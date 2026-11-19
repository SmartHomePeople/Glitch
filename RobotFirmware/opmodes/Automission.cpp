// =====================================================================
//  GLITCHED (Team 190) - NRL 2026 Autonomous Skill Challenge
//  File: RobotFirmware/opmodes/AutoMission.cpp
//
//  One OpMode lives in this file (2026-11-19: Stage 1 and Stage 2 removed to declutter the
//  AUTO menu; Start-to-Park is now "Park Run" in AutoPark.cpp):
//    "Box + Pick Test"      TELEOP  bench tests for the box steps and the arm
//  NOTE: its D-pad RIGHT uses the SDK's followToBox(), which needs all 8 channels dark; with
//  channel 7 dead it may never see the box.
//
//  Start the robot CENTRED in the right-hand Start box, facing right (towards
//  the line that leaves the box), after a LINE TEST calibration on this mat.
//
//  STATUS: written against the NRL SDK v1.5.0 headers. NOT yet compiled or
//  run on the robot. Every value marked TUNE needs measuring on the mat.
// =====================================================================
#include "NRL.h"

// ---------------------------------------------------------------------
//  Hardware (same pins and motor settings as AutoLinos.cpp)
// ---------------------------------------------------------------------
static HexaDCMotor leftMotor{{
    .dirPin = MOTOR_L_DIR,
    .pwmPin = MOTOR_L_PWM,
    .minDrive = 34        // Min Drive test 2026-11-19: left max 31 (pivot) + 3
}};

static HexaDCMotor rightMotor{{
    .dirPin = MOTOR_R_DIR,
    .pwmPin = MOTOR_R_PWM,
    .flipped = true,      // 2026-11-19: both motors reversed (was left only) - the new motors drove backward and turned the wrong way
    .minDrive = 32        // Min Drive test 2026-11-19: right max 29 (pivot) + 3
}};

static TankDrive drive(leftMotor, rightMotor);
static HexaLineArray line{hexaLineConfig()};
static LineActions lineActions(drive, line);

// ---------------------------------------------------------------------
//  Arm and gripper (angles measured with SERVO TEST; lower gripper angle = open)
// ---------------------------------------------------------------------
constexpr float ARM_STOW   = 110.0f;   // start pose: robot inside the Start box
constexpr float ARM_DOWN   = 148.0f;   // gripper at floor level
constexpr float ARM_UP     =  85.0f;   // carrying pose
constexpr float GRIP_OPEN  =   5.0f;
constexpr float GRIP_CORE  =  75.0f;   // closed on a CORE
constexpr float GRIP_EMPTY =  85.0f;   // closed on nothing
constexpr float ARM_DEG_S  = 100.0f;   // ramp speeds: slow enough not to fling the CORE
constexpr float GRIP_DEG_S = 300.0f;

// Servos start at the stowed pose and HOLD it (settleMs defaults to 0), so INIT moves nothing.
static HexaServo arm{{.signalPin = SERVO_1, .startAngle = ARM_STOW}};
static HexaServo gripper{{.signalPin = SERVO_2, .startAngle = GRIP_EMPTY}};

// ---------------------------------------------------------------------
//  Tunable mission values - measure these on the mat (see the Reach sheet)
// ---------------------------------------------------------------------
static long  parkMs      = 400;    // TUNE: ms to drive on into the box after followToBox() (about 250 mm = middle)
constexpr float REACH_IN = 0.0f;   // TUNE: extra inches forward at an end bar before closing on the CORE (c - (s + Rg))
constexpr float DROP_IN  = 0.0f;   // TUNE: extra inches forward at an end bar before releasing the CORE

// ---------------------------------------------------------------------
//  Run state shown on the controller screen
// ---------------------------------------------------------------------
static const char *stage    = "ready";
static bool        carrying = false;       // true while a CORE is held (one CORE at a time)

static ActionPtr at(const char *name)
{
    return instant([name] { stage = name; });
}

// Move a servo smoothly to `to`, then wait `dwellMs` more. The servo's ramp only
// advances while setPosition() keeps being called, so this calls it every tick.
static ActionPtr moveServo(HexaServo &s, float to, unsigned long dwellMs)
{
    auto arrivedMs = std::make_shared<uint32_t>(0);
    return sequential({
        instant([&s, arrivedMs] {
            s.setPosition(s.getPosition());   // restart the ramp clock without moving
            *arrivedMs = 0;
        }),
        wait_until([&s, to, dwellMs, arrivedMs] {
            if (*arrivedMs == 0)
            {
                s.setPosition(to);
                if (fabsf(s.getPosition() - to) < 0.5f)
                    *arrivedMs = millis() | 1;
                return false;
            }
            return millis() - *arrivedMs >= dwellMs;
        }),
    });
}

// Pick up: open, arm down, (optional creep forward), close on the CORE, arm up.
static ActionPtr pickCore()
{
    std::vector<ActionPtr> steps;
    steps.push_back(at("PICK"));
    steps.push_back(moveServo(gripper, GRIP_OPEN, 150));
    steps.push_back(moveServo(arm, ARM_DOWN, 250));
    if (REACH_IN > 0.01f)
        steps.push_back(lineActions.driveInches(REACH_IN));
    steps.push_back(moveServo(gripper, GRIP_CORE, 300));
    steps.push_back(moveServo(arm, ARM_UP, 200));
    steps.push_back(instant([] { carrying = true; }));
    return sequential(steps);
}

// Drop: (optional creep forward), arm down, open, arm up, close empty.
static ActionPtr dropCore()
{
    std::vector<ActionPtr> steps;
    steps.push_back(at("DROP"));
    if (DROP_IN > 0.01f)
        steps.push_back(lineActions.driveInches(DROP_IN));
    steps.push_back(moveServo(arm, ARM_DOWN, 250));
    steps.push_back(moveServo(gripper, GRIP_OPEN, 250));
    steps.push_back(moveServo(arm, ARM_UP, 200));
    steps.push_back(moveServo(gripper, GRIP_EMPTY, 100));
    steps.push_back(instant([] { carrying = false; }));
    return sequential(steps);
}

// ---------------------------------------------------------------------
//  Bench test (TELEOP): run ONE step at a time from a chosen spot.
//    D-pad up    leaveBox                      (robot placed in the Start box)
//    D-pad left  followToJunction              (stops past the next junction)
//    X           followToEnd                   (stops on the end bar of a terminal)
//    D-pad right followToBox + park(parkMs)    (run towards a box)
//    A           pick up routine (arm and gripper only, robot stays still)
//    Y           drop routine    (arm and gripper only, robot stays still)
//    LB / RB     parkMs  -50 / +50
//    LT          stop (the engine exit control)
// ---------------------------------------------------------------------
class BoxPickTest : public NRLOpMode
{
public:
    void init() override
    {
        leftMotor.begin();
        rightMotor.begin();
        line.begin();
        arm.setRampRate(ARM_DEG_S);
        gripper.setRampRate(GRIP_DEG_S);
        arm.begin();
        gripper.begin();
    }

    void start() override
    {
        lineActions.clearFault();
        lineActions.resetCounts();
        lineActions.clearRoute();
        carrying = false;
        stage    = "ready";
    }

    void loop() override
    {
        if (gamepad1.justPressed(BTN_LB)) parkMs -= 50;
        if (gamepad1.justPressed(BTN_RB)) parkMs += 50;

        if (!isActionRunning())
        {
            if (gamepad1.justPressed(BTN_DPAD_UP))
            {
                lineActions.clearFault();
                runAction(lineActions.leaveBox());
            }
            if (gamepad1.justPressed(BTN_DPAD_LEFT))
            {
                lineActions.clearFault();
                runAction(lineActions.followToJunction());
            }
            if (gamepad1.justPressed(BTN_X))
            {
                lineActions.clearFault();
                runAction(lineActions.followToEnd());
            }
            if (gamepad1.justPressed(BTN_DPAD_RIGHT))
            {
                lineActions.clearFault();
                runAction(lineActions.auto_().followToBox().park(parkMs).build());
            }
            if (gamepad1.justPressed(BTN_A)) runAction(pickCore());
            if (gamepad1.justPressed(BTN_Y)) runAction(dropCore());
        }

        telemetry.addData("stage", stage);
        telemetry.addData("fault", lineActions.faultName());
        telemetry.addData("busy", isActionRunning() ? "yes" : "no");
        telemetry.addData("parkMs", (float)parkMs);
        telemetry.addData("jx", (float)lineActions.junctionCount());
        telemetry.addData("err", (float)line.error());
        telemetry.addData("turn", lineActions.lastTurnDegrees());
        telemetry.addData("hold", carrying ? "CORE" : "-");
    }

    void stop() override { drive.stop(); }
};

REGISTER_OPMODE(BoxPickTest, "Box + Pick Test", TELEOP);