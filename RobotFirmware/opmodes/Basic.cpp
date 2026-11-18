// =====================================================================
//  GLITCHED (Team 190) - driver-controlled OpMode
//  File: RobotFirmware/opmodes/Basic.cpp
//
//  Controller map (gamepad1):
//    Left stick up/down ... drive forward / backward
//    Right stick left/right  turn
//    Right stick up/down ... move the arm
//    A (HOME) ............. toggle TURBO mode
//    LB ................... toggle EXTREME PRECISION mode
//    RB ................... toggle the gripper open / closed on a CORE
//    D-pad UP ............. EMERGENCY STOP (latched, see below)
//    LT toggle ............ engine exit / stop (handled by the engine, not remapped here)
//
//  Drive modes (A and LB are mutually exclusive; press again to go back to NORMAL)
//    NORMAL   75% top speed, soft centre (more precise than before)
//    TURBO    100% top speed, linear response
//    PRECISE  30% top speed, very soft centre
//
//  Emergency stop: D-pad UP freezes the robot. The motors stop, all other
//  inputs are ignored, and the arm and gripper stay exactly where they are.
//  It stays latched until the OpMode is stopped and started again.
//
//  STATUS: not yet compiled or run on the robot.
// =====================================================================
#include "NRL.h"

static HexaDCMotor leftMotor{{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor rightMotor{{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM }};
static TankDrive drive(leftMotor, rightMotor);

// ---------------------------------------------------------------------
//  Arm and gripper angles (measured with SERVO TEST; a LOWER gripper angle is open)
// ---------------------------------------------------------------------
constexpr float ARM_START   = 110.0f;   // stowed pose, robot fits the start box
constexpr float ARM_MIN_DEG =  85.0f;   // arm up (carry)
constexpr float ARM_MAX_DEG = 160.0f;   // arm down for picking up a CORE (was 140 = floor level). If it presses on the floor or the servo strains, lower this in small steps
constexpr float ARM_STEP    =   1.0f;   // degrees per 10 ms tick at full stick

constexpr float GRIP_OPEN   =   5.0f;   // open: the gripper STARTS here
constexpr float GRIP_CORE   =  75.0f;   // closed on a CORE: tighter hold (was 70, originally 65). Fully closed with nothing in it is 85, so keep this below 85
constexpr float GRIP_DEG_S  = 300.0f;   // closing and opening speed in degrees per second (was 150)
constexpr float ARM_DEG_S   = 120.0f;

// settleMs = 0: the servos HOLD their angle. The gripper starts open and the arm starts
// stowed, so INIT does not sweep anything (R8.11). No setPosition() calls are needed in init().
static HexaServo arm{{ .signalPin = SERVO_1, .startAngle = ARM_START, .settleMs = 0 }};
static HexaServo gripper{{ .signalPin = SERVO_2, .startAngle = GRIP_OPEN, .settleMs = 0 }};

// ---------------------------------------------------------------------
//  Drive modes
// ---------------------------------------------------------------------
enum DriveMode { MODE_NORMAL, MODE_TURBO, MODE_PRECISE };

struct ModeSettings
{
    const char *name;
    float scale;       // top speed as a fraction of full power
    float expo;        // 0 = linear stick, 1 = fully cubic (very soft near the centre)
    float ramp;        // how fast motor commands may change per tick
};

static const ModeSettings MODES[] = {
    {"NORMAL",  0.75f, 0.5f, 0.20f},
    {"TURBO",   1.00f, 0.0f, 0.30f},
    {"PRECISE", 0.30f, 0.8f, 0.12f},
};

constexpr float DEADBAND = 0.05f;   // ignore tiny stick drift

// Soft centre: blends the stick with its cube, so small pushes give small speeds
// and full deflection still gives full speed.
static float shape(float x, float expo)
{
    if (fabsf(x) < DEADBAND) return 0.0f;
    return (1.0f - expo) * x + expo * x * x * x;
}

class Basic : public NRLOpMode
{
public:
    void init() override
    {
        leftMotor.begin();
        rightMotor.begin();
        arm.setRampRate(ARM_DEG_S);
        gripper.setRampRate(GRIP_DEG_S);
        arm.begin();
        gripper.begin();
        applyMode(MODE_NORMAL);
    }

    void start() override
    {
        estop       = false;
        gripClosed  = false;      // the gripper starts OPEN
        armAngle    = ARM_START;
        applyMode(MODE_NORMAL);
    }

    void loop() override
    {
        // Emergency stop: latched. The motors are held at zero, nothing else is commanded,
        // so the arm and gripper stay exactly where they were.
        if (gamepad1.justPressed(BTN_DPAD_UP)) estop = true;
        if (estop)
        {
            drive.stop();
            telemetry.addData("STATE", "E-STOP");
            return;
        }

        // A and LB toggle between their mode and NORMAL.
        if (gamepad1.justPressed(BTN_A)) applyMode(mode == MODE_TURBO ? MODE_NORMAL : MODE_TURBO);
        if (gamepad1.justPressed(BTN_LB)) applyMode(mode == MODE_PRECISE ? MODE_NORMAL : MODE_PRECISE);

        // Drive: left stick up/down = forward/back, right stick left/right = turn.
        //
        // NOTE: the arguments go into drive() in the opposite order to the SDK's
        // drive(forward, turn), exactly as in the original Basic.cpp. That order and these
        // signs were set from the robot: after testing, left/right was correct and forward
        // was inverted, so only the forward sign changed. If the motors are rewired or
        // `flipped` is set later, this will need re-checking.
        const ModeSettings &m = MODES[mode];
        const float turnStick = shape(gamepad1.rightX(), m.expo);
        const float fwdStick  = shape(gamepad1.leftY(), m.expo);   // was -leftY; flipped to fix inverted forward
        drive.drive(turnStick, fwdStick);

        // Gripper: one press closes on the CORE, the next press opens. Calling
        // setPosition() every tick lets the ramp close it gently.
        if (gamepad1.justPressed(BTN_RB)) gripClosed = !gripClosed;
        gripper.setPosition(gripClosed ? GRIP_CORE : GRIP_OPEN);

        // Arm: the stick moves the target angle, which is kept inside the safe range.
        const float rightY = gamepad1.rightY();   // was -rightY; flipped because stick up moved the arm down
        if (fabsf(rightY) > DEADBAND) armAngle += rightY * ARM_STEP;
        if (armAngle < ARM_MIN_DEG) armAngle = ARM_MIN_DEG;
        if (armAngle > ARM_MAX_DEG) armAngle = ARM_MAX_DEG;
        arm.setPosition(armAngle);

        telemetry.addData("mode", m.name);
        telemetry.addData("grip", gripClosed ? "CLOSED" : "OPEN");
        telemetry.addData("arm", armAngle);
        telemetry.addData("fwd", fwdStick);
        telemetry.addData("turn", turnStick);
    }

    void stop() override
    {
        rightMotor.stop();
        leftMotor.stop();
        arm.detach();
        gripper.detach();
    }

private:
    void applyMode(DriveMode m)
    {
        mode = m;
        drive.setScale(MODES[m].scale);
        drive.setRampRate(MODES[m].ramp);
    }

    DriveMode mode       = MODE_NORMAL;
    bool      estop      = false;
    bool      gripClosed = false;
    float     armAngle   = ARM_START;
};

REGISTER_OPMODE(Basic, "Basic", TELEOP);