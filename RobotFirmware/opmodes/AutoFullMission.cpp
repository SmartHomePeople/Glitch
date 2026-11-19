// =====================================================================
//  GLITCHED (Team 190) - NRL 2026 Autonomous Skill Challenge
//  File: RobotFirmware/opmodes/AutoFullMission.cpp
//
//  OpMode "Full Mission" (AUTO): leave the START box, collect CORE from
//  Pick-Up terminals, drop them in the DROP ZONE one at a time, and park.
//
//  HOW THE ROUTE WORKS
//  The robot drives one way round the loop: east out of START, north up the
//  right side, west along the top, south down the left side, east into PARK.
//  Going that way EVERY Pick-Up terminal is a branch on the RIGHT and every
//  Drop terminal is a branch on the LEFT, so the whole mission is built from
//  one pick pattern and one drop pattern. Corners turn by themselves.
//
//      terminals in travel order:
//      P1(5) D1 P2 | P3(gold) D2 P4(gold) | P5 D3 P6
//
//  Default plan (forward-only, golds included):
//      P1 -> D1      red   5 + 10
//      P3 -> D2      gold  5 + 25
//      P4 -> D3      gold  5 + 25
//      P6 pick only  blue  5          (carried to PARK; no drop points)
//      leave 5 + park 30               = 115 points if everything lands
//  Phases are skipped automatically when the clock runs short, so the run
//  always tries to finish with a park.
//
//  START POSITION: centred in the right-hand START box, FACING EAST (towards
//  the short piece of line that leaves the box towards the bottom-right
//  corner), square to that line. Calibrate LINE TEST on this mat first.
//
//  STATUS: written against the NRL SDK v1.5.0 headers. NOT compiled, NOT run
//  on the robot. Every value marked TUNE must be measured on the mat.
// =====================================================================
#include "NRL.h"

// ---------------------------------------------------------------------
//  Hardware
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
//  Arm and gripper (measured with SERVO TEST; a LOWER gripper angle is open)
// ---------------------------------------------------------------------
constexpr float ARM_STOW   = 110.0f;
constexpr float ARM_DOWN   = 148.0f;
constexpr float ARM_UP     =  85.0f;
constexpr float GRIP_OPEN  =   5.0f;
constexpr float GRIP_CORE  =  75.0f;
constexpr float GRIP_EMPTY =  85.0f;
constexpr float ARM_DEG_S  = 100.0f;
constexpr float GRIP_DEG_S = 300.0f;

// settleMs defaults to 0, so the servos hold their start pose and INIT moves nothing (R8.11).
static HexaServo arm{{.signalPin = SERVO_1, .startAngle = ARM_STOW}};
static HexaServo gripper{{.signalPin = SERVO_2, .startAngle = GRIP_EMPTY}};

// ---------------------------------------------------------------------
//  Mission tuning - all of these want measuring on the real mat
// ---------------------------------------------------------------------
constexpr long          PARK_MS        = 400;      // TUNE: ms into the PARK box after followToBox()
constexpr float         REACH_IN       = 0.0f;     // TUNE: inches to creep at a Pick-Up end bar before gripping
constexpr float         DROP_IN        = 0.0f;     // TUNE: inches to creep at a Drop end bar before releasing
constexpr unsigned long RUN_LIMIT_MS   = 150000;   // official limit
constexpr unsigned long SOFT_STOP_MS   = 145000;   // our own stop, a little before the limit
constexpr unsigned long PARK_RESERVE_MS = 40000;   // TUNE: time kept back to reach PARK from the far side
constexpr bool          PARK_AFTER_FAULT = true;   // after a fault, still try to park (one attempt)

// ---------------------------------------------------------------------
//  Run state (shown on the controller screen)
// ---------------------------------------------------------------------
static const char *stage     = "ready";
static bool        carrying  = false;   // one CORE at a time
static int         picks     = 0;
static int         drops     = 0;
static uint32_t    runStart  = 0;
static bool        recovered = false;

static ActionPtr at(const char *name)
{
    return instant([name] { stage = name; });
}

// Move a servo smoothly to `to`, then dwell. The servo's ramp only advances
// while setPosition() keeps being called, so this calls it every tick.
static ActionPtr moveServo(HexaServo &s, float to, unsigned long dwellMs)
{
    auto arrivedMs = std::make_shared<uint32_t>(0);
    return sequential({
        instant([&s, arrivedMs] {
            s.setPosition(s.getPosition());
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

static ActionPtr pickCore()
{
    std::vector<ActionPtr> s;
    s.push_back(at("PICK"));
    s.push_back(moveServo(gripper, GRIP_OPEN, 150));
    s.push_back(moveServo(arm, ARM_DOWN, 250));
    if (REACH_IN > 0.01f) s.push_back(lineActions.driveInches(REACH_IN));
    s.push_back(moveServo(gripper, GRIP_CORE, 300));
    s.push_back(moveServo(arm, ARM_UP, 200));
    s.push_back(instant([] { carrying = true; picks++; }));
    return sequential(s);
}

static ActionPtr dropCore()
{
    std::vector<ActionPtr> s;
    s.push_back(at("DROP"));
    if (DROP_IN > 0.01f) s.push_back(lineActions.driveInches(DROP_IN));
    s.push_back(moveServo(arm, ARM_DOWN, 250));
    s.push_back(moveServo(gripper, GRIP_OPEN, 250));
    s.push_back(moveServo(arm, ARM_UP, 200));
    s.push_back(moveServo(gripper, GRIP_EMPTY, 100));
    s.push_back(instant([] { carrying = false; drops++; }));
    return sequential(s);
}

// ---------------------------------------------------------------------
//  Route building blocks
//
//  Travelling one way round the loop, a Pick-Up terminal is always a branch
//  on the RIGHT and a Drop terminal always a branch on the LEFT.
//
//  Leaving a terminal the robot has turned round, so it meets the main line
//  facing the opposite way: after a Pick-Up it resumes with another right
//  turn, after a Drop with another left turn. That is why each pattern ends
//  with the same turn it started with.
//
//  `skip` is how many terminals on that side to pass before turning in, so
//  skip = 0 means the next one.
// ---------------------------------------------------------------------
static ActionPtr pickAt(const char *label, int skip)
{
    return lineActions.auto_()
        .then(at(label))
        .followToJunction(LineSide::RIGHT, skip + 1)   // stop just past the junction
        .right()                                       // onto the Pick-Up terminal
        .followToEnd()                                 // up to its end bar
        .then(pickCore())
        .uTurn()
        .followToJunction()                            // back out to the main line
        .right()                                       // resume the way we were going
        .build();
}

static ActionPtr dropAt(const char *label, int skip)
{
    return lineActions.auto_()
        .then(at(label))
        .followToJunction(LineSide::LEFT, skip + 1)
        .left()                                        // onto the Drop terminal
        .followToEnd()                                 // up to its end bar, at the DROP ZONE
        .then(dropCore())
        .uTurn()
        .followToJunction()
        .left()                                        // resume the way we were going
        .build();
}

// ---------------------------------------------------------------------
//  The plan: one phase per line. `costMs` is this phase's time estimate and
//  is only used to decide whether to start it - TUNE them from real runs.
// ---------------------------------------------------------------------
struct Phase
{
    const char *name;
    ActionPtr (*make)();
    unsigned long costMs;
    bool          optional;    // false = run it even if the clock looks tight
};

static ActionPtr phLeave()  { return lineActions.auto_().then(at("leave box")).leaveBox().build(); }
static ActionPtr phP1()     { return pickAt("P1 pick", 0); }                 // first right
static ActionPtr phD1()     { return dropAt("D1 drop", 0); }                 // first left
static ActionPtr phP3()     { return pickAt("P3 pick", 1); }                 // pass P2, turn in at P3 (gold)
static ActionPtr phD2()     { return dropAt("D2 drop", 0); }
static ActionPtr phP4()     { return pickAt("P4 pick", 0); }                 // gold
static ActionPtr phD3()     { return dropAt("D3 drop", 0); }                 // passes P5 (a right branch)
static ActionPtr phP6()     { return pickAt("P6 pick", 0); }                 // carried to PARK
static ActionPtr phPark()
{
    return lineActions.auto_()
        .then(at("to PARK"))
        .followToBox()
        .then(at("parking"))
        .park(PARK_MS)
        .then(at("DONE"))
        .build();
}

static const Phase PLAN[] = {
    {"leave",   phLeave, 6000,  false},
    {"P1 pick", phP1,   16000,  true},
    {"D1 drop", phD1,   16000,  true},
    {"P3 pick", phP3,   20000,  true},
    {"D2 drop", phD2,   16000,  true},
    {"P4 pick", phP4,   16000,  true},
    {"D3 drop", phD3,   20000,  true},
    {"P6 pick", phP6,   16000,  true},
};
constexpr int PLAN_LEN = sizeof(PLAN) / sizeof(PLAN[0]);

// ---------------------------------------------------------------------
//  OpMode
// ---------------------------------------------------------------------
class FullMission : public NRLOpMode
{
public:
    // INIT must not move anything (R8.11): begin() only holds the start poses.
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
        lineActions.clearRoute();      // also fixes grid 0 degrees = the way the robot faces now
        carrying  = false;
        picks = drops = 0;
        recovered = false;
        _next     = 0;
        _parking  = false;
        _finished = false;
        stage     = "start";
        runStart  = millis();
    }

    void loop() override
    {
        const uint32_t elapsed   = millis() - runStart;
        const long     remaining = (long)RUN_LIMIT_MS - (long)elapsed;

        // Our own safety net, a little before the official limit.
        if (!_finished && elapsed > SOFT_STOP_MS)
        {
            if (isActionRunning()) cancelActions();
            drive.stop();
            stage     = "TIME";
            _finished = true;
        }

        if (!_finished && !isActionRunning())
        {
            // A phase that faulted stops the robot and skips its own later steps.
            if (lineActions.fault() != LineFault::NONE)
            {
                if (PARK_AFTER_FAULT && !recovered && !_parking)
                {
                    recovered = true;              // one attempt only
                    lineActions.clearFault();
                    startParking();
                }
                else
                {
                    drive.stop();
                    stage     = "FAULT STOP";
                    _finished = true;
                }
            }
            else if (_parking)
            {
                drive.stop();                      // park() finished: the run is over
                stage     = "PARKED";
                _finished = true;
            }
            else
            {
                // Pick the next phase we still have time for; otherwise go and park.
                while (_next < PLAN_LEN &&
                       PLAN[_next].optional &&
                       remaining < (long)(PLAN[_next].costMs + PARK_RESERVE_MS))
                    _next++;                       // skipped: not enough clock left

                if (_next < PLAN_LEN)
                    runAction(PLAN[_next++].make());
                else
                    startParking();
            }
        }

        const LineJunction &j = lineActions.lastJunction();
        char lrs[4] = {j.left ? 'L' : '-', j.right ? 'R' : '-', j.straight ? 'S' : '-', 0};

        telemetry.addData("stage", stage);
        telemetry.addData("fault", lineActions.faultName());
        telemetry.addData("t", (float)(elapsed / 1000));
        telemetry.addData("phase", (float)_next);
        telemetry.addData("pick", (float)picks);
        telemetry.addData("drop", (float)drops);
        telemetry.addData("jx", (float)lineActions.junctionCount());
        telemetry.addData("last", lrs);
        telemetry.addData("hold", carrying ? "CORE" : "-");
    }

    void stop() override { drive.stop(); }

private:
    void startParking()
    {
        _parking = true;
        runAction(phPark());
    }

    int  _next     = 0;
    bool _parking  = false;
    bool _finished = false;
};

REGISTER_OPMODE(FullMission, "Full Mission", AUTO);