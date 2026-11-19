// =====================================================================
//  GLITCHED (Team 190) - autonomous test ladder (line following only, no arm)
//  File: RobotFirmware/opmodes/AutoLadder.cpp
//
//  Four AUTO OpModes, each a bigger piece of the course than the one before. Run them in
//  order. Each one stops itself and shows where it got to, so a failure points at one step.
//
//    "L1 Straight"         leave the Start box, follow the line straight, stop before corner C1
//    "L2 First Junction"   leave the box, round corner C1, follow to the first junction
//                          (the P1 branch) and stop just past it
//    "L3 Reach P1 End"     the same, then turn right into the P1 terminal and stop on its end bar
//    "L4 Course + Park"    leave the box, all four left corners, into the Park box, park
//
//  Start the robot CENTRED in the right-hand (START) box, facing the line that leaves it,
//  after a LINE TEST calibration on this mat. Calibrate white first, then black.
//
//  SCREEN (8 lines)
//    stage   what it is doing now        fault   why it stopped (NONE while all is well)
//    t       seconds since start         jx      junctions counted
//    cnr     corners counted             last    last junction as L / R / S (left, right, straight)
//    turn    degrees of the last turn    pat     what the line array sees right now
//
//  Dead channel: sensor 7 is dead and reads 0. The SDK's own box detector (followToBox) wants
//  ALL 8 channels dark, which cannot happen. L4 therefore uses the array's TERMINAL pattern
//  (7 of 8 dark) held for BOX_CONFIRM_MS, which a dead channel does not stop.
//
//  If the robot stops with a fault, send me: stage, fault, jx, cnr, last, turn, pat.
//
//  STATUS: written against the NRL SDK v1.5.0 headers. NOT compiled or run on the robot yet.
// =====================================================================
#include "NRL.h"

// ---------------------------------------------------------------------
//  Hardware (same motor settings as AutoMission.cpp: both motors reversed, 2026-11-19)
// ---------------------------------------------------------------------
static HexaDCMotor leftMotor{{
    .dirPin = MOTOR_L_DIR,
    .pwmPin = MOTOR_L_PWM,
    .minDrive = 29        // TUNE: re-measure after any motor change
}};

static HexaDCMotor rightMotor{{
    .dirPin = MOTOR_R_DIR,
    .pwmPin = MOTOR_R_PWM,
    .flipped = true,
    .minDrive = 32        // TUNE: re-measure after any motor change
}};

static TankDrive drive(leftMotor, rightMotor);
static HexaLineArray line{hexaLineConfig()};

// ---------------------------------------------------------------------
//  Tuning. Defaults are the SDK's, written out so you can see and change them.
// ---------------------------------------------------------------------
constexpr unsigned long STRAIGHT_MS    = 700;   // TUNE: L1 follow time. About 0.4 m: short of corner C1
constexpr unsigned long BOX_CONFIRM_MS = 150;   // L4: TERMINAL pattern held this long = inside the box
constexpr long          PARK_MS        = 400;   // TUNE: L4 drive on into the box (about 250 mm = the middle)
constexpr unsigned long SOFT_STOP_MS   = 145000;

static LineTuning makeTuning()
{
    LineTuning t;
    t.followSpeed   = 0.35f;
    t.junctionSpeed = 0.25f;
    t.steerSign     = 1;        // -1 if the robot steers AWAY from the line
    // Corner turn: the SDK rolls forward pivotMs so the chassis pivot sits on the corner, then
    // spins until the line is centred. 60 ms was tuned on a different robot: measure the
    // distance from your sensor bar to the wheel axle and send it to me if corners fail.
    t.pivotMs       = 60;       // TUNE
    t.pivotSpeed    = 0.25f;
    t.turnSpeed     = 0.45f;
    t.turnSlowSpeed = 0.30f;
    t.turnMinMs     = 200;
    return t;
}

static LineActions lineActions(drive, line, makeTuning());

// ---------------------------------------------------------------------
//  Run state shown on the controller screen
// ---------------------------------------------------------------------
static const char *stage    = "ready";
static uint32_t    runStart = 0;
static uint32_t    boxSince = 0;

static ActionPtr at(const char *name)
{
    return instant([name] { stage = name; });
}

// True once the array has seen TERMINAL (7 or more channels dark) for BOX_CONFIRM_MS in a row.
// Crossing a tape only darkens the array for a few tens of ms, so it does not trigger this.
static bool inBox()
{
    if (line.pattern() == LinePattern::TERMINAL)
    {
        if (boxSince == 0) boxSince = millis() | 1;
        return millis() - boxSince >= BOX_CONFIRM_MS;
    }
    boxSince = 0;
    return false;
}

// ---------------------------------------------------------------------
//  Missions
// ---------------------------------------------------------------------
static ActionPtr l1Straight()
{
    return lineActions.auto_()
        .then(at("leave box")).leaveBox()
        .then(at("straight")).followFor(STRAIGHT_MS)
        .then(at("DONE"))
        .build();
}

static ActionPtr l2FirstJunction()
{
    return lineActions.auto_()
        .then(at("leave box")).leaveBox()
        .then(at("to junction")).followToJunction()       // corners turn by themselves
        .then(at("DONE"))
        .build();
}

static ActionPtr l3ReachP1End()
{
    return lineActions.auto_()
        .then(at("leave box")).leaveBox()
        .then(at("to P1")).followToJunction(LineSide::RIGHT, 1)
        .then(at("turn in")).right()
        .then(at("P1 to end")).followToEnd()
        .then(at("DONE"))
        .build();
}

static ActionPtr l4CoursePark()
{
    return lineActions.auto_()
        .then(instant([] { boxSince = 0; }))
        .then(at("leave box")).leaveBox()
        .then(at("round course")).followUntil([] { return inBox(); })
        .then(at("parking")).park(PARK_MS)
        .then(at("DONE"))
        .build();
}

// ---------------------------------------------------------------------
//  Shared OpMode behaviour
// ---------------------------------------------------------------------
class LadderBase : public NRLOpMode
{
public:
    // INIT stays motionless (R8.11).
    void init() override
    {
        leftMotor.begin();
        rightMotor.begin();
        line.begin();
    }

    void start() override
    {
        lineActions.clearFault();
        lineActions.resetCounts();
        lineActions.clearRoute();      // also sets grid 0 degrees = the way the robot faces now
        stage    = "ready";
        runStart = millis();
        runAction(mission());
    }

    void loop() override
    {
        const uint32_t elapsed = millis() - runStart;

        // Own safety net: stop before the 150 s limit.
        if (elapsed > SOFT_STOP_MS && isActionRunning())
        {
            cancelActions();
            drive.stop();
            stage = "TIME";
        }

        const LineJunction &j = lineActions.lastJunction();
        char lrs[4] = {j.left ? 'L' : '-', j.right ? 'R' : '-', j.straight ? 'S' : '-', 0};

        telemetry.addData("stage", stage);
        telemetry.addData("fault", lineActions.faultName());
        telemetry.addData("t", (float)(elapsed / 1000));
        telemetry.addData("jx", (float)lineActions.junctionCount());
        telemetry.addData("cnr", (float)lineActions.cornerCount());
        telemetry.addData("last", lrs);
        telemetry.addData("turn", lineActions.lastTurnDegrees());
        telemetry.addData("pat", linePatternName(line.pattern()));
    }

    void stop() override { drive.stop(); }

protected:
    virtual ActionPtr mission() = 0;
};

class L1Straight : public LadderBase
{
protected:
    ActionPtr mission() override { return l1Straight(); }
};

class L2FirstJunction : public LadderBase
{
protected:
    ActionPtr mission() override { return l2FirstJunction(); }
};

class L3ReachP1End : public LadderBase
{
protected:
    ActionPtr mission() override { return l3ReachP1End(); }
};

class L4CoursePark : public LadderBase
{
protected:
    ActionPtr mission() override { return l4CoursePark(); }
};

REGISTER_OPMODE(L1Straight,      "L1 Straight",        AUTO);
REGISTER_OPMODE(L2FirstJunction, "L2 First Junction",  AUTO);
REGISTER_OPMODE(L3ReachP1End,    "L3 Reach P1 End",    AUTO);
REGISTER_OPMODE(L4CoursePark,    "L4 Course + Park",   AUTO);