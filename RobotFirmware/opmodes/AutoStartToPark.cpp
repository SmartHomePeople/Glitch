// ============================================================
//  AutoStartToPark.cpp — "Start to Park"  (AUTO)
//  GLITCHED, Team 190, robot "Pixel".  NOT YET RUN ON THE ROBOT.
// ============================================================
//  Route (Autonomous Skill Challenge manual, pages 5-6):
//
//    START box (bottom line, right of centre)
//      -> 300 mm east to corner C1 (bottom-right)      corner: auto left turn
//      -> up the right side, passing P1, D1, P2         junctions: drive through
//      -> corner C2 (top-right)                         corner: auto left turn
//      -> along the top, passing P3, D2, P4
//      -> corner C3 (top-left)                          corner: auto left turn
//      -> down the left side, passing P5, D3, P6
//      -> corner C4 (bottom-left)                       corner: auto left turn
//      -> 300 mm east into the PARK box, stop           Park = 30 pts
//
//  PLACE THE ROBOT: centred and square in the START box, FACING RIGHT (east,
//  toward C1), sensor bar on the main line's axis. LineActions records "grid
//  0 degrees" from the gyro at the first step, so square matters.
//
//  Everything line-related is the SDK's LineActions (same engine as the
//  "First Junction" fallback): leaveBox() / followToBox() handle the black
//  START and PARK squares, corners turn themselves, and the T-junctions
//  (P1-P6, D1-D3 all have the line carrying on straight) are passed.
//  The last stretch into the PARK box is the SDK's timed park(ms) (no encoders).
//
//  Needs LINE TEST calibration on the mat (white first, then black).
// ============================================================
#include "NRL.h"

// ── Park ──────────────────────────────────────────────────────────────────
// Park: followToBox() brakes with the sensor bar just inside the PARK square;
// then the SDK's park(ms) drives straight on at tuning.parkSpeed for PARK_MS
// and stops. NO ENCODERS (20 Nov). Same goal as before: ~0.25 m further, so
// the chassis middle sits near the box centre. 600 ms at 0.22 is an ESTIMATE
// (speed at 0.22 not measured). Tune on the mat: all drive wheels must be on
// the black square -> wheels short of the box: raise; robot past it: lower.
static constexpr float    PARK_SPEED       = 0.22f;
static constexpr uint16_t PARK_MS          = 600;

// ── Hardware ───────────────────────────────────────────────────────────────
// Motor direction + floor from the 19 Nov Min Drive test: left NOT flipped,
// right flipped. (AutoLinos.cpp still has the older left-flipped values.)
static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM,
                                .flipped = false, .minDrive = 34 }};
static HexaDCMotor rightMotor{{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM,
                                .flipped = true,  .minDrive = 32 }};
static TankDrive drive(leftMotor, rightMotor);

// Arm held in the stowed pose so it can't sag in front of the sensor bar.
// startAngle = stowed (110) and settleMs = 0: begin() commands the pose the
// arm is already in, so INIT does not move it (R8.11) as long as the arm is
// stowed by hand before INIT. The gripper is not touched in this OpMode.
static HexaServo arm{{ .signalPin = SERVO_1, .startAngle = 110.0f, .settleMs = 0 }};

static HexaLineArray line{ hexaLineConfig() };

// ── Line-following tuning ──────────────────────────────────────────────────
// SDK defaults (LineTuning.h) except where noted. First knobs to touch:
//   pivotMs    — corner/turn lands short -> raise; turns past the line -> lower.
//   followSpeed — lower if corners or junctions are missed.
static LineTuning parkTuning() {
    LineTuning t;
    t.followSpeed   = 0.35f;   // SDK default
    t.junctionSpeed = 0.25f;   // SDK default
    // Runs 1-2 (20 Nov): the roll before the corner looked right at 60 and the
    // turn itself looked good, but it stopped just short of the new line and
    // faulted TURN_TIMEOUT (C2 on run 1, C1 on run 2). Cause not known yet:
    // read "mask" and "turn deg" on screen at the next fault.
    t.pivotMs       = 60;      // SDK default; looked right on the robot
    t.pivotSpeed    = 0.25f;   // SDK default
    t.turnSpeed     = 0.45f;   // SDK default
    t.turnSlowSpeed = 0.30f;   // SDK default; must stay above the spin floor
    t.parkSpeed     = PARK_SPEED;   // park(ms) speed, as the old park drive
    return t;
}
static LineActions lineActions(drive, line, parkTuning());

// What the routine is doing, for telemetry. Set by instant() steps, which the
// builder skips after a fault, so it shows the last step that actually ran.
static const char* stepName = "idle";
static ActionPtr step(const char* name) { return instant([name] { stepName = name; }); }

class AutoStartToPark : public NRLOpMode {
public:
    void init() override {
        // Nothing here moves: motors begin stopped, arm holds its stowed pose.
        leftMotor.begin();
        rightMotor.begin();
        arm.begin();
        line.begin();
        stepName = "ready";
    }

    void start() override {
        // Fresh run every START: no state carried over from a previous run.
        lineActions.clearFault();
        lineActions.resetCounts();
        lineActions.clearRoute();
        stepName = "start";
        recoveries = 0;
        faultLog = "none";
        queueRoute(false);
    }

    void loop() override {
        LineFault f = lineActions.fault();
        if (f != LineFault::NONE) faultLog = lineActions.faultName();

        // TURN_TIMEOUT recovery. On runs 1-2 the corner turn stopped with the
        // new line just beside the middle sensors. Instead of giving up, pivot
        // toward where the line was last seen (searchLine) and carry on to
        // PARK. Only for TURN_TIMEOUT, only while the line is still under the
        // bar, at most MAX_RECOVERIES times. Wait until the faulted routine
        // has finished skipping its remaining steps first.
        if (f == LineFault::TURN_TIMEOUT && !isActionRunning()
                && recoveries < MAX_RECOVERIES && line.onLine()) {
            recoveries++;
            lineActions.clearFault();
            queueRoute(true);
        }

        // 8 lines max on the Hub screen.
        static char counts[24];
        snprintf(counts, sizeof(counts), "J%d C%d R%d",
                 lineActions.junctionCount(),   // expect J9
                 lineActions.cornerCount(),     // expect C4
                 recoveries);                   // recoveries used
        static char maskStr[9];                 // ch0 (LEFT-most) first
        uint8_t m = line.mask();
        for (int i = 0; i < 8; i++) maskStr[i] = (m & (1 << i)) ? '#' : '.';
        maskStr[8] = 0;

        telemetry.addData("step", f != LineFault::NONE ? "FAULT" : stepName);
        telemetry.addData("fault", faultLog);           // last fault seen this run
        telemetry.addData("count", counts);
        telemetry.addData("mask", maskStr);             // at a fault: where the line is
        telemetry.addData("turn deg", lineActions.lastTurnDegrees());
        telemetry.addData("grid deg", lineActions.gridHeading());
        telemetry.addData("err", (float)line.error());
    }

private:
    static constexpr int MAX_RECOVERIES = 3;
    int recoveries = 0;
    const char* faultLog = "none";

    // The route from wherever the robot is to PARK. fromStart = false is the
    // normal run; resume = true picks up after a turn timeout (search, then follow on).
    void queueRoute(bool resume) {
        LineActions::Builder b = lineActions.auto_();
        if (!resume) {
            b.then(step("leave START"))
             .leaveBox();                   // straight out until bar is on the line
        } else {
            b.then(step("recover"))
             .searchLine();                 // pivot toward last-seen side until centred
        }
        b.then(step("loop to PARK"))
         .followToBox()                     // corners auto, junctions passed,
                                            // brakes just inside the PARK square
         .then(step("park"))
         .then(lineActions.park(PARK_MS))
         .then(step("PARKED"))
         .stop();
        runAction(b.build());
    }

public:
    void stop() override {
        cancelActions();
        drive.stop();
    }
};

REGISTER_OPMODE(AutoStartToPark, "Start to Park", AUTO);
