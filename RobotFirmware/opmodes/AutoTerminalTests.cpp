// ============================================================
//  AutoTerminalTests.cpp — "TerminalTests"  (AUTO)
//  GLITCHED, Team 190, robot "Pixel".  NOT YET RUN ON THE ROBOT.
//
//  Built from "Start to Park" (worked on the robot, 20 Nov), plus ONE
//  terminal visit, to get terminals right one step at a time:
//
//    START -> leave box -> follow to P1 (J1, first RIGHT junction after C1)
//          -> markHere(P1) -> right() into P1 -> followToEnd() (end bar)
//          -> uTurn() -> retraceTo(P1) (stops ON the P1 junction)
//          -> right() (= forward, north) -> follow round the loop into PARK
//          -> park(ms) -> stop
//
//  Terminal pattern from the NRL programming guide (SDK 1.5.0), 5.5 / 7.1:
//    junction -> mark -> turn in -> followToEnd() -> action -> uTurn()
//    -> retraceTo(mark) -> turn to carry on.
//  retraceTo() uses route memory: it stops on the marked junction and checks
//  it looks like the remembered one (else WRONG JUNCTION, not a wrong turn).
//
//  No arm/gripper moves yet. Test WITHOUT a CORE on P1.
//
//  SPEED_SCALE: the driving speeds (follow, junction, pivot roll, leaving
//  START, park) are multiplied by it and the SDK time limits divided by it.
//  Turn / spin / search speeds stay at the Start to Park values.
//  SPEED_SCALE = 1.0 gives exactly the Start to Park tuning.
// ============================================================
#include "NRL.h"
#include "GlitchTuning.h"

namespace {   // file-local: other OpModes have their own helpers

using namespace glitch;

constexpr int MAX_RECOVERIES = 4;
// After a recovery, facing within this many degrees of BACK the way the robot
// was travelling = it caught the old line: U-turn and try again.
constexpr float BACKWARDS_DEG = 50.0f;

float angleDiff(float a, float b) {        // a - b, wrapped to -180..180
    float d = fmodf(a - b, 360.0f);
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

// ── Hardware (same as Start to Park) ───────────────────────────────────────
HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM,
                         .flipped = false, .minDrive = 34 }};
HexaDCMotor rightMotor{{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM,
                         .flipped = true,  .minDrive = 32 }};
TankDrive drive(leftMotor, rightMotor);

// Arm held stowed (110). Stow it by hand before INIT so INIT doesn't move it.
HexaServo arm{{ .signalPin = SERVO_1, .startAngle = 110.0f, .settleMs = 0 }};

HexaLineArray line{ hexaLineConfig() };

// Tuning: SPEED_SCALE, park and testTuning() live in GlitchTuning.h (shared
// with the Line Bench TELEOP).
LineActions lineActions(drive, line, testTuning());

// ── The route, one step at a time ──────────────────────────────────────────
enum class Op : uint8_t { LEAVE_BOX, TO_JUNCTION, MARK, RETRACE, LEFT, RIGHT,
                          UTURN, TO_END, TO_BOX, PARK_DRIVE, STOP };

// Route-memory mark for the P1 junction. Must be static/global: markHere()
// writes it and retraceTo() reads it later, while the routine runs (guide 4.3).
int markP1 = -1;
struct Step {
    Op op;
    LineSide side;
    int count;
    const char* name;   // shown on the Hub screen
};

const Step STEPS[] = {
    // Robot's point of view at every junction (guide 5.2).
    { Op::LEAVE_BOX,   LineSide::ANY,   1, "leave START" },  // #0  out of the black box
    { Op::TO_JUNCTION, LineSide::RIGHT, 1, "to P1" },        // #1  C1 corner (auto), stop
                                                             //     past P1, last = -RS
    { Op::MARK,        LineSide::ANY,   1, "mark P1" },      // #2  remember this junction
    { Op::RIGHT,       LineSide::ANY,   1, "into P1" },      // #3  branch is on the right
    { Op::TO_END,      LineSide::ANY,   1, "to end bar" },   // #4  stops ON the bar, LR-
    { Op::UTURN,       LineSide::ANY,   1, "U-turn" },       // #5  face back to the T
    { Op::RETRACE,     LineSide::ANY,   1, "back to P1" },   // #6  route memory, stops on P1
    { Op::RIGHT,       LineSide::ANY,   1, "right=north" },  // #7  heading west: north is right
    { Op::TO_BOX,      LineSide::ANY,   1, "to PARK" },      // #8  rest of the loop
    { Op::PARK_DRIVE,  LineSide::ANY,   1, "park" },         // #9
    { Op::STOP,        LineSide::ANY,   1, "stop" },         // #10
};
constexpr int N_STEPS = sizeof(STEPS) / sizeof(STEPS[0]);

// Steps that follow a line (and can meet a corner on the way).
bool isFollow(Op op) {
    return op == Op::TO_JUNCTION || op == Op::RETRACE || op == Op::TO_BOX;
}

ActionPtr makeAction(const Step& s, int count) {
    switch (s.op) {
        case Op::LEAVE_BOX:   return lineActions.leaveBox();
        case Op::TO_JUNCTION: return lineActions.followToJunction(s.side, count);
        case Op::MARK:        return lineActions.markHere(&markP1);
        case Op::RETRACE:     return lineActions.retraceTo(&markP1);
        case Op::LEFT:        return lineActions.left();
        case Op::RIGHT:       return lineActions.right();
        case Op::UTURN:       return lineActions.uTurn();
        case Op::TO_END:      return lineActions.followToEnd();
        case Op::TO_BOX:      return lineActions.followToBox();
        case Op::PARK_DRIVE:  return lineActions.park(PARK_MS);
        case Op::STOP:        return lineActions.stop();
    }
    return lineActions.stop();
}

}  // namespace

class AutoTerminalTests : public NRLOpMode {
public:
    void init() override {
        // Nothing here moves: motors begin stopped, arm holds its stowed pose.
        leftMotor.begin();
        rightMotor.begin();
        arm.begin();
        line.begin();
    }

    void start() override {
        // Fresh run every START: nothing carried over from a previous run.
        lineActions.setTuning(testTuning());
        lineActions.clearFault();
        lineActions.resetCounts();
        lineActions.clearRoute();
        markP1 = -1;
        recoveries = 0;
        backwardsFixes = 0;
        travelHeading = -999.0f;
        headingAtFault = -999.0f;
        faultLog = "none";
        runStartMs = millis();
        mode = Mode::RUN;
        beginStep(0);
    }

    void loop() override {
        LineFault f = lineActions.fault();
        if (f != LineFault::NONE) faultLog = lineActions.faultName();

        // Travel direction: gyro grid heading while following a line steadily
        // (centred). Used to spot a recovery that turned the robot backwards.
        if (mode == Mode::RUN && f == LineFault::NONE && cur >= 0 &&
            isFollow(STEPS[cur].op) &&
            line.onLine() && abs(line.error()) < 500) {
            float h = lineActions.gridHeading();
            if (h > -900.0f) travelHeading = h;
        }

        if ((mode == Mode::RUN || mode == Mode::SEARCH) && !isActionRunning()) {
            if (f == LineFault::NONE) {
                if (mode == Mode::SEARCH) {
                    // Back on a line after a turn timeout.
                    mode = Mode::RUN;
                    Op op = STEPS[cur].op;
                    if (isFollow(op)) {
                        // Timed out at a corner inside a follow step: redo it,
                        // minus junctions it already passed (none on this route).
                        int left = STEPS[cur].count - (lineActions.junctionCount() - stepJcStart);
                        if (left < 1) left = 1;
                        float h = lineActions.gridHeading();
                        bool known = (h > -900.0f) && (headingAtFault > -900.0f);
                        if (known && fabsf(angleDiff(h, headingAtFault + 180.0f)) < BACKWARDS_DEG) {
                            // Facing back the way it came (C4, 20 Nov): turn
                            // round onto the old line, follow back to the
                            // corner and let the SDK turn it again.
                            backwardsFixes++;
                            runAction(lineActions.auto_()
                                .uTurn()
                                .then(makeAction(STEPS[cur], left))
                                .build());
                        } else {
                            runAction(makeAction(STEPS[cur], left));
                        }
                    } else {
                        beginStep(cur + 1);   // a turn timed out but found the line
                    }
                } else {
                    beginStep(cur + 1);
                }
            } else if (f == LineFault::TURN_TIMEOUT && recoveries < MAX_RECOVERIES) {
                // Same recovery that saved Start to Park: pivot toward where
                // the line was last seen, then carry on.
                headingAtFault = travelHeading;   // the way it was going before
                recoveries++;
                lineActions.clearFault();
                mode = Mode::SEARCH;
                runAction(lineActions.searchLine());
            } else {
                mode = Mode::FAILED;
                drive.stop();
            }
        }

        // ── Telemetry: 8 lines max ──
        static char stepStr[32], counts[24], maskStr[9], timeStr[16];
        const char* nm = (cur >= 0 && cur < N_STEPS) ? STEPS[cur].name : "-";
        if (mode == Mode::DONE)        snprintf(stepStr, sizeof(stepStr), "PARKED");
        else if (mode == Mode::FAILED) snprintf(stepStr, sizeof(stepStr), "FAIL %s #%d", nm, cur);
        else if (mode == Mode::SEARCH) snprintf(stepStr, sizeof(stepStr), "search #%d", cur);
        else                           snprintf(stepStr, sizeof(stepStr), "%s #%d", nm, cur);
        snprintf(counts, sizeof(counts), "J%d C%d R%d B%d",
                 lineActions.junctionCount(), lineActions.cornerCount(), recoveries,
                 backwardsFixes);   // B = recoveries that faced backwards
        uint8_t m = line.mask();
        for (int i = 0; i < 8; i++) maskStr[i] = (m & (1 << i)) ? '#' : '.';
        maskStr[8] = 0;
        float t = (mode == Mode::IDLE) ? 0.0f : (millis() - runStartMs) / 1000.0f;
        if (mode == Mode::DONE) t = (doneMs - runStartMs) / 1000.0f;
        snprintf(timeStr, sizeof(timeStr), "%.1f s", t);
        // Last junction as L/R/S + route-memory depth (guide 3.3 / 5.3).
        static char lastStr[16];
        const LineJunction& j = lineActions.lastJunction();
        snprintf(lastStr, sizeof(lastStr), "%c%c%c d%d",
                 j.left ? 'L' : '-', j.right ? 'R' : '-', j.straight ? 'S' : '-',
                 lineActions.routeDepth());

        telemetry.addData("step", stepStr);
        telemetry.addData("fault", faultLog);
        telemetry.addData("count", counts);        // expect C4 R0 B0 at PARK
        telemetry.addData("mask", maskStr);        // ch0 (LEFT-most) first
        telemetry.addData("turn deg", lineActions.lastTurnDegrees());
        telemetry.addData("grid deg", lineActions.gridHeading());
        telemetry.addData("last", lastStr);        // -RS at P1, LR- on the end bar
        telemetry.addData("time", timeStr);
    }

    void stop() override {
        cancelActions();
        drive.stop();
        mode = Mode::IDLE;
    }

private:
    enum class Mode : uint8_t { IDLE, RUN, SEARCH, DONE, FAILED };
    Mode mode = Mode::IDLE;
    int cur = -1;
    int stepJcStart = 0;
    int recoveries = 0;
    int backwardsFixes = 0;
    float travelHeading = -999.0f;
    float headingAtFault = -999.0f;
    const char* faultLog = "none";
    uint32_t runStartMs = 0, doneMs = 0;

    void beginStep(int i) {
        if (i >= N_STEPS) { mode = Mode::DONE; doneMs = millis(); drive.stop(); return; }
        cur = i;
        stepJcStart = lineActions.junctionCount();
        runAction(makeAction(STEPS[cur], STEPS[cur].count));
    }
};

REGISTER_OPMODE(AutoTerminalTests, "TerminalTests", AUTO);
