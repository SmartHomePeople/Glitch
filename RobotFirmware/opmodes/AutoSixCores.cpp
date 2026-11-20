// ============================================================
//  AutoSixCores.cpp — "Six Cores"  (AUTO)
//  GLITCHED, Team 190, robot "Pixel".  NOT YET RUN ON THE ROBOT.
//
//  STAGE 1: MOVEMENT ONLY. PICK and DROP are placeholders (the robot pauses
//  there). The arm / gripper / dumper go into pickCore() / dropCore() next.
//  Test WITHOUT CORE on the mat first: the robot drives right up to the end
//  bar, where a CORE would be pushed.
// ============================================================
//
//  Route: counter-clockwise from START, the same way as "Start to Park".
//  "Forward" = the direction of travel round the loop. Every Pick-Up
//  terminal is on the robot's RIGHT going forward, every Drop terminal on
//  its LEFT. Each side of the loop is done the same way:
//
//      Pa ----- D ----- Pb          (Pa = the nearer Pick-Up, Pb = the farther)
//
//    leg A  pick Pa:  -> Pa, right, to end bar, PICK, U-turn, back, right (= forward)
//    leg B  drop D:   -> D,  left,  to end bar, DROP, U-turn, back, left  (= forward)
//    leg C  pick Pb, drop D:
//                     -> Pb, right, to end bar, PICK, U-turn, back, left (= BACKWARD)
//                     -> D (now on the right), right, to end bar, DROP,
//                        U-turn, back, left (= forward)
//
//    right side (heading north): Pa = P1, D = D1, Pb = P2   RED
//    top side   (heading west):  Pa = P3, D = D2, Pb = P4   GOLD
//    left side  (heading south): Pa = P5, D = D3, Pb = P6   BLUE
//    then follow into the PARK box (corners and junctions as in Start to Park).
//
//  TIME BUDGET. Before each leg the robot checks it still has time for that
//  leg AND the drive to PARK (30 pts). If not, it skips straight to PARK.
//  Leg times are measured during the run; until measured, the estimates
//  below are used. The time estimates are GUESSES — tune from the "leg" line.
//
//  TURN_TIMEOUT RECOVERY (from Start to Park, worked on the robot 20 Nov):
//  pivot toward the line (searchLine), then redo the follow step that
//  faulted, or carry on with the next step if a turn faulted.
// ============================================================
#include "NRL.h"

namespace {   // file-local: other OpModes have their own helpers

// ── Park ──────────────────────────────────────────────────────────────────
// Park: followToBox() brakes with the sensor bar just inside the PARK square;
// then the SDK's park(ms) drives straight on at tuning.parkSpeed for PARK_MS
// and stops. NO ENCODERS (20 Nov). Same goal as before: ~0.25 m further, so
// the chassis middle sits near the box centre. 600 ms at 0.22 is an ESTIMATE
// (speed at 0.22 not measured). Tune on the mat: all drive wheels must be on
// the black square -> wheels short of the box: raise; robot past it: lower.
constexpr float    PARK_SPEED      = 0.22f;
constexpr uint16_t PARK_MS          = 600;

// ── Placeholders until the mechanism code is in ────────────────────────────
constexpr uint32_t PICK_PAUSE_MS = 500;
constexpr uint32_t DROP_PAUSE_MS = 500;

// ── Time budget (GUESSES: tune from telemetry) ─────────────────────────────
constexpr float RUN_S          = 150.0f;
constexpr float MARGIN_S       = 5.0f;   // spare at the end
constexpr float EST_PICK_S     = 16.0f;  // leg A, until measured
constexpr float EST_DROP_S     = 14.0f;  // leg B, until measured
constexpr float EST_PICKDROP_S = 32.0f;  // leg C, until measured
constexpr float PARK_S_PER_M   = 2.5f;   // main-line following
constexpr float PARK_S_CORNER  = 3.0f;   // each corner
constexpr float PARK_FINISH_S  = 3.0f;   // into the box + timed park

// Arena (manual p.6), metres along the loop from corner C1, travelling CCW.
// Side s runs from 2s to 2s+2: Pa at +0.5, D at +1.0, Pb at +1.5.
// PARK box edge is 0.3 m past C4 (at 6.0).
constexpr float PARK_EDGE_POS = 6.3f;

constexpr int MAX_RECOVERIES = 4;

// ── Hardware (same as Start to Park) ───────────────────────────────────────
HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM,
                         .flipped = false, .minDrive = 34 }};
HexaDCMotor rightMotor{{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM,
                         .flipped = true,  .minDrive = 32 }};
TankDrive drive(leftMotor, rightMotor);

// Arm held stowed (110). Stow it by hand before INIT so INIT doesn't move it.
HexaServo arm{{ .signalPin = SERVO_1, .startAngle = 110.0f, .settleMs = 0 }};

HexaLineArray line{ hexaLineConfig() };

LineTuning missionTuning() {
    LineTuning t;           // SDK defaults — same values that parked on 20 Nov
    t.followSpeed   = 0.35f;
    t.junctionSpeed = 0.25f;
    t.pivotMs       = 60;
    t.pivotSpeed    = 0.25f;
    t.turnSpeed     = 0.45f;
    t.turnSlowSpeed = 0.30f;
    t.parkSpeed     = PARK_SPEED;   // park(ms) speed, as the old park drive
    return t;
}
LineActions lineActions(drive, line, missionTuning());

// Inside a terminal (turn-in -> end bar) the robot follows slower, so it
// stops closer to the end bar. Run 1 (20 Nov): at 0.35 it always rolled a
// bit past the end bar before the U-turn, which would knock a CORE away.
// 0.22 is a first guess: lower it if it still overshoots, raise it if the
// robot crawls or stalls.
constexpr float TERMINAL_SPEED = 0.22f;
LineTuning terminalTuning() {
    LineTuning t = missionTuning();
    t.followSpeed = TERMINAL_SPEED;
    return t;
}

// ── CORE being carried (record for telemetry and later decisions) ─────────
enum Core : uint8_t { NONE, RED, GOLD, BLUE };
const char* coreName(Core c) {
    switch (c) { case RED: return "RED"; case GOLD: return "GOLD"; case BLUE: return "BLUE"; default: return "none"; }
}
Core carrying = NONE;

// ======== MECHANISM HOOKS — stage 2 replaces these ========================
// Called with the sensor bar stopped on the terminal's end bar.
ActionPtr pickCore(Core c) {
    return sequential({
        instant([c] { carrying = c; }),
        sleep_ms(PICK_PAUSE_MS),          // TODO: arm down, grip, arm up
    });
}
ActionPtr dropCore() {
    return sequential({
        sleep_ms(DROP_PAUSE_MS),          // TODO: release beyond the end bar
        instant([] { carrying = NONE; }),
    });
}
// ===========================================================================

// ── The mission as a list of steps, grouped into legs ─────────────────────
enum class Op : uint8_t { LEAVE_BOX, TO_JUNCTION, FORGET_ROUTE, LEFT, RIGHT, UTURN,
                          TO_END, TO_BOX, PICK, DROP, PARK_DRIVE, STOP };
enum class LegKind : uint8_t { START, PICK, DROP, PICK_DROP, PARK };

struct Leg {
    const char* name;
    LegKind kind;
    float endPos;       // where the leg leaves the robot on the loop (m from C1)
    int cornersAfter;   // corners between there and PARK
};
struct Step {
    Op op;
    LineSide side;
    int count;
    int leg;
    Core core;          // for PICK
};

constexpr int MAX_LEGS  = 12;
constexpr int MAX_STEPS = 96;
Leg  legs[MAX_LEGS];
Step steps[MAX_STEPS];
int  nLegs = 0, nSteps = 0;
int  parkStep = 0;      // first step of the PARK leg

int addLeg(const char* name, LegKind kind, float endPos, int cornersAfter) {
    legs[nLegs] = Leg{ name, kind, endPos, cornersAfter };
    return nLegs++;
}
void add(int leg, Op op, LineSide side = LineSide::ANY, int count = 1, Core core = NONE) {
    steps[nSteps++] = Step{ op, side, count, leg, core };
}

void buildMission() {
    nLegs = 0; nSteps = 0;
    static const char* const NAME_A[3] = { "P1", "P3", "P5" };
    static const char* const NAME_B[3] = { "D1", "D2", "D3" };
    static const char* const NAME_C[3] = { "P2>D1", "P4>D2", "P6>D3" };
    static const Core CORE[3] = { RED, GOLD, BLUE };

    int L = addLeg("leave", LegKind::START, -0.3f, 4);
    add(L, Op::LEAVE_BOX);

    for (int s = 0; s < 3; s++) {
        float base = 2.0f * s;
        int corners = 3 - s;   // corners still ahead after this side's legs

        // leg A: pick Pa. From START, Pa is the first right junction; from
        // the previous side's D, Pb of that side comes first, so the 2nd.
        L = addLeg(NAME_A[s], LegKind::PICK, base + 0.5f, corners);
        add(L, Op::TO_JUNCTION, LineSide::RIGHT, s == 0 ? 1 : 2);
        add(L, Op::RIGHT);
        add(L, Op::TO_END);
        add(L, Op::PICK, LineSide::ANY, 1, CORE[s]);
        add(L, Op::UTURN);
        add(L, Op::TO_JUNCTION);              // back on the main line (a T)
        add(L, Op::RIGHT);                    // forward

        // leg B: drop at D.
        L = addLeg(NAME_B[s], LegKind::DROP, base + 1.0f, corners);
        add(L, Op::TO_JUNCTION, LineSide::LEFT, 1);
        add(L, Op::LEFT);
        add(L, Op::TO_END);
        add(L, Op::DROP);
        add(L, Op::UTURN);
        add(L, Op::TO_JUNCTION);
        add(L, Op::LEFT);                     // forward

        // leg C: pick Pb, come back to D, drop.
        L = addLeg(NAME_C[s], LegKind::PICK_DROP, base + 1.0f, corners);
        add(L, Op::TO_JUNCTION, LineSide::RIGHT, 1);
        add(L, Op::RIGHT);
        add(L, Op::TO_END);
        add(L, Op::PICK, LineSide::ANY, 1, CORE[s]);
        add(L, Op::UTURN);
        add(L, Op::TO_JUNCTION);
        add(L, Op::LEFT);                     // BACKWARD, toward D
        add(L, Op::TO_JUNCTION, LineSide::RIGHT, 1);   // D is on the right now
        add(L, Op::RIGHT);
        add(L, Op::TO_END);
        add(L, Op::DROP);
        add(L, Op::UTURN);
        add(L, Op::TO_JUNCTION);
        add(L, Op::FORGET_ROUTE);             // see makeAction()
        add(L, Op::LEFT);                     // forward again
    }

    L = addLeg("park", LegKind::PARK, PARK_EDGE_POS, 0);
    parkStep = nSteps;
    add(L, Op::TO_BOX);
    add(L, Op::PARK_DRIVE);
    add(L, Op::STOP);
}

ActionPtr makeAction(const Step& s, int count) {
    switch (s.op) {
        case Op::LEAVE_BOX:   return lineActions.leaveBox();
        case Op::TO_JUNCTION: return lineActions.followToJunction(s.side, count);
        case Op::FORGET_ROUTE:
            // Run 2 (20 Nov): NO_BRANCH on the final left() of leg P2>D1 (#28),
            // robot on the T. The SDK's route memory ("the way home") was
            // confused by our route visiting D1 twice from different
            // directions. A clearRoute() before EVERY follow (tried next) made
            // step #14 fail instead, twice (runs 5 and 7), where run 2 had
            // passed it. So the route is forgotten only here, right before
            // that one turn. It also re-sets grid 0 deg from the current
            // heading at the turn.
            return instant([] { lineActions.clearRoute(); });
        case Op::LEFT:        return lineActions.left();
        case Op::RIGHT:       return lineActions.right();
        case Op::UTURN:       return lineActions.uTurn();
        case Op::TO_END:
            // Slow tuning for the terminal, normal tuning back afterwards. The
            // restore also runs if followToEnd() faults (it still finishes).
            return sequential({
                instant([] { lineActions.setTuning(terminalTuning()); }),
                lineActions.followToEnd(),
                instant([] { lineActions.setTuning(missionTuning()); }),
            });
        case Op::TO_BOX:      return lineActions.followToBox();
        case Op::PICK:        return pickCore(s.core);
        case Op::DROP:        return dropCore();
        case Op::PARK_DRIVE:  return lineActions.park(PARK_MS);
        case Op::STOP:        return lineActions.stop();
    }
    return lineActions.stop();
}

const char* opName(Op op) {
    switch (op) {
        case Op::LEAVE_BOX:   return "leave box";
        case Op::TO_JUNCTION: return "to junction";
        case Op::FORGET_ROUTE: return "forget route";
        case Op::LEFT:        return "left";
        case Op::RIGHT:       return "right";
        case Op::UTURN:       return "U-turn";
        case Op::TO_END:      return "to end bar";
        case Op::TO_BOX:      return "to PARK box";
        case Op::PICK:        return "PICK";
        case Op::DROP:        return "DROP";
        case Op::PARK_DRIVE:  return "park drive";
        case Op::STOP:        return "stop";
    }
    return "?";
}

}  // namespace

class AutoSixCores : public NRLOpMode {
public:
    void init() override {
        // Nothing here moves: motors begin stopped, arm holds its stowed pose.
        leftMotor.begin();
        rightMotor.begin();
        arm.begin();
        line.begin();
        buildMission();
    }

    void start() override {
        // Fresh run every START: nothing carried over from a previous run.
        lineActions.clearFault();
        lineActions.resetCounts();
        lineActions.clearRoute();
        // A run stopped inside a terminal never switched back from the slow
        // terminal tuning; without this the next run would follow at 0.22.
        lineActions.setTuning(missionTuning());
        carrying = NONE;
        recoveries = 0;
        faultLog = "none";
        skippedToPark = false;
        estS[(int)LegKind::PICK]      = EST_PICK_S;
        estS[(int)LegKind::DROP]      = EST_DROP_S;
        estS[(int)LegKind::PICK_DROP] = EST_PICKDROP_S;
        runStartMs = millis();
        curLeg = -1;
        mode = Mode::RUN;
        beginStep(0);
    }

    void loop() override {
        LineFault f = lineActions.fault();
        if (f != LineFault::NONE) faultLog = lineActions.faultName();

        if ((mode == Mode::RUN || mode == Mode::SEARCH) && !isActionRunning()) {
            if (f == LineFault::NONE) {
                if (mode == Mode::SEARCH) {
                    // Back on a line after a turn timeout.
                    mode = Mode::RUN;
                    Op op = steps[cur].op;
                    if (op == Op::TO_JUNCTION || op == Op::TO_BOX) {
                        // The fault was at a corner inside a follow step: redo
                        // it, minus the junctions it already passed. (On this
                        // route every junction a follow step passes is on its
                        // target side, so this count is right.)
                        int passed = lineActions.junctionCount() - stepJcStart;
                        int left = steps[cur].count - passed;
                        if (left < 1) left = 1;
                        runAction(makeAction(steps[cur], left));
                    } else {
                        beginStep(cur + 1);   // a turn faulted but found the line
                    }
                } else {
                    beginStep(cur + 1);
                }
            } else if (f == LineFault::TURN_TIMEOUT && recoveries < MAX_RECOVERIES) {
                // Run 1 of Six Cores (20 Nov): TURN_TIMEOUT in leg D2 with the
                // line NOT under the bar, so the old "line.onLine()" condition
                // gave up. searchLine() pivots toward the side the line was
                // last seen and has its own timeout, so try it anyway.
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
        float t = (millis() - runStartMs) / 1000.0f;
        static char legStr[32], stepStr[32], counts[24], maskStr[9];
        const char* legName = (curLeg >= 0) ? legs[curLeg].name : "-";
        snprintf(legStr, sizeof(legStr), "%s %d/%d t%.0f%s", legName, curLeg + 1, nLegs,
                 mode == Mode::IDLE ? 0.0f : t, skippedToPark ? " SKIP" : "");
        // Step number = index in buildMission() order, so a failure can be
        // matched to the exact step.
        const char* opTxt = (cur >= 0 && cur < nSteps) ? opName(steps[cur].op) : "-";
        if (mode == Mode::DONE)        snprintf(stepStr, sizeof(stepStr), "PARKED");
        else if (mode == Mode::FAILED) snprintf(stepStr, sizeof(stepStr), "FAIL %s #%d", opTxt, cur);
        else if (mode == Mode::SEARCH) snprintf(stepStr, sizeof(stepStr), "search #%d", cur);
        else                           snprintf(stepStr, sizeof(stepStr), "%s #%d", opTxt, cur);
        snprintf(counts, sizeof(counts), "J%d C%d R%d",
                 lineActions.junctionCount(), lineActions.cornerCount(), recoveries);
        uint8_t m = line.mask();
        for (int i = 0; i < 8; i++) maskStr[i] = (m & (1 << i)) ? '#' : '.';
        maskStr[8] = 0;

        telemetry.addData("leg", legStr);
        telemetry.addData("step", stepStr);
        telemetry.addData("fault", faultLog);
        telemetry.addData("count", counts);
        telemetry.addData("mask", maskStr);
        telemetry.addData("carry", coreName(carrying));
        telemetry.addData("turn deg", lineActions.lastTurnDegrees());
        // Last measured leg times, seconds: A = pick, B = drop, C = pick+drop.
        // (Shows the estimates until a leg of that kind has finished.)
        static char times[32];
        snprintf(times, sizeof(times), "A%.0f B%.0f C%.0f",
                 estS[(int)LegKind::PICK], estS[(int)LegKind::DROP],
                 estS[(int)LegKind::PICK_DROP]);
        telemetry.addData("leg s", times);
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
    int curLeg = -1;
    int stepJcStart = 0;
    int recoveries = 0;
    bool skippedToPark = false;
    const char* faultLog = "none";
    uint32_t runStartMs = 0, legStartMs = 0;
    float estS[5] = { 0, EST_PICK_S, EST_DROP_S, EST_PICKDROP_S, 0 };

    float elapsedS() const { return (millis() - runStartMs) / 1000.0f; }

    // Time for this leg plus the drive from where it ends to PARK.
    bool legFits(int legIdx) const {
        const Leg& lg = legs[legIdx];
        if (lg.kind == LegKind::START || lg.kind == LegKind::PARK) return true;
        float park = (PARK_EDGE_POS - lg.endPos) * PARK_S_PER_M
                   + lg.cornersAfter * PARK_S_CORNER + PARK_FINISH_S;
        return elapsedS() + estS[(int)lg.kind] + park <= RUN_S - MARGIN_S;
    }

    void beginStep(int i) {
        if (i >= nSteps) { mode = Mode::DONE; drive.stop(); return; }
        if (steps[i].leg != curLeg) {
            // A leg just finished: remember how long it really took.
            if (curLeg >= 0) {
                LegKind k = legs[curLeg].kind;
                if (k == LegKind::PICK || k == LegKind::DROP || k == LegKind::PICK_DROP)
                    estS[(int)k] = (millis() - legStartMs) / 1000.0f;
            }
            if (!legFits(steps[i].leg)) {
                i = parkStep;           // not enough time: go and park
                skippedToPark = true;
            }
            curLeg = steps[i].leg;
            legStartMs = millis();
        }
        cur = i;
        stepJcStart = lineActions.junctionCount();
        runAction(makeAction(steps[cur], steps[cur].count));
    }
};

REGISTER_OPMODE(AutoSixCores, "Six Cores", AUTO);
