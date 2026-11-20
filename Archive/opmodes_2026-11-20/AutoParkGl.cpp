// =====================================================================
//  GLITCHED (Team 190) - "Park Gyro+Line": START -> C1 -> C2 -> C3 -> C4 -> PARK
//  File: RobotFirmware/opmodes/AutoParkGL.cpp
//
//  Uses ONLY the gyro (NRLComms heading) and the IR line array. No encoders, no other input.
//  Other files are untouched.
//
//  ROUTE: out of START (facing east), follow the line, turn LEFT at all four corners, cross the
//  three junctions on each side (P1 D1 P2 / P3 D2 P4 / P5 D3 P6) without turning, stop in PARK.
//
//  HOW A CORNER IS TOLD APART FROM A JUNCTION (no distance needed):
//    A junction on the side: extra tape appears at an edge of the array, then the line CONTINUES
//    straight ahead. A corner: the line ENDS. So whenever a branch / bar shows up the robot drives
//    straight across it (gyro holds the heading) and looks: line ahead again = crossing (counted),
//    no line = the line ended.
//  HOW THE TURN POINT IS FOUND (no distance needed):
//    After the line ends the robot stops, then backs up slowly until the array is on the corner
//    tape again: that is an exact, repeatable position. From there it drives forward for ROLL_MS
//    (the chassis middle onto the corner), stops, and turns left by gyro until the line is centred.
//
//  SAFETY: every state has a timeout; every failure ends in FAULT (motors stopped, LED red,
//  reason on the screen). A lost line: creep straight a short time, then a bounded search
//  (+/- SEARCH_DEG), then FAULT. A turn never goes past its window; it searches back once, then FAULT.
//
//  Start: robot centred in START, facing the line that leaves it, after LINE TEST calibration.
//  LED: off while running, GREEN = parked, RED = fault.
//  STATUS: compiled against the SDK v1.5.0 headers only. NOT run on the robot yet.
//  The two TIME values (ROLL_MS, PARK_MS) must be tuned on the robot - see TUNING below.
// =====================================================================
#include "NRL.h"

namespace {

// =====================================================================
//  HARDWARE - motor setup copied from AutoPark.cpp (verified on the robot 2026-11-19)
// =====================================================================
HexaDCMotor leftMotor{{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .minDrive = 34 }};
HexaDCMotor rightMotor{{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true, .minDrive = 32 }};
TankDrive     drive(leftMotor, rightMotor);
HexaLineArray line{hexaLineConfig()};

// =====================================================================
//  TUNING
// =====================================================================
// --- the two time values to tune on the robot -------------------------------------------
// ROLL_MS: from standing with the array ON the corner tape, how long to drive forward so the
//   chassis middle is on the corner. Tune with the "land" number on the screen after a turn:
//   land > +800  -> the new line was RIGHT of centre -> the robot did not go far enough -> raise ROLL_MS
//   land < -800  -> went too far -> lower ROLL_MS.   (8 cm at ROLL_SPEED; first guess)
constexpr uint32_t ROLL_MS  = 250;
// PARK_MS: after the array enters the PARK box, how long to keep driving so the robot is fully
//   inside (chassis middle about 33 cm past the box edge). Also stops early if the array leaves
//   the far side of the box.
constexpr uint32_t PARK_MS  = 700;

// Speeds (TankDrive units -1..1). Slow on purpose.
constexpr float LEAVE_SPEED  = 0.22f;
constexpr float FOLLOW_SPEED = 0.30f;
constexpr float SLOW_SPEED   = 0.22f;   // first leg, and after the last junction before a corner
constexpr float CROSS_SPEED  = 0.20f;   // straight across a junction
constexpr float LOCATE_SPEED = 0.16f;   // backing up onto the corner tape
constexpr float ROLL_SPEED   = 0.18f;
constexpr float PARK_SPEED   = 0.20f;
constexpr float TURN_FAST    = 0.20f;
constexpr float TURN_SLOW    = 0.17f;
constexpr float RAMP_RATE    = 0.10f;

// Line following PD (gains from AutoPark)
constexpr int   STEER_SIGN     = 1;     // +1: channel 0 is the LEFT-most sensor (confirmed 2026-11-19)
constexpr float STEER_MAX      = 0.25f;
constexpr float STEER_STEP_MAX = 0.025f;
constexpr float SPEED_DROP     = 0.50f;
HexaPID steerPid{ HexaPIDConfig{ .kP = 0.22f, .kD = 0.08f, .outMin = -STEER_MAX, .outMax = STEER_MAX, .dAlpha = 0.5f } };

// Gyro heading hold while driving straight without the line (crossing, creeping, locate, park)
constexpr float HOLD_KP  = 0.010f;      // steering per degree off the planned heading
constexpr float HOLD_MAX = 0.10f;

// Line reading (channel 0 = left)
constexpr int   BAR_DARK      = 6;      // >= this many dark channels = bar / crossing / box
constexpr int   PLAIN_MAX     = 5;      // a plain line never darkens more channels than this
constexpr int   EDGE_L = 1, EDGE_R = 6; // a branch reaches channel 1 / 6 (works with a dead outer channel)
constexpr int   CONFIRM_TICKS = 3;      // the same reading this many ticks in a row
constexpr int   BOX_TICKS     = 6;
constexpr int   LOST_TICKS    = 5;      // no line this many ticks = gone
constexpr int   ACQUIRE_ERR   = 1200;   // a turn stops on a line this close to centre (1000 = one sensor)
constexpr int   ALIGN_OK      = 700;
constexpr int   ALIGN_DONE    = 350;

// Plan
constexpr int   LEG_COUNT     = 5;      // START->C1, C1->C2, C2->C3, C3->C4, C4->PARK
constexpr int   EXPECT_CROSS[LEG_COUNT] = { 0, 3, 3, 3, 0 };   // junctions to cross on each leg
constexpr const char *LEG_NAME[LEG_COUNT] = { "START->C1", "C1->C2", "C2->C3", "C3->C4", "C4->PARK" };
constexpr int   TURN_DIR      = -1;     // every corner is a LEFT turn (drive turn -1 = left)

// Turns
constexpr float TURN_DEG       = 90.0f;
constexpr float TURN_LO_DEG    = 60.0f; // accept the new line only between these angles
constexpr float TURN_HI_DEG    = 115.0f;
constexpr float SLOW_DEG       = 40.0f;
constexpr float SEARCH_DEG     = 30.0f;
constexpr float ALIGN_MAX_DEG  = 20.0f;
constexpr float CORNER_MAX_DEG = 20.0f; // a line end while this far off the planned heading is NOT a corner
constexpr float GRID_FAULT_DEG = 60.0f; // after a turn this far from the plan = wrong line -> FAULT

// Times (ms)
constexpr uint32_t LEAVE_MIN_MS   = 200;
constexpr uint32_t LEAVE_MAX_MS   = 4000;
constexpr uint32_t FIND_MAX_MS    = 600;    // after the box: line must appear within this
constexpr uint32_t LEG_MAX_MS     = 20000;
constexpr uint32_t CROSS_MAX_MS   = 900;    // straight across a junction, at most
constexpr uint32_t CROSS_IGNORE_MS = 150;   // after a crossing: ignore junction readings this long (same junction)
constexpr uint32_t LOST_CREEP_MS  = 250;    // unexpected loss: creep straight this long first
constexpr uint32_t SETTLE_MS      = 400;    // no encoders: a stop always waits this long
constexpr uint32_t LOCATE_MAX_MS  = 2500;
constexpr uint32_t TURN_MAX_MS    = 7000;
constexpr uint32_t TURN_BACK_MS   = 4000;
constexpr uint32_t TAPE_OFF_MS    = 150;    // gyro not ready: array must be off every line this long
constexpr uint32_t SEARCH_SIDE_MS = 1500;   // gyro not ready: each search side by time
constexpr uint32_t ALIGN_MAX_MS   = 1500;
constexpr uint32_t VERIFY_MAX_MS  = 400;
constexpr int      VERIFY_TICKS   = 5;
constexpr uint32_t PARK_EXIT_TICKS = 5;     // array all white this long during park-in = far edge
constexpr uint32_t MATCH_STOP_MS  = 145000;

// =====================================================================
//  SENSOR LAYER
// =====================================================================
struct LineView
{
    uint8_t mask = 0;
    int  count = 0, err = 0, lo = -1, hi = -1;
    bool on = false, bar = false, leftEdge = false, rightEdge = false, plain = false;
};
LineView v;
float normLo = 3.0f, normHi = 4.0f;   // where the plain line sits on the array (learned each run)

LineView readLine()
{
    LineView r;
    r.mask = line.mask();
    r.count = line.count();
    r.on = line.onLine();
    r.err = line.error();
    for (int i = 0; i < 8; i++)
        if (r.mask & (1 << i)) { if (r.lo < 0) r.lo = i; r.hi = i; }
    r.bar = r.count >= BAR_DARK;
    // A branch edge: reaches channel 1 / 6, still covers the middle, grew outward from where the
    // plain line sits while its other side did not move (a drifting line moves both sides).
    const int nLo = (int)(normLo + 0.5f), nHi = (int)(normHi + 0.5f);
    r.leftEdge  = r.on && r.lo <= EDGE_L && r.hi >= 4 && r.lo <= nLo - 1 && r.hi >= nHi;
    r.rightEdge = r.on && r.hi >= EDGE_R && r.lo <= 3 && r.hi >= nHi + 1 && r.lo <= nLo;
    r.plain = r.on && !r.bar && !r.leftEdge && !r.rightEdge && r.count <= PLAIN_MAX;
    return r;
}

void learnLinePlace()
{
    if (!v.plain) return;
    normLo += 0.08f * ((float)v.lo - normLo);
    normHi += 0.08f * ((float)v.hi - normHi);
}

bool junctionSeen() { return v.bar || v.leftEdge || v.rightEdge; }

// =====================================================================
//  GYRO LAYER
// =====================================================================
float wrap180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// Angle spun since startSpin(), always positive (signed changes summed, then |sum|).
bool  gyroOk = false;
float spinAcc = 0.0f, spinPrev = 0.0f;
void  startSpin() { gyroOk = NRLComms::isHeadingReady(); spinAcc = 0.0f; spinPrev = NRLComms::getHeadingFree(); }
void  updateSpin()
{
    const float h = NRLComms::getHeadingFree();
    spinAcc += wrap180(h - spinPrev);
    spinPrev = h;
}
float turned() { return fabsf(spinAcc); }

// Heading against the plan. The SDK's sign convention is learned on the first corner turn.
float freeStart = 0.0f;
float planHeading = 0.0f;   // + = right of the start heading
int   gyroSign = 0;         // +1 heading grows on right turns, -1 on left turns, 0 unknown
bool  gridKnown() { return gyroSign != 0 && NRLComms::isHeadingReady(); }
float gridError()           // + = robot is turned RIGHT of where the plan says
{
    if (!gridKnown()) return 0.0f;
    return wrap180(gyroSign * (NRLComms::getHeadingFree() - freeStart) - planHeading);
}

// =====================================================================
//  DRIVE SKILLS
// =====================================================================
float lastSteer = 0.0f;
void resetSteering() { steerPid.reset(); lastSteer = 0.0f; }
void halt() { drive.stop(); }
void spin(int dir, float speed) { drive.drive(0.0f, dir * speed); }

// Straight at `speed` (negative = reverse), holding the planned heading with the gyro once
// its sign is known (before the first turn: no correction).
void driveStraight(float speed)
{
    float t = 0.0f;
    if (gridKnown())
    {
        t = -HOLD_KP * gridError();                 // turned right of plan -> steer left
        if (t > HOLD_MAX) t = HOLD_MAX;
        if (t < -HOLD_MAX) t = -HOLD_MAX;           // (a turn command rotates the robot the same way
    }                                               //  whether it is driving forward or backward)
    drive.drive(speed, t);
}

void followLine(float speed)                        // only with v.on
{
    const float e = (float)v.err / 3500.0f;
    float t = STEER_SIGN * steerPid.update(e);
    if (t > lastSteer + STEER_STEP_MAX) t = lastSteer + STEER_STEP_MAX;
    if (t < lastSteer - STEER_STEP_MAX) t = lastSteer - STEER_STEP_MAX;
    lastSteer = t;
    float f = speed * (1.0f - SPEED_DROP * fabsf(e));
    if (f < 0.5f * speed) f = 0.5f * speed;
    drive.drive(f, t);
}

bool lineAcquired() { return v.plain && abs(v.err) <= ACQUIRE_ERR; }

// =====================================================================
//  STATE MACHINE
// =====================================================================
enum class St : uint8_t
{
    LEAVE_BOX,   // straight out of the filled START box
    FIND_LINE,   // the line must be under the array just outside the box
    FOLLOW,      // follow the line; a junction -> CROSS, the PARK box -> PARK_IN, line gone -> CREEP
    CROSS,       // straight across a junction: line ahead again = crossing, no line = line ended
    CREEP,       // line lost with no junction: straight a short time, maybe it comes back
    LOCATE,      // after a line end: back up slowly until the array is on the corner tape
    ROLL,        // forward ROLL_MS from the corner tape: chassis middle onto the corner
    SETTLE,      // motors off for SETTLE_MS, then -> next
    TURN,        // left by gyro, stop on the new line
    TURN_BACK,   // spun past the window: back through it once
    ALIGN,       // small rotation until the line is centred
    VERIFY,      // standing: line seen steadily before driving on
    SEARCH,      // bounded search for a lost line
    PARK_IN,     // into the box for PARK_MS
    DONE,
    FAULT
};

St          st = St::LEAVE_BOX, afterSettle = St::FAULT;
int         legIdx = 0, crossed = 0;
uint32_t    tState = 0, tRun = 0, tLeg = 0, tOffLine = 0, tLastCross = 0;
const char *why = "-";
const char *note = "-";
int         nLost = 0, nPlain = 0, nJunction = 0, nBox = 0, nWhite = 0;
int         searchPhase = 0, searchDir = 1, searchesThisLeg = 0;
int         alignDir = 0, verifyGood = 0;
bool        turnPending = false, leftStartLine = false, afterLastJunction = false;
float       turnAcc = 0.0f;
int         landErr = 0;                 // line error when the last turn stopped (for tuning ROLL_MS)

void enter(St s, const char *n = nullptr) { st = s; tState = millis(); if (n) note = n; }

void fault(const char *reason)
{
    why = reason;
    halt();
    enter(St::FAULT);
    userLed.setSolid(255, 0, 0);
}

void settleThen(St next) { halt(); afterSettle = next; enter(St::SETTLE); }

void startLeg(int idx)
{
    legIdx = idx;
    crossed = 0;
    afterLastJunction = false;
    searchesThisLeg = 0;
    tLeg = millis();
    nLost = nPlain = nJunction = nBox = 0;
    resetSteering();
    enter(St::FOLLOW, LEG_NAME[legIdx]);
}

void startSearch(const char *reason)
{
    if (++searchesThisLeg > 2) { fault("line lost (2 searches failed)"); return; }
    const int side = line.lastSeenSide();
    searchDir = (side != 0) ? STEER_SIGN * side : 1;
    searchPhase = 0;
    note = reason;
    settleThen(St::SEARCH);
}

// The line has ended. Is this the corner of this leg?
void lineEnded()
{
    const bool lastLeg = legIdx >= LEG_COUNT - 1;
    const bool enoughCrossed = crossed >= EXPECT_CROSS[legIdx] - 1;   // allow one missed count
    const bool square = !gridKnown() || fabsf(gridError()) <= CORNER_MAX_DEG;
    if (lastLeg)             { startSearch("line ended before PARK"); return; }
    if (!enoughCrossed)      { startSearch("line ended too early"); return; }
    if (!square)             { startSearch("line ended while crooked"); return; }
    turnPending = true;
    note = "corner: locate";
    settleThen(St::LOCATE);
}

void startTurn()
{
    startSpin();
    tOffLine = millis();
    leftStartLine = false;
    enter(St::TURN, "turn left");
}

// ---------------------------------------------------------------------
void tick()
{
    const uint32_t now = millis();
    v = readLine();
    if (st == St::TURN || st == St::TURN_BACK || st == St::ALIGN || st == St::SEARCH || st == St::SETTLE)
        updateSpin();
    if (st != St::DONE && st != St::FAULT && now - tRun > MATCH_STOP_MS) { fault("match time"); return; }
    const uint32_t inState = now - tState;

    switch (st)
    {
    case St::LEAVE_BOX:
    {
        driveStraight(LEAVE_SPEED);
        nPlain = (!v.bar && v.count <= PLAIN_MAX) ? nPlain + 1 : 0;
        if (inState > LEAVE_MIN_MS && nPlain >= CONFIRM_TICKS) { nPlain = 0; enter(St::FIND_LINE, "box exit"); return; }
        if (inState > LEAVE_MAX_MS) fault("still in START box");
        return;
    }
    case St::FIND_LINE:
    {
        nPlain = v.plain ? nPlain + 1 : 0;
        if (nPlain >= CONFIRM_TICKS)
        {
            normLo = (float)v.lo; normHi = (float)v.hi;   // where the plain line sits
            startLeg(0);
            return;
        }
        driveStraight(LEAVE_SPEED);
        if (inState > FIND_MAX_MS) startSearch("no line at box exit");
        return;
    }
    case St::FOLLOW:
    {
        if (now - tLeg > LEG_MAX_MS) { fault("leg took too long"); return; }
        const bool lastLeg = legIdx >= LEG_COUNT - 1;

        // Last leg: the filled PARK box.
        if (lastLeg)
        {
            nBox = v.bar ? nBox + 1 : 0;
            if (nBox >= BOX_TICKS) { nWhite = 0; enter(St::PARK_IN, "park in"); return; }
        }
        else
        {
            nJunction = (junctionSeen() && now - tLastCross > CROSS_IGNORE_MS) ? nJunction + 1 : 0;
            if (nJunction >= CONFIRM_TICKS) { nJunction = 0; nPlain = 0; nLost = 0; enter(St::CROSS, "junction"); return; }
        }

        if (!v.on)
        {
            if (++nLost >= LOST_TICKS) { nLost = 0; enter(St::CREEP, "line lost"); }
            else driveStraight(SLOW_SPEED);
            return;
        }
        nLost = 0;

        learnLinePlace();
        const bool slow = (legIdx == 0) || afterLastJunction || lastLeg || (now - tLeg < 600);
        if (!v.bar && !v.leftEdge && !v.rightEdge) followLine(slow ? SLOW_SPEED : FOLLOW_SPEED);
        else { driveStraight(CROSS_SPEED); lastSteer = 0.0f; }   // a junction is starting: no steering
        return;
    }
    case St::CROSS:
    {
        // Straight across. A plain line ahead again (junction gone) = it was a crossing.
        // Nothing at all for LOST_TICKS = the line ended here.
        driveStraight(CROSS_SPEED);
        nPlain = v.plain ? nPlain + 1 : 0;
        nLost  = !v.on ? nLost + 1 : 0;
        if (nPlain >= CONFIRM_TICKS)
        {
            crossed++;
            tLastCross = now;
            if (crossed >= EXPECT_CROSS[legIdx]) afterLastJunction = true;
            resetSteering();
            enter(St::FOLLOW, LEG_NAME[legIdx]);
            return;
        }
        if (nLost >= LOST_TICKS) { lineEnded(); return; }
        if (inState > CROSS_MAX_MS)
        {
            if (!v.on) { lineEnded(); return; }
            fault("junction never cleared");
        }
        return;
    }
    case St::CREEP:
    {
        // Lost with no junction before it: maybe a dropout. Straight for a moment.
        driveStraight(SLOW_SPEED);
        nPlain = v.plain ? nPlain + 1 : 0;
        if (nPlain >= CONFIRM_TICKS) { resetSteering(); enter(St::FOLLOW, LEG_NAME[legIdx]); return; }
        if (inState > LOST_CREEP_MS) lineEnded();     // still nothing: it ended (or we left it)
        return;
    }
    case St::LOCATE:
    {
        // Back up slowly until the array is on the corner tape again (any dark under it).
        if (v.on && nPlain++ >= 1) { nPlain = 0; settleThen(St::ROLL); return; }
        if (!v.on) nPlain = 0;
        driveStraight(-LOCATE_SPEED);
        if (inState > LOCATE_MAX_MS) fault("corner tape not found backing up");
        return;
    }
    case St::ROLL:
    {
        driveStraight(ROLL_SPEED);
        if (inState >= ROLL_MS) settleThen(St::TURN);
        return;
    }
    case St::SETTLE:
    {
        halt();
        if (inState < SETTLE_MS) return;
        switch (afterSettle)
        {
        case St::LOCATE:    nPlain = 0; enter(St::LOCATE); return;
        case St::ROLL:      enter(St::ROLL, "roll to corner"); return;
        case St::TURN:      startTurn(); return;
        case St::TURN_BACK: enter(St::TURN_BACK, "turn back"); return;
        case St::ALIGN:     startSpin(); alignDir = 0; enter(St::ALIGN); return;
        case St::VERIFY:    verifyGood = 0; enter(St::VERIFY); return;
        case St::SEARCH:    startSpin(); searchPhase = 0; enter(St::SEARCH); return;
        case St::DONE:      why = "-"; enter(St::DONE, "PARKED"); userLed.setSolid(0, 255, 0); return;
        default:            fault("settle: no next state"); return;
        }
    }
    case St::TURN:
    {
        if (inState > TURN_MAX_MS) { fault("turn timeout"); return; }
        if (gyroOk)
        {
            const float a = turned();
            if ((a >= TURN_LO_DEG && lineAcquired()) || (a >= TURN_DEG && v.plain))
            { turnAcc = spinAcc; landErr = v.err; settleThen(St::ALIGN); return; }
            if (a > TURN_HI_DEG) { note = "no line: turn back"; settleThen(St::TURN_BACK); return; }
            spin(TURN_DIR, (a < TURN_DEG - SLOW_DEG) ? TURN_FAST : TURN_SLOW);
            return;
        }
        // Gyro not ready: leave whatever line is under the array, stop on the next one.
        if (v.on) tOffLine = now;
        if (!leftStartLine && now - tOffLine >= TAPE_OFF_MS) leftStartLine = true;
        if (leftStartLine && v.plain) { turnAcc = 0.0f; landErr = v.err; settleThen(St::ALIGN); return; }
        spin(TURN_DIR, TURN_SLOW);
        return;
    }
    case St::TURN_BACK:
    {
        const float a = turned();
        if (a >= TURN_LO_DEG && a <= TURN_HI_DEG && v.plain) { turnAcc = spinAcc; landErr = v.err; settleThen(St::ALIGN); return; }
        if (a < TURN_LO_DEG - 5.0f || inState > TURN_BACK_MS) { fault("no line after turn"); return; }
        spin(-TURN_DIR, TURN_SLOW);
        return;
    }
    case St::ALIGN:
    {
        if (inState > ALIGN_MAX_MS || (gyroOk && turned() > ALIGN_MAX_DEG))
        {
            if (v.on && abs(v.err) <= ACQUIRE_ERR) { settleThen(St::VERIFY); return; }
            fault("align: line not centred");
            return;
        }
        if (v.on && abs(v.err) <= (alignDir == 0 ? ALIGN_OK : ALIGN_DONE)) { settleThen(St::VERIFY); return; }
        int want;
        if (v.on) want = (v.err > 0) ? STEER_SIGN : -STEER_SIGN;           // +1 = turn right
        else      want = (line.lastSeenSide() != 0) ? STEER_SIGN * line.lastSeenSide() : 0;
        if (want == 0) { fault("align: no line"); return; }
        if (alignDir != 0 && want != alignDir) { settleThen(St::VERIFY); return; }   // crossed the centre
        alignDir = want;
        spin(alignDir, TURN_SLOW);
        return;
    }
    case St::VERIFY:
    {
        halt();
        if (v.on && !v.bar && abs(v.err) <= ACQUIRE_ERR) verifyGood++;
        if (verifyGood >= VERIFY_TICKS)
        {
            if (turnPending)
            {
                turnPending = false;
                planHeading = wrap180(planHeading + TURN_DIR * TURN_DEG);
                if (gyroSign == 0 && fabsf(turnAcc) > 45.0f)
                    gyroSign = ((turnAcc > 0.0f) ? 1 : -1) * TURN_DIR;
                if (gridKnown() && fabsf(gridError()) > GRID_FAULT_DEG) { fault("wrong heading after turn"); return; }
                startLeg(legIdx + 1);
            }
            else { resetSteering(); nLost = 0; enter(St::FOLLOW, LEG_NAME[legIdx]); }
            return;
        }
        if (inState > VERIFY_MAX_MS) startSearch("verify failed");
        return;
    }
    case St::SEARCH:
    {
        if (v.on && !v.bar && abs(v.err) <= ACQUIRE_ERR) { settleThen(St::ALIGN); return; }
        const float a = gyroOk ? turned() : 0.0f;
        if (searchPhase == 0)
        {
            const bool done = gyroOk ? (a >= SEARCH_DEG) : (inState > SEARCH_SIDE_MS);
            if (done) { searchPhase = 1; startSpin(); tState = now; return; }
            spin(searchDir, TURN_SLOW);
            return;
        }
        const bool done = gyroOk ? (a >= 2.0f * SEARCH_DEG) : (inState > 2 * SEARCH_SIDE_MS);
        if (done || inState > 6000) { fault("search: line not found"); return; }
        spin(-searchDir, TURN_SLOW);
        return;
    }
    case St::PARK_IN:
    {
        // Straight in for PARK_MS. If the array runs out of the far side of the box first
        // (all white), stop there.
        nWhite = !v.on ? nWhite + 1 : 0;
        if (inState >= PARK_MS || nWhite >= (int)PARK_EXIT_TICKS) { settleThen(St::DONE); return; }
        driveStraight(PARK_SPEED);
        return;
    }
    case St::DONE:
    case St::FAULT:
        halt();
        return;
    }
}

const char *stateName(St s)
{
    switch (s)
    {
    case St::LEAVE_BOX: return "LEAVE BOX";
    case St::FIND_LINE: return "FIND LINE";
    case St::FOLLOW:    return "FOLLOW";
    case St::CROSS:     return "CROSS";
    case St::CREEP:     return "CREEP";
    case St::LOCATE:    return "LOCATE";
    case St::ROLL:      return "ROLL";
    case St::SETTLE:    return "SETTLE";
    case St::TURN:      return "TURN";
    case St::TURN_BACK: return "TURN BACK";
    case St::ALIGN:     return "ALIGN";
    case St::VERIFY:    return "VERIFY";
    case St::SEARCH:    return "SEARCH";
    case St::PARK_IN:   return "PARK IN";
    case St::DONE:      return "DONE";
    case St::FAULT:     return "FAULT";
    }
    return "?";
}

char maskStr[9];
const char *maskText()
{
    for (int i = 0; i < 8; i++) maskStr[i] = (v.mask & (1 << i)) ? '#' : '.';
    maskStr[8] = 0;
    return maskStr;
}

// =====================================================================
//  OPMODE
// =====================================================================
class ParkGyroLine : public NRLOpMode
{
public:
    void init() override            // INIT does not move anything (R8.11)
    {
        leftMotor.begin();
        rightMotor.begin();
        line.begin();
    }

    void start() override
    {
        // Everything from scratch on every run.
        tRun = tLeg = millis();
        tLastCross = 0;
        legIdx = 0; crossed = 0;
        turnPending = false; afterLastJunction = false;
        searchesThisLeg = 0;
        nLost = nPlain = nJunction = nBox = nWhite = 0;
        normLo = 3.0f; normHi = 4.0f;
        planHeading = 0.0f; gyroSign = 0;
        freeStart = NRLComms::getHeadingFree();   // robot placed square in START
        landErr = 0; turnAcc = 0.0f;
        why = "-"; note = "-";
        resetSteering();
        drive.setRampRate(RAMP_RATE);
        userLed.setOff();
        if (!line.isCalibrated()) { fault("line array not calibrated"); return; }
        enter(St::LEAVE_BOX, "START");
    }

    void loop() override
    {
        tick();
        static char crossStr[8];
        crossStr[0] = (char)('0' + crossed % 10); crossStr[1] = '/';
        crossStr[2] = (char)('0' + EXPECT_CROSS[legIdx < LEG_COUNT ? legIdx : LEG_COUNT - 1]); crossStr[3] = 0;
        telemetry.addData("leg", LEG_NAME[legIdx < LEG_COUNT ? legIdx : LEG_COUNT - 1]);
        telemetry.addData("state", stateName(st));
        telemetry.addData("why", (st == St::FAULT) ? why : note);
        telemetry.addData("crossed", crossStr);
        telemetry.addData("mask", maskText());
        telemetry.addData("err", (float)v.err);
        telemetry.addData("land", (float)landErr);
        telemetry.addData("gyro", NRLComms::isHeadingReady() ? (gyroSign != 0 ? "ok, sign known" : "ok") : "NOT READY");
    }

    void stop() override { drive.stop(); }
};

}  // namespace

REGISTER_OPMODE(ParkGyroLine, "Park Gyro+Line", AUTO);