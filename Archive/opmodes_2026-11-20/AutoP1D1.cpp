// =====================================================================
//  GLITCHED (Team 190) - "P1-D1 Clean": START -> P1 -> D1 -> PARK
//  File: RobotFirmware/opmodes/AutoP1D1.cpp
//
//  A from-scratch rewrite. It shares no code with AutoPark.cpp, and that file is
//  untouched ("Park Run", "P1-D1 Run", "Park Run Tape", "First Junction" stay as they are).
//
//  ROUTE (seen from above, C4 bottom-left, START on the bottom line facing east):
//    START -> C1 (left) -> P1 branch (right) -> P1 end bar -> U-turn -> main line (right)
//          -> D1 branch (left) -> D1 end bar -> U-turn -> main line (left)
//          -> C2 (left) -> C3 (left) -> C4 (left) -> PARK box -> stop
//
//  HOW IT IS BUILT (four layers, each only talks to the one below):
//    1. SENSOR    readLine(): one snapshot of the IR array per tick, plus debounce counters.
//    2. MOTION    encoder distance along the current leg, "is the robot still", gyro angle turned.
//    3. SKILLS    follow the line, roll to a pivot point, turn by gyro + tape, align, search.
//    4. MISSION   a table of LEGS. Each leg = follow the line until ONE expected event
//                 (corner, branch, end bar, T, park box) inside a distance WINDOW, then act.
//                 Anything seen outside the window is crossed straight over and ignored.
//
//  SAFETY RULES
//    - Every state has a timeout. Every failure path ends in FAULT: motors stopped, held stopped.
//    - Lost line: creep straight at most DROPOUT_M, stop, bounded search (+/- SEARCH_DEG), else FAULT.
//    - A turn never spins more than its target + TURN_LATE_DEG, then searches back, else FAULT.
//    - An event is only accepted after CONFIRM_TICKS identical readings in a row, inside its window.
//    - A branch that was not seen by the end of its window is looked for by backing up a
//      bounded distance; if still not found the robot STOPS (it never skips P1 or D1).
//
//    - Nothing learned is kept between runs: every START begins from the same values.
//
//  Start: robot centred in the START box, facing the line that leaves it (east), after a
//  LINE TEST calibration on this mat. Values marked MEASURE are not measured yet.
//  User LED: off while running, GREEN = parked (DONE), RED = stopped on a fault ("why" on screen).
//
//  STATUS: compiled against the SDK v1.5.0 headers and run in the team simulator only.
//          NOT run on the robot yet.
// =====================================================================
#include "NRL.h"

namespace {

// =====================================================================
//  HARDWARE - copied from AutoPark.cpp (motor setup verified on the robot 2026-11-19)
// =====================================================================
HexaDCMotor leftMotor{{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .minDrive = 34 }};
HexaDCMotor rightMotor{{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true, .minDrive = 32 }};
TankDrive     drive(leftMotor, rightMotor);
HexaLineArray line{hexaLineConfig()};

// Encoders: channel A only, rising edges, so counts only ever go UP (forward, reverse or spin).
constexpr int      LEFT_ENC_PIN  = DIGITAL_D3;   // GPIO37
constexpr int      RIGHT_ENC_PIN = DIGITAL_D2;   // GPIO36
constexpr float    CPM           = 1300.0f;      // counts per metre (Encoder Accuracy, 2026-11-19)
constexpr uint32_t GLITCH_US     = 50;

volatile uint32_t encL = 0, encR = 0, lastUsL = 0, lastUsR = 0;
void IRAM_ATTR leftIsr()  { const uint32_t t = micros(); if (t - lastUsL > GLITCH_US) { encL = encL + 1; lastUsL = t; } }
void IRAM_ATTR rightIsr() { const uint32_t t = micros(); if (t - lastUsR > GLITCH_US) { encR = encR + 1; lastUsR = t; } }

// =====================================================================
//  TUNING
// =====================================================================
// Geometry
constexpr float PIVOT_FWD_M  = 0.08f;    // sensor bar ahead of the chassis middle (measured 2026-11-19)
constexpr float LINE_HALF_M  = 0.0125f;  // half of the 25 mm tape
constexpr float PARK_IN_M    = 0.25f + PIVOT_FWD_M;   // after the bar reaches the PARK box edge: chassis middle to box middle
constexpr float ROLL_COAST_M = 0.010f;   // MEASURE: roll after the motors cut at ROLL_SPEED (stop this much early).
                                         // Only the first guess: each run measures it after every roll and
                                         // uses that for the next one (reset at START, never kept between runs).
constexpr float COAST_MAX_M  = 0.10f;
constexpr float PIVOT_TRIM_M = 0.0f;     // TUNE: + = every turn pivots later, - = earlier
constexpr float PIVOT_TOL_M  = 0.02f;    // stopped further than this from the pivot point: one slow correction

// Speeds (TankDrive units, -1..1). Slow on purpose: reliability over speed.
constexpr float LEAVE_SPEED  = 0.22f;    // straight out of the START box
constexpr float FOLLOW_SPEED = 0.30f;    // main line, away from any expected event
constexpr float NEAR_SPEED   = 0.20f;    // inside / just before an event window, first part of a leg
constexpr float BRANCH_SPEED = 0.20f;    // on a terminal branch (P1, D1)
constexpr float ROLL_SPEED   = 0.18f;    // roll to the pivot point, park-in, back-up
constexpr float TRIM_SPEED   = 0.15f;    // the one correction move to the pivot point (slower = less coast)
constexpr float TURN_FAST    = 0.20f;    // spin speed
constexpr float TURN_SLOW    = 0.17f;    //   ...for the last SLOW_DEG, searches and alignment
constexpr float RAMP_RATE    = 0.10f;    // max motor change per tick

// Line following: PD on the line error (-1..+1). Gains from AutoPark (simulator: insensitive).
constexpr int   STEER_SIGN     = 1;      // +1: channel 0 is the LEFT-most sensor (confirmed 2026-11-19)
constexpr float STEER_KP       = 0.22f;
constexpr float STEER_KD       = 0.08f;
constexpr float STEER_MAX      = 0.25f;
constexpr float STEER_STEP_MAX = 0.025f; // steering may change at most this much per tick
constexpr float SPEED_DROP     = 0.50f;  // slow down as the line moves off centre
HexaPID steerPid{ HexaPIDConfig{ .kP = STEER_KP, .kD = STEER_KD, .outMin = -STEER_MAX, .outMax = STEER_MAX, .dAlpha = 0.5f } };

// Line reading (channel indices, 0 = left)
constexpr int BAR_DARK      = 6;         // >= this many dark channels = bar / box / crossing (allows 2 dead)
constexpr int PLAIN_MAX     = 5;         // a plain line never darkens more than this many channels
// Where the plain, centred line sits on the array (left-most / right-most dark channel). First
// guess = 2 channels in the middle; each run learns the real values while following, because
// they depend on the sensor pitch. A branch pushes ONE side of the dark area outward while the
// other side stays where the line normally is; a line that drifts moves BOTH sides.
constexpr float NORM_LO_START = 3.0f;
constexpr float NORM_HI_START = 4.0f;
constexpr int   EDGE_GROW     = 1;       // the branch side must reach at least this many channels past normal
                                         // (the other side must not have moved away from normal)
constexpr int CONFIRM_TICKS = 3;         // identical readings in a row before an event is accepted
constexpr int BOX_TICKS     = 5;         // ...for the PARK box
constexpr int LOST_TICKS    = 6;         // no line this many ticks = line ended (only counts as an event
                                         // if the event's own bar / edge was seen in the last SIG_RECENT_M)
constexpr float SIG_RECENT_M = 0.05f;
constexpr int ACQUIRE_ERR   = 1200;      // a turn stops on a line within this of centre (1000 = one sensor)
constexpr int ALIGN_OK      = 700;       // after a turn: closer than this is centred
constexpr int ALIGN_DONE    = 350;       //   ...alignment rotates until within this

// Recovery limits
constexpr float DROPOUT_M     = 0.05f;   // unexpected line loss: creep straight at most this far
constexpr float CROSS_MAX_M   = 0.08f;   // crossing an ignored junction: no steering for at most this far
constexpr float BACKTRACK_M   = 0.25f;   // missed event: back up at most this far looking for it
constexpr float SEARCH_DEG    = 30.0f;   // bounded search: each side
constexpr float ALIGN_MAX_DEG = 20.0f;
constexpr float TURN_EARLY_DEG = 30.0f;  // a 90 deg turn accepts a line from 60 deg ...
constexpr float TURN_LATE_DEG  = 25.0f;  //   ... to 115 deg; never further
constexpr float UTURN_EARLY_DEG = 35.0f; // a U-turn accepts a line from 145 to 210 deg
constexpr float UTURN_LATE_DEG  = 30.0f;
constexpr float SLOW_DEG       = 40.0f;  // spin slower for the last 40 deg
constexpr float GRID_FAULT_DEG = 60.0f;  // after a turn, heading this far from the plan = wrong line -> FAULT

// Timeouts (ms)
constexpr uint32_t LEAVE_MAX_MS    = 5000;
constexpr uint32_t LEG_MAX_MS      = 20000;   // one leg of following (the longest is about 2 m)
constexpr uint32_t ROLL_MAX_MS     = 2500;
constexpr uint32_t SETTLE_MIN_MS   = 150;
constexpr uint32_t SETTLE_STILL_MS = 200;     // no encoder counts this long = stopped
constexpr uint32_t SETTLE_MAX_MS   = 900;
constexpr uint32_t TURN_MAX_MS     = 7000;
constexpr uint32_t TAPE_OFF_MS     = 150;     // gyro not ready: the array must be off every line this long
                                              // (it has left the line it started on); the next line is the target
constexpr uint32_t SEARCH_SIDE_MS  = 1500;    // gyro not ready: each search side by time
constexpr uint32_t ALIGN_MAX_MS    = 1500;
constexpr uint32_t VERIFY_MAX_MS   = 400;
constexpr int      VERIFY_TICKS    = 5;
constexpr uint32_t PARK_MAX_MS     = 4000;
constexpr uint32_t ENC_STALL_MS    = 800;     // driving forward with no counts on EITHER side = encoders lost
constexpr uint32_t ENC_SIDE_MS     = 400;     // one side silent this long while the other counts = that side dead
constexpr uint32_t MATCH_STOP_MS   = 145000;  // stop before the 2:30 limit

// =====================================================================
//  MISSION TABLE
// =====================================================================
// Distances are measured along the leg from where it starts:
//   - after a 90 deg turn the sensor bar is PIVOT_FWD_M past the junction it turned on;
//   - after a U-turn on an end bar it is about 2 x PIVOT_FWD_M back from the bar;
//   - the first leg starts where the bar leaves the START box (box edge to C1 = 0.30 m).
// Manual: main line 2 m square, terminals 0.5 m, P1 / D1 / P2 at 0.5 / 1.0 / 1.5 m from C1.
enum class Ev : uint8_t { CORNER_L, BRANCH_R, BRANCH_L, END_BAR, T_JUNCTION, PARK_BOX };

struct Leg
{
    const char *name;    // mission stage, shown on the screen
    Ev    ev;            // the ONE event that ends this leg
    float winMin, winMax;// the event is accepted only between these leg distances (m)
    int   turnDir;       // then: -1 turn left, +1 turn right, 0 no turn
    float turnDeg;       // 90, 180 (U-turn), 0
    bool  onBranch;      // a terminal branch: drive slower
};

constexpr Leg LEGS[] = {
    // name           event            window (m)      turn   deg     branch   expected
    {"START->C1",     Ev::CORNER_L,    0.12f, 0.55f,   -1,    90.0f,  false},  // 0.28
    {"C1->P1",        Ev::BRANCH_R,    0.28f, 0.62f,   +1,    90.0f,  false},  // 0.42
    {"P1 terminal",   Ev::END_BAR,     0.25f, 0.70f,   -1,   180.0f,  true },  // 0.42
    {"P1->main",      Ev::T_JUNCTION,  0.18f, 0.60f,   +1,    90.0f,  true },  // 0.34, right = north
    {"main->D1",      Ev::BRANCH_L,    0.28f, 0.62f,   -1,    90.0f,  false},  // 0.42
    {"D1 terminal",   Ev::END_BAR,     0.25f, 0.70f,   -1,   180.0f,  true },  // 0.42
    {"D1->main",      Ev::T_JUNCTION,  0.18f, 0.60f,   -1,    90.0f,  true },  // 0.34, left = north
    {"main->C2",      Ev::CORNER_L,    0.70f, 1.15f,   -1,    90.0f,  false},  // 0.92 (P2 at 0.42 crossed)
    {"C2->C3",        Ev::CORNER_L,    1.65f, 2.20f,   -1,    90.0f,  false},  // 1.92 (P3 D2 P4 crossed)
    {"C3->C4",        Ev::CORNER_L,    1.65f, 2.20f,   -1,    90.0f,  false},  // 1.92 (P5 D3 P6 crossed)
    {"C4->PARK",      Ev::PARK_BOX,    0.08f, 0.50f,    0,     0.0f,  false},  // 0.22
};
constexpr int LEG_COUNT = sizeof(LEGS) / sizeof(LEGS[0]);

// =====================================================================
//  1. SENSOR LAYER
// =====================================================================
struct LineView
{
    uint8_t mask = 0;
    int  count = 0;
    bool on = false;          // any channel dark this tick
    int  err = 0;             // -3500..3500, + = line right of centre (valid when on)
    int  lo = -1, hi = -1;    // left-most / right-most dark channel
    bool bar = false;         // most of the array dark: bar, crossing, box
    bool leftEdge = false;    // a line reaching the LEFT edge from the middle (left branch / corner)
    bool rightEdge = false;   // a line reaching the RIGHT edge from the middle
    bool plain = false;       // an ordinary single line, nothing else
};

LineView v;
float normLo = NORM_LO_START, normHi = NORM_HI_START;   // learned this run (see above)
// A branch must reach the second channel from the edge (1 / 6), not the last one, so a dead or
// silent outer channel cannot hide a branch.
constexpr int EDGE_L = 1, EDGE_R = 6;

LineView readLine()
{
    LineView r;
    r.mask  = line.mask();
    r.count = line.count();
    r.on    = line.onLine();
    r.err   = line.error();
    for (int i = 0; i < 8; i++)
        if (r.mask & (1 << i)) { if (r.lo < 0) r.lo = i; r.hi = i; }
    r.bar = r.count >= BAR_DARK;
    // A branch edge: the dark area reaches the outermost working channel on that side and has
    // grown outward from where the plain line sits, while its other side has not moved.
    // Both tests must pass: the fixed geometry (reaches channel 1 / 6 AND still covers the middle)
    // and the learned one (grew outward from where the line sits; the other side did not move).
    const int nLo = (int)(normLo + 0.5f), nHi = (int)(normHi + 0.5f);   // whole channels
    r.leftEdge  = r.on && r.lo <= EDGE_L && r.hi >= 4 && r.lo <= nLo - EDGE_GROW && r.hi >= nHi;
    r.rightEdge = r.on && r.hi >= EDGE_R && r.lo <= 3 && r.hi >= nHi + EDGE_GROW && r.lo <= nLo;
    r.plain = r.on && !r.bar && !r.leftEdge && !r.rightEdge && r.count <= PLAIN_MAX;
    return r;
}

// Counts how many ticks in a row a condition has held, and remembers the leg distance where
// the current run of ticks began (where the feature was FIRST seen, not where it was confirmed).
struct Debounce
{
    int   n = 0;
    float startM = 0.0f;
    void  feed(bool c, float atM) { if (c) { if (n == 0) startM = atM; n++; } else n = 0; }
    bool  held(int k) const { return n >= k; }
    void  clear() { n = 0; }
};

// =====================================================================
//  2. MOTION LAYER
// =====================================================================
float    legM = 0.0f;                 // distance along the current leg (m), forward +, reverse -
uint32_t lastEncL = 0, lastEncR = 0;
uint32_t tLastAnyCount = 0, tLastL = 0, tLastR = 0;
bool     deadL = false, deadR = false;
int      moveSign = 0;                // +1 driving forward, -1 reversing, 0 not moving straight
int      coastSign = 0;               // direction of the last straight drive (counts while coasting to a stop)
uint32_t tLastMove = 0;               // last encoder count of any kind (for "still")

// Called once per tick. Only straight driving (and the coast after it) adds to legM: spins also
// make counts (the encoders cannot tell direction) and must not count as distance.
void updateOdometry(uint32_t now, bool settling)
{
    const uint32_t l = encL, r = encR;
    const uint32_t dL = l - lastEncL, dR = r - lastEncR;
    lastEncL = l; lastEncR = r;
    if (dL || dR) tLastMove = now;
    if (moveSign == 0)
    {
        tLastAnyCount = tLastL = tLastR = now;
        if (settling && coastSign != 0) legM += coastSign * 0.5f * (float)(dL + dR) / CPM;
        return;
    }
    if (dL) tLastL = now;
    if (dR) tLastR = now;
    if (dL || dR) tLastAnyCount = now;
    // One side silent while the other keeps counting: use the working side from now on.
    if (!deadL && !deadR)
    {
        if (now - tLastL > ENC_SIDE_MS && now - tLastR < 50) deadL = true;
        if (now - tLastR > ENC_SIDE_MS && now - tLastL < 50) deadR = true;
    }
    float counts;
    if (deadL)      counts = (float)dR;
    else if (deadR) counts = (float)dL;
    else            counts = 0.5f * (float)(dL + dR);
    legM += moveSign * counts / CPM;
}

bool encodersStalled(uint32_t now) { return moveSign != 0 && now - tLastAnyCount > ENC_STALL_MS; }

float wrap180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// Gyro angle turned since startSpin(), always positive whichever way the robot spins.
// (The sign of the SDK heading is never needed: we accumulate signed changes and take |sum|.)
bool  gyroOk = false;
float spinAcc = 0.0f, spinPrev = 0.0f;
void  startSpin()   { gyroOk = NRLComms::isHeadingReady(); spinAcc = 0.0f; spinPrev = NRLComms::getHeadingFree(); }
void  updateSpin()
{
    const float h = NRLComms::getHeadingFree();
    spinAcc += wrap180(h - spinPrev);
    spinPrev = h;
}
float turned() { return fabsf(spinAcc); }

// Heading check against the plan ("grid"): every planned turn changes the expected heading by
// +/-90 or 180. The SDK's sign convention is learned on the first gyro turn.
float freeStart = 0.0f;
float planHeading = 0.0f;             // expected heading, degrees, + = right turns
int   gyroSign = 0;                   // +1 if heading grows on a right turn, -1 if it shrinks, 0 = unknown
float gridError()
{
    if (gyroSign == 0) return 0.0f;
    return wrap180(gyroSign * (NRLComms::getHeadingFree() - freeStart) - planHeading);
}

// =====================================================================
//  3. SKILLS: follow, roll, turn, align, search
// =====================================================================
float lastSteer = 0.0f;

void driveStraight(float speed) { moveSign = (speed > 0.0f) ? 1 : -1; drive.drive(speed, 0.0f); }
void spin(int dir, float speed) { moveSign = 0; coastSign = 0; drive.drive(0.0f, dir * speed); }
void halt()                     { if (moveSign != 0) coastSign = moveSign; moveSign = 0; drive.stop(); }

// One tick of PD line following at up to `speed`. Only call with v.on.
void followLine(float speed)
{
    moveSign = 1;
    const float e = (float)v.err / 3500.0f;
    float t = STEER_SIGN * steerPid.update(e);
    if (t > lastSteer + STEER_STEP_MAX) t = lastSteer + STEER_STEP_MAX;
    if (t < lastSteer - STEER_STEP_MAX) t = lastSteer - STEER_STEP_MAX;
    lastSteer = t;
    float f = speed * (1.0f - SPEED_DROP * fabsf(e));
    if (f < 0.5f * speed) f = 0.5f * speed;
    drive.drive(f, t);
}

void resetSteering() { steerPid.reset(); lastSteer = 0.0f; }

// A line good enough to stop a turn on: a plain line close to the centre.
bool lineAcquired() { return v.plain && abs(v.err) <= ACQUIRE_ERR; }

// =====================================================================
//  4. MISSION STATE MACHINE
// =====================================================================
enum class St : uint8_t
{
    LEAVE_BOX,     // straight out of the filled START box
    FIND_LINE,     // just out of the box: the line must be under the bar
    FOLLOW,        // follow the line until this leg's event (inside its window)
    BACKTRACK,     // the event was not seen by the end of its window: back up and look for it
    ROLL,          // drive straight so the chassis middle is on the junction, then stop
    SETTLE,        // motors off, wait until the robot is still, then -> afterSettle
    TURN,          // gyro-measured spin toward the new line, stops on the line
    TURN_BACK,     // spun too far without a line: spin back through the window once
    ALIGN,         // after a stop: rotate a little until the line is centred
    VERIFY,        // stationary: the line must be seen steadily before driving on
    SEARCH,        // lost line: bounded search both sides
    PARK_IN,       // in the PARK box: drive in until the robot is fully inside
    DONE,
    FAULT
};

St          st = St::LEAVE_BOX;
St          afterSettle = St::FAULT;
int         legIdx = 0;
uint32_t    tState = 0, tRun = 0, tLeg = 0;
const char *why = "-";
const char *note = "-";

// per-state working values
Debounce dbLeft, dbRight, dbBar, dbLost, dbPlain;
float    crossUntilM = -1.0f;   // crossing an ignored junction until this leg distance
float    rollTarget = 0.0f;     // ROLL: leg distance to stop at
int      rollDir = 1;           // ROLL: +1 forward, -1 backward
bool     rollTrim = false;      // ROLL: this is the correction move (slow, no further correction)
float    pivotM = 0.0f;         // where the chassis middle should stop (leg distance of the bar)
float    coastM = ROLL_COAST_M; // roll after a stop at ROLL_SPEED, measured during THIS run
float    rollStopM = 0.0f;      // where the last ROLL cut the motors
bool     coastMeasure = false;  // the next SETTLE ends a ROLL: measure the coast
float    turnAcc = 0.0f;        // signed gyro angle of the last mission turn (for the heading check)
float    backFromM = 0.0f;      // BACKTRACK: where it started
int      turnDir = 0;           // TURN / TURN_BACK
float    turnTarget = 90.0f;
float    turnLo = 60.0f, turnHi = 115.0f;
int      searchPhase = 0, searchDir = 1;
int      searchesThisLeg = 0;
int      alignDir = 0;
int      verifyGood = 0;
uint32_t tOffLine = 0;          // TURN without gyro: when the array last saw a line
bool     leftStartLine = false; // TURN without gyro: the array has left the line it started on
float    sigStartM = 0.0f, sigLastM = -1.0f;   // FOLLOW: the leg's own event pattern, first / last seen
bool     turnPending = false;   // VERIFY is the end of a mission turn: start the next leg after it
float    parkFromM = 0.0f;

const Leg &leg() { return LEGS[legIdx < LEG_COUNT ? legIdx : LEG_COUNT - 1]; }

void enter(St s, const char *n = nullptr)
{
    st = s;
    tState = millis();
    if (n) note = n;
}

void fault(const char *reason)
{
    why = reason;
    halt();
    enter(St::FAULT);
    userLed.setSolid(255, 0, 0);          // red: stopped on a fault (reason on the screen)
}

void settleThen(St next)
{
    halt();
    afterSettle = next;
    enter(St::SETTLE);
}

void startLeg(int idx)
{
    legIdx = idx;
    legM = 0.0f;
    tLeg = millis();
    crossUntilM = -1.0f;
    searchesThisLeg = 0;
    sigLastM = -1.0f;
    dbLeft.clear(); dbRight.clear(); dbBar.clear(); dbLost.clear(); dbPlain.clear();
    resetSteering();
    enter(St::FOLLOW, leg().name);
}

// Leg event -> where is the middle of the tape that triggered it (leg distance)?
//   edge / bar seen: the bar touched the NEAR edge of the tape at `startM`  -> centre = startM + half tape
//   line lost:       the bar left the FAR edge of the tape at `startM`      -> centre = startM - half tape
void beginPivot(float tapeCentreM)
{
    pivotM = tapeCentreM + PIVOT_FWD_M + PIVOT_TRIM_M;
    rollTarget = pivotM - coastM;
    rollTrim = false;
    resetSteering();
    if (fabsf(rollTarget - legM) < 0.005f) { settleThen(St::TURN); return; }
    rollDir = (rollTarget > legM) ? 1 : -1;
    enter(St::ROLL, "roll to pivot");
}

void startTurn(int dir, float deg)
{
    turnDir = dir;
    turnTarget = deg;
    const bool u = deg > 135.0f;
    turnLo = deg - (u ? UTURN_EARLY_DEG : TURN_EARLY_DEG);
    turnHi = deg + (u ? UTURN_LATE_DEG : TURN_LATE_DEG);
    startSpin();
    tOffLine = millis();
    leftStartLine = false;
    enter(St::TURN, u ? "U-turn" : (dir > 0 ? "turn right" : "turn left"));
}

// The current leg's event has been confirmed. tapeCentreM: see beginPivot().
void eventReached(float tapeCentreM)
{
    const Leg &L = leg();
    switch (L.ev)
    {
    case Ev::PARK_BOX:
        parkFromM = legM;
        resetSteering();
        enter(St::PARK_IN, "park in");
        return;
    case Ev::END_BAR:
        // "Stop completely at the terminal strip", then U-turn in place.
        turnPending = true;
        settleThen(St::TURN);
        return;
    default:
        turnPending = true;
        beginPivot(tapeCentreM);
        return;
    }
}

void startSearch(const char *reason)
{
    if (++searchesThisLeg > 2) { fault("line lost (2 searches failed)"); return; }
    note = reason;
    const int side = line.lastSeenSide();
    searchDir = (side != 0) ? STEER_SIGN * side : 1;
    searchPhase = 0;
    halt();
    afterSettle = St::SEARCH;
    enter(St::SETTLE, reason);
}

// Is the leg's own event visible right now? (used by FOLLOW inside the window and by BACKTRACK)
bool eventSignature(Ev ev)
{
    switch (ev)
    {
    case Ev::CORNER_L:   return v.leftEdge || v.bar;
    case Ev::BRANCH_R:   return v.rightEdge;
    case Ev::BRANCH_L:   return v.leftEdge;
    case Ev::END_BAR:    return v.bar;
    case Ev::T_JUNCTION: return v.bar || (v.leftEdge && v.rightEdge);
    case Ev::PARK_BOX:   return v.bar;
    }
    return false;
}

// Does losing the line inside the window also mean "the event is here"? (L corner, end of a
// terminal, top of a T: the line really ends there. A side branch: the main line continues.)
bool lostMeansEvent(Ev ev) { return ev == Ev::CORNER_L || ev == Ev::END_BAR || ev == Ev::T_JUNCTION; }

// ---------------------------------------------------------------------
//  One tick of the state machine
// ---------------------------------------------------------------------
void tick()
{
    const uint32_t now = millis();
    v = readLine();
    updateOdometry(now, st == St::SETTLE);
    if (st == St::TURN || st == St::TURN_BACK || st == St::ALIGN || st == St::SEARCH || st == St::SETTLE)
        updateSpin();   // SETTLE too: a spin keeps coasting after the motors stop

    if (st != St::DONE && st != St::FAULT)
    {
        if (now - tRun > MATCH_STOP_MS) { fault("match time"); return; }
        if (encodersStalled(now))       { fault("encoders: no counts"); return; }
    }

    switch (st)
    {
    // -----------------------------------------------------------------
    case St::LEAVE_BOX:
    {
        // Straight, no steering: inside the filled box the array is all dark and position()
        // means nothing. The box is behind the bar once a plain line (or nothing) is seen for
        // CONFIRM_TICKS after at least 5 cm.
        driveStraight(LEAVE_SPEED);
        dbPlain.feed(!v.bar && v.count <= PLAIN_MAX, legM);
        if (legM > 0.05f && dbPlain.held(CONFIRM_TICKS))
        {
            legM = 0.0f;
            enter(St::FIND_LINE, "box exit");
            return;
        }
        if (legM > 0.45f || now - tState > LEAVE_MAX_MS) fault("still in START box");
        return;
    }
    // -----------------------------------------------------------------
    case St::FIND_LINE:
    {
        dbPlain.feed(v.plain, legM);
        if (dbPlain.held(CONFIRM_TICKS))
        {
            normLo = (float)v.lo;         // first sight of the plain line: where it sits on the array
            normHi = (float)v.hi;
            const float out = legM;       // the first leg is measured from the box edge
            startLeg(0);
            legM = out;
            return;
        }
        driveStraight(LEAVE_SPEED);
        if (legM > 0.06f || now - tState > 1500) startSearch("no line at box exit");
        return;
    }
    // -----------------------------------------------------------------
    case St::FOLLOW:
    {
        const Leg &L = leg();
        if (now - tLeg > LEG_MAX_MS) { fault("leg took too long"); return; }
        const bool inWindow = legM >= L.winMin && legM <= L.winMax;

        dbLeft.feed(v.leftEdge, legM);
        dbRight.feed(v.rightEdge, legM);
        dbBar.feed(v.bar, legM);
        dbLost.feed(!v.on, legM);
        if (eventSignature(L.ev)) { if (sigLastM < 0.0f || legM - sigLastM > 0.02f) sigStartM = legM; sigLastM = legM; }

        // --- the expected event, only inside its window -------------------------------
        if (inWindow)
        {
            const int need = (L.ev == Ev::PARK_BOX) ? BOX_TICKS : CONFIRM_TICKS;
            Debounce *d = nullptr;
            switch (L.ev)
            {
            case Ev::CORNER_L:   d = dbLeft.held(need) ? &dbLeft : (dbBar.held(need) ? &dbBar : nullptr); break;
            case Ev::BRANCH_R:   d = dbRight.held(need) ? &dbRight : nullptr; break;
            case Ev::BRANCH_L:   d = dbLeft.held(need) ? &dbLeft : nullptr; break;
            case Ev::END_BAR:    d = dbBar.held(need) ? &dbBar : nullptr; break;
            case Ev::T_JUNCTION: d = dbBar.held(need) ? &dbBar : ((dbLeft.held(need) && dbRight.held(need)) ? &dbLeft : nullptr); break;
            case Ev::PARK_BOX:   d = dbBar.held(need) ? &dbBar : nullptr; break;
            }
            if (d) { why = "-"; eventReached(d->startM + LINE_HALF_M); return; }
            // The line ended right after this leg's bar / edge was seen: that IS the event (the
            // confirmation was cut short by noise). A plain dropout without it is NOT.
            if (lostMeansEvent(L.ev) && dbLost.held(LOST_TICKS) && sigLastM >= 0.0f && dbLost.startM - sigLastM < SIG_RECENT_M)
            { eventReached(sigStartM + LINE_HALF_M); return; }
        }

        // --- past the window without the event: go back and look for it ----------------
        if (legM > L.winMax)
        {
            backFromM = legM;
            halt();
            afterSettle = St::BACKTRACK;
            enter(St::SETTLE, "missed: backtrack");
            return;
        }

        // --- the line is gone (not an expected event) -----------------------------------
        if (!v.on)
        {
            if (legM - dbLost.startM > DROPOUT_M || dbLost.n > 60)
            {
                // Still gone after the dropout creep. Inside a window where the line really ends
                // (corner, T, end bar) that is the event, even if noise hid its bar / edge:
                // the tape was where the line was last seen, so the robot rolls BACK to pivot.
                if (inWindow && lostMeansEvent(L.ev)) { note = "line end (no pattern)"; eventReached(dbLost.startM - LINE_HALF_M); return; }
                startSearch("line lost");
                return;
            }
            // creep straight a short way: a dropout usually comes back within a few cm
            driveStraight(NEAR_SPEED);
            return;
        }

        // --- a junction that is not ours (or not yet): cross it straight -----------------
        if (v.leftEdge || v.rightEdge || v.bar)
        {
            if (crossUntilM < 0.0f) crossUntilM = legM + CROSS_MAX_M;
        }
        if (crossUntilM >= 0.0f)
        {
            if (legM < crossUntilM && !v.plain) { driveStraight(NEAR_SPEED); lastSteer = 0.0f; return; }
            crossUntilM = -1.0f;
            resetSteering();
        }

        // --- ordinary following ------------------------------------------------------------
        if (v.plain && crossUntilM < 0.0f)            // learn where the plain line sits (even if off-centre)
        {
            normLo += 0.08f * ((float)v.lo - normLo);
            normHi += 0.08f * ((float)v.hi - normHi);
        }
        float speed = L.onBranch ? BRANCH_SPEED : FOLLOW_SPEED;
        if (legM < 0.10f || legM > L.winMin - 0.15f) speed = (speed < NEAR_SPEED) ? speed : NEAR_SPEED;
        followLine(speed);
        return;
    }
    // -----------------------------------------------------------------
    case St::BACKTRACK:
    {
        // Reverse straight (no steering) looking for the event's tape. While reversing the bar
        // meets the FAR edge of that tape first: its middle is half a tape behind.
        const Leg &L = leg();
        if (eventSignature(L.ev))
        {
            dbBar.feed(true, legM);
            if (dbBar.held(CONFIRM_TICKS))
            {
                halt();
                note = "found on backtrack";
                eventReached(dbBar.startM - LINE_HALF_M);
                return;
            }
        }
        else dbBar.clear();
        driveStraight(-ROLL_SPEED);
        if (backFromM - legM > BACKTRACK_M || now - tState > 4000) fault("missed event (not found backing up)");
        return;
    }
    // -----------------------------------------------------------------
    case St::ROLL:
    {
        if ((rollTarget - legM) * rollDir <= 0.0f)
        {
            coastMeasure = !rollTrim && rollDir > 0;
            rollStopM = legM;
            settleThen(St::TURN);
            return;
        }
        driveStraight(rollDir * (rollTrim ? TRIM_SPEED : ROLL_SPEED));
        if (now - tState > ROLL_MAX_MS) fault("roll timeout");
        return;
    }
    // -----------------------------------------------------------------
    case St::SETTLE:
    {
        halt();
        if (now - tLastMove < SETTLE_STILL_MS && now - tState < SETTLE_MAX_MS) return;
        if (now - tState < SETTLE_MIN_MS) return;
        const St next = afterSettle;
        if (coastMeasure)
        {
            coastMeasure = false;
            float c = legM - rollStopM;
            if (c < 0.0f) c = 0.0f;
            if (c > COAST_MAX_M) c = COAST_MAX_M;
            coastM = 0.5f * coastM + 0.5f * c;
        }
        if (next == St::TURN)
        {
            // Coasted too far (or stopped short)? One slow correction move, then turn.
            if (turnPending && leg().ev != Ev::END_BAR && !rollTrim && fabsf(legM - pivotM) > PIVOT_TOL_M)
            {
                rollTrim = true;
                rollTarget = pivotM - ((pivotM > legM) ? 1.0f : -1.0f) * coastM * (TRIM_SPEED / ROLL_SPEED);
                rollDir = (pivotM > legM) ? 1 : -1;
                enter(St::ROLL, "pivot correction");
                return;
            }
            if (turnPending) startTurn(leg().turnDir, leg().turnDeg);
            else fault("turn without a plan");
            return;
        }
        if (next == St::SEARCH) { startSpin(); searchPhase = 0; enter(St::SEARCH); return; }
        if (next == St::ALIGN)  { startSpin(); alignDir = 0; enter(St::ALIGN); return; }
        if (next == St::TURN_BACK) { enter(St::TURN_BACK, "turn back"); return; }
        if (next == St::BACKTRACK) { dbBar.clear(); enter(St::BACKTRACK); return; }
        if (next == St::VERIFY) { verifyGood = 0; enter(St::VERIFY); return; }
        if (next == St::DONE)   { why = "-"; enter(St::DONE, "PARKED"); userLed.setSolid(0, 255, 0); return; }   // green
        fault("settle: no next state");
        return;
    }
    // -----------------------------------------------------------------
    case St::TURN:
    {
        if (now - tState > TURN_MAX_MS) { fault("turn timeout"); return; }
        if (gyroOk)
        {
            const float a = turned();
            if ((a >= turnLo && lineAcquired()) || (a >= turnTarget && v.plain))   // off-centre: ALIGN centres it
            { turnAcc = spinAcc; settleThen(St::ALIGN); return; }
            if (a > turnHi) { afterSettle = St::TURN_BACK; halt(); enter(St::SETTLE, "no line: turn back"); return; }
            spin(turnDir, (a < turnTarget - SLOW_DEG) ? TURN_FAST : TURN_SLOW);
            return;
        }
        // Gyro not ready: tape only. First the array must leave the line it starts on (the main
        // line ahead at a branch, the end bar at a U-turn); the NEXT line it meets is the target.
        if (v.on) tOffLine = now;
        if (!leftStartLine && now - tOffLine >= TAPE_OFF_MS) leftStartLine = true;
        if (leftStartLine && v.plain) { turnAcc = 0.0f; settleThen(St::ALIGN); return; }
        if (now - tState > TURN_MAX_MS - 1000) { fault("tape turn: no line"); return; }
        spin(turnDir, TURN_SLOW);
        return;
    }
    // -----------------------------------------------------------------
    case St::TURN_BACK:
    {
        // One slow pass back through the acceptance window. Gyro only (TURN without a gyro
        // never comes here).
        const float a = turned();
        if (a >= turnLo && a <= turnHi && v.plain) { turnAcc = spinAcc; settleThen(St::ALIGN); return; }
        if (a < turnLo - 5.0f || now - tState > 4000) { fault("no line after turn"); return; }
        spin(-turnDir, TURN_SLOW);
        return;
    }
    // -----------------------------------------------------------------
    case St::ALIGN:
    {
        // Robot is still. Rotate slowly toward the line until it is near the centre.
        if (now - tState > ALIGN_MAX_MS || (gyroOk && turned() > ALIGN_MAX_DEG))
        {
            if (v.on && abs(v.err) <= ACQUIRE_ERR) { settleThen(St::VERIFY); return; }
            fault("align: line not centred");
            return;
        }
        if (v.on && abs(v.err) <= (alignDir == 0 ? ALIGN_OK : ALIGN_DONE)) { settleThen(St::VERIFY); return; }
        int want;
        if (v.on) want = (v.err > 0) ? STEER_SIGN : -STEER_SIGN;            // +1 = turn right
        else      want = (line.lastSeenSide() != 0) ? STEER_SIGN * line.lastSeenSide() : 0;
        if (want == 0) { fault("align: no line"); return; }
        if (alignDir != 0 && want != alignDir) { settleThen(St::VERIFY); return; }   // crossed the centre
        alignDir = want;
        spin(alignDir, TURN_SLOW);
        return;
    }
    // -----------------------------------------------------------------
    case St::VERIFY:
    {
        halt();
        if (v.on && !v.bar && abs(v.err) <= ACQUIRE_ERR) verifyGood++;
        if (verifyGood >= VERIFY_TICKS)
        {
            if (turnPending)
            {
                // A mission turn is complete. Book the new planned heading and check it.
                turnPending = false;
                planHeading = wrap180(planHeading + leg().turnDir * leg().turnDeg);
                if (gyroSign == 0 && leg().turnDeg < 135.0f && fabsf(turnAcc) > 45.0f)
                    gyroSign = ((turnAcc > 0.0f) ? 1 : -1) * leg().turnDir;
                if (NRLComms::isHeadingReady() && fabsf(gridError()) > GRID_FAULT_DEG)
                { fault("wrong heading after turn"); return; }
                if (legIdx + 1 >= LEG_COUNT) { fault("plan ended without PARK"); return; }
                startLeg(legIdx + 1);
            }
            else
            {
                // back on the line after a search: continue the same leg
                dbLost.clear();
                resetSteering();
                enter(St::FOLLOW, leg().name);
            }
            return;
        }
        if (now - tState > VERIFY_MAX_MS) startSearch("verify failed");
        return;
    }
    // -----------------------------------------------------------------
    case St::SEARCH:
    {
        // Phase 0: toward the side the line was last seen, up to SEARCH_DEG.
        // Phase 1: back across to SEARCH_DEG on the other side. Then give up.
        if (v.on && !v.bar && abs(v.err) <= ACQUIRE_ERR) { settleThen(St::ALIGN); return; }
        const float a = gyroOk ? turned() : 0.0f;
        const uint32_t el = now - tState;
        if (searchPhase == 0)
        {
            const bool doneSide = gyroOk ? (a >= SEARCH_DEG) : (el > SEARCH_SIDE_MS);
            if (doneSide) { searchPhase = 1; startSpin(); tState = now; return; }
            spin(searchDir, TURN_SLOW);
            return;
        }
        const bool doneSide = gyroOk ? (a >= 2.0f * SEARCH_DEG) : (el > 2 * SEARCH_SIDE_MS);
        if (doneSide || el > 6000) { fault("search: line not found"); return; }
        spin(-searchDir, TURN_SLOW);
        return;
    }
    // -----------------------------------------------------------------
    case St::PARK_IN:
    {
        // Inside the filled box: straight, no steering, a measured distance.
        driveStraight(ROLL_SPEED);
        if (legM - parkFromM >= PARK_IN_M - coastM) { settleThen(St::DONE); return; }
        if (now - tState > PARK_MAX_MS) fault("park-in timeout");
        return;
    }
    // -----------------------------------------------------------------
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
    case St::BACKTRACK: return "BACKTRACK";
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
class P1D1Clean : public NRLOpMode
{
public:
    void init() override            // INIT does not move anything (R8.11)
    {
        leftMotor.begin();
        rightMotor.begin();
        line.begin();
        pinMode(LEFT_ENC_PIN, INPUT_PULLUP);
        pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), leftIsr, RISING);
        attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightIsr, RISING);
    }

    void start() override
    {
        // Everything from scratch on every run: nothing is carried over from an earlier run.
        const uint32_t now = millis();
        tRun = now;
        tLeg = now;
        coastSign = 0;
        turnAcc = 0.0f;
        coastM = ROLL_COAST_M;
        coastMeasure = false;
        sigLastM = -1.0f;
        normLo = NORM_LO_START; normHi = NORM_HI_START;
        userLed.setOff();
        lastEncL = encL; lastEncR = encR;
        tLastAnyCount = tLastL = tLastR = tLastMove = now;
        deadL = deadR = false;
        moveSign = 0;
        legIdx = 0;
        legM = 0.0f;
        turnPending = false;
        searchesThisLeg = 0;
        crossUntilM = -1.0f;
        planHeading = 0.0f;
        gyroSign = 0;
        freeStart = NRLComms::getHeadingFree();   // robot placed square in the START box
        why = "-";
        note = "-";
        dbLeft.clear(); dbRight.clear(); dbBar.clear(); dbLost.clear(); dbPlain.clear();
        resetSteering();
        drive.setRampRate(RAMP_RATE);
        if (!line.isCalibrated()) { fault("line array not calibrated"); return; }
        enter(St::LEAVE_BOX, "START");
    }

    void loop() override
    {
        tick();
        // 8 lines fit on the Hub screen.
        telemetry.addData("stage", leg().name);
        telemetry.addData("state", stateName(st));
        telemetry.addData("why", (st == St::FAULT) ? why : note);
        telemetry.addData("leg cm", legM * 100.0f);
        telemetry.addData("mask", maskText());
        telemetry.addData("err", (float)v.err);
        telemetry.addData("deg", turned());
        telemetry.addData("enc", deadL ? "L dead" : (deadR ? "R dead" : (NRLComms::isHeadingReady() ? "ok gyro ok" : "ok NO GYRO")));
    }

    void stop() override { drive.stop(); }
};

}  // namespace

// REGISTER_OPMODE(P1D1Clean, "P1-D1 Clean", AUTO);