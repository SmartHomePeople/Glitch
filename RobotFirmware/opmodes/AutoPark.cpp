// =====================================================================
//  GLITCHED (Team 190) - Start zone to Park zone
//  File: RobotFirmware/opmodes/AutoPark.cpp
//
//  Four AUTO OpModes:
//    "Park Run"        MAIN. Our own follower: encoders anticipate each corner, the tape
//                      confirms it, the gyro makes the turn. If anything does not add up,
//                      that corner is done by tape alone. (35 pts: leave + park)
//    "P1-D1 Run"       Park Run plus two detours, no arm or gripper: into P1 to its end bar,
//                      U-turn, back to the main line, into D1 to its end bar, U-turn, back,
//                      then the rest of the course and park. The steps are in P1D1_PLAN.
//    "Park Run Tape"   FALLBACK. The SDK's own tape-only follower, with softer steering.
//    "First Junction"  FALLBACK. Leave the box and stop just past the first junction.
//
//  Start the robot CENTRED in the START box, facing the line that leaves it, after a
//  LINE TEST calibration on this mat (white first, then black; "1 DEAD" for channel 7 is ok).
//
//  ---------------------------------------------------------------------------------
//  HOW EVERY TURN IS TAKEN (corners, branches, the T back onto the main line, U-turns)
//    1  FOLLOW     Gentle line following. Encoders predict the next event; the robot eases
//                  down to CREEP_SPEED over the last 30 cm before it.
//    2  DETECT     Only the tape triggers: line ends (corner), branch on the expected side seen
//                  3 ticks in a row while the robot is square to the grid (branch), wide bar
//                  (end bar / T). Encoders only decide WHERE to look. STOP fully.
//    3  VERIFY     Robot still: the tape must still agree, else it carries on following.
//    4  LOCATE     Creep to the exact tape edge: corner = back up until the line end is under
//                  the sensor bar; branch / T = back off the tape, creep forward onto its edge.
//    5  ADJUST     From that edge, move exactly PIVOT_FWD_M -/+ half a tape width, so the
//                  middle of the chassis sits on the junction. (End bar: back up 8 cm instead.)
//    6  TURN       Slow gyro turn (90 or 180 deg). STOP. Tape must show a line, else a slow
//                  search between target -25 and +25 deg.
//    7  SQUARE     Square the robot to the arena grid with the gyro (lines are all at
//                  0/90/180/270 deg), up to 3 slow passes with stops. If the line is under the
//                  array: drive; the follower removes any small sideways offset. If not: slow
//                  tape centring passes.
//    FALLBACKS     Any check that does not add up turns that one event by tape alone, with spin
//                  limits; nothing ever spins more than target + 30 deg; if lost it stops (FAULT).
//    "turns" stores every turn: G = gyro, S = gyro + search, T = tape. P = parked.
//
//  SCREEN (8 lines)
//    state   FOLLOW / STOP / VERIFY / BACKUP / ROLL / GYRO TURN / CHECK / CORRECT / SWEEP / CENTRE /
//            TAPE TURN /
//            PARK IN / DONE / FAULT
//    why     why the last fallback or fault happened ("-" while all is well)
//    leg     C1..C4 or BOX, and the expected distance in cm
//    d cm    distance travelled on this leg (encoders)
//    turns   corners done so far, e.g. "GGTG"
//    deg     degrees of the last turn (gyro)
//    enc     ok, or OFF if the encoders stopped counting (then distances are ignored)
//    t       seconds since start
//
//  STATUS: written against the NRL SDK v1.5.0 headers. NOT compiled or run on the robot yet.
// =====================================================================
#include "NRL.h"
#include <string.h>

// ---------------------------------------------------------------------
//  Hardware (same as the other autos: both motors reversed, 2026-11-19)
// ---------------------------------------------------------------------
static HexaDCMotor leftMotor{{
    .dirPin = MOTOR_L_DIR,
    .pwmPin = MOTOR_L_PWM,
    .minDrive = 34        // Min Drive test 2026-11-19: left max 31 (pivot) + 3
}};

static HexaDCMotor rightMotor{{
    .dirPin = MOTOR_R_DIR,
    .pwmPin = MOTOR_R_PWM,
    .flipped = true,
    .minDrive = 32        // Min Drive test 2026-11-19: right max 29 (pivot) + 3
}};

static TankDrive drive(leftMotor, rightMotor);
static HexaLineArray line{hexaLineConfig()};

// The SDK follower is used for leaving the box and for the two fallback OpModes.
// Its default steering (kP 0.50) was too aggressive, so it is softened here.
static LineTuning makeLineTuning()
{
    LineTuning t;
    t.pid = HexaPIDConfig{ .kP = 0.25f, .kD = 0.06f, .dAlpha = 0.5f };   // was kP 0.50, kD 0.04 (SDK default)
    t.rampRate = 0.12f;   // motors change more slowly per tick (SDK default 0.25): smaller, smoother corrections
    return t;
}
static LineActions lineActions(drive, line, makeLineTuning());

// ---------------------------------------------------------------------
//  Encoders (channel A only, same wiring as Encoder Accuracy)
// ---------------------------------------------------------------------
constexpr int      LEFT_ENC_PIN  = DIGITAL_D3;   // back-left:  board port D2 (GPIO37)
constexpr int      RIGHT_ENC_PIN = DIGITAL_D2;   // back-right: board port D1 (GPIO36)
constexpr float    CPM           = 1300.0f;      // counts per metre (Encoder Accuracy, 2026-11-19)
constexpr uint32_t GLITCH_US     = 50;

static volatile uint32_t encL = 0, encR = 0, lastUsL = 0, lastUsR = 0;

static void IRAM_ATTR leftIsr()
{
    const uint32_t now = micros();
    if (now - lastUsL > GLITCH_US) { encL = encL + 1; lastUsL = now; }
}

static void IRAM_ATTR rightIsr()
{
    const uint32_t now = micros();
    if (now - lastUsR > GLITCH_US) { encR = encR + 1; lastUsR = now; }
}

// ---------------------------------------------------------------------
//  Arena and robot. TUNE the three marked MEASURE values on the real arena.
// ---------------------------------------------------------------------
constexpr float SIDE_M      = 2.00f;   // manual: the main line is a 2000 x 2000 mm square
constexpr float FIRST_LEG_M = 0.28f;   // START box edge to corner C1 = 30 cm (measured 2026-11-19);
                                      // leaveBox() stops a little past the edge, so 28 cm. Was a 50 cm guess,
                                      // which made C1 look "too early": it was treated as a wobble and searched.
constexpr float LAST_LEG_M  = 0.30f;   // corner C4 to the edge of the PARK box = 30 cm (measured 2026-11-19)
constexpr float PIVOT_FWD_M = 0.08f;   // sensor bar back to the middle of the chassis = 8 cm (measured 2026-11-19)
constexpr float TURN_ROLL_M = 0.08f;   /* FALLBACK only (used if LOCATE cannot find the line end) */   // TUNE: from where the line ended to where it turns. Now measured from the
                                      // detection point INCLUDING the coast (was 0.09 + coast), so +2 cm
                                      // (was 0.10, then 0.06; 0.09 because it turned too early). Bigger = later turn.
constexpr float WINDOW_M    = 0.20f;   // a corner is "expected" within +/- this of the predicted distance
constexpr float SLOW_M      = 0.30f;   // start slowing this far before the predicted corner
constexpr float PARK_IN_M   = 0.25f + PIVOT_FWD_M;   // array past the box edge so the chassis sits mid-box

// ---------------------------------------------------------------------
//  Driving. Speeds are TankDrive units (-1..1).
// ---------------------------------------------------------------------
constexpr float FOLLOW_SPEED   = 0.35f;
constexpr float APPROACH_SPEED = 0.22f;   // first 15 cm of a step, crossing into a box
constexpr float CREEP_SPEED    = 0.18f;   // speed reached at the predicted event: the robot slows down
                                          // gradually from FOLLOW_SPEED over the last SLOW_M
constexpr float JUNCTION_SPEED = 0.25f;   // straight across a branch, no steering
constexpr float ROLL_SPEED     = 0.22f;
constexpr float TURN_FAST      = 0.20f;   // was 0.40 / 0.32 / 0.30 / 0.24: slow and steady instead of corrections
constexpr float TURN_SLOW      = 0.17f;   // last part of a turn, and tape turns (was 0.20). If the robot stalls
                                          // mid-turn (stiff left wheel), raise this back toward 0.25
constexpr float CORR_SPEED     = 0.17f;   // search and centring speed (was 0.20)
constexpr int   MAX_CORRECTIONS = 0;      // gyro corrections after a turn: 0 = none (they looked jittery)
constexpr float SPEED_DROP     = 0.50f;   // slow down as the line moves off centre
constexpr float RAMP_RATE      = 0.10f;   // how fast motor commands may change per tick (SDK uses 0.25)
constexpr float MAX_STEER_STEP = 0.025f;  // the steering may change at most this much per tick, so a
                                          // correction grows gradually instead of jerking

// Steering: a gentle PD on the line error (-1..+1). TUNE: if it still zig-zags lower kP;
// if it runs wide off the line raise it. +1 when channel 0 is the LEFT-most sensor.
constexpr int   STEER_SIGN = 1;      // confirmed 2026-11-19: channel 0 is on the LEFT (dead channel 7 is right-most)
static HexaPID steer{ HexaPIDConfig{ .kP = 0.22f, .kD = 0.08f, .outMin = -0.25f, .outMax = 0.25f, .dAlpha = 0.5f } };
static float   lastSteer = 0.0f;   // the steering used last tick (for MAX_STEER_STEP)

// Turns
constexpr float TURN_DEG       = 90.0f;
constexpr float STOP_EARLY_DEG = 8.0f;    // cut the motors this early; the robot coasts the rest (was 6)
constexpr float TURN_TOL_DEG   = 4.0f;    // after stopping, 86..94 deg is accepted; outside that it corrects by gyro
constexpr float SWEEP_DEG      = 25.0f;   // no line after the turn: search only 65..115 deg, never further
constexpr float MAX_SPIN_DEG   = 120.0f;  // a tape turn never spins more than this (stops the 180 deg reversal)
constexpr float WOBBLE_DEG     = 60.0f;   // an early line-end is searched this far toward the last-seen side
constexpr int   CENTRE_ERR     = 250;     // tape centring target when the gyro grid is not available
constexpr int   LATERAL_MAX    = 1800;    // after squaring: line further off-centre than this -> twist toward it
constexpr int   LATERAL_OK     = 800;     //   ...until it is within this (a small angle; the follower does the rest)
constexpr float FLICKER_M      = 0.04f;   // after a 'line back' check, a lost line within this is a wobble
constexpr int   CENTRE_PASSES  = 3;       // slow centring passes, each ending in a full stop
constexpr float LEG_START_M    = 0.15f;   // drive slowly for the first part of every leg
constexpr float NEAR_CORNER_M  = 0.40f;   // a line lost this soon after a corner is searched both ways, never turned as a corner
constexpr float SLOW_DEG       = 45.0f;   // slow down for the last 45 degrees (was 35)
constexpr int   TAPE_ACCEPT_ERR = 1200;   // a tape turn stops once the line is this close to centre;
                                          // the gentle follower straightens the rest
constexpr int   CORNER_DIR     = -1;      // every corner on the way to PARK is a LEFT turn

// Timings
constexpr int           LOST_TICKS     = 3;      // line gone this many ticks in a row = line ended
constexpr unsigned long BOX_CONFIRM_MS = 150;    // TERMINAL (7 of 8 dark) this long = in the box
constexpr unsigned long CROSS_MAX_MS   = 600;    // longest straight crossing of a branch
constexpr unsigned long ROLL_MS        = 150;    // roll when encoders are OFF (was 250)
constexpr unsigned long PARK_MS        = 700;    // park drive when encoders are OFF
constexpr long          TAPE_PARK_MS   = 400;    // "Park Run Tape": SDK park() time into the box (about 250 mm)
constexpr unsigned long SETTLE_MS      = 250;    // still (no encoder counts) this long = fully stopped
constexpr unsigned long SETTLE_MAX_MS  = 800;
constexpr unsigned long CENTRE_MAX_MS  = 1500;
constexpr unsigned long GYRO_TURN_MS   = 4500;   // a gyro turn longer than this does not add up (slower turns now)
constexpr unsigned long TAPE_MIN_MS    = 200;    // spin at least this long before accepting a corner line
constexpr unsigned long TAPE_MAX_MS    = 2500;   // time limit when the gyro is not ready
constexpr unsigned long ENC_STALL_MS   = 500;    // no counts while driving this long = encoders OFF
constexpr unsigned long SOFT_STOP_MS   = 145000;

// ---------------------------------------------------------------------
//  Branches and terminals (manual: terminals are 500 mm long, P1 is 500 mm past C1,
//  D1 is 500 mm past P1, C2 is 1000 mm past D1)
// ---------------------------------------------------------------------
constexpr float TERM_M        = 0.50f;   // terminal length, main line to end bar (confirmed on the arena)
constexpr float GAP_M         = 0.50f;   // C1 -> P1 and P1 -> D1
constexpr float BRANCH_ROLL_M = PIVOT_FWD_M + 0.05f;   /* from the detection point incl. coast (was +0.03 + coast) */   // TUNE (+3 cm: turned too early): roll after seeing a branch / the main line, so the
                                               // middle of the chassis is on the junction before turning
constexpr float MIN_EVENT_M   = 0.05f;   // ignore events in the first 5 cm of a step (still leaving a junction)
constexpr int   WIDE_COUNT    = 5;       // this many dark channels = a bar across the line
constexpr int   CONFIRM_TICKS = 3;       // a branch must be seen this many ticks in a row
constexpr float BACKUP_MAX_M  = 0.08f;   // VERIFY: back up at most this far to find the branch again
constexpr unsigned long UTURN_MIN_MS = 1500;  // a tape U-turn spins at least this long before accepting a line
                                              // (was 450: it could grab the end bar itself half-way round)
constexpr float UTURN_BACK_M  = 0.08f;   // TUNE: drive back this far from the end bar before the 180 deg turn
constexpr unsigned long UTURN_BACK_MS = 300;   // the same, by time, when the encoders are OFF
constexpr int   BAR_MIN_DARK  = 6;       // a bar across the array: at least this many dark channels (7 work),
                                         // and at least 3 more than the plain line (was 4: the plain line itself
                                         // could read 4, so it kept "finding" the bar and stopping)

// LOCATE: after stopping, find the exact tape edge by creeping, then move by a measured amount.
constexpr float LINE_HALF_M   = 0.0125f; // half the 25 mm tape width
constexpr float LOCATE_SPEED  = 0.16f;   // creep speed while looking for the tape edge
constexpr float LOCATE_MAX_M  = 0.15f;   // give up looking for the edge after this far (then use the old roll)
constexpr float CORNER_TRIM_M = 0.0f;    // TUNE: + = turn later at corners,  - = earlier (after PIVOT_FWD_M is measured)
constexpr float BRANCH_TRIM_M = 0.0f;    // TUNE: + = turn later at branches / T, - = earlier
constexpr float RIGHT_TURN_TRIM_M = -0.015f;  // right turns at a branch / T pivot 1.5 cm early, so the new line
                                              // lands LEFT of centre: channel 7 (right-most) is dead, and a line
                                              // on the right edge kept dropping out (the P1 jerking)
constexpr float SENSOR_PITCH_M = 0.010f; // MEASURE: distance between two neighbouring IR sensors on the bar
constexpr float LEARN_GAIN    = 0.5f;    // how much of each turn's landing error is corrected next time
constexpr float LEARN_MAX_M   = 0.04f;   // learned turn-position correction limit (+/-)
constexpr float CROSS_ZONE_M  = 0.10f;   // within this of a known junction: drive straight on the gyro
constexpr float HOLD_KP       = 0.010f;  // gyro heading hold while crossing: steering per degree off square
constexpr float HOLD_MAX      = 0.12f;
constexpr float TERMINAL_SPEED = 0.20f;  // top speed on a terminal (cores sit just past the end bar)
constexpr float BAR_CREEP_SPEED = 0.14f; // last BAR_CREEP_M before an end bar
constexpr float BAR_CREEP_M    = 0.15f;
constexpr float OFF_COURSE_DEG = 30.0f;  // a 'corner' while this far off square = followed a branch by mistake
constexpr unsigned long SOFT_LOST_MS = 500;   // line lost where no end is expected: creep and steer toward it
constexpr float SOFT_TURN     = 0.12f;        //   for up to this long, without stopping

// GRID: the arena lines are all at 0/90/180/270 deg. After every turn the robot squares itself
// to the grid with the gyro (not by twisting toward the tape, which left it crooked).
constexpr float GRID_TOL_DEG     = 2.0f;   // square to the grid within this
constexpr int   GRID_PASSES      = 3;      // gyro squaring passes, each ending in a full stop
constexpr float BRANCH_MAX_ANGLE = 12.0f;  // a branch only counts while the robot is this square to the grid

// ---------------------------------------------------------------------
//  The plan: a list of steps. Each step follows the line until one EVENT, then turns.
// ---------------------------------------------------------------------
enum class Ev : uint8_t { CORNER, BRANCH_R, BRANCH_L, END_BAR, MAIN_LINE, BOX };

struct Step
{
    const char *name;    // shown on the screen
    Ev    ev;            // what ends the step
    float expM;          // expected encoder distance from the start of the step to the event
    int   turnDir;       // then turn: -1 left, +1 right
    float turnDeg;       // 90, or 180 for a U-turn (0 for BOX)
    float cross[3];      // junctions to CROSS on the way (distance from the step start), 0 = none
};

// Junctions along a side are 500 / 1000 / 1500 mm from its corner; the sensor bar starts a leg
// PIVOT_FWD_M past the corner.
#define SIDE_CROSS { 0.5f - PIVOT_FWD_M, 1.0f - PIVOT_FWD_M, 1.5f - PIVOT_FWD_M }
#define NO_CROSS   { 0.0f, 0.0f, 0.0f }

// After every turn the sensor bar is PIVOT_FWD_M past the junction / corner it turned on.
// After a U-turn on an end bar it is about 2 x PIVOT_FWD_M back from the bar.
static const Step PARK_PLAN[] = {
    {"C1",      Ev::CORNER,    FIRST_LEG_M,                 -1,  90.0f, NO_CROSS},
    {"C2",      Ev::CORNER,    SIDE_M - PIVOT_FWD_M,        -1,  90.0f, SIDE_CROSS},   // P1 D1 P2
    {"C3",      Ev::CORNER,    SIDE_M - PIVOT_FWD_M,        -1,  90.0f, SIDE_CROSS},   // P3 D2 P4
    {"C4",      Ev::CORNER,    SIDE_M - PIVOT_FWD_M,        -1,  90.0f, SIDE_CROSS},   // P5 D3 P6
    {"BOX",     Ev::BOX,       LAST_LEG_M - PIVOT_FWD_M,     0,   0.0f, NO_CROSS},
};

static const Step P1D1_PLAN[] = {
    {"C1",      Ev::CORNER,    FIRST_LEG_M,                 -1,  90.0f, NO_CROSS},
    {"to P1",   Ev::BRANCH_R,  GAP_M - PIVOT_FWD_M,         +1,  90.0f, NO_CROSS},   // turn right into P1
    {"P1 end",  Ev::END_BAR,   TERM_M - PIVOT_FWD_M,        -1, 180.0f, NO_CROSS},   // U-turn on the end bar
    {"P1 back", Ev::MAIN_LINE, TERM_M - 2.0f * PIVOT_FWD_M - UTURN_BACK_M, +1, 90.0f, NO_CROSS},   // right = north again
    {"to D1",   Ev::BRANCH_L,  GAP_M - PIVOT_FWD_M,         -1,  90.0f, NO_CROSS},   // turn left into D1
    {"D1 end",  Ev::END_BAR,   TERM_M - PIVOT_FWD_M,        -1, 180.0f, NO_CROSS},
    {"D1 back", Ev::MAIN_LINE, TERM_M - 2.0f * PIVOT_FWD_M - UTURN_BACK_M, -1, 90.0f, NO_CROSS},   // left = north again
    {"C2",      Ev::CORNER,    2.0f * GAP_M - PIVOT_FWD_M,  -1,  90.0f, {GAP_M - PIVOT_FWD_M, 0.0f, 0.0f}},   // cross P2
    {"C3",      Ev::CORNER,    SIDE_M - PIVOT_FWD_M,        -1,  90.0f, SIDE_CROSS},   // cross P3 D2 P4
    {"C4",      Ev::CORNER,    SIDE_M - PIVOT_FWD_M,        -1,  90.0f, SIDE_CROSS},   // cross P5 D3 P6
    {"BOX",     Ev::BOX,       LAST_LEG_M - PIVOT_FWD_M,     0,   0.0f, NO_CROSS},
};

static const Step *plan     = PARK_PLAN;
static int         planLen  = sizeof(PARK_PLAN) / sizeof(PARK_PLAN[0]);
static int         stepIdx  = 0;

static const Step &step() { return plan[stepIdx < planLen ? stepIdx : planLen - 1]; }

// ---------------------------------------------------------------------
//  Run state machine (shared by "Park Run" and "P1-D1 Run")
// ---------------------------------------------------------------------
enum class St { FOLLOW, STOP, VERIFY, BACKUP, LOCATE, ROLL, GYRO_TURN, CHECK, CORRECT, SWEEP, GRID_ALIGN, GRID_CHECK, CENTRE, CENTRE_CHECK, TAPE_TURN, PARK_IN, DONE, FAULT };

static St          st        = St::FOLLOW;
static const char *why       = "-";
static char        turnsLog[16] = "";
static float       lastDeg   = 0.0f;
static bool        encOk     = true;
static uint32_t    legL = 0, legR = 0;           // encoder counts at the start of the step
static uint32_t    seenL = 0, seenR = 0;         // for the stall check
static unsigned long tState = 0, tLastCount = 0, tCross = 0, tBox = 0;
static int         lostTicks = 0;
static unsigned long tLost = 0;
static bool        anchored[3] = {false, false, false};   // junctions of this leg already used to re-anchor
static int         jxSeen = 0;                           // junctions on this leg seen by the tape
static float       learnCorner = 0.0f;                   // learned turn-position corrections (metres)
static float       learnBranch[2] = {0.0f, 0.0f};        //   [0] = left turn at a branch / T, [1] = right
static bool        lastLocated = false;                  // this turn's position came from LOCATE
static bool        lastLocCorner = false;
static int         lastLocDir = 0;
static int         lastTurnErr = 0;                      // line error right after squaring (screen)
static int         branchTicks = 0;
static bool        crossing  = false;
static char        mode      = 'G';              // G gyro turn, T tape turn, W wobble check, V search back
static int         spinDir   = CORNER_DIR;
static float       targetDeg = TURN_DEG;
static float       rollFrom  = 0.0f;
static uint32_t    detL = 0, detR = 0;            // encoder counts where the event was detected
static uint32_t    adjL = 0, adjR = 0;            // encoder counts when the ADJUST move started
static float       adjM = 0.0f;                   // ADJUST distance (metres, always positive)
static int         adjDir = 1;                    // ADJUST direction: +1 forward, -1 backward
static int         locPhase = 0;                  // LOCATE: -1 find the tape, 0 back off it, 1 creep onto its edge
static bool        locCorner = false;             // LOCATE for a corner (line end) instead of a branch / T
static uint32_t    locL = 0, locR = 0;
static float       freeStart = 0.0f;              // free gyro heading at the start (robot square in the box)
static float       gridTurn  = 0.0f;              // sum of all turns made so far (+ right), in degrees
static int         freeSign  = 0;                 // +1 / -1 once learned: how the free heading moves on a right turn
static float       freeBefore = 0.0f;             // free heading when the current turn started
static int         alignSign = 1;
static int         gridPasses = 0;
static float       alignStartErr = 0.0f;
static float       rollM     = 0.0f;
static uint32_t    runStart  = 0;
static St          afterStop = St::FOLLOW;       // where STOP goes once the robot is still
static uint32_t    stopL = 0, stopR = 0;
static unsigned long tStill = 0;
static int         corrections = 0;
static int         sweepPhase  = 0;
static int         centreDir   = 1;
static bool        centreFlipped = false;
static int         centrePasses  = 0;
static int         centreTarget  = CENTRE_ERR;
static float       flickerUntilM = 0.0f;
static uint32_t    backL = 0, backR = 0;          // encoder counts when BACKUP started

// Turn angle, unwrapped so a 180 deg U-turn is measured correctly.
static float accDeg = 0.0f, prevHeading = 0.0f;

static void zeroTurn()
{
    NRLComms::zeroHeading();
    prevHeading = NRLComms::getHeading();
    accDeg = 0.0f;
}

static void updateTurn()
{
    const float h = NRLComms::getHeading();
    float d = h - prevHeading;
    if (d > 180.0f)  d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    accDeg += d;
    prevHeading = h;
}

static float turned() { return fabsf(accDeg); }

static float wrap180(float a)
{
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// How far the robot is from square to the grid line it should be on (deg). Only valid once
// freeSign is known (learned on the first turn).
static float gridErr()
{
    return wrap180((NRLComms::getHeadingFree() - freeStart) - freeSign * gridTurn);
}

static float legM()
{
    return 0.5f * ((float)(encL - legL) + (float)(encR - legR)) / CPM;
}

static void startLeg()
{
    legL = encL;
    legR = encR;
    lostTicks = 0;
    branchTicks = 0;
    crossing = false;
    tBox = 0;
    flickerUntilM = 0.0f;
    steer.reset();
    lastSteer = 0.0f;
    anchored[0] = anchored[1] = anchored[2] = false;
    jxSeen = 0;
}

static void enter(St s)
{
    st = s;
    tState = millis();
}

static void fault(const char *reason)
{
    why = reason;
    drive.stop();
    enter(St::FAULT);
}

static void logTurn(char c)
{
    const size_t n = strlen(turnsLog);
    if (n < sizeof(turnsLog) - 1) { turnsLog[n] = c; turnsLog[n + 1] = 0; }
}

static void stopThen(St next);

// The step's turn is finished: store it and start the next step.
static void stepDone(char c)
{
    drive.stop();
    lastDeg = NRLComms::isHeadingReady() ? turned() : -1.0f;
    logTurn(c);
    // Learn which way the free heading moves (first turn only), then record this turn.
    const float dFree = wrap180(NRLComms::getHeadingFree() - freeBefore);
    if (freeSign == 0 && fabsf(dFree) > 45.0f && step().turnDir != 0 && step().turnDeg < 135.0f)   // not from a U-turn (+/-180 is ambiguous)
        freeSign = ((dFree > 0.0f) ? 1 : -1) * step().turnDir;
    gridTurn += step().turnDir * step().turnDeg;
    stepIdx++;
    // Square up to the grid with the gyro, then (if needed) find the tape. Full stops throughout.
    gridPasses = 0;
    centrePasses = 0;
    centreTarget = CENTRE_ERR;
    if (freeSign != 0 && NRLComms::isHeadingReady()) stopThen(St::GRID_CHECK);
    else                                             stopThen(St::CENTRE_CHECK);
}

// Stop fully, wait until the robot is still, then go to `next`.
static void stopThen(St next)
{
    drive.stop();
    afterStop = next;
    stopL = encL;
    stopR = encR;
    tStill = millis();
    enter(St::STOP);
}

// Pure tape turn from a full stop. 'T' = the step's turn, 'W' = wobble (toward where the
// line was last seen), 'V' = search back the other way.
static void startTapeTurn(const char *reason, char m)
{
    if (reason) why = reason;
    mode = m;
    if (m == 'W')
    {
        const int side = line.lastSeenSide();           // -1 left, +1 right
        spinDir = (side != 0) ? STEER_SIGN * side : CORNER_DIR;
        targetDeg = WOBBLE_DEG;
    }
    else if (m == 'V')
    {
        spinDir = -spinDir;
        targetDeg = 2.0f * WOBBLE_DEG;
    }
    else
    {
        spinDir = step().turnDir;
        targetDeg = step().turnDeg;
    }
    stopThen(St::TAPE_TURN);
}

// The event that ends this step has been seen. Turn by gyro if everything agrees, else by tape.
static void rememberMask();

static void eventSeen(bool inWindow, float roll)
{
    spinDir   = step().turnDir;
    targetDeg = step().turnDeg;
    rollM     = roll;               // where to put the robot, measured from the detection point
    lastLocated = false;
    detL      = encL;               // (+ forward, - backward)
    detR      = encR;
    rememberMask();
    if ((inWindow || step().turnDeg > 135.0f) && NRLComms::isHeadingReady())
    {
        mode = 'G';
    }
    else
    {
        why = !encOk ? "enc off" : (!NRLComms::isHeadingReady() ? "gyro not ready" : "off window");
        mode = 'T';
    }
    stopThen(St::VERIFY);       // nothing turns until the tape confirms it with the robot stopped
}

static void verified();
static bool branchOn(int side);
static bool barSeen();

static bool locPred()
{
    const Step &s = step();
    if (locCorner) return line.onLine();
    if (s.ev == Ev::BRANCH_R) return branchOn(+1);
    if (s.ev == Ev::BRANCH_L) return branchOn(-1);
    return barSeen();                                                                // the T / main line
}

// Creep to the exact tape edge before the final move. Corner: back up until the line end is
// under the array. Branch / T: back off the tape, then creep forward onto its near edge.
static void startLocate(bool corner)
{
    locCorner = corner;
    locPhase  = corner ? 0 : (locPred() ? 0 : -1);
    locL = encL;
    locR = encR;
    enter(St::LOCATE);
}

// The tape edge is under the sensor bar: from here the move is a measured distance.
static void locateDone()
{
    drive.stop();
    detL = encL;
    detR = encR;
    const int dir = step().turnDir;
    rollM = locCorner ? (PIVOT_FWD_M - LINE_HALF_M + CORNER_TRIM_M + learnCorner)   // just past the corner's far edge
                      : (PIVOT_FWD_M + LINE_HALF_M + BRANCH_TRIM_M                  // on the branch's near edge
                         + (dir > 0 ? RIGHT_TURN_TRIM_M : 0.0f) + learnBranch[dir > 0 ? 1 : 0]);
    lastLocated   = (step().turnDeg < 135.0f);   // only 90 deg turns teach the turn position
    lastLocCorner = locCorner;
    lastLocDir    = dir;
    verified();
}

// After VERIFY: roll (if this event needs it), then turn.
static void verified()
{
    // The robot coasted a little after the detection. Move forward or backward so it ends up
    // exactly rollM from the detection point.
    float remaining = rollM;
    if (encOk)
    {
        const float travelled = 0.5f * ((float)(encL - detL) + (float)(encR - detR)) / CPM;
        remaining = rollM - travelled;
    }
    if (fabsf(remaining) < 0.01f)
    {
        stopThen(mode == 'G' ? St::GYRO_TURN : St::TAPE_TURN);
        return;
    }
    adjDir = (remaining > 0.0f) ? 1 : -1;
    adjM   = fabsf(remaining);
    adjL   = encL;
    adjR   = encR;
    enter(St::ROLL);
}

// VERIFY failed: it was not the event. Carry on following on the same step.
static void notConfirmed(const char *reason)
{
    why = reason;
    lostTicks = 0;
    branchTicks = 0;
    steer.reset();
    lastSteer = 0.0f;
    flickerUntilM = legM() + FLICKER_M;   // if the line drops again right away: wobble search, not a stop
    enter(St::FOLLOW);
}

// The line has ended (nothing under the array for LOST_TICKS).
static void lineEnded()
{
    const Step &s = step();
    const float d = legM();
    const bool inWindow = encOk && fabsf(d - s.expM) <= WINDOW_M;
    const bool early    = encOk && d < s.expM - WINDOW_M;

    switch (s.ev)
    {
    case Ev::END_BAR:            // ran past the end bar: that is the bar
    case Ev::MAIN_LINE:          // crossed the main line
        // Before half the expected distance a lost line cannot be the bar: the line slid off the
        // array (channel 7, the right-most, is dead). Search for it instead of stopping.
        if ((encOk && d < 0.5f * s.expM) || d < flickerUntilM) { startTapeTurn("line lost", 'W'); return; }
        eventSeen(inWindow, s.ev == Ev::MAIN_LINE ? BRANCH_ROLL_M * 0.5f : -UTURN_BACK_M);
        return;
    case Ev::BRANCH_R:
    case Ev::BRANCH_L:
        if (early) { startTapeTurn("early end", 'W'); return; }
        fault("missed branch");  // the line ended before the branch was seen
        return;
    case Ev::BOX:
        startTapeTurn("line lost", 'W');
        return;
    case Ev::CORNER:
        if (freeSign != 0 && NRLComms::isHeadingReady() && fabsf(gridErr()) > OFF_COURSE_DEG)
        {
            fault("off course");   // the line ended while the robot faces the wrong way: stop
            return;
        }
        if (early || d < flickerUntilM) { startTapeTurn("early end", 'W'); return; }
        eventSeen(inWindow, TURN_ROLL_M);
        return;
    }
}

// How many channels the plain line normally covers (learned while following straight).
static float normalDark = 3.0f;

// True when the array shows a real branch on `side` (+1 right, -1 left).
// A branch ADDS tape: the main line is still under the centre, the two outer channels on that
// side are dark, AND the dark area is at least 2 channels wider than the plain line normally is.
// (The old test trusted the SDK's RIGHT_BRANCH / LEFT_BRANCH label on its own; with channel 7
// dead, a line sitting a little right of centre could get that label, so the robot "found" P1
// before reaching it.)
static bool branchOn(int side)
{
    const uint8_t m = line.mask();
    const int     c = line.count();
    const bool right  = side * STEER_SIGN > 0;
    const bool centre = (m & 0x18) != 0;                          // ch3 or ch4: main line still there
    const bool edge   = right ? ((m & 0x60) == 0x60)              // ch5 and ch6 (ch7 is dead)
                              : ((m & 0x03) == 0x03);             // ch0 and ch1
    const bool wide   = c >= (int)(normalDark + 0.5f) + 2;
    return centre && edge && wide;
}

// A bar across the array (end bar, or the main line met at a T): much wider than the plain line.
static bool barSeen()
{
    if (line.pattern() == LinePattern::TERMINAL) return true;
    int need = (int)(normalDark + 0.5f) + 3;
    if (need < BAR_MIN_DARK) need = BAR_MIN_DARK;
    return line.count() >= need;
}

// Last event's sensor picture for the screen: '#' = dark, '.' = light, channel 0 on the left.
static char lastMaskStr[12] = "-";

static void rememberMask()
{
    const uint8_t m = line.mask();
    for (int i = 0; i < 8; i++) lastMaskStr[i] = (m & (1 << i)) ? '#' : '.';
    lastMaskStr[8] = 0;
}

// One control tick of the whole run. Returns true when finished (DONE or FAULT).
static bool runTick()
{
    const unsigned long now = millis();
    const bool gyroOk = NRLComms::isHeadingReady();
    updateTurn();

    // Encoder health: counts must keep coming while we drive forward.
    const bool driving = (st == St::FOLLOW || st == St::ROLL || st == St::PARK_IN || st == St::BACKUP);
    if (encL != seenL || encR != seenR) { seenL = encL; seenR = encR; tLastCount = now; }
    if (!driving) tLastCount = now;
    if (encOk && driving && now - tLastCount > ENC_STALL_MS) { encOk = false; why = "enc off"; }

    switch (st)
    {
    case St::FOLLOW:
    {
        const Step &s = step();
        const float d = legM();
        // Speed: slow for the first 15 cm, then normal, then easing down to CREEP_SPEED over the
        // last SLOW_M before the predicted event (and staying there if it comes late).
        float speed = FOLLOW_SPEED;
        if (encOk)
        {
            if (d < LEG_START_M) speed = APPROACH_SPEED;
            else if (d >= s.expM - SLOW_M)
            {
                float f = (d - (s.expM - SLOW_M)) / SLOW_M;
                if (f > 1.0f) f = 1.0f;
                speed = FOLLOW_SPEED - f * (FOLLOW_SPEED - CREEP_SPEED);
            }
        }
        if (s.ev == Ev::END_BAR)                              // slow and steady toward a core
        {
            if (speed > TERMINAL_SPEED) speed = TERMINAL_SPEED;
            if (!encOk || d >= s.expM - BAR_CREEP_M) speed = BAR_CREEP_SPEED;
        }
        const LinePattern p = line.pattern();
        const bool inWindow = encOk && fabsf(d - s.expM) <= WINDOW_M;
        const bool late     = encOk && d > s.expM + WINDOW_M;
        const bool started  = !encOk || d >= MIN_EVENT_M;
        // A bar only counts after half the expected distance: right after the U-turn the end bar is
        // still close behind, and must not be taken for the main line (that skipped "P1 back").
        const bool barStarted = !encOk || d >= ((0.5f * s.expM > MIN_EVENT_M) ? 0.5f * s.expM : MIN_EVENT_M);

        // BOX: the PARK box reads as TERMINAL (7 or more channels dark).
        if (s.ev == Ev::BOX && p == LinePattern::TERMINAL)
        {
            if (tBox == 0) tBox = now | 1;
            drive.drive(APPROACH_SPEED, 0.0f);
            if (now - tBox >= BOX_CONFIRM_MS)
            {
                if (encOk && d < s.expM - WINDOW_M) why = "box early";
                rollFrom = legM();
                enter(St::PARK_IN);
            }
            return false;
        }
        tBox = 0;

        // END_BAR / MAIN_LINE: a bar across the line (many channels dark).
        if ((s.ev == Ev::END_BAR || s.ev == Ev::MAIN_LINE) && barStarted && d >= flickerUntilM && barSeen())
        {
            if (encOk && d < s.expM - WINDOW_M) why = (s.ev == Ev::END_BAR) ? "bar early" : "main early";
            eventSeen(true, s.ev == Ev::MAIN_LINE ? BRANCH_ROLL_M : -UTURN_BACK_M);   // a bar: the tape decides
            return false;
        }

        // BRANCH: inside the window (or once late, or with encoders off) take the branch.
        if ((s.ev == Ev::BRANCH_R || s.ev == Ev::BRANCH_L) && started)
        {
            const int side = (s.ev == Ev::BRANCH_R) ? +1 : -1;
            const bool square  = (freeSign == 0) || fabsf(gridErr()) <= BRANCH_MAX_ANGLE;
            const bool allowed = (inWindow || late || !encOk) && square;
            if (allowed && branchOn(side))
            {
                if (++branchTicks >= CONFIRM_TICKS)
                {
                    if (late) why = "branch late";
                    eventSeen(inWindow, BRANCH_ROLL_M);
                    return false;
                }
            }
            else
            {
                branchTicks = 0;
            }
        }

        // Known junction on this leg (P / D branches we pass): drive straight across it, square to
        // the grid by gyro. The tape is ignored here, so a branch can never pull the robot off.
        if (encOk)
        {
            int zi = -1;
            for (int i = 0; i < 3; i++)
                if (s.cross[i] > 0.0f && fabsf(d - s.cross[i]) <= CROSS_ZONE_M) zi = i;
            if (zi >= 0)
            {
                // The tape sees the junction (line much wider than normal): count it, and correct
                // the encoder distance to its known position (array on its near edge).
                if (!anchored[zi] && (p == LinePattern::TERMINAL || line.count() >= (int)(normalDark + 0.5f) + 2))
                {
                    anchored[zi] = true;
                    jxSeen++;
                    const float target = s.cross[zi] - LINE_HALF_M;
                    const int32_t shift = (int32_t)((d - target) * CPM);
                    legL += shift;
                    legR += shift;
                }
                float t = 0.0f;
                if (freeSign != 0 && gyroOk)
                {
                    t = -freeSign * HOLD_KP * gridErr();
                    if (t >  HOLD_MAX) t =  HOLD_MAX;
                    if (t < -HOLD_MAX) t = -HOLD_MAX;
                }
                drive.drive(JUNCTION_SPEED, t);
                lastSteer = t;
                lostTicks = 0;
                crossing = true;                               // re-start the steering cleanly after
                tCross = now;
                return false;
            }
        }

        // Line gone.
        if (!line.onLine())
        {
            if (lostTicks == 0) tLost = now;
            lostTicks++;
            const bool endExpected =
                (s.ev == Ev::CORNER && !(encOk && d < s.expM - WINDOW_M)) ||
                ((s.ev == Ev::END_BAR || s.ev == Ev::MAIN_LINE) && barStarted);
            if (endExpected)
            {
                // A real line end is due here: straight on slowly, then treat it as the end.
                drive.drive(APPROACH_SPEED, 0.0f);
                if (lostTicks >= LOST_TICKS) lineEnded();
                return false;
            }
            // No end expected (e.g. on a terminal): the line slid off an edge. Keep creeping and
            // steer gently toward where it was last seen - no stop. Only if that fails, search.
            if (now - tLost < SOFT_LOST_MS)
            {
                const int side = line.lastSeenSide();
                const float t = (side != 0) ? STEER_SIGN * side * SOFT_TURN : 0.0f;
                drive.drive(CREEP_SPEED * 0.8f, t);
                lastSteer = t;
                return false;
            }
            lineEnded();
            return false;
        }
        lostTicks = 0;

        // Any other branch: cross it straight, no steering.
        // Only legs on the main line have branches to cross; a terminal (END_BAR / MAIN_LINE step)
        // has none, so steering is never switched off there. Width only: with channel 7 dead the
        // SDK's LEFT/RIGHT_BRANCH labels can appear on a plain off-centre line.
        const bool canCross = (s.ev == Ev::CORNER || s.ev == Ev::BRANCH_R || s.ev == Ev::BRANCH_L || s.ev == Ev::BOX);
        const bool branch = canCross &&
                            (p == LinePattern::TERMINAL || line.count() >= (int)(normalDark + 0.5f) + 2);
        if (branch)
        {
            if (!crossing) { crossing = true; tCross = now; }
            if (now - tCross < CROSS_MAX_MS)
            {
                drive.drive(JUNCTION_SPEED, 0.0f);
                return false;
            }
        }
        else if (crossing)
        {
            crossing = false;
            steer.reset();
            lastSteer = 0.0f;
        }

        // Learn how wide the plain line looks (for the branch test).
        if (p == LinePattern::LINE && abs(line.error()) < 1500)
            normalDark += 0.05f * ((float)line.count() - normalDark);

        // Normal gentle following.
        const float err = (float)line.error() / 3500.0f;
        float turn = STEER_SIGN * steer.update(err);
        if (turn > lastSteer + MAX_STEER_STEP) turn = lastSteer + MAX_STEER_STEP;
        if (turn < lastSteer - MAX_STEER_STEP) turn = lastSteer - MAX_STEER_STEP;
        lastSteer = turn;
        float fwd = speed * (1.0f - SPEED_DROP * fabsf(err));
        if (fwd < 0.4f * speed) fwd = 0.4f * speed;
        drive.drive(fwd, turn);
        return false;
    }

    case St::STOP:
    {
        drive.stop();
        if (encL != stopL || encR != stopR) { stopL = encL; stopR = encR; tStill = now; }
        const bool still = encOk ? (now - tStill >= SETTLE_MS) : (now - tState >= SETTLE_MS);
        if (still || now - tState >= SETTLE_MAX_MS)
        {
            const St next = afterStop;
            if (next == St::ROLL) rollFrom = legM();
            if (next == St::GYRO_TURN || next == St::TAPE_TURN)
            {
                zeroTurn();                            // measure the turn from here
                corrections = 0;
                if (next == St::GYRO_TURN || mode == 'T') freeBefore = NRLComms::getHeadingFree();
            }
            enter(next);
        }
        return false;
    }

    case St::VERIFY:
    {
        // The robot is stopped. The tape must confirm the event before anything turns.
        const Step &s = step();
        const bool wide = barSeen();
        switch (s.ev)
        {
        case Ev::CORNER:
            if (!line.onLine()) startLocate(true);             // the line really has ended
            else notConfirmed("not a corner");                 // line is back: it was a gap
            return false;
        case Ev::END_BAR:
            if (wide || !line.onLine()) verified();            // on the bar, or just past it
            else notConfirmed("no bar");
            return false;
        case Ev::MAIN_LINE:
            if (wide || !line.onLine()) startLocate(false);    // on the main line, or just past it
            else notConfirmed("no main line");
            return false;
        case Ev::BRANCH_R:
        case Ev::BRANCH_L:
        {
            const int side = (s.ev == Ev::BRANCH_R) ? +1 : -1;
            if (branchOn(side)) { startLocate(false); return false; }  // still over the branch
            backL = encL;                                      // coasted past it: back up to find it
            backR = encR;
            enter(St::BACKUP);
            return false;
        }
        case Ev::BOX:
            verified();
            return false;
        }
        return false;
    }

    case St::BACKUP:
    {
        // Reverse slowly until the branch is under the array again (at most BACKUP_MAX_M).
        const int side = (step().ev == Ev::BRANCH_R) ? +1 : -1;
        const float backed = 0.5f * ((float)(encL - backL) + (float)(encR - backR)) / CPM;
        if (branchOn(side))
        {
            drive.stop();
            detL = encL;                                       // the branch is here (fallback point)
            detR = encR;
            startLocate(false);
            return false;
        }
        if ((encOk && backed >= BACKUP_MAX_M) || now - tState > 1500)
        {
            drive.stop();
            // Encoders count both ways: remove the backing up (and the drive back over it).
            legL += 2 * (encL - backL);
            legR += 2 * (encR - backR);
            notConfirmed("branch not confirmed");
            return false;
        }
        drive.drive(-ROLL_SPEED, 0.0f);
        return false;
    }

    case St::LOCATE:
    {
        const float moved = 0.5f * ((float)(encL - locL) + (float)(encR - locR)) / CPM;
        const bool p = locPred();
        if ((encOk && moved > LOCATE_MAX_M) || now - tState > 3000)
        {
            drive.stop();
            why = "edge not found";                            // fall back to the old roll distance
            verified();
            return false;
        }
        if (locCorner)
        {
            if (p) { locateDone(); return false; }             // line end back under the bar
            drive.drive(-LOCATE_SPEED, 0.0f);
        }
        else if (locPhase < 0)                                 // passed it: back up onto it
        {
            if (p) locPhase = 0;
            drive.drive(-LOCATE_SPEED, 0.0f);
        }
        else if (locPhase == 0)                                // back off it
        {
            if (!p)
            {
                drive.stop();
                locPhase = 1;
                locL = encL;
                locR = encR;
                tState = now;
                return false;
            }
            drive.drive(-LOCATE_SPEED, 0.0f);
        }
        else                                                   // creep forward onto its near edge
        {
            if (p) { locateDone(); return false; }
            drive.drive(LOCATE_SPEED, 0.0f);
        }
        return false;
    }

    case St::GRID_CHECK:
    {
        // Robot is still. Square to the grid with the gyro (up to GRID_PASSES slow passes).
        const float e = gridErr();
        if (fabsf(e) > GRID_TOL_DEG && gridPasses < GRID_PASSES && fabsf(e) < 45.0f)
        {
            gridPasses++;
            alignSign = (e > 0.0f) ? 1 : -1;
            alignStartErr = fabsf(e);
            enter(St::GRID_ALIGN);
            return false;
        }
        if (line.onLine())
        {
            // Square to the grid now, so any line offset is sideways only: it shows how far the
            // pivot was from the junction. Learn from it for the next turn of the same kind.
            lastTurnErr = line.error();
            if (lastLocated && fabsf(e) <= GRID_TOL_DEG * 2.0f)
            {
                const float off   = (float)lastTurnErr / 1000.0f * SENSOR_PITCH_M;   // + = line right
                const float delta = lastLocDir * off;                                // + = turned too late
                float *t = lastLocCorner ? &learnCorner : &learnBranch[lastLocDir > 0 ? 1 : 0];
                *t -= LEARN_GAIN * delta;
                if (*t >  LEARN_MAX_M) *t =  LEARN_MAX_M;
                if (*t < -LEARN_MAX_M) *t = -LEARN_MAX_M;
            }
            lastLocated = false;
        }
        if (line.onLine() && abs(line.error()) > LATERAL_MAX)  // square but line near an edge:
        {                                                      // twist toward it a little first
            centreTarget = LATERAL_OK;
            centrePasses = 0;
            centreDir = (line.error() > 0) ? STEER_SIGN : -STEER_SIGN;
            centreFlipped = false;
            enter(St::CENTRE);
            return false;
        }
        if (line.onLine())                                     // square and on the tape: go
        {
            startLeg();
            enter(St::FOLLOW);
            return false;
        }
        centreTarget = CENTRE_ERR;
        why = "no line after squaring";                        // square but no tape: tape search
        enter(St::CENTRE_CHECK);
        return false;
    }

    case St::GRID_ALIGN:
    {
        const float e = gridErr();
        const int eSign = (e > 0.0f) ? 1 : -1;
        if (fabsf(e) > alignStartErr + 5.0f)                   // getting worse: wrong way, stop using the grid
        {
            freeSign = 0;
            why = "grid sign wrong";
            stopThen(St::CENTRE_CHECK);
            return false;
        }
        if (fabsf(e) <= GRID_TOL_DEG * 0.5f || eSign != alignSign || now - tState > 2000)
        {
            stopThen(St::GRID_CHECK);
            return false;
        }
        drive.drive(0.0f, -alignSign * freeSign * CORR_SPEED);
        return false;
    }

    case St::ROLL:
    {
        // ADJUST: move forward or backward to the turning spot.
        drive.drive(adjDir * ROLL_SPEED, 0.0f);
        const float moved = 0.5f * ((float)(encL - adjL) + (float)(encR - adjR)) / CPM;
        const bool done = encOk ? (moved >= adjM)
                                : (now - tState >= (adjDir > 0 ? ROLL_MS : UTURN_BACK_MS));
        if (done || now - tState > 2500)
        {
            if (mode == 'G') stopThen(St::GYRO_TURN);
            else             stopThen(St::TAPE_TURN);
        }
        return false;
    }

    case St::GYRO_TURN:
    {
        const float left = targetDeg - STOP_EARLY_DEG - turned();
        if (left <= 0.0f)
        {
            stopThen(St::CHECK);
            return false;
        }
        if (now - tState > GYRO_TURN_MS * (targetDeg > 135.0f ? 2 : 1))   // does not add up
        {
            startTapeTurn("gyro slow", 'T');
            return false;
        }
        drive.drive(0.0f, spinDir * (left < SLOW_DEG ? TURN_SLOW : TURN_FAST));
        return false;
    }

    case St::CHECK:
    {
        // Robot is still. First the gyro: correct an over- or under-turn (at most twice).
        const float errDeg = turned() - targetDeg;           // + = turned too far
        if (fabsf(errDeg) > TURN_TOL_DEG && corrections < MAX_CORRECTIONS)
        {
            corrections++;
            enter(St::CORRECT);
            return false;
        }
        // Then the tape must agree.
        if (line.onLine())
        {
            stepDone('G');
            return false;
        }
        why = "no line after turn";
        sweepPhase = 0;
        enter(St::SWEEP);
        return false;
    }

    case St::CORRECT:
    {
        const float errDeg = turned() - targetDeg;
        if (fabsf(errDeg) <= TURN_TOL_DEG * 0.5f || now - tState > 1500)
        {
            stopThen(St::CHECK);
            return false;
        }
        drive.drive(0.0f, (errDeg > 0.0f ? -spinDir : spinDir) * CORR_SPEED);
        return false;
    }

    case St::SWEEP:
    {
        // Slow search for the line, only within +/- SWEEP_DEG of the target.
        if (line.onLine())
        {
            stepDone('S');
            return false;
        }
        const float t = turned();
        if (sweepPhase == 0)
        {
            if (t >= targetDeg + SWEEP_DEG || now - tState > 2500) { sweepPhase = 1; tState = now; }
            else drive.drive(0.0f, spinDir * CORR_SPEED);
        }
        else
        {
            if (t <= targetDeg - SWEEP_DEG || now - tState > 4000) { fault("no line at turn"); return true; }
            drive.drive(0.0f, -spinDir * CORR_SPEED);
        }
        return false;
    }

    case St::CENTRE_CHECK:
    {
        // Robot is still. Centred enough (or out of passes): start the next step from here.
        const bool centred = line.onLine() && abs(line.error()) <= centreTarget;
        if (centred || centrePasses >= CENTRE_PASSES)
        {
            startLeg();                                        // distance counts from here
            enter(St::FOLLOW);
            return false;
        }
        centrePasses++;
        if (line.onLine())
            centreDir = (line.error() > 0) ? STEER_SIGN : -STEER_SIGN;
        else
            centreDir = (line.lastSeenSide() != 0) ? STEER_SIGN * line.lastSeenSide() : -spinDir;
        centreFlipped = false;
        enter(St::CENTRE);
        return false;
    }

    case St::CENTRE:
    {
        // One slow pass in one direction; stop when the line reaches the middle or crosses it.
        if (!line.onLine())
        {
            if (!centreFlipped && now - tState > CENTRE_MAX_MS / 2) { centreDir = -centreDir; centreFlipped = true; tState = now; }
            else if (centreFlipped && now - tState > CENTRE_MAX_MS) { fault("lost after turn"); return true; }
            drive.drive(0.0f, centreDir * CORR_SPEED);
            return false;
        }
        const int err = line.error();
        const int errDir = (err > 0) ? STEER_SIGN : -STEER_SIGN;
        if (abs(err) <= centreTarget || errDir != centreDir || now - tState > CENTRE_MAX_MS)
        {
            stopThen(St::CENTRE_CHECK);
            return false;
        }
        drive.drive(0.0f, centreDir * CORR_SPEED);
        return false;
    }

    case St::TAPE_TURN:
    {
        // Pure tape: spin until the line is under the array, with a hard limit on how far.
        unsigned long minMs = TAPE_MIN_MS;
        if (mode == 'W' || mode == 'V') minMs = 0;
        else if (targetDeg > 135.0f)     minMs = UTURN_MIN_MS;
        const bool onBar = barSeen();
        if (now - tState >= minMs && line.onLine() && !onBar && abs(line.error()) <= TAPE_ACCEPT_ERR)
        {
            drive.stop();
            if (mode == 'W' || mode == 'V')
            {
                lostTicks = 0;                                 // a wobble: same step, distance kept
                steer.reset();
                lastSteer = 0.0f;
                stopThen(St::FOLLOW);
                return false;
            }
            stepDone('T');
            return false;
        }
        const float limit = (mode == 'W' || mode == 'V') ? targetDeg : targetDeg + 30.0f;
        const bool tooFar = gyroOk ? (turned() >= limit)
                                   : (now - tState > TAPE_MAX_MS * (targetDeg > 135.0f ? 2 : 1));
        if (tooFar)
        {
            if (mode == 'W')
            {
                // Not found on the last-seen side. Just after a turn, on the box leg, or on a
                // step that is not a corner, it cannot be a corner: search back the other way.
                if (step().ev != Ev::CORNER || legM() < NEAR_CORNER_M) startTapeTurn("search back", 'V');
                else                                                   startTapeTurn("early corner", 'T');
                return false;
            }
            fault("no line");
            return true;
        }
        drive.drive(0.0f, spinDir * TURN_SLOW);
        return false;
    }

    case St::PARK_IN:
    {
        drive.drive(APPROACH_SPEED, 0.0f);
        const bool done = encOk ? (legM() - rollFrom >= PARK_IN_M) : (now - tState >= PARK_MS);
        if (done)
        {
            drive.stop();
            logTurn('P');
            enter(St::DONE);
            return true;
        }
        return false;
    }

    case St::DONE:
    case St::FAULT:
        drive.stop();
        return true;
    }
    return true;
}

static const char *stateName()
{
    switch (st)
    {
    case St::FOLLOW:    return "FOLLOW";
    case St::STOP:      return "STOP";
    case St::VERIFY:    return "VERIFY";
    case St::BACKUP:    return "BACKUP";
    case St::LOCATE:    return "LOCATE";
    case St::GRID_ALIGN: return "SQUARE";
    case St::GRID_CHECK: return "SQUARE CHK";
    case St::ROLL:      return "ADJUST";
    case St::GYRO_TURN: return "GYRO TURN";
    case St::CHECK:     return "CHECK";
    case St::CORRECT:   return "CORRECT";
    case St::SWEEP:     return "SWEEP";
    case St::CENTRE:    return "CENTRE";
    case St::CENTRE_CHECK: return "CENTRE CHK";
    case St::TAPE_TURN: return "TAPE TURN";
    case St::PARK_IN:   return "PARK IN";
    case St::DONE:      return "DONE";
    case St::FAULT:     return "FAULT";
    }
    return "?";
}

// ---------------------------------------------------------------------
//  SDK-only helpers for the fallback OpModes
// ---------------------------------------------------------------------
static uint32_t boxSince = 0;

// TERMINAL (7 of 8 dark) held for BOX_CONFIRM_MS. The SDK's followToBox() wants all 8 dark,
// which a dead channel 7 never gives.
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
//  OpModes
// ---------------------------------------------------------------------
class LineRunBase : public NRLOpMode
{
public:
    void init() override        // INIT does not move anything (R8.11)
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
        lineActions.clearFault();
        lineActions.resetCounts();
        lineActions.clearRoute();
        choosePlan();
        stepIdx = 0;
        freeStart = NRLComms::getHeadingFree();   // the robot is placed square in the START box
        gridTurn  = 0.0f;
        freeSign  = 0;
        st = St::FOLLOW;
        why = "-";
        turnsLog[0] = 0;
        lastDeg = 0.0f;
        encOk = true;
        runStart = millis();
        tLastCount = runStart;
        normalDark = 3.0f;
        lastMaskStr[0] = '-'; lastMaskStr[1] = 0;

        runAction(sequential({
            lineActions.leaveBox(),                                     // SDK: straight out of the box
            instant([] {
                if (lineActions.fault() != LineFault::NONE) { fault(lineActions.faultName()); return; }
                drive.setRampRate(RAMP_RATE);
                startLeg();
                enter(St::FOLLOW);
            }),
            wait_until([] { return runTick(); }),
        }));
    }

    void loop() override
    {
        const uint32_t elapsed = millis() - runStart;
        if (elapsed > SOFT_STOP_MS && isActionRunning())
        {
            cancelActions();
            drive.stop();
            why = "time";
        }

        static char legLine[16], dLine[16];
        snprintf(legLine, sizeof(legLine), "%s exp %d", step().name, (int)(step().expM * 100.0f + 0.5f));
        snprintf(dLine, sizeof(dLine), "%d", (int)(legM() * 100.0f + 0.5f));

        telemetry.addData("state", stateName());
        telemetry.addData("why", why);
        telemetry.addData("leg", legLine);
        telemetry.addData("d cm", dLine);
        telemetry.addData("turns", turnsLog[0] ? turnsLog : "-");
        static char maskLine[16];
        snprintf(maskLine, sizeof(maskLine), "%s %d", lastMaskStr, lastTurnErr);
        telemetry.addData("mask", maskLine);   // last event: sensors 0..7, and where the line landed after the last turn
        static char jxLine[16];
        int jxExp = 0;
        for (int i = 0; i < 3; i++) if (step().cross[i] > 0.0f) jxExp++;
        snprintf(jxLine, sizeof(jxLine), "%d/%d%s", jxSeen, jxExp, encOk ? "" : " encOFF");
        telemetry.addData("jx", jxLine);       // junctions crossed on this leg / expected
        telemetry.addData("t", (float)(elapsed / 1000));
    }

    void stop() override
    {
        drive.stop();
        detachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN));
        detachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN));
    }

protected:
    virtual void choosePlan() = 0;
};

class ParkRun : public LineRunBase
{
protected:
    void choosePlan() override
    {
        plan = PARK_PLAN;
        planLen = sizeof(PARK_PLAN) / sizeof(PARK_PLAN[0]);
    }
};

class P1D1Run : public LineRunBase
{
protected:
    void choosePlan() override
    {
        plan = P1D1_PLAN;
        planLen = sizeof(P1D1_PLAN) / sizeof(P1D1_PLAN[0]);
    }
};

// Shared base for the two SDK-only fallbacks.
static const char *fbStage = "ready";
static uint32_t    fbStart = 0;

static ActionPtr fbAt(const char *name)
{
    return instant([name] { fbStage = name; });
}

class FallbackBase : public NRLOpMode
{
public:
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
        lineActions.clearRoute();
        boxSince = 0;
        fbStage = "ready";
        fbStart = millis();
        runAction(mission());
    }

    void loop() override
    {
        const uint32_t elapsed = millis() - fbStart;
        if (elapsed > SOFT_STOP_MS && isActionRunning())
        {
            cancelActions();
            drive.stop();
            fbStage = "TIME";
        }
        const LineJunction &j = lineActions.lastJunction();
        char lrs[4] = {j.left ? 'L' : '-', j.right ? 'R' : '-', j.straight ? 'S' : '-', 0};

        telemetry.addData("stage", fbStage);
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

class ParkRunTape : public FallbackBase
{
protected:
    ActionPtr mission() override
    {
        return lineActions.auto_()
            .then(fbAt("leave box")).leaveBox()
            .then(fbAt("round course")).followUntil([] { return inBox(); })
            .then(fbAt("parking")).park(TAPE_PARK_MS)
            .then(fbAt("DONE"))
            .build();
    }
};

class FirstJunction : public FallbackBase
{
protected:
    ActionPtr mission() override
    {
        return lineActions.auto_()
            .then(fbAt("leave box")).leaveBox()
            .then(fbAt("to junction")).followToJunction()
            .then(fbAt("DONE"))
            .build();
    }
};

REGISTER_OPMODE(ParkRun,       "Park Run",       AUTO);
REGISTER_OPMODE(P1D1Run,       "P1-D1 Run",      AUTO);
REGISTER_OPMODE(ParkRunTape,   "Park Run Tape",  AUTO);
REGISTER_OPMODE(FirstJunction, "First Junction", AUTO);