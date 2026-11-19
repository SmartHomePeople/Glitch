// =====================================================================
//  GLITCHED (Team 190) - Start zone to Park zone
//  File: RobotFirmware/opmodes/AutoPark.cpp
//
//  Three AUTO OpModes:
//    "Park Run"        MAIN. Our own follower: encoders anticipate each corner, the tape
//                      confirms it, the gyro makes the turn. If anything does not add up,
//                      that corner is done by tape alone. (35 pts: leave + park)
//    "Park Run Tape"   FALLBACK. The SDK's own tape-only follower, with softer steering.
//    "First Junction"  FALLBACK. Leave the box and stop just past the first junction.
//
//  Start the robot CENTRED in the START box, facing the line that leaves it, after a
//  LINE TEST calibration on this mat (white first, then black; "1 DEAD" for channel 7 is ok).
//
//  ---------------------------------------------------------------------------------
//  HOW "Park Run" TAKES A CORNER (the robot comes to a FULL STOP before and after each move)
//    1  ENCODERS   It knows how far each corner is (the arena is a 2000 mm square). Near
//                  the expected distance it slows down.
//    2  TAPE       The corner is confirmed when the line ends under the array. STOP.
//    3  ROLL       Roll forward so the middle of the chassis is on the corner. STOP.
//    4  GYRO       Turn about 90 deg by gyro, slowing near the end. STOP.
//    5  CHECK      Read the gyro while still: outside 86..94 deg it corrects by gyro (max 2
//                  times, stopping each time). Then the tape must show the line. If not, it
//                  searches slowly between 65 and 115 deg only.
//    6  CENTRE     If the line is off to one side, turn toward it on the spot (one way only,
//                  stopping when it reaches the middle). Then follow, slowly for the first 15 cm.
//    FALLBACK      If it does not add up (line ended too early/late, encoders stopped, gyro
//                  not ready or too slow) that corner is turned by TAPE: stop, spin toward the
//                  corner until the line is under the array, never more than 120 deg.
//    "turns" stores every corner: G = gyro, S = gyro + search, T = tape. P = parked.
//    A line that ends too early is searched 60 deg toward where it was last seen; if found it
//    was a wobble (not counted), otherwise the corner is done by tape.
//
//  SCREEN (8 lines)
//    state   FOLLOW / STOP / ROLL / GYRO TURN / CHECK / CORRECT / SWEEP / CENTRE / TAPE TURN /
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
    .minDrive = 29
}};

static HexaDCMotor rightMotor{{
    .dirPin = MOTOR_R_DIR,
    .pwmPin = MOTOR_R_PWM,
    .flipped = true,
    .minDrive = 32
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
constexpr float FIRST_LEG_M = 0.50f;   // MEASURE: from the edge of the START box (where the line leaves it) to corner C1
constexpr float LAST_LEG_M  = 0.50f;   // MEASURE: from corner C4 to the edge of the PARK box
constexpr float PIVOT_FWD_M = 0.10f;   // MEASURE: from the sensor bar back to the middle of the chassis
                                       //          (halfway between the front and back wheels)
constexpr float TURN_ROLL_M = 0.06f;   // TUNE: how far it rolls forward after the line ends, before turning
                                      // (was PIVOT_FWD_M = 0.10). Smaller = turns closer to where it stopped.
constexpr float WINDOW_M    = 0.20f;   // a corner is "expected" within +/- this of the predicted distance
constexpr float SLOW_M      = 0.30f;   // start slowing this far before the predicted corner
constexpr float PARK_IN_M   = 0.25f + PIVOT_FWD_M;   // array past the box edge so the chassis sits mid-box

// ---------------------------------------------------------------------
//  Driving. Speeds are TankDrive units (-1..1).
// ---------------------------------------------------------------------
constexpr float FOLLOW_SPEED   = 0.35f;
constexpr float APPROACH_SPEED = 0.22f;   // near a predicted corner / the box
constexpr float JUNCTION_SPEED = 0.25f;   // straight across a branch, no steering
constexpr float ROLL_SPEED     = 0.22f;
constexpr float TURN_FAST      = 0.30f;   // was 0.40 / 0.32: it overshot the corner
constexpr float TURN_SLOW      = 0.25f;   // last part of a turn, and tape turns (lower may stall the stiff left wheel)
constexpr float CORR_SPEED     = 0.25f;   // small gyro correction after a turn
constexpr float SPEED_DROP     = 0.50f;   // slow down as the line moves off centre
constexpr float RAMP_RATE      = 0.10f;   // how fast motor commands may change per tick (SDK uses 0.25)
constexpr float MAX_STEER_STEP = 0.025f;  // the steering may change at most this much per tick, so a
                                          // correction grows gradually instead of jerking

// Steering: a gentle PD on the line error (-1..+1). TUNE: if it still zig-zags lower kP;
// if it runs wide off the line raise it. +1 when channel 0 is the LEFT-most sensor.
constexpr int   STEER_SIGN = 1;
static HexaPID steer{ HexaPIDConfig{ .kP = 0.22f, .kD = 0.08f, .outMin = -0.25f, .outMax = 0.25f, .dAlpha = 0.5f } };
static float   lastSteer = 0.0f;   // the steering used last tick (for MAX_STEER_STEP)

// Turns
constexpr float TURN_DEG       = 90.0f;
constexpr float STOP_EARLY_DEG = 6.0f;    // cut the motors this early; the robot coasts the rest
constexpr float TURN_TOL_DEG   = 4.0f;    // after stopping, 86..94 deg is accepted; outside that it corrects by gyro
constexpr float SWEEP_DEG      = 25.0f;   // no line after the turn: search only 65..115 deg, never further
constexpr float MAX_SPIN_DEG   = 120.0f;  // a tape turn never spins more than this (stops the 180 deg reversal)
constexpr float WOBBLE_DEG     = 60.0f;   // an early line-end is searched this far toward the last-seen side
constexpr int   CENTRE_ERR     = 500;     // after a turn, pivot (one way only) until the line is this close to centre
constexpr float LEG_START_M    = 0.15f;   // drive slowly for the first part of every leg
constexpr float NEAR_CORNER_M  = 0.40f;   // a line lost this soon after a corner is searched both ways, never turned as a corner
constexpr float SLOW_DEG       = 35.0f;   // slow down for the last 35 degrees
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
constexpr unsigned long GYRO_TURN_MS   = 3000;   // a gyro turn longer than this does not add up
constexpr unsigned long TAPE_MIN_MS    = 200;    // spin at least this long before accepting a corner line
constexpr unsigned long TAPE_MAX_MS    = 2500;   // time limit when the gyro is not ready
constexpr unsigned long ENC_STALL_MS   = 500;    // no counts while driving this long = encoders OFF
constexpr unsigned long SOFT_STOP_MS   = 145000;

// ---------------------------------------------------------------------
//  The course: legs 0..3 end at corners C1..C4, leg 4 ends in the PARK box
// ---------------------------------------------------------------------
constexpr int LAST_LEG = 4;

static float expectedM(int leg)
{
    if (leg == 0)        return FIRST_LEG_M;
    if (leg == LAST_LEG) return LAST_LEG_M - PIVOT_FWD_M;
    return SIDE_M - PIVOT_FWD_M;      // after a turn the array is PIVOT_FWD_M past the corner
}

static const char *legName(int leg)
{
    static const char *names[] = {"C1", "C2", "C3", "C4", "BOX"};
    return names[leg < 0 ? 0 : (leg > LAST_LEG ? LAST_LEG : leg)];
}

// ---------------------------------------------------------------------
//  Park Run state machine
// ---------------------------------------------------------------------
enum class St { FOLLOW, STOP, ROLL, GYRO_TURN, CHECK, CORRECT, SWEEP, CENTRE, TAPE_TURN, PARK_IN, DONE, FAULT };

static St          st        = St::FOLLOW;
static const char *why       = "-";
static int         leg       = 0;
static char        turnsLog[8] = "";
static float       lastDeg   = 0.0f;
static bool        encOk     = true;
static uint32_t    legL = 0, legR = 0;           // encoder counts at the start of the leg
static uint32_t    seenL = 0, seenR = 0;         // for the stall check
static unsigned long tState = 0, tLastCount = 0, tCross = 0, tBox = 0;
static int         lostTicks = 0;
static bool        crossing  = false;
static char        mode      = 'G';              // G = gyro corner, T = tape corner, W = wobble check
static int         spinDir   = CORNER_DIR;
static float       rollFrom  = 0.0f;
static float       rollM     = 0.0f;
static uint32_t    runStart  = 0;
static St          afterStop = St::FOLLOW;   // where STOP goes once the robot is still
static uint32_t    stopL = 0, stopR = 0;
static unsigned long tStill = 0;
static int         corrections = 0;
static int         sweepPhase  = 0;
static int         centreDir   = 1;
static bool        centreFlipped = false;

static float legM()
{
    return 0.5f * ((float)(encL - legL) + (float)(encR - legR)) / CPM;
}

static void startLeg()
{
    legL = encL;
    legR = encR;
    lostTicks = 0;
    crossing = false;
    tBox = 0;
    steer.reset();
    lastSteer = 0.0f;
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

static float turned() { return fabsf(NRLComms::getHeading()); }

// A corner is finished: store it and start the next leg.
static void cornerDone(char c)
{
    drive.stop();
    lastDeg = NRLComms::isHeadingReady() ? turned() : -1.0f;
    logTurn(c);
    leg++;
    startLeg();
    // Line off to one side: turn toward it on the spot first, then drive. Driving forward
    // with the line at the edge is what lost it at corner C2.
    if (line.onLine() && abs(line.error()) > CENTRE_ERR)
    {
        centreDir = (line.error() > 0) ? STEER_SIGN : -STEER_SIGN;
        centreFlipped = false;
        enter(St::CENTRE);
        return;
    }
    enter(St::FOLLOW);
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

// Pure tape turn: from a full stop, spin until the line is under the array.
static void startTapeTurn(const char *reason, char m)
{
    if (reason) why = reason;
    mode = m;
    if (m == 'W')
    {
        const int side = line.lastSeenSide();           // -1 left, +1 right
        spinDir = (side != 0) ? STEER_SIGN * side : CORNER_DIR;
    }
    else if (m == 'V')
    {
        spinDir = -spinDir;                             // search back the other way
    }
    else
    {
        spinDir = CORNER_DIR;                           // we know every corner here is a left turn
    }
    stopThen(St::TAPE_TURN);
}

// The line has ended. Decide how to take it.
static void lineEnded()
{
    const float d = legM();
    const float e = expectedM(leg);
    const bool inWindow = encOk && fabsf(d - e) <= WINDOW_M;
    const bool early    = encOk && d < e - WINDOW_M;

    if (leg == LAST_LEG)                       // no corner left: the line should run into the box
    {
        startTapeTurn("line lost", 'W');
        return;
    }
    if (early)                                 // too soon for a corner: probably a wobble
    {
        startTapeTurn("early end", 'W');
        return;
    }
    spinDir = CORNER_DIR;
    rollM = TURN_ROLL_M;
    if (inWindow && NRLComms::isHeadingReady())   // everything agrees: gyro corner
    {
        mode = 'G';
    }
    else                                          // late, encoders off, or gyro not ready
    {
        why = !encOk ? "enc off" : (!NRLComms::isHeadingReady() ? "gyro not ready" : "late corner");
        mode = 'T';
    }
    stopThen(St::ROLL);
}

// One control tick of the whole run. Returns true when finished (DONE or FAULT).
static bool parkRunTick()
{
    const unsigned long now = millis();
    const bool gyroOk = NRLComms::isHeadingReady();

    // Encoder health: counts must keep coming while we drive forward.
    const bool driving = (st == St::FOLLOW || st == St::ROLL || st == St::PARK_IN);
    if (encL != seenL || encR != seenR) { seenL = encL; seenR = encR; tLastCount = now; }
    if (!driving) tLastCount = now;
    if (encOk && driving && now - tLastCount > ENC_STALL_MS) { encOk = false; why = "enc off"; }

    switch (st)
    {
    case St::FOLLOW:
    {
        const float d = legM();
        const float e = expectedM(leg);
        const bool  slow  = encOk && (d >= e - SLOW_M || d < LEG_START_M);
        const float speed = slow ? APPROACH_SPEED : FOLLOW_SPEED;
        const LinePattern p = line.pattern();

        // Last leg: the PARK box reads as TERMINAL (7 or more channels dark).
        if (leg == LAST_LEG && p == LinePattern::TERMINAL)
        {
            if (tBox == 0) tBox = now | 1;
            drive.drive(APPROACH_SPEED, 0.0f);
            if (now - tBox >= BOX_CONFIRM_MS)
            {
                if (encOk && d < e - WINDOW_M) why = "box early";
                rollFrom = legM();
                enter(St::PARK_IN);
            }
            return false;
        }
        tBox = 0;

        // Line gone: drive straight on slowly, and after LOST_TICKS treat it as the line ending.
        if (!line.onLine())
        {
            lostTicks++;
            drive.drive(APPROACH_SPEED, 0.0f);
            if (lostTicks >= LOST_TICKS) lineEnded();
            return false;
        }
        lostTicks = 0;

        // A branch (or the start of a corner): cross it straight, no steering.
        const bool branch = (p == LinePattern::LEFT_BRANCH || p == LinePattern::RIGHT_BRANCH ||
                             p == LinePattern::TERMINAL || line.count() >= 4);
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

        // Normal gentle following.
        const float err  = (float)line.error() / 3500.0f;
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
                NRLComms::zeroHeading();               // measure the turn from here
                corrections = 0;
            }
            enter(next);
        }
        return false;
    }

    case St::ROLL:
    {
        drive.drive(ROLL_SPEED, 0.0f);
        const bool done = encOk ? (legM() - rollFrom >= rollM) : (now - tState >= ROLL_MS);
        if (done || now - tState > 2000)
        {
            if (mode == 'G') stopThen(St::GYRO_TURN);
            else             stopThen(St::TAPE_TURN);
        }
        return false;
    }

    case St::GYRO_TURN:
    {
        const float left = TURN_DEG - STOP_EARLY_DEG - turned();
        if (left <= 0.0f)
        {
            stopThen(St::CHECK);
            return false;
        }
        if (now - tState > GYRO_TURN_MS)                       // does not add up
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
        const float errDeg = turned() - TURN_DEG;            // + = turned too far
        if (fabsf(errDeg) > TURN_TOL_DEG && corrections < 2)
        {
            corrections++;
            enter(St::CORRECT);
            return false;
        }
        // Then the tape must agree.
        if (line.onLine())
        {
            cornerDone('G');
            return false;
        }
        why = "no line after turn";
        sweepPhase = 0;
        enter(St::SWEEP);
        return false;
    }

    case St::CORRECT:
    {
        const float errDeg = turned() - TURN_DEG;
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
        // Slow search for the line, only between 65 and 115 degrees.
        if (line.onLine())
        {
            cornerDone('S');
            return false;
        }
        const float t = turned();
        if (sweepPhase == 0)
        {
            if (t >= TURN_DEG + SWEEP_DEG || now - tState > 2500) { sweepPhase = 1; tState = now; }
            else drive.drive(0.0f, spinDir * CORR_SPEED);
        }
        else
        {
            if (t <= TURN_DEG - SWEEP_DEG || now - tState > 4000) { fault("no line at corner"); return true; }
            drive.drive(0.0f, -spinDir * CORR_SPEED);
        }
        return false;
    }

    case St::CENTRE:
    {
        // One direction only: stop as soon as the line is near centre or crosses it.
        if (!line.onLine())
        {
            if (!centreFlipped) { centreDir = -centreDir; centreFlipped = true; tState = now; }
            else if (now - tState > CENTRE_MAX_MS) { fault("lost after turn"); return true; }
            drive.drive(0.0f, centreDir * CORR_SPEED);
            return false;
        }
        const int err = line.error();
        const int errDir = (err > 0) ? STEER_SIGN : -STEER_SIGN;
        if (abs(err) <= CENTRE_ERR || errDir != centreDir || now - tState > CENTRE_MAX_MS)
        {
            stopThen(St::FOLLOW);
            return false;
        }
        drive.drive(0.0f, centreDir * CORR_SPEED);
        return false;
    }

    case St::TAPE_TURN:
    {
        // Pure tape: spin until the line is under the array, with a hard limit on how far.
        const unsigned long minMs = (mode == 'W') ? 0 : TAPE_MIN_MS;
        if (now - tState >= minMs && line.onLine() && abs(line.error()) <= TAPE_ACCEPT_ERR)
        {
            drive.stop();
            if (mode == 'W' || mode == 'V')
            {
                lostTicks = 0;                                 // a wobble: same leg, distance kept
                steer.reset();
                lastSteer = 0.0f;
                stopThen(St::FOLLOW);
                return false;
            }
            cornerDone('T');
            return false;
        }
        const float limit = (mode == 'W') ? WOBBLE_DEG : (mode == 'V' ? 2.0f * WOBBLE_DEG : MAX_SPIN_DEG);
        const bool tooFar = gyroOk ? (turned() >= limit) : (now - tState > TAPE_MAX_MS);
        if (tooFar)
        {
            if (mode == 'W')
            {
                // Not found on the last-seen side. Just after a corner (or on the last leg) it
                // cannot be a corner: search back the other way. Otherwise it was the corner.
                if (leg == LAST_LEG || legM() < NEAR_CORNER_M) startTapeTurn("search back", 'V');
                else                                           startTapeTurn("early corner", 'T');
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
    case St::ROLL:      return "ROLL";
    case St::GYRO_TURN: return "GYRO TURN";
    case St::CHECK:     return "CHECK";
    case St::CORRECT:   return "CORRECT";
    case St::SWEEP:     return "SWEEP";
    case St::CENTRE:    return "CENTRE";
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
class ParkRun : public NRLOpMode
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
        st = St::FOLLOW;
        why = "-";
        leg = 0;
        turnsLog[0] = 0;
        lastDeg = 0.0f;
        encOk = true;
        runStart = millis();
        tLastCount = runStart;

        runAction(sequential({
            lineActions.leaveBox(),                                     // SDK: straight out of the box
            instant([] {
                if (lineActions.fault() != LineFault::NONE) { fault(lineActions.faultName()); return; }
                drive.setRampRate(RAMP_RATE);
                startLeg();
                enter(St::FOLLOW);
            }),
            wait_until([] { return parkRunTick(); }),
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
        snprintf(legLine, sizeof(legLine), "%s exp %d", legName(leg), (int)(expectedM(leg) * 100.0f + 0.5f));
        snprintf(dLine, sizeof(dLine), "%d", (int)(legM() * 100.0f + 0.5f));

        telemetry.addData("state", stateName());
        telemetry.addData("why", why);
        telemetry.addData("leg", legLine);
        telemetry.addData("d cm", dLine);
        telemetry.addData("turns", turnsLog[0] ? turnsLog : "-");
        telemetry.addData("deg", lastDeg);
        telemetry.addData("enc", encOk ? "ok" : "OFF");
        telemetry.addData("t", (float)(elapsed / 1000));
    }

    void stop() override
    {
        drive.stop();
        detachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN));
        detachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN));
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
REGISTER_OPMODE(ParkRunTape,   "Park Run Tape",  AUTO);
REGISTER_OPMODE(FirstJunction, "First Junction", AUTO);