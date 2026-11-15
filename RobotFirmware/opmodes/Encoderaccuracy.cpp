// =====================================================================
//  GLITCHED (Team 190) - encoder ACCURACY test
//  File: RobotFirmware/opmodes/EncoderAccuracy.cpp
//
//  OpMode "Encoder Accuracy" (TELEOP). Counts rising edges of channel A of
//  two OE-28 encoders and helps you find out how far to trust them.
//  Wiring: left encoder channel A -> D6 (DIGITAL_6, GPIO36), right encoder
//  channel A -> D7 (DIGITAL_7, GPIO37), the 3.3V / GND / D6 / D7 header. Change
//  LEFT_ENC_PIN / RIGHT_ENC_PIN below if you wired them the other way round.
//
//  Channel A only: the count rises in either direction, which is fine for
//  straight driving and distance checks.
//
//  TESTS (do them in this order)
//   1  NOISE      Robot still, motors idle, 30 s. Both counts must stay at 0.
//                 Any count that creeps up = electrical noise (check grounds, wiring).
//   2  WHEEL      Turn each wheel by hand. Its own count must rise, the other must not.
//                 (If left/right are swapped, swap the two pin constants below.)
//   3  HAND PUSH  Mark a start line and a line exactly TRIAL_DIST_M away.
//                 Press X to zero, push the robot by hand to the second line,
//                 press A to log the trial. Repeat 5-10 times. The screen shows
//                 counts per metre for each side, the spread between trials
//                 (repeatability) and the left/right ratio.
//   4  DRIVE      D-pad UP drives forward until the encoders say TARGET_M, then stops
//                 and shows where the encoders think it ended up. Measure the real
//                 distance with a tape and compare. The difference is the accuracy
//                 of the whole chain (counts, calibration and braking).
//   5  SPEED      D-pad LEFT drives for SPEED_RUN_MS at TEST_SPEED and shows the
//                 average speed each encoder measured.
//
//  CONTROLS   X = zero counts     A = log hand-push trial     Y = clear trials + stop
//             D-pad UP = drive to TARGET_M     D-pad LEFT = timed speed run
//             LT = stop (engine)
//
//  Safety: every drive has a timeout, and stops itself if an encoder gives no
//  pulses (shown as NO PULSES) or if the two sides disagree badly (shown as L/R MISMATCH).
//  If the robot drives BACKWARD, set FWD_SIGN to -1.
//
//  STATUS: written against the NRL SDK v1.5.0 headers. NOT compiled or run on the
//  robot yet. The limits (10 % mismatch, 1.5 s no-pulse) are my own starting values.
// =====================================================================
#include "NRL.h"

// ---------------------------------------------------------------------
//  Hardware (same motor settings as the other OpModes)
// ---------------------------------------------------------------------
static HexaDCMotor leftMotor{{
    .dirPin = MOTOR_L_DIR,
    .pwmPin = MOTOR_L_PWM,
    .flipped = true,
    .minDrive = 29        // TUNE: re-measure after any motor change
}};

static HexaDCMotor rightMotor{{
    .dirPin = MOTOR_R_DIR,
    .pwmPin = MOTOR_R_PWM,
    .minDrive = 32        // TUNE: re-measure after any motor change
}};

static TankDrive drive(leftMotor, rightMotor);

// ---------------------------------------------------------------------
//  Tuning
// ---------------------------------------------------------------------
constexpr int   LEFT_ENC_PIN   = DIGITAL_6;    // left-side encoder channel A  (D6, GPIO36)
constexpr int   RIGHT_ENC_PIN  = DIGITAL_7;    // right-side encoder channel A (D7, GPIO37)
constexpr bool  USE_PULLUP     = false;         // internal pull-up on the encoder inputs (see init)
constexpr float FWD_SIGN       = 1.0f;         // set to -1.0f if the robot drives backward
constexpr float TRIAL_DIST_M   = 1.0f;         // length of the hand-push course
constexpr float COUNTS_PER_M   = 1000.0f;      // TUNE: placeholder used until you log trials
constexpr float TARGET_M       = 1.0f;         // distance for the drive test
constexpr float TEST_SPEED     = 0.35f;        // same as followSpeed
constexpr unsigned long DRIVE_TIMEOUT_MS = 6000;
constexpr unsigned long SPEED_RUN_MS     = 2000;
constexpr unsigned long SETTLE_MS        = 700;   // wait after stopping so the robot finishes rolling
constexpr unsigned long NO_PULSE_MS      = 1500;  // no counts for this long while driving = fault
constexpr float MISMATCH_FRACTION        = 0.10f; // L and R differing by more than this = fault
constexpr int   MAX_TRIALS               = 10;

// ---------------------------------------------------------------------
//  Pulse counters (interrupts). A 100 microsecond glitch filter ignores
//  noise faster than any real wheel can produce.
// ---------------------------------------------------------------------
static volatile uint32_t leftCount   = 0;
static volatile uint32_t rightCount  = 0;
static volatile uint32_t leftLastUs  = 0;
static volatile uint32_t rightLastUs = 0;

static void IRAM_ATTR leftIsr()
{
    const uint32_t now = micros();
    if (now - leftLastUs > 100) { leftCount = leftCount + 1; leftLastUs = now; }
}

static void IRAM_ATTR rightIsr()
{
    const uint32_t now = micros();
    if (now - rightLastUs > 100) { rightCount = rightCount + 1; rightLastUs = now; }
}

static void zeroCounts()
{
    leftCount  = 0;
    rightCount = 0;
}

// Wiring diagnostic: the raw pin level, read directly (no interrupts). If a level
// changes here while a wheel turns but the counts stay 0, the problem is the interrupt
// code. If the level never changes, the problem is wiring or encoder power.
static int      pollLastL = 0, pollLastR = 0;
static uint32_t pollChangesL = 0, pollChangesR = 0;

// ---------------------------------------------------------------------
//  Trial log (hand push)
// ---------------------------------------------------------------------
static uint32_t trialL[MAX_TRIALS];
static uint32_t trialR[MAX_TRIALS];
static int      trials = 0;

static float meanOf(const uint32_t *v, int n)
{
    if (n == 0) return 0.0f;
    float s = 0.0f;
    for (int i = 0; i < n; i++) s += (float)v[i];
    return s / (float)n;
}

// Spread of the trials as a percentage of their mean: (max - min) / mean * 100.
static float spreadPct(const uint32_t *v, int n)
{
    if (n < 2) return 0.0f;
    uint32_t lo = v[0], hi = v[0];
    for (int i = 1; i < n; i++)
    {
        if (v[i] < lo) lo = v[i];
        if (v[i] > hi) hi = v[i];
    }
    const float mean = meanOf(v, n);
    return mean > 0.0f ? 100.0f * (float)(hi - lo) / mean : 0.0f;
}

// Counts per metre from the trials, or the placeholder until a trial is logged.
static float cpmLeft()  { return trials > 0 ? meanOf(trialL, trials) / TRIAL_DIST_M : COUNTS_PER_M; }
static float cpmRight() { return trials > 0 ? meanOf(trialR, trials) / TRIAL_DIST_M : COUNTS_PER_M; }

// ---------------------------------------------------------------------
//  Run state (shared with the action lambdas)
// ---------------------------------------------------------------------
static const char *state   = "idle";
static const char *fault   = "none";
static float runCpmL       = COUNTS_PER_M;   // calibration frozen at the start of a drive
static float runCpmR       = COUNTS_PER_M;
static float resultLm      = 0.0f;           // where the encoders say each side ended up
static float resultRm      = 0.0f;
static float resultSpeedL  = 0.0f;           // m/s from the timed run
static float resultSpeedR  = 0.0f;
static unsigned long tStart = 0;
static unsigned long tStop  = 0;
static unsigned long tLastPulse = 0;
static uint32_t lastSeenL = 0, lastSeenR = 0;

static float leftM()  { return (float)leftCount  / runCpmL; }
static float rightM() { return (float)rightCount / runCpmR; }

// Returns true (and records the fault) when a drive should be aborted.
static bool driveFault()
{
    const uint32_t l = leftCount, r = rightCount;
    const unsigned long now = millis();

    if (l != lastSeenL || r != lastSeenR)
    {
        lastSeenL = l;
        lastSeenR = r;
        tLastPulse = now;
    }
    if (now - tLastPulse > NO_PULSE_MS) { fault = "NO PULSES"; return true; }

    // Only compare the sides once there is enough travel to compare.
    const float lm = leftM(), rm = rightM();
    const float big = lm > rm ? lm : rm;
    const float diff = lm > rm ? lm - rm : rm - lm;
    if (big > 0.20f && diff > MISMATCH_FRACTION * big) { fault = "L/R MISMATCH"; return true; }
    return false;
}

static void beginDrive(const char *name)
{
    zeroCounts();
    runCpmL = cpmLeft();
    runCpmR = cpmRight();
    fault = "none";
    state = name;
    lastSeenL = 0;
    lastSeenR = 0;
    tStart = millis();
    tLastPulse = tStart;
}

static void endDrive()
{
    drive.stop();
    tStop = millis();
    state = "settling";
}

// ---------------------------------------------------------------------
//  OpMode
// ---------------------------------------------------------------------
class EncoderAccuracy : public NRLOpMode
{
public:
    void init() override
    {
        leftMotor.begin();
        rightMotor.begin();
        // INPUT_PULLUP is harmless if the encoder output is push-pull, and required if it is
        // open-collector. Set USE_PULLUP to false to test with a plain input.
        pinMode(LEFT_ENC_PIN, USE_PULLUP ? INPUT_PULLUP : INPUT);
        pinMode(RIGHT_ENC_PIN, USE_PULLUP ? INPUT_PULLUP : INPUT);
        pollLastL = digitalRead(LEFT_ENC_PIN);
        pollLastR = digitalRead(RIGHT_ENC_PIN);
        attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), leftIsr, RISING);
        attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightIsr, RISING);
    }

    void start() override
    {
        zeroCounts();
        trials = 0;
        state = "idle";
        fault = "none";
    }

    void loop() override
    {
        const bool busy = isActionRunning();

        // Raw level polling (diagnostic only; the real counting is done by the interrupts).
        const int lvlL = digitalRead(LEFT_ENC_PIN);
        const int lvlR = digitalRead(RIGHT_ENC_PIN);
        if (lvlL != pollLastL) { pollChangesL++; pollLastL = lvlL; }
        if (lvlR != pollLastR) { pollChangesR++; pollLastR = lvlR; }
        telemetry.addData("L pin level", (float)lvlL);
        telemetry.addData("R pin level", (float)lvlR);
        telemetry.addData("L level changes", (float)pollChangesL);
        telemetry.addData("R level changes", (float)pollChangesR);

        // Y: clear the trial log and stop everything.
        if (gamepad1.justPressed(BTN_Y))
        {
            cancelActions();
            drive.stop();
            trials = 0;
            zeroCounts();
            state = "idle";
            fault = "none";
        }

        if (!busy)
        {
            if (gamepad1.justPressed(BTN_X)) zeroCounts();

            // A: log a hand-push trial (robot is pushed by hand, motors idle).
            if (gamepad1.justPressed(BTN_A) && trials < MAX_TRIALS)
            {
                trialL[trials] = leftCount;
                trialR[trials] = rightCount;
                trials++;
            }

            // D-pad UP: drive until the encoders say TARGET_M.
            if (gamepad1.justPressed(BTN_DPAD_UP))
            {
                beginDrive("driving");
                runAction(sequential({
                    instant([] { drive.drive(FWD_SIGN * TEST_SPEED, 0.0f); }),
                    wait_until([] {
                        return 0.5f * (leftM() + rightM()) >= TARGET_M
                            || millis() - tStart > DRIVE_TIMEOUT_MS
                            || driveFault();
                    }),
                    instant([] { endDrive(); }),
                    wait_until([] { return millis() - tStop > SETTLE_MS; }),
                    instant([] {
                        resultLm = leftM();
                        resultRm = rightM();
                        state = "done";
                    }),
                }));
            }

            // D-pad LEFT: timed run, reports the speed each encoder measured.
            if (gamepad1.justPressed(BTN_DPAD_LEFT))
            {
                beginDrive("speed run");
                runAction(sequential({
                    instant([] { drive.drive(FWD_SIGN * TEST_SPEED, 0.0f); }),
                    wait_until([] {
                        return millis() - tStart >= SPEED_RUN_MS || driveFault();
                    }),
                    instant([] {
                        const float secs = (float)(millis() - tStart) / 1000.0f;
                        resultSpeedL = secs > 0.0f ? leftM() / secs : 0.0f;
                        resultSpeedR = secs > 0.0f ? rightM() / secs : 0.0f;
                        endDrive();
                    }),
                    wait_until([] { return millis() - tStop > SETTLE_MS; }),
                    instant([] {
                        resultLm = leftM();
                        resultRm = rightM();
                        state = "done";
                    }),
                }));
            }
        }

        // ---- Telemetry ----
        telemetry.addData("state", state);
        telemetry.addData("fault", fault);
        telemetry.addData("L count", (float)leftCount);
        telemetry.addData("R count", (float)rightCount);
        telemetry.addData("trials", (float)trials);
        telemetry.addData("L cnt/m", cpmLeft());
        telemetry.addData("R cnt/m", cpmRight());
        telemetry.addData("L spread %", spreadPct(trialL, trials));
        telemetry.addData("R spread %", spreadPct(trialR, trials));
        const float cl = cpmLeft(), cr = cpmRight();
        telemetry.addData("L/R ratio", cr > 0.0f ? cl / cr : 0.0f);
        telemetry.addData("end L m", resultLm);
        telemetry.addData("end R m", resultRm);
        telemetry.addData("speed L m/s", resultSpeedL);
        telemetry.addData("speed R m/s", resultSpeedR);
    }

    void stop() override
    {
        drive.stop();
        detachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN));
        detachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN));
    }
};

REGISTER_OPMODE(EncoderAccuracy, "Encoder Accuracy", TELEOP);