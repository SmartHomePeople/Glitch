// =====================================================================
//  GLITCHED (Team 190) - encoder accuracy test (motor-driven only, no hand pushing)
//  File: RobotFirmware/opmodes/EncoderAccuracy.cpp
//
//  OpMode "Encoder Accuracy" (TELEOP). Two encoders, channel A only, counted by interrupts.
//
//  Wiring (board labels, measured with Pin Finder):
//    back-left  encoder channel A -> port D2 (GPIO37)
//    back-right encoder channel A -> port D1 (GPIO36)
//    both encoders: Vcc (pin 3) -> 5V, GND (pin 4) -> GND
//
//  STEP 1 - CALIBRATION RUNS (D-pad UP). The robot drives forward for CAL_RUN_MS at
//    TEST_SPEED, stops, waits for the rolling to finish, and keeps showing the final
//    "L count" and "R count". Measure the real distance it travelled with a tape and
//    write down the counts. Counts per metre = count / distance (metres). Repeat 5 times,
//    putting the robot back on the start line each time. Keep 2.5 m clear ahead.
//    Then send me the numbers: I work out CPM_LEFT / CPM_RIGHT, you paste them below.
//
//  STEP 2 - VERIFY RUNS (D-pad RIGHT). Needs CPM_LEFT and CPM_RIGHT filled in. The robot
//    drives until the encoders say TARGET_M, stops, and shows "end L m" / "end R m".
//    Measure the real distance with a tape and compare. Repeat 5 times.
//
//  CONTROLS   D-pad UP = calibration run     D-pad RIGHT = verify run (drive TARGET_M)
//             X = zero the counts            Y = stop and clear the run log
//             LT = stop (engine)
//
//  SCREEN (8 lines)
//    state      idle / cal run / verify run / settling / done, or SET CPM FIRST / NO PULSES / TIMEOUT
//    L count    left encoder pulses (live during a run, final value kept after it)
//    R count    right encoder pulses
//    runs       completed calibration runs
//    mean L     average final L count over the calibration runs
//    mean R     average final R count
//    end L m    left distance the encoder reported at the end of the last verify run
//    end R m    right distance the encoder reported at the end of the last verify run
//
//  If the robot drives BACKWARD, change FWD_SIGN. Every run stops itself on a timeout,
//  or if neither encoder counts for NO_PULSE_MS.
//
//  STATUS: written against the NRL SDK v1.5.0 headers. NOT compiled or run on the robot yet.
// =====================================================================
#include "NRL.h"

// ---------------------------------------------------------------------
//  Hardware (same drive settings as the other OpModes)
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
constexpr int   LEFT_ENC_PIN   = DIGITAL_D3;   // back-left channel A:  board port D2 = GPIO37
constexpr int   RIGHT_ENC_PIN  = DIGITAL_D2;   // back-right channel A: board port D1 = GPIO36
constexpr float FWD_SIGN       = -1.0f;        // the robot drove backward with +1, so forward is -1
constexpr float TEST_SPEED     = 0.35f;        // same as the line-follow speed
constexpr unsigned long CAL_RUN_MS = 2000;     // length of a calibration run

// Counts per metre. 0 means "not calibrated yet" and blocks the verify run.
// From 10 calibration runs (2026-11-18): average of both sides' counts / centre distance
// = 1360 counts per metre, run-to-run spread about 1.3 %. The same value is used for both
// sides (assumes identical wheels and gearing), so a robot that veers still reports the
// centre distance. Re-measure after any wheel, motor or encoder change.
// Verify runs (3, 2026-11-19): encoders averaged 1.012 m against a tape average of 1.06 m,
// so the value was lowered to 1300 (1360 x 1.012 / 1.06). Based on 3 runs only.
constexpr float CPM_LEFT       = 1300.0f;
constexpr float CPM_RIGHT      = 1300.0f;

constexpr float TARGET_M       = 1.0f;         // distance for the verify run
constexpr uint32_t GLITCH_US   = 50;           // ignore edges closer together than this
constexpr unsigned long VERIFY_TIMEOUT_MS = 6000;
constexpr unsigned long SETTLE_MS         = 700;    // wait after stopping so the robot finishes rolling
constexpr unsigned long NO_PULSE_MS       = 1500;   // no counts for this long while driving = fault

// ---------------------------------------------------------------------
//  Pulse counters (interrupts). The Hub pin's pull-up holds the line high; the encoder
//  pulls it low, so each rising edge is one pulse.
// ---------------------------------------------------------------------
static volatile uint32_t leftCount   = 0;
static volatile uint32_t rightCount  = 0;
static volatile uint32_t leftLastUs  = 0;
static volatile uint32_t rightLastUs = 0;

static void IRAM_ATTR leftIsr()
{
    const uint32_t now = micros();
    if (now - leftLastUs > GLITCH_US) { leftCount = leftCount + 1; leftLastUs = now; }
}

static void IRAM_ATTR rightIsr()
{
    const uint32_t now = micros();
    if (now - rightLastUs > GLITCH_US) { rightCount = rightCount + 1; rightLastUs = now; }
}

static void zeroCounts()
{
    leftCount  = 0;
    rightCount = 0;
}

// ---------------------------------------------------------------------
//  Run state (shared with the action lambdas)
// ---------------------------------------------------------------------
static const char *state    = "idle";
static const char *faultMsg = "";
static bool   faulted       = false;
static int    runs          = 0;      // completed calibration runs
static float  sumL          = 0.0f;   // sum of final counts over the calibration runs
static float  sumR          = 0.0f;
static float  endLm         = 0.0f;   // verify run result
static float  endRm         = 0.0f;
static unsigned long tStart     = 0;
static unsigned long tStop      = 0;
static unsigned long tLastPulse = 0;
static uint32_t lastSeenL = 0, lastSeenR = 0;

static float leftM()  { return (float)leftCount  / CPM_LEFT; }
static float rightM() { return (float)rightCount / CPM_RIGHT; }
static float distM()  { return 0.5f * (leftM() + rightM()); }   // average of both sides

static void beginRun(const char *name)
{
    zeroCounts();
    faulted = false;
    faultMsg = "";
    lastSeenL = 0;
    lastSeenR = 0;
    tStart = millis();
    tLastPulse = tStart;
    state = name;
}

// True when neither encoder has counted for NO_PULSE_MS (and records the fault).
static bool noPulses()
{
    const unsigned long now = millis();
    const uint32_t l = leftCount, r = rightCount;
    if (l != lastSeenL || r != lastSeenR) { lastSeenL = l; lastSeenR = r; tLastPulse = now; }
    if (now - tLastPulse > NO_PULSE_MS) { faulted = true; faultMsg = "NO PULSES"; return true; }
    return false;
}

// ---------------------------------------------------------------------
//  Actions
// ---------------------------------------------------------------------
static ActionPtr calibrationRun()
{
    return sequential({
        instant([] { beginRun("cal run"); drive.drive(FWD_SIGN * TEST_SPEED, 0.0f); }),
        wait_until([] { return millis() - tStart >= CAL_RUN_MS || noPulses(); }),
        instant([] { drive.stop(); tStop = millis(); state = "settling"; }),
        wait_until([] { return millis() - tStop > SETTLE_MS; }),
        instant([] {
            if (!faulted) { sumL += (float)leftCount; sumR += (float)rightCount; runs++; }
            state = faulted ? faultMsg : "done";
        }),
    });
}

static ActionPtr verifyRun()
{
    return sequential({
        instant([] { beginRun("verify run"); drive.drive(FWD_SIGN * TEST_SPEED, 0.0f); }),
        wait_until([] {
            if (distM() >= TARGET_M) return true;
            if (millis() - tStart > VERIFY_TIMEOUT_MS) { faulted = true; faultMsg = "TIMEOUT"; return true; }
            return noPulses();
        }),
        instant([] { drive.stop(); tStop = millis(); state = "settling"; }),
        wait_until([] { return millis() - tStop > SETTLE_MS; }),
        instant([] {
            endLm = leftM();
            endRm = rightM();
            state = faulted ? faultMsg : "done";
        }),
    });
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
        pinMode(LEFT_ENC_PIN,  INPUT_PULLUP);
        pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN),  leftIsr,  RISING);
        attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightIsr, RISING);
    }

    void start() override
    {
        zeroCounts();
        runs = 0;
        sumL = 0.0f;
        sumR = 0.0f;
        endLm = 0.0f;
        endRm = 0.0f;
        state = "idle";
    }

    void loop() override
    {
        // Y: stop everything and clear the run log.
        if (gamepad1.justPressed(BTN_Y))
        {
            cancelActions();
            drive.stop();
            zeroCounts();
            runs = 0;
            sumL = 0.0f;
            sumR = 0.0f;
            endLm = 0.0f;
            endRm = 0.0f;
            state = "idle";
        }

        if (!isActionRunning())
        {
            if (gamepad1.justPressed(BTN_X)) zeroCounts();

            // D-pad UP: calibration run (timed).
            if (gamepad1.justPressed(BTN_DPAD_UP)) runAction(calibrationRun());

            // D-pad RIGHT: verify run (drive TARGET_M using the calibrated counts per metre).
            if (gamepad1.justPressed(BTN_DPAD_RIGHT))
            {
                if (CPM_LEFT <= 0.0f || CPM_RIGHT <= 0.0f) state = "SET CPM FIRST";
                else runAction(verifyRun());
            }
        }

        // ---- Screen: exactly 8 lines ----
        telemetry.addData("state", state);
        telemetry.addData("L count", (float)leftCount);
        telemetry.addData("R count", (float)rightCount);
        telemetry.addData("runs", (float)runs);
        telemetry.addData("mean L", runs > 0 ? sumL / (float)runs : 0.0f);
        telemetry.addData("mean R", runs > 0 ? sumR / (float)runs : 0.0f);
        telemetry.addData("end L m", endLm);
        telemetry.addData("end R m", endRm);
    }

    void stop() override
    {
        drive.stop();
        detachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN));
        detachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN));
    }
};

REGISTER_OPMODE(EncoderAccuracy, "Encoder Accuracy", TELEOP);  