// =====================================================================
//  GLITCHED (Team 190) - gyro turn vs encoder cross-check
//  File: RobotFirmware/opmodes/TurnCheck.cpp
//
//  OpMode "Turn Check" (TELEOP). The robot turns on the spot using the SDK's
//  gyro-based turn (DriveActions::turn). While it turns, the two wheel
//  encoders count how far each side travelled. The screen then compares:
//
//    gyro angle     what the IMU says the robot turned
//    enc angle      what the encoders say, using TRACK_WIDTH_M
//    T_eff          the "effective track width" that would make the two agree
//    L/R            left distance / right distance (about 1.0 for a clean pivot)
//
//  A pivot turn moves the two sides in opposite directions, each one an arc of
//  (track width / 2) x angle, so:
//      enc angle (rad) = (dL + dR) / track width        T_eff = (dL + dR) / gyro angle (rad)
//  Channel A has no direction, but both sides count upward, which is all this needs.
//
//  HOW TO RUN
//   1  Calibrate the encoders first ("Encoder Accuracy", hand-push test) and paste the
//      counts per metre into CPM_LEFT / CPM_RIGHT below.
//   2  Measure TRACK_WIDTH_M: distance between the left and right wheel contact points.
//      A four-wheel skid-steer robot behaves as if it were wider, so expect T_eff to
//      come out larger than your tape measure. Use the mean T_eff shown after a few turns.
//   3  Press Start and keep the robot STILL until "heading ready" says yes (about 0.5 s).
//   4  Stick tape on the floor along the robot's side. Use D-pad RIGHT / LEFT for single
//      90 degree turns and check them against a protractor or a square corner.
//   5  Press A for four 90 degree turns in a row (a square). Compare where the robot
//      ends up with where it started, and with "free heading total" (about +/-360).
//
//  CONTROLS   D-pad RIGHT = turn right 90 deg       D-pad LEFT = turn left 90 deg
//             A = four right turns (a square)        X = zero the counts and the averages
//             Y = stop everything                    LT = stop (engine)
//
//  The gyro turn stops within the SDK's tolerance (DriveTuning::turnToleranceDeg, 2 deg by
//  default). That is a setting, not a measured result: the protractor check is what shows
//  how close it really gets.
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
    .minDrive = 34        // Min Drive test 2026-11-19: left max 31 (pivot) + 3
}};

static HexaDCMotor rightMotor{{
    .dirPin = MOTOR_R_DIR,
    .pwmPin = MOTOR_R_PWM,
    .minDrive = 32        // Min Drive test 2026-11-19: right max 29 (pivot) + 3
}};

static TankDrive    drive(leftMotor, rightMotor);
// Slower, gentler turns so they are easy to read with a protractor. Values are SDK
// DriveTuning fields (DriveTuning.h); the defaults are turnSpeed 255, turnSlowdownDeg 60,
// turnBrakeMs 200. The default 200 ms reverse brake pulse after each turn is what pulled the
// robot back on the spot and left the first turn at 86 deg, so it is shortened here.
// TUNE: if turns end SHORT of 90, set turnBrakeMs to 0; if they end PAST 90, raise it.
static DriveTuning makeTurnTuning()
{
    DriveTuning t;
    t.turnSpeed        = 150;     // max turn PWM (default 255)
    t.turnMinSpeed     = 80;      // floor near the target; stiff wheels need this much to keep moving
    t.turnSlowdownDeg  = 60.0f;   // start slowing 60 deg before the target
    t.turnToleranceDeg = 2.0f;    // stop within 2 deg of the target
    t.turnBrakeMs      = 60;      // short brake pulse (default 200)
    return t;
}

static DriveActions driveActions(drive, makeTurnTuning());

// ---------------------------------------------------------------------
//  Tuning
// ---------------------------------------------------------------------
constexpr int   LEFT_ENC_PIN   = DIGITAL_D3;   // back-left encoder channel A: physical port D2 = GPIO37 (measured with Pin Finder)
constexpr int   RIGHT_ENC_PIN  = DIGITAL_D2;   // back-right encoder channel A: physical port D1 = GPIO36 (measured with Pin Finder)
constexpr bool  USE_PULLUP     = true;         // internal pull-up (see Encoder Accuracy for the wiring notes)
constexpr uint32_t GLITCH_US   = 50;           // ignore edges closer than this
constexpr float CPM_LEFT       = 1300.0f;      // counts per metre, from Encoder Accuracy verify runs (3 runs)
constexpr float CPM_RIGHT      = 1300.0f;      // counts per metre, from Encoder Accuracy verify runs (3 runs)
constexpr float TRACK_WIDTH_M  = 0.316f;       // measured by hand, wheel centre to wheel centre (31.6 cm); T_eff will show the effective value
constexpr unsigned long SETTLE_MS = 700;       // wait after each turn so the robot stops rolling

// ---------------------------------------------------------------------
//  Pulse counters (interrupts)
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
//  Results
// ---------------------------------------------------------------------
constexpr float RAD_TO_DEG_F = 57.2957795f;

static const char *state = "idle";
static int   turnsDone   = 0;
static float lastGyroDeg = 0.0f;    // what the IMU says the last turn was
static float lastLm      = 0.0f;    // left distance during the last turn
static float lastRm      = 0.0f;    // right distance during the last turn
static float lastEncDeg  = 0.0f;    // angle from the encoders using TRACK_WIDTH_M
static float lastTeff    = 0.0f;    // effective track width from the last turn
static float lastLR      = 0.0f;    // left / right distance
static float sumTeff     = 0.0f;
static int   nTeff       = 0;
static float freeStart   = 0.0f;    // NRLComms::getHeadingFree() when the OpMode started
static unsigned long tStop = 0;     // when the last turn finished

static void captureResult()
{
    const float gyroDeg = fabsf(NRLComms::getHeading());   // turn() zeroes the heading first, so this is the turn made
    const float dL = (float)leftCount  / CPM_LEFT;
    const float dR = (float)rightCount / CPM_RIGHT;

    lastGyroDeg = gyroDeg;
    lastLm      = dL;
    lastRm      = dR;
    lastEncDeg  = ((dL + dR) / TRACK_WIDTH_M) * RAD_TO_DEG_F;
    lastLR      = dR > 0.0f ? dL / dR : 0.0f;

    if (gyroDeg > 1.0f)
    {
        lastTeff = (dL + dR) / (gyroDeg / RAD_TO_DEG_F);
        sumTeff += lastTeff;
        nTeff++;
    }
    turnsDone++;
    state = "done";
}

// One gyro turn, then wait for the robot to settle, then record the encoder cross-check.
static ActionPtr oneTurn(float degrees)
{
    return sequential({
        instant([] { zeroCounts(); state = "turning"; }),
        driveActions.turn(degrees),
        instant([] { drive.stop(); tStop = millis(); state = "settling"; }),
        wait_until([] { return millis() - tStop > SETTLE_MS; }),
        instant([] { captureResult(); }),
    });
}

// ---------------------------------------------------------------------
//  OpMode
// ---------------------------------------------------------------------
class TurnCheck : public NRLOpMode
{
public:
    void init() override
    {
        leftMotor.begin();
        rightMotor.begin();
        pinMode(LEFT_ENC_PIN,  USE_PULLUP ? INPUT_PULLUP : INPUT);
        pinMode(RIGHT_ENC_PIN, USE_PULLUP ? INPUT_PULLUP : INPUT);
        attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN),  leftIsr,  RISING);
        attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightIsr, RISING);
    }

    void start() override
    {
        zeroCounts();
        turnsDone = 0;
        sumTeff   = 0.0f;
        nTeff     = 0;
        state     = "idle";
        NRLComms::resetHeading();                 // re-collect the gyro bias: keep the robot still
        freeStart = NRLComms::getHeadingFree();
    }

    void loop() override
    {
        const bool busy  = isActionRunning();
        const bool ready = NRLComms::isHeadingReady();

        if (gamepad1.justPressed(BTN_Y))
        {
            cancelActions();
            drive.stop();
            state = "stopped";
        }

        if (!busy && ready)
        {
            // Measured on the robot: turn(+90) turns the robot LEFT, so the signs are swapped
            // to make D-pad RIGHT turn right.
            if (gamepad1.justPressed(BTN_DPAD_RIGHT)) runAction(oneTurn(-90.0f));
            if (gamepad1.justPressed(BTN_DPAD_LEFT))  runAction(oneTurn(90.0f));
            if (gamepad1.justPressed(BTN_A))
            {
                runAction(sequential({ oneTurn(-90.0f), oneTurn(-90.0f), oneTurn(-90.0f), oneTurn(-90.0f) }));
            }
            if (gamepad1.justPressed(BTN_X))
            {
                zeroCounts();
                turnsDone = 0;
                sumTeff   = 0.0f;
                nTeff     = 0;
                freeStart = NRLComms::getHeadingFree();
                state     = "idle";
            }
        }

        // The Hub screen shows about 8 lines, so the important ones come first.
        telemetry.addData("state", ready ? state : "WAIT keep still");
        telemetry.addData("gyro deg", lastGyroDeg);
        telemetry.addData("enc deg", lastEncDeg);
        telemetry.addData("L m", lastLm);
        telemetry.addData("R m", lastRm);
        telemetry.addData("T_eff m", lastTeff);
        telemetry.addData("T_eff mean", nTeff > 0 ? sumTeff / (float)nTeff : 0.0f);
        telemetry.addData("turns done", (float)turnsDone);
        telemetry.addData("L/R", lastLR);
        telemetry.addData("free hdg total", NRLComms::getHeadingFree() - freeStart);
        telemetry.addData("L count", (float)leftCount);
        telemetry.addData("R count", (float)rightCount);
    }

    void stop() override
    {
        drive.stop();
        detachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN));
        detachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN));
    }
};

REGISTER_OPMODE(TurnCheck, "Turn Check", TELEOP);