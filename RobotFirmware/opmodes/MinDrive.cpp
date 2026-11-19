// =====================================================================
//  GLITCHED (Team 190) - minDrive finder
//  File: RobotFirmware/opmodes/MinDrive.cpp
//
//  OpMode "Min Drive" (TELEOP). Finds the lowest motor power (PWM out of 255) at which each
//  side of the robot really moves, using the wheel encoders, in steps of 1 instead of the
//  calibration screen's steps of 25.
//
//  HOW TO USE  (robot ON THE FLOOR on the arena mat, about 1 m clear around it)
//    D-pad UP     straight test: both sides ramp up forward together
//    D-pad LEFT   pivot-left test: left side backward, right side forward (like a corner turn)
//    D-pad RIGHT  pivot-right test
//    Y            stop
//  Each test raises the power by 1 every RAMP_STEP_MS from START_PWM, and notes the power at
//  which each side's encoder starts counting steadily. It stops once both sides moved.
//  Run each test 2-3 times (put the robot back each time) and send me the numbers.
//
//  SCREEN (8 lines)
//    state, pwm (power right now), then the result of each test for the left and right side.
//    "-" = not measured yet. The minDrive to use = the highest number for that side + 3.
//
//  Motors: same directions as the autos (left normal, right flipped) but minDrive = 0, so the
//  power on screen is the raw PWM.
//
//  STATUS: written against the NRL SDK v1.5.0 headers. NOT compiled or run on the robot yet.
// =====================================================================
#include "NRL.h"

static HexaDCMotor leftMotor{{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .minDrive = 0 }};
static HexaDCMotor rightMotor{{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true, .minDrive = 0 }};

constexpr int      LEFT_ENC_PIN  = DIGITAL_D3;   // back-left:  board port D2 (GPIO37)
constexpr int      RIGHT_ENC_PIN = DIGITAL_D2;   // back-right: board port D1 (GPIO36)
constexpr uint32_t GLITCH_US     = 50;

constexpr int           START_PWM    = 15;
constexpr int           MAX_PWM      = 200;
constexpr unsigned long RAMP_STEP_MS = 120;   // +1 PWM this often
constexpr uint32_t      MOVE_COUNTS  = 2;     // counts per step that mean "moving"
constexpr int           MOVE_STEPS   = 2;     // ...for this many steps in a row

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

enum Test { NONE = -1, STRAIGHT = 0, PIVOT_L = 1, PIVOT_R = 2 };

static Test          test = NONE;
static const char   *state = "idle";
static int           pwm = 0;
static unsigned long tStep = 0;
static uint32_t      prevL = 0, prevR = 0;
static int           goodL = 0, goodR = 0;
static int           resL[3] = {-1, -1, -1};   // per test: PWM at which the left side moved
static int           resR[3] = {-1, -1, -1};

static void allStop()
{
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    leftMotor.stop();
    rightMotor.stop();
}

static void startTest(Test t)
{
    test = t;
    pwm = START_PWM;
    tStep = millis();
    prevL = encL;
    prevR = encR;
    goodL = goodR = 0;
    resL[t] = -1;
    resR[t] = -1;
    state = (t == STRAIGHT) ? "straight" : (t == PIVOT_L ? "pivot left" : "pivot right");
}

static void apply()
{
    // Left / right sign for each test: straight = both forward; pivot left = left back, right fwd.
    int sl = 1, sr = 1;
    if (test == PIVOT_L) { sl = -1; sr = 1; }
    if (test == PIVOT_R) { sl = 1;  sr = -1; }
    leftMotor.setSpeed(resL[test] < 0 || resR[test] < 0 ? sl * pwm : 0);
    rightMotor.setSpeed(resL[test] < 0 || resR[test] < 0 ? sr * pwm : 0);
}

static void fmt(char *out, size_t n, int v)
{
    if (v < 0) snprintf(out, n, "-");
    else       snprintf(out, n, "%d", v);
}

class MinDrive : public NRLOpMode
{
public:
    void init() override        // INIT does not move anything (R8.11)
    {
        leftMotor.begin();
        rightMotor.begin();
        pinMode(LEFT_ENC_PIN, INPUT_PULLUP);
        pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), leftIsr, RISING);
        attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightIsr, RISING);
    }

    void start() override
    {
        test = NONE;
        state = "idle";
        pwm = 0;
        allStop();
    }

    void loop() override
    {
        if (gamepad1.justPressed(BTN_Y)) { test = NONE; state = "stopped"; pwm = 0; allStop(); }

        if (test == NONE)
        {
            if (gamepad1.justPressed(BTN_DPAD_UP))    startTest(STRAIGHT);
            if (gamepad1.justPressed(BTN_DPAD_LEFT))  startTest(PIVOT_L);
            if (gamepad1.justPressed(BTN_DPAD_RIGHT)) startTest(PIVOT_R);
        }

        if (test != NONE)
        {
            const unsigned long now = millis();
            if (now - tStep >= RAMP_STEP_MS)
            {
                tStep = now;
                const uint32_t l = encL, r = encR;
                // A side is "moving" once it counts MOVE_COUNTS per step for MOVE_STEPS steps.
                goodL = (l - prevL >= MOVE_COUNTS) ? goodL + 1 : 0;
                goodR = (r - prevR >= MOVE_COUNTS) ? goodR + 1 : 0;
                prevL = l;
                prevR = r;
                if (resL[test] < 0 && goodL >= MOVE_STEPS) resL[test] = pwm - (MOVE_STEPS - 1);
                if (resR[test] < 0 && goodR >= MOVE_STEPS) resR[test] = pwm - (MOVE_STEPS - 1);

                if ((resL[test] >= 0 && resR[test] >= 0) || pwm >= MAX_PWM)
                {
                    state = (pwm >= MAX_PWM) ? "MAX reached" : "done";
                    test = NONE;
                    allStop();
                }
                else
                {
                    pwm++;
                }
            }
            if (test != NONE) apply();
        }

        static char b[6][8];
        fmt(b[0], sizeof(b[0]), resL[STRAIGHT]);
        fmt(b[1], sizeof(b[1]), resR[STRAIGHT]);
        fmt(b[2], sizeof(b[2]), resL[PIVOT_L]);
        fmt(b[3], sizeof(b[3]), resR[PIVOT_L]);
        fmt(b[4], sizeof(b[4]), resL[PIVOT_R]);
        fmt(b[5], sizeof(b[5]), resR[PIVOT_R]);

        telemetry.addData("state", state);
        telemetry.addData("pwm", (float)pwm);
        telemetry.addData("L straight", b[0]);
        telemetry.addData("R straight", b[1]);
        telemetry.addData("L pivot L", b[2]);
        telemetry.addData("R pivot L", b[3]);
        telemetry.addData("L pivot R", b[4]);
        telemetry.addData("R pivot R", b[5]);
    }

    void stop() override
    {
        allStop();
        detachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN));
        detachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN));
    }
};

REGISTER_OPMODE(MinDrive, "Min Drive", TELEOP);