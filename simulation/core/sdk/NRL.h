// =====================================================================
//  SIMULATION STAND-IN for the NRL SDK v1.5.0 public API.
//
//  The real SDK bodies live in Xtensa (ESP32-S3) static libraries and cannot run on a
//  desktop. This header reproduces the PUBLIC declarations that robot opmodes compile
//  against (same names, same struct fields in the same order, same defaults), so
//  RobotFirmware/opmodes/AutoPark.cpp compiles here UNCHANGED.
//
//  Behaviour is implemented in ../sim_core.cpp from each header's documentation.
//  Anything a header does not specify is marked "ASSUMPTION" there.
//  Source of truth for every declaration below:
//    RobotFirmware/lib/HexaNRL/src/*.h, lib/HexaLineArray/src/HexaLineArray.h,
//    lib/HexaServos/src/HexaDCMotor.h
// =====================================================================
#pragma once
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

#define IRAM_ATTR

// ---------------- Arduino core (subset) ----------------
unsigned long millis();
unsigned long micros();
enum { INPUT = 0x01, OUTPUT = 0x03, INPUT_PULLUP = 0x05 };
enum { RISING = 0x01, FALLING = 0x02, CHANGE = 0x03 };
enum { LOW = 0, HIGH = 1 };
void pinMode(uint8_t pin, uint8_t mode);
int  digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, uint8_t val);
int  analogRead(uint8_t pin);
void delayMicroseconds(unsigned int us);
int  digitalPinToInterrupt(int pin);
void attachInterrupt(int irq, void (*isr)(), int mode);
void detachInterrupt(int irq);

// ---------------- BoardPins.h (values copied) ----------------
constexpr int MOTOR_L_DIR = 4, MOTOR_L_PWM = 5, MOTOR_R_DIR = 6, MOTOR_R_PWM = 7;   // placeholders; pins unused in sim
constexpr int SERVO_1 = 8, SERVO_2 = 9;
constexpr int DIGITAL_D1 = 35;   // J14 PIN2  (BoardPins.h)
constexpr int DIGITAL_D2 = 36;   // J15 PIN2  (BoardPins.h)
constexpr int DIGITAL_D3 = 37;   // J16 PIN2  (BoardPins.h)

// ---------------- HexaDCMotor.h ----------------
struct HexaDCMotorConfig {
    uint8_t dirPin;
    uint8_t pwmPin;
    bool    flipped = false;
    uint8_t minDrive = 0;
};
class HexaDCMotor {
public:
    HexaDCMotor(const HexaDCMotorConfig &config);
    bool begin();
    void setSpeed(int speed);
    int  getSpeed() const { return m_currentSpeed; }
    void stop() { setSpeed(0); }
    void setFlipped(bool f) { m_cfg.flipped = f; }
    bool isReady() const { return m_ready; }
    static void stopAll();
    static void clearRegistry();
    // --- simulator side ---
    int  simPwm() const { return m_pwm; }        // signed duty actually applied (after deadband + minDrive)
    const HexaDCMotorConfig &simConfig() const { return m_cfg; }
private:
    HexaDCMotorConfig m_cfg;
    int  m_currentSpeed = 0;
    int  m_pwm = 0;
    bool m_ready = false;
    static constexpr int MAX_SPEED = 255;
    static constexpr int MOTOR_DEADBAND = 10;
};

// ---------------- TankDrive.h ----------------
class TankDrive {
public:
    TankDrive(HexaDCMotor &left, HexaDCMotor &right) : _left(left), _right(right) {}
    ~TankDrive();
    void  drive(float forward, float turn);
    void  stop();
    void  setScale(float scale) { _scale = scale; }
    float getScale() const { return _scale; }
    void  setRampRate(float maxDeltaPerTick) { _maxDelta = maxDeltaPerTick; }
    static void tickAll();
    static void stopAll();
    static void clearRegistry();
    // --- simulator side ---
    float simOutL() const { return _curL; }
    float simOutR() const { return _curR; }
    HexaDCMotor &simLeft() { return _left; }
    HexaDCMotor &simRight() { return _right; }
private:
    HexaDCMotor &_left;
    HexaDCMotor &_right;
    float _scale = 1.0f, _maxDelta = 0.1f, _curL = 0, _curR = 0, _targetL = 0, _targetR = 0;
    void _register();
    void _tick();
};

// ---------------- HexaPID.h ----------------
struct HexaPIDConfig {
    float kP = 0.0f;
    float kI = 0.0f;
    float kD = 0.0f;
    float outMin = -1.0f;
    float outMax = 1.0f;
    float iMax = 0.5f;
    float dAlpha = 0.0f;
    uint16_t maxDtMs = 50;
};
class HexaPID {
public:
    HexaPID() : HexaPID(HexaPIDConfig{}) {}
    explicit HexaPID(const HexaPIDConfig &config);
    float update(float error);
    float update(float target, float measured) { return update(target - measured); }
    float updateDt(float error, float dtSec);
    void  reset();
    void  setGains(float kP, float kI, float kD) { m_cfg.kP = kP; m_cfg.kI = kI; m_cfg.kD = kD; }
    void  setOutputLimits(float a, float b) { m_cfg.outMin = a; m_cfg.outMax = b; }
    void  setConfig(const HexaPIDConfig &c) { m_cfg = c; }
    const HexaPIDConfig &config() const { return m_cfg; }
    float p() const { return m_p; }
    float i() const { return m_i; }
    float d() const { return m_d; }
    float output() const { return m_out; }
    float error() const { return m_lastErr; }
private:
    HexaPIDConfig m_cfg;
    float m_iSum = 0, m_lastErr = 0, m_dFilt = 0;
    bool  m_primed = false;
    uint32_t m_lastUs = 0;
    float m_p = 0, m_i = 0, m_d = 0, m_out = 0;
};

// ---------------- HexaLineArray.h ----------------
static constexpr uint8_t HEXA_LINE_CHANNELS = 8;
static constexpr int HEXA_LINE_CENTRE = 3500;
enum class LinePattern : uint8_t { LOST, LINE, LEFT_BRANCH, RIGHT_BRANCH, TERMINAL };
const char *linePatternName(LinePattern p);
enum class LineRef : uint8_t { WHITE, BLACK, GREY };
struct HexaLineArrayConfig {
    uint8_t  selPins[3] = {18, 15, 16};
    uint8_t  sigPin = 17;
    uint8_t  settleUs = 15;
    uint16_t darkThreshold = 250;
    uint16_t weightThreshold = 250;
    uint16_t minSpan = 300;
    uint16_t weakSpan = 800;
    uint16_t lostStopMs = 1200;
    uint16_t greyHardLine = 700;
    uint16_t greyMinGap = 80;
    uint8_t  greyMinChannels = 5;
    bool     reversed = false;
};
inline HexaLineArrayConfig hexaLineConfig() { return HexaLineArrayConfig{}; }
class HexaLineArray {
public:
    HexaLineArray() : HexaLineArray(HexaLineArrayConfig{}) {}
    explicit HexaLineArray(const HexaLineArrayConfig &config);
    bool begin();
    bool isReady() const { return m_ready; }
    void tick();
    static void tickAll();
    static void clearRegistry();
    uint16_t readRaw(uint8_t ch) const { return ch < 8 ? m_raw[ch] : 0; }
    uint16_t read(uint8_t ch) const { return ch < 8 ? m_norm[ch] : 0; }
    uint8_t  mask() const { return m_mask; }
    uint8_t  count() const;
    int  position() const { return m_position; }
    int  error() const { return (m_position < 0) ? 0 : m_position - HEXA_LINE_CENTRE; }
    bool onLine() const { return m_onLine; }
    bool lost() const { return m_hardLost; }
    int  lastSeenSide() const { return m_lastSeenSide; }
    int  lastSeenMagnitude() const { return m_lastSeenMag; }
    LinePattern pattern() const { return m_pattern; }
    bool onGrey() const { return false; }
    bool isCalibrated() const { return m_calibrated; }
    uint16_t span(uint8_t ch) const { return ch < 8 ? (uint16_t)(m_calMax[ch] - m_calMin[ch]) : 0; }
    bool channelOk(uint8_t ch) const { return span(ch) >= m_cfg.minSpan; }
    const HexaLineArrayConfig &config() const { return m_cfg; }
    // --- simulator side ---
    void simCalibrate(const uint16_t white[8], const uint16_t black[8]);
private:
    HexaLineArrayConfig m_cfg;
    bool m_ready = false, m_calibrated = false, m_onLine = false, m_hardLost = false;
    uint16_t m_raw[8] = {}, m_norm[8] = {}, m_calMin[8] = {}, m_calMax[8] = {};
    uint8_t m_mask = 0;
    int m_position = HEXA_LINE_CENTRE, m_lastSeenSide = 0, m_lastSeenMag = 0;
    unsigned long m_lineSeenMs = 0;
    LinePattern m_pattern = LinePattern::LOST;
};

// ---------------- NRLComms.h (heading API) ----------------
class NRLComms {
public:
    static float getHeading();
    static float getHeadingFree();
    static bool  isHeadingReady();
    static void  zeroHeading();
    static void  resetHeading();
};

// ---------------- NRLAction.h ----------------
class Action {
public:
    virtual ~Action() = default;
    virtual bool run() = 0;
    virtual void reset() {}
};
using ActionPtr = std::shared_ptr<Action>;
ActionPtr instant(std::function<void()> fn);
ActionPtr sequential(std::initializer_list<ActionPtr> actions);
ActionPtr sequential(std::vector<ActionPtr> actions);
ActionPtr parallel(std::initializer_list<ActionPtr> actions);
ActionPtr sleep_ms(unsigned long ms);
ActionPtr wait_until(std::function<bool()> cond);
ActionPtr repeat(ActionPtr inner);

// ---------------- LineTuning.h (fields used by opmodes; defaults copied) ----------------
struct LineTuning {
    HexaPIDConfig pid{.kP = 0.50f, .kI = 0.0f, .kD = 0.04f};
    float followSpeed = 0.35f;
    float junctionSpeed = 0.25f;
    float rampRate = 0.25f;
    int   steerSign = 1;
    float startSpeed = 0.35f;
    uint16_t startTimeoutMs = 4000;
    uint16_t hardLine = 700;
    uint16_t boxConfirmMs = 150;
    uint8_t  boxLeaveTicks = 2;
    uint16_t parkMs = 300;
    float parkSpeed = 0.25f;
    unsigned long pivotMs = 60;
    float pivotSpeed = 0.25f, turnSpeed = 0.45f, turnSlowSpeed = 0.30f;
    unsigned long turnMinMs = 200;
};

// ---------------- LineActions.h (subset used by opmodes) ----------------
enum class LineSide : uint8_t { ANY, LEFT, RIGHT };
enum class LineFault : uint8_t { NONE, NOT_CALIBRATED, LINE_LOST, UNEXPECTED_END, TURN_TIMEOUT, NO_GREY,
                                 NO_LINE, WRONG_JUNCTION, NO_ROUTE, NO_BRANCH };
const char *lineFaultName(LineFault f);
struct LineJunction { bool left = false, right = false, straight = false; };
class LineActions {
public:
    LineActions(TankDrive &drive, HexaLineArray &line, LineTuning tuning = LineTuning{});
    ActionPtr leaveBox();
    LineFault fault() const { return _fault; }
    const char *faultName() const { return lineFaultName(_fault); }
    void clearFault() { _fault = LineFault::NONE; }
    void resetCounts() {}
    void clearRoute() {}
    int  junctionCount() const { return 0; }
    int  cornerCount() const { return 0; }
    float lastTurnDegrees() const { return 0.0f; }
    const LineJunction &lastJunction() const { return _lj; }
    const LineTuning &tuning() const { return _tuning; }
    // The fallback OpModes' builder is accepted but NOT simulated (Park Run / P1-D1 Run do not use it).
    struct Builder {
        Builder &then(ActionPtr) { return *this; }
        Builder &leaveBox() { return *this; }
        Builder &followUntil(std::function<bool()>) { return *this; }
        Builder &park(long) { return *this; }
        Builder &followToJunction(LineSide = LineSide::ANY, int = 1) { return *this; }
        ActionPtr build();
    };
    Builder auto_() { return Builder{}; }
    // --- simulator side ---
    TankDrive &simDrive() { return _drive; }
    HexaLineArray &simLine() { return _line; }
    LineFault _fault = LineFault::NONE;
private:
    TankDrive &_drive;
    HexaLineArray &_line;
    LineTuning _tuning;
    LineJunction _lj;
};

// ---------------- Gamepad / telemetry / OpMode ----------------
enum { BTN_A, BTN_B, BTN_X, BTN_Y, BTN_LB, BTN_RB, BTN_LT, BTN_RT, BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_DPAD_LEFT, BTN_DPAD_RIGHT };
struct NRLGamepadSim {
    bool justPressed(int) const { return false; }
    bool isPressed(int) const { return false; }
    float leftX() const { return 0; } float leftY() const { return 0; }
    float rightX() const { return 0; } float rightY() const { return 0; }
};
extern NRLGamepadSim gamepad1;

// HexaLED (real: lib/HexaLED/src/HexaLED.h, global userLed from RobotConfig.h). No effect in the simulator.
class HexaLED {
public:
    void setSolid(uint8_t, uint8_t, uint8_t) {}
    void setBlink(uint8_t, uint8_t, uint8_t, uint16_t) {}
    void setPulse(uint8_t, uint8_t, uint8_t, uint16_t) {}
    void setOff() {}
};
inline HexaLED userLed;

class NRLTelemetry {
public:
    void addData(const char *key, float value);
    void addData(const char *key, const char *value);
};

class NRLOpMode {
public:
    virtual ~NRLOpMode() = default;
    NRLTelemetry telemetry;
    virtual void init() {}
    virtual void start() {}
    virtual void loop() {}
    virtual void stop() {}
    void runAction(ActionPtr action);
    void cancelActions();
    bool isActionRunning() const;
    void _advanceAction();
private:
    std::vector<ActionPtr> _actions, _pending;
    bool _advancing = false, _cancelRequested = false;
};

enum class OpModeType { TELEOP, AUTO };
void simRegisterOpMode(const char *name, OpModeType type, NRLOpMode *(*factory)());
template <typename T>
class OpModeRegistrar {
public:
    OpModeRegistrar(const char *name, OpModeType type, bool = false, int = 1000) {
        simRegisterOpMode(name, type, []() -> NRLOpMode * { return new T(); });
    }
};
#define REGISTER_OPMODE(ClassName, DisplayName, Type) \
    static OpModeRegistrar<ClassName> _reg_##ClassName(DisplayName, OpModeType::Type, false)
