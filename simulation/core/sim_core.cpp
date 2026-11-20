// =====================================================================
//  GLITCHED SIL simulator core.
//
//  Compiled together with an UNCHANGED copy of RobotFirmware/opmodes/AutoPark.cpp into a
//  shared library. Implements:
//    1. the SDK runtime the opmode links against (sdk/NRL.h), from the headers' docs;
//    2. the robot physics (differential / skid-steer drive, motors with stiction);
//    3. virtual sensors: 8-channel IR array over the arena tape, encoders, gyro;
//    4. fault injection;
//    5. a C API driven from Python (simulation/simulator/engine.py).
//
//  Every behaviour the SDK headers do not pin down is marked ASSUMPTION.
//  Units inside: metres, seconds, radians (heading CCW-positive, 0 = +x / east),
//  except where a name says _mm, _ms, _deg.
// =====================================================================
#include "sdk/NRL.h"
#include <algorithm>
#include <map>
#include <random>
#include <sstream>
#include <string>

// =====================================================================
//  Parameters (all set from Python JSON configs; defaults here are only fallbacks)
// =====================================================================
static std::map<std::string, double> &params() { static std::map<std::string, double> p; return p; }
static double P(const char *k, double def) { auto it = params().find(k); return it == params().end() ? def : it->second; }

static std::mt19937 &rng() { static std::mt19937 r(1); return r; }
static double nrand(double s) { if (s <= 0) return 0; std::normal_distribution<double> d(0, s); return d(rng()); }
static double urand() { std::uniform_real_distribution<double> d(0, 1); return d(rng()); }

// =====================================================================
//  Simulated clock, pins, interrupts
// =====================================================================
static unsigned long g_ms = 0;
static unsigned long g_us = 0;
unsigned long millis() { return g_ms; }
unsigned long micros() { return g_us; }
void pinMode(uint8_t, uint8_t) {}
int  digitalRead(uint8_t) { return 1; }
void digitalWrite(uint8_t, uint8_t) {}
int  analogRead(uint8_t) { return 0; }
void delayMicroseconds(unsigned int) {}
int  digitalPinToInterrupt(int pin) { return pin; }
static std::map<int, void (*)()> &isrs() { static std::map<int, void (*)()> m; return m; }
void attachInterrupt(int irq, void (*isr)(), int) { isrs()[irq] = isr; }
void detachInterrupt(int irq) { isrs().erase(irq); }
NRLGamepadSim gamepad1;

// =====================================================================
//  Event log + telemetry capture
// =====================================================================
static std::string g_events;
static std::map<std::string, std::string> g_tele;
static void logEvent(const std::string &s)
{
    char b[64]; snprintf(b, sizeof b, "%.2f ", g_ms / 1000.0);
    g_events += b; g_events += s; g_events += "\n";
}
void NRLTelemetry::addData(const char *k, float v) { char b[32]; snprintf(b, sizeof b, "%.2f", v); g_tele[k] = b; }
void NRLTelemetry::addData(const char *k, const char *v) { g_tele[k] = v ? v : ""; }

// =====================================================================
//  HexaDCMotor  (HexaDCMotor.h: -255..255, deadband 10 -> 0, minDrive remaps |cmd| into [minDrive,255],
//  0 = coast stop, flipped = software reverse)
// =====================================================================
static std::vector<HexaDCMotor *> &motors() { static std::vector<HexaDCMotor *> v; return v; }
HexaDCMotor::HexaDCMotor(const HexaDCMotorConfig &c) : m_cfg(c) {}
bool HexaDCMotor::begin() { m_ready = true; if (std::find(motors().begin(), motors().end(), this) == motors().end()) motors().push_back(this); return true; }
void HexaDCMotor::setSpeed(int s)
{
    s = std::max(-MAX_SPEED, std::min(MAX_SPEED, s));
    m_currentSpeed = s;
    int a = std::abs(s);
    if (a <= MOTOR_DEADBAND) { m_pwm = 0; return; }       // ASSUMPTION: "within +/-10" inclusive
    if (m_cfg.minDrive > 0)                              // ASSUMPTION: linear remap of 1..255 onto minDrive..255
        a = m_cfg.minDrive + (int)lround((a - 1) * (255.0 - m_cfg.minDrive) / 254.0);
    // The robot's "flipped" setting corrects its own wiring, so the logical sign IS the wheel's
    // forward/backward. A wiring fault can be injected with motor_reversed_l / _r.
    m_pwm = (s < 0) ? -a : a;
}
void HexaDCMotor::stopAll() { for (auto m : motors()) m->setSpeed(0); }
void HexaDCMotor::clearRegistry() { motors().clear(); }

// =====================================================================
//  TankDrive  (TankDrive.h: L = (f+t)*scale, R = (f-t)*scale, ramp _maxDelta per tick toward the
//  command, stop() zeroes command AND output with no ramp, tickAll() after loop())
// =====================================================================
static std::vector<TankDrive *> &drives() { static std::vector<TankDrive *> v; return v; }
TankDrive::~TankDrive() {}
void TankDrive::_register() { if (std::find(drives().begin(), drives().end(), this) == drives().end()) drives().push_back(this); }
void TankDrive::drive(float f, float t)
{
    _register();
    _targetL = std::max(-1.0f, std::min(1.0f, (f + t) * _scale));
    _targetR = std::max(-1.0f, std::min(1.0f, (f - t) * _scale));
}
void TankDrive::stop() { _register(); _targetL = _targetR = _curL = _curR = 0; _left.setSpeed(0); _right.setSpeed(0); }
void TankDrive::_tick()
{
    auto step = [&](float c, float t) { float d = t - c; d = std::max(-_maxDelta, std::min(_maxDelta, d)); return c + d; };
    _curL = step(_curL, _targetL); _curR = step(_curR, _targetR);
    _left.setSpeed((int)lroundf(_curL * 255.0f));        // ASSUMPTION: output * 255, rounded
    _right.setSpeed((int)lroundf(_curR * 255.0f));
}
void TankDrive::tickAll() { for (auto d : drives()) d->_tick(); }
void TankDrive::stopAll() { for (auto d : drives()) d->stop(); }
void TankDrive::clearRegistry() { drives().clear(); }

// =====================================================================
//  HexaPID  (HexaPID.h: dt measured, capped at maxDtMs; first update after reset has no D;
//  D filter = dAlpha*old + (1-dAlpha)*new; I clamp iMax in output units + anti-windup)
//  Simulator extra: optional gain OVERRIDE for experiments (never touches the firmware).
// =====================================================================
static std::vector<HexaPID *> &pids() { static std::vector<HexaPID *> v; return v; }
HexaPID::HexaPID(const HexaPIDConfig &c) : m_cfg(c) { pids().push_back(this); }
void HexaPID::reset() { m_iSum = 0; m_lastErr = 0; m_dFilt = 0; m_primed = false; }
float HexaPID::update(float e)
{
    float dt = 0.01f;
    if (m_primed) {
        dt = (micros() - m_lastUs) / 1e6f;
        if (dt * 1000.0f > m_cfg.maxDtMs) dt = m_cfg.maxDtMs / 1000.0f;
        if (dt <= 0) dt = 0.001f;
    }
    m_lastUs = micros();
    return updateDt(e, dt);
}
float HexaPID::updateDt(float e, float dt)
{
    m_p = m_cfg.kP * e;
    float dRaw = 0;
    if (m_primed && dt > 0) dRaw = (e - m_lastErr) / dt;
    m_dFilt = m_primed ? (m_cfg.dAlpha * m_dFilt + (1.0f - m_cfg.dAlpha) * dRaw) : 0.0f;
    m_d = m_cfg.kD * m_dFilt;
    float iTry = m_iSum + e * dt;
    float iOut = m_cfg.kI * iTry;
    if (iOut > m_cfg.iMax) iOut = m_cfg.iMax;
    if (iOut < -m_cfg.iMax) iOut = -m_cfg.iMax;
    float raw = m_p + iOut + m_d;
    float out = std::max(m_cfg.outMin, std::min(m_cfg.outMax, raw));
    bool pinned = (raw > m_cfg.outMax && e > 0) || (raw < m_cfg.outMin && e < 0);
    if (!pinned) m_iSum = iTry;                         // anti-windup
    m_i = iOut;
    m_out = out;
    m_lastErr = e;
    m_primed = true;
    return out;
}

// =====================================================================
//  Arena surface (rectangles of black tape, mm; built from config/arena.json)
// =====================================================================
struct Rect { double x0, y0, x1, y1; };
static std::vector<Rect> g_tape;
static bool darkAt_mm(double x, double y)
{
    for (const auto &r : g_tape) if (x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1) return true;
    return false;
}

// =====================================================================
//  World / robot state
// =====================================================================
struct World {
    double x = 0, y = 0, th = 0;      // chassis middle (m), heading (rad, CCW)
    double thUnwrapped = 0;
    double vL = 0, vR = 0;            // wheel surface speeds (m/s)
    double encAccL = 0, encAccR = 0;
    long   pulsesL = 0, pulsesR = 0;  // pulses delivered to the robot's interrupt handlers
    double gyroZero = 0;              // zeroHeading() reference (measured units)
    double gyroFree0 = 0;
    double dropoutUntil = 0;          // intermittent line-loss fault: array blind until this time
    double lastMoveT = 0, lastX = 0, lastY = 0, lastTh = 0;
};
static World W;

// =====================================================================
//  Gyro (NRLComms heading API). Clockwise-positive degrees, like DriveActions.
// =====================================================================
static double gyroTrueCW() { return -W.thUnwrapped * 180.0 / M_PI; }
static double gyroMeasured()
{
    if (P("gyro_unavailable", 0) > 0) return 0;
    const double t = g_ms / 1000.0;
    double h = gyroTrueCW() * P("gyro_scale", 1.0);
    if (P("gyro_sign_flip", 0) > 0) h = -h;
    h += P("gyro_bias_deg", 0) + P("gyro_drift_dps", 0) * t + nrand(P("gyro_noise_deg", 0.05));
    const double tj = P("gyro_jump_t", -1);
    if (tj >= 0 && t >= tj) h += P("gyro_jump_deg", 0);
    return h;
}
static double wrap180(double a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
float NRLComms::getHeading() { return (float)wrap180(gyroMeasured() - W.gyroZero); }
float NRLComms::getHeadingFree() { return (float)wrap180(gyroMeasured() - W.gyroFree0); }
bool  NRLComms::isHeadingReady() { return P("gyro_unavailable", 0) <= 0 && g_ms >= (unsigned long)P("gyro_ready_ms", 0); }
void  NRLComms::zeroHeading() { W.gyroZero = gyroMeasured(); }
void  NRLComms::resetHeading() { W.gyroZero = gyroMeasured(); }

// =====================================================================
//  IR line array: geometry + analog model + the documented HexaLineArray processing
// =====================================================================
static std::vector<HexaLineArray *> &arrays() { static std::vector<HexaLineArray *> v; return v; }
static double g_cover[8];          // physical tape coverage of each spot (0..1), for the viewer
static double g_sensorXY[8][2];    // sensor positions (m), for the viewer

const char *linePatternName(LinePattern p)
{
    switch (p) {
    case LinePattern::LOST: return "LOST"; case LinePattern::LINE: return "LINE";
    case LinePattern::LEFT_BRANCH: return "LEFT_BRANCH"; case LinePattern::RIGHT_BRANCH: return "RIGHT_BRANCH";
    case LinePattern::TERMINAL: return "TERMINAL";
    }
    return "?";
}
// Registers at construction: the real array's calibration lives in flash (NVS) before begin().
HexaLineArray::HexaLineArray(const HexaLineArrayConfig &c) : m_cfg(c) { arrays().push_back(this); }
bool HexaLineArray::begin()
{
    m_ready = true;
    if (std::find(arrays().begin(), arrays().end(), this) == arrays().end()) arrays().push_back(this);
    return m_calibrated;
}
void HexaLineArray::simCalibrate(const uint16_t w[8], const uint16_t b[8])
{
    for (int i = 0; i < 8; i++) { m_calMin[i] = w[i]; m_calMax[i] = b[i]; }
    m_calibrated = true;
}
uint8_t HexaLineArray::count() const { uint8_t c = 0; for (int i = 0; i < 8; i++) if (m_mask & (1 << i)) c++; return c; }

static void physicalCoverage()
{
    const double pf = P("sensor_fwd_m", 0.08), pitch = P("sensor_pitch_m", 0.010);
    const double lat0 = P("sensor_lateral_offset_m", 0.0), ang = P("sensor_angle_deg", 0.0) * M_PI / 180.0;
    const double r = P("sensor_spot_radius_m", 0.003);
    const double hx = cos(W.th), hy = sin(W.th), lx = -sin(W.th), ly = cos(W.th);
    const double cx = W.x + pf * hx + lat0 * lx, cy = W.y + pf * hy + lat0 * ly;
    // bar direction (robot-left), optionally rotated on the chassis
    const double bx = lx * cos(ang) - hx * sin(ang), by = ly * cos(ang) - hy * sin(ang);
    for (int i = 0; i < 8; i++) {
        const double off = (3.5 - i) * pitch;           // channel 0 is the LEFT-most
        const double sx = cx + bx * off, sy = cy + by * off;
        g_sensorXY[i][0] = sx; g_sensorXY[i][1] = sy;
        int dark = 0, n = 0;
        for (int a = -3; a <= 3; a++) for (int c = -3; c <= 3; c++) {
            if (a * a + c * c > 9) continue;
            n++;
            if (darkAt_mm((sx + a * r / 3.0) * 1000.0, (sy + c * r / 3.0) * 1000.0)) dark++;
        }
        g_cover[i] = (double)dark / n;
    }
    if (g_ms / 1000.0 < W.dropoutUntil) for (int i = 0; i < 8; i++) g_cover[i] = 0;   // injected blind spell
}

void HexaLineArray::tick()
{
    // --- raw ADC model (white reads LOW, black HIGH; header doc) ---
    const int deadMask = (int)P("sensor_dead_mask", 128);   // bit i = channel i dead (default: ch7)
    for (int i = 0; i < 8; i++) {
        const double w = P((std::string("raw_white_") + char('0' + i)).c_str(), P("raw_white", 400));
        const double b = P((std::string("raw_black_") + char('0' + i)).c_str(), P("raw_black", 3000));
        double v = (deadMask & (1 << i)) ? w : w + g_cover[i] * (b - w);
        v += nrand(P("raw_noise", 40));
        m_raw[i] = (uint16_t)std::max(0.0, std::min(4095.0, v));
    }
    // --- normalisation (header formula), dead channel = span < minSpan -> 0 ---
    m_mask = 0;
    double sw = 0, s = 0;
    for (int i = 0; i < 8; i++) {
        const int span = (int)m_calMax[i] - (int)m_calMin[i];
        int n = 0;
        if (m_calibrated && span >= m_cfg.minSpan) n = (int)lround(((double)m_raw[i] - m_calMin[i]) / span * 1000.0);
        n = std::max(0, std::min(1000, n));
        m_norm[i] = (uint16_t)n;
        if (n >= m_cfg.darkThreshold) m_mask |= (1 << i);
        if (n >= m_cfg.weightThreshold) { sw += n; s += (double)n * i * 1000.0; }
    }
    m_onLine = m_mask != 0;
    if (m_onLine && sw > 0) {
        m_position = (int)lround(s / sw);
        m_lineSeenMs = millis();
        m_hardLost = false;
        const int e = m_position - HEXA_LINE_CENTRE;
        if (e != 0) { m_lastSeenSide = (e < 0) ? -1 : 1; }   // ASSUMPTION: any non-zero offset sets the side
        m_lastSeenMag = std::abs(e);
    } else {
        if (millis() - m_lineSeenMs > m_cfg.lostStopMs) { m_hardLost = true; m_position = -1; }
        else m_position = (m_lastSeenSide < 0) ? 0 : 7000;   // header: extreme on the side last seen
    }
    // --- pattern (header: zones ch0-1, ch3-4, ch6-7; TERMINAL = 7+ dark) ---
    // ASSUMPTION: "middle dark" = ch3 OR ch4; edge = either channel of the pair; both edges -> LEFT_BRANCH.
    const int c = count();
    const bool mid = (m_mask & 0x18) != 0, le = (m_mask & 0x03) != 0, re = (m_mask & 0xC0) != 0;
    if (c == 0) m_pattern = LinePattern::LOST;
    else if (c >= 7) m_pattern = LinePattern::TERMINAL;
    else if (mid && le) m_pattern = LinePattern::LEFT_BRANCH;
    else if (mid && re) m_pattern = LinePattern::RIGHT_BRANCH;
    else m_pattern = LinePattern::LINE;
}
void HexaLineArray::tickAll() { for (auto a : arrays()) a->tick(); }
void HexaLineArray::clearRegistry() { arrays().clear(); }

// =====================================================================
//  Actions engine (NRLAction.h / NRLOpMode.h)
//  ASSUMPTION: sequential() moves to its next child on the NEXT tick (one child step per tick).
//  Set seq_chain=1 to run following children in the same tick instead.
// =====================================================================
namespace {
struct Instant : Action { std::function<void()> f; bool run() override { f(); return true; } };
struct WaitUntil : Action { std::function<bool()> f; bool run() override { return f(); } };
struct SleepMs : Action { unsigned long ms, t0 = 0; bool started = false;
    bool run() override { if (!started) { started = true; t0 = millis(); } return millis() - t0 >= ms; }
    void reset() override { started = false; } };
struct Seq : Action { std::vector<ActionPtr> c; size_t i = 0;
    bool run() override {
        while (i < c.size()) {
            if (!c[i]->run()) return false;
            i++;
            if (P("seq_chain", 0) <= 0) return i >= c.size();
        }
        return true; }
    void reset() override { i = 0; for (auto &a : c) a->reset(); } };
struct Par : Action { std::vector<ActionPtr> c; std::vector<bool> done;
    bool run() override { bool all = true; for (size_t k = 0; k < c.size(); k++) { if (!done[k]) done[k] = c[k]->run(); all = all && done[k]; } return all; } };
struct Rep : Action { ActionPtr a; bool run() override { if (a->run()) a->reset(); return false; } };
}
ActionPtr instant(std::function<void()> fn) { auto a = std::make_shared<Instant>(); a->f = fn; return a; }
ActionPtr wait_until(std::function<bool()> f) { auto a = std::make_shared<WaitUntil>(); a->f = f; return a; }
ActionPtr sleep_ms(unsigned long ms) { auto a = std::make_shared<SleepMs>(); a->ms = ms; return a; }
ActionPtr sequential(std::initializer_list<ActionPtr> l) { auto a = std::make_shared<Seq>(); a->c = l; return a; }
ActionPtr sequential(std::vector<ActionPtr> l) { auto a = std::make_shared<Seq>(); a->c = l; return a; }
ActionPtr parallel(std::initializer_list<ActionPtr> l) { auto a = std::make_shared<Par>(); a->c = l; a->done.assign(a->c.size(), false); return a; }
ActionPtr repeat(ActionPtr inner) { auto a = std::make_shared<Rep>(); a->a = inner; return a; }

void NRLOpMode::runAction(ActionPtr a)
{
    if (_actions.size() + _pending.size() >= 8) { logEvent("WARN action queue full, action dropped"); return; }
    _pending.push_back(a);                      // NRLOpMode.h: "queued since the last tick"
}
void NRLOpMode::cancelActions() { _actions.clear(); _pending.clear(); if (_advancing) _cancelRequested = true; }
bool NRLOpMode::isActionRunning() const { return !_actions.empty() || !_pending.empty(); }
void NRLOpMode::_advanceAction()
{
    for (auto &a : _pending) _actions.push_back(a);
    _pending.clear();
    _advancing = true; _cancelRequested = false;
    for (size_t i = 0; i < _actions.size();) {
        if (_actions[i]->run()) { if (_cancelRequested) break; _actions.erase(_actions.begin() + i); }
        else { if (_cancelRequested) break; i++; }
    }
    if (_cancelRequested) _actions.clear();
    _advancing = false;
}

// =====================================================================
//  LineActions (only leaveBox() is simulated; LineActions.h / LineTuning.h docs):
//  "Drive straight out of a BLACK start box until the bar is on the line leaving it:
//   ch3 or ch4 black, ch0/ch1/ch6/ch7 all white", boxLeaveTicks in a row, at startSpeed,
//   NO_LINE after startTimeoutMs.  ASSUMPTION: "straight" = drive(startSpeed, 0), no gyro hold.
// =====================================================================
const char *lineFaultName(LineFault f)
{
    static const char *n[] = {"NONE", "NOT_CALIBRATED", "LINE_LOST", "UNEXPECTED_END", "TURN_TIMEOUT", "NO_GREY",
                              "NO_LINE", "WRONG_JUNCTION", "NO_ROUTE", "NO_BRANCH"};
    return n[(int)f];
}
LineActions::LineActions(TankDrive &d, HexaLineArray &l, LineTuning t) : _drive(d), _line(l), _tuning(t) {}
namespace {
struct LeaveBox : Action {
    LineActions *la; int ok = 0; unsigned long t0 = 0; bool started = false; float h0 = 0;
    bool run() override {
        if (!started) { started = true; t0 = millis(); h0 = NRLComms::getHeading(); la->simDrive().setRampRate(la->tuning().rampRate); }
        HexaLineArray &l = la->simLine();
        const uint8_t m = l.mask();
        if ((m & 0x18) && !(m & 0xC3)) ok++; else ok = 0;
        if (ok >= la->tuning().boxLeaveTicks) { la->simDrive().stop(); return true; }
        if (millis() - t0 > la->tuning().startTimeoutMs) { la->simDrive().stop(); la->_fault = LineFault::NO_LINE; return true; }
        // ASSUMPTION switch: the header says only "straight out". sdk_leavebox_gyro_hold=1 models a
        // gyro heading hold (P on getHeading drift since the action started), 0 = open loop.
        float turn = 0.0f;
        if (P("sdk_leavebox_gyro_hold", 0) > 0 && NRLComms::isHeadingReady()) {
            const float h = wrap180(NRLComms::getHeading() - h0);
            turn = std::max(-0.1f, std::min(0.1f, -0.02f * h));
        }
        la->simDrive().drive(la->tuning().startSpeed, turn);
        return false;
    }
};
}
ActionPtr LineActions::leaveBox() { auto a = std::make_shared<LeaveBox>(); a->la = this; return a; }
ActionPtr LineActions::Builder::build() { logEvent("NOTE builder-based fallback opmode is not simulated"); return instant([] {}); }

// =====================================================================
//  OpMode registry
// =====================================================================
struct Reg { std::string name; OpModeType type; NRLOpMode *(*make)(); };
static std::vector<Reg> &registry() { static std::vector<Reg> r; return r; }
void simRegisterOpMode(const char *n, OpModeType t, NRLOpMode *(*f)()) { registry().push_back({n, t, f}); }

// =====================================================================
//  Physics
// =====================================================================
static HexaDCMotor *g_mL = nullptr, *g_mR = nullptr;
static TankDrive *g_drive = nullptr;

static double wheelTarget(int pwm, bool pivot, bool left, double vNow)
{
    if (left && P("motor_reversed_l", 0) > 0) pwm = -pwm;
    if (!left && P("motor_reversed_r", 0) > 0) pwm = -pwm;
    const double t = g_ms / 1000.0;
    const double stallT = left ? P("motor_stall_l_t", -1) : P("motor_stall_r_t", -1);
    if (stallT >= 0 && t >= stallT) return 0;
    if (pwm == 0) return 0;
    const double thr = left ? (pivot ? P("thr_pivot_l", 29) : P("thr_straight_l", 24.5))
                            : (pivot ? P("thr_pivot_r", 23) : P("thr_straight_r", 22.5));
    const bool moving = fabs(vNow) > 0.01;
    const double eff = moving ? thr * P("kinetic_ratio", 0.8) : thr;
    const double a = std::abs(pwm) - eff;
    if (a <= 0) return 0;
    const double rpm = P("motor_rpm", 300), r = P("wheel_radius_m", 0.05);
    const double vmax = rpm / 60.0 * 2 * M_PI * r;
    const double kv = vmax / (255.0 - thr);
    const double gain = left ? P("motor_gain_l", 1.0) : P("motor_gain_r", 1.05);
    return (pwm > 0 ? 1 : -1) * kv * gain * a;
}

static void fireEncoder(int pin, int n, unsigned long t0us, double dtUs)
{
    auto it = isrs().find(pin);
    if (it == isrs().end() || n <= 0) return;
    for (int k = 0; k < n; k++) { g_us = t0us + (unsigned long)((k + 1) * dtUs / (n + 1)); it->second(); }
}

static void physicsStep(double dt)
{
    const int pl = g_mL ? g_mL->simPwm() : 0, pr = g_mR ? g_mR->simPwm() : 0;
    const bool pivot = pl != 0 && pr != 0 && ((pl > 0) != (pr > 0));
    const double tL = wheelTarget(pl, pivot, true, W.vL), tR = wheelTarget(pr, pivot, false, W.vR);
    auto approach = [&](double v, double tgt) {
        const bool speedingUp = fabs(tgt) > fabs(v) && (tgt * v >= 0);
        const double tau = speedingUp ? P("tau_up_s", 0.08) : (tgt == 0 ? P("tau_coast_s", 0.10) : P("tau_down_s", 0.08));
        return v + (tgt - v) * std::min(1.0, dt / tau);
    };
    W.vL = approach(W.vL, tL); W.vR = approach(W.vR, tR);
    const double slip = P("slip_long", 0.97);
    const double v = 0.5 * (W.vL + W.vR) * slip;
    const double w = (W.vR - W.vL) / P("track_eff_m", 0.45);
    W.x += v * cos(W.th) * dt; W.y += v * sin(W.th) * dt;
    W.th += w * dt; W.thUnwrapped += w * dt;

    // encoders: channel A only, a pulse per edge whichever way the wheel turns (AutoPark ISR counts up)
    const unsigned long t0 = g_us;
    auto pulses = [&](double &acc, double vWheel, double cpm, bool dead, double miss, double spur) {
        acc += fabs(vWheel) * dt * cpm + (spur > 0 ? (urand() < spur * dt ? 1.0 : 0.0) : 0.0);
        int n = (int)acc; acc -= n;
        if (dead) return 0;
        int k = 0; for (int j = 0; j < n; j++) if (urand() >= miss) k++;
        return k;
    };
    const double t = g_ms / 1000.0;
    const bool deadL = P("enc_dead_l", 0) > 0 || (P("enc_dead_l_t", -1) >= 0 && t >= P("enc_dead_l_t", -1));
    const bool deadR = P("enc_dead_r", 0) > 0 || (P("enc_dead_r_t", -1) >= 0 && t >= P("enc_dead_r_t", -1));
    const int nL = pulses(W.encAccL, W.vL, P("enc_cpm_l", 1300), deadL, P("enc_miss_l", 0), P("enc_spurious_hz_l", 0));
    const int nR = pulses(W.encAccR, W.vR, P("enc_cpm_r", 1300), deadR, P("enc_miss_r", 0), P("enc_spurious_hz_r", 0));
    W.pulsesL += nL; W.pulsesR += nR;
    fireEncoder((int)P("enc_pin_l", DIGITAL_D3), nL, t0, dt * 1e6);
    fireEncoder((int)P("enc_pin_r", DIGITAL_D2), nR, t0, dt * 1e6);
    g_us = t0;

    // intermittent line-loss fault
    const double rate = P("line_dropout_per_s", 0);
    if (rate > 0 && t >= W.dropoutUntil && urand() < rate * dt) W.dropoutUntil = t + P("line_dropout_ms", 60) / 1000.0;
}

// =====================================================================
//  C API
// =====================================================================
static NRLOpMode *g_op = nullptr;
static std::string g_lastState, g_lastTurns, g_lastLeg, g_status = "idle";
static std::string g_teleStr, g_eventsOut;
static int g_stat = 0;   // 0 running, 1 done, 2 fault, 3 timeout, 4 left field, 5 stuck, 6 no opmode

extern "C" {

void sim_set_param(const char *key, double value) { params()[key] = value; }
void sim_arena_clear() { g_tape.clear(); }
void sim_arena_rect(double x0, double y0, double x1, double y1) { g_tape.push_back({std::min(x0, x1), std::min(y0, y1), std::max(x0, x1), std::max(y0, y1)}); }

const char *sim_list_opmodes()
{
    static std::string s; s.clear();
    for (auto &r : registry()) { s += r.name; s += (r.type == OpModeType::AUTO ? "|AUTO\n" : "|TELEOP\n"); }
    return s.c_str();
}

// Start an opmode: calibrate the array (as LINE TEST would), place the robot, init() + start().
int sim_start(const char *name)
{
    rng().seed((unsigned)P("seed", 1));
    g_ms = 1000; g_us = g_ms * 1000;               // the Hub has been on for a second
    W = World();
    W.x = P("start_x_m", 1.45); W.y = P("start_y_m", 0.0);
    W.th = P("start_heading_deg", 0.0) * M_PI / 180.0; W.thUnwrapped = W.th;
    W.gyroFree0 = 0; W.gyroZero = 0;
    W.lastX = W.x; W.lastY = W.y; W.lastTh = W.th; W.lastMoveT = g_ms / 1000.0;
    g_events.clear(); g_tele.clear(); g_lastState = g_lastTurns = g_lastLeg = "";
    g_stat = 0;
    const Reg *found = nullptr;
    for (auto &r : registry()) if (r.name == name) found = &r;
    if (!found) { g_stat = 6; return -1; }
    g_op = found->make();
    // per-channel calibration references: the white / black raw levels (+ calibration noise)
    uint16_t w[8], b[8];
    for (int i = 0; i < 8; i++) {
        const double wi = P((std::string("raw_white_") + char('0' + i)).c_str(), P("raw_white", 400));
        const double bi = P((std::string("raw_black_") + char('0' + i)).c_str(), P("raw_black", 3000));
        const bool dead = ((int)P("sensor_dead_mask", 128) & (1 << i)) != 0;
        w[i] = (uint16_t)std::max(0.0, wi + nrand(P("cal_noise", 10)));
        b[i] = (uint16_t)std::max(0.0, (dead ? wi : bi) + nrand(P("cal_noise", 10)));
    }
    if (P("uncalibrated", 0) <= 0) for (auto a : arrays()) a->simCalibrate(w, b);
    logEvent(std::string("START ") + name);
    g_op->init();
    // find the drive the opmode built (the first registered TankDrive / motors)
    for (auto a : arrays()) a->begin();
    if (!motors().empty()) { g_mL = motors()[0]; g_mR = motors().size() > 1 ? motors()[1] : nullptr; }
    g_op->start();
    // pids: optional experimental gain override for pid #k
    for (size_t k = 0; k < pids().size(); k++) {
        char kk[32];
        snprintf(kk, sizeof kk, "pid%zu_kp", k); if (params().count(kk)) { auto c = pids()[k]->config(); c.kP = (float)P(kk, 0); pids()[k]->setConfig(c); }
        snprintf(kk, sizeof kk, "pid%zu_kd", k); if (params().count(kk)) { auto c = pids()[k]->config(); c.kD = (float)P(kk, 0); pids()[k]->setConfig(c); }
        snprintf(kk, sizeof kk, "pid%zu_ki", k); if (params().count(kk)) { auto c = pids()[k]->config(); c.kI = (float)P(kk, 0); pids()[k]->setConfig(c); }
    }
    return 0;
}

// One 10 ms robot tick: line array tick -> actions -> loop() -> drive ramp -> physics.
static void oneTick()
{
    const double dt = P("dt_s", 0.01);
    physicalCoverage();
    HexaLineArray::tickAll();
    g_op->_advanceAction();                         // NRLOpMode.h: "each tick runs your actions FIRST, then loop()"
    g_op->loop();
    if (!drives().empty()) g_drive = drives()[0];
    TankDrive::tickAll();
    physicsStep(dt);
    g_ms += (unsigned long)lround(dt * 1000); g_us = g_ms * 1000;

    const std::string st = g_tele.count("state") ? g_tele["state"] : "";
    const std::string tu = g_tele.count("turns") ? g_tele["turns"] : "";
    const std::string lg = g_tele.count("leg") ? g_tele["leg"] : "";
    char pose[96]; snprintf(pose, sizeof pose, " @(%.3f,%.3f,%.1fdeg)", W.x, W.y, W.th * 180 / M_PI);
    if (st != g_lastState) { logEvent("STATE " + g_lastState + " -> " + st + " [" + lg + "] why=" + (g_tele.count("why") ? g_tele["why"] : "") + pose); g_lastState = st; }
    if (tu != g_lastTurns) { logEvent("TURN log=" + tu + pose); g_lastTurns = tu; }
    if (lg != g_lastLeg) { logEvent("STEP " + lg + pose); g_lastLeg = lg; }
}

// Advance n ticks. Returns status.
int sim_step(int n)
{
    if (!g_op) return 6;
    for (int k = 0; k < n && g_stat == 0; k++) {
        oneTick();
        const double t = g_ms / 1000.0;
        const std::string st = g_lastState;
        if ((st == "DONE" || st == "FAULT") && !g_op->isActionRunning()) {
            // let the robot come to rest so the final pose is the parked pose
            for (int j = 0; j < 200; j++) { HexaLineArray::tickAll(); g_op->loop(); TankDrive::tickAll(); physicsStep(0.01); g_ms += 10; g_us = g_ms * 1000; }
            g_stat = (st == "DONE") ? 1 : 2;
            logEvent(std::string("END ") + st);
            break;
        }
        if (t - 1.0 > P("timeout_s", 150)) { g_stat = 3; logEvent("END TIMEOUT"); break; }
        const double fx0 = P("field_x0_m", -0.829), fx1 = P("field_x1_m", 2.829), fy0 = P("field_y0_m", -0.829), fy1 = P("field_y1_m", 2.829);
        if (W.x < fx0 || W.x > fx1 || W.y < fy0 || W.y > fy1) { g_stat = 4; logEvent("END LEFT FIELD"); break; }
        if (hypot(W.x - W.lastX, W.y - W.lastY) > 0.003 || fabs(W.th - W.lastTh) > 0.01) { W.lastX = W.x; W.lastY = W.y; W.lastTh = W.th; W.lastMoveT = t; }
        else if (t - W.lastMoveT > P("stuck_s", 8)) { g_stat = 5; logEvent("END STUCK (no motion for " + std::to_string((int)P("stuck_s", 8)) + " s)"); break; }
    }
    return g_stat;
}

void sim_stop()
{
    if (g_op) { g_op->stop(); }
}

// State vector for Python (see engine.py STATE_FIELDS for the order).
int sim_get_state(double *o, int n)
{
    std::vector<double> v;
    v.push_back(g_ms / 1000.0 - 1.0);
    v.push_back(W.x); v.push_back(W.y); v.push_back(W.th * 180 / M_PI);
    v.push_back(W.vL); v.push_back(W.vR);
    v.push_back(g_drive ? g_drive->simOutL() : 0); v.push_back(g_drive ? g_drive->simOutR() : 0);
    v.push_back(g_mL ? g_mL->simPwm() : 0); v.push_back(g_mR ? g_mR->simPwm() : 0);
    v.push_back((double)W.pulsesL); v.push_back((double)W.pulsesR);
    v.push_back(wrap180(gyroTrueCW()));   // true heading, CW+ (reading the noisy gyro here would consume RNG draws)
    HexaLineArray *a = arrays().empty() ? nullptr : arrays()[0];
    v.push_back(a ? a->mask() : 0); v.push_back(a ? a->position() : 0); v.push_back(a ? a->error() : 0);
    v.push_back(a ? a->onLine() : 0); v.push_back(a ? (int)a->pattern() : 0);
    for (int i = 0; i < 8; i++) v.push_back(a ? a->read(i) : 0);
    for (int i = 0; i < 8; i++) { v.push_back(g_sensorXY[i][0]); v.push_back(g_sensorXY[i][1]); }
    HexaPID *p = pids().empty() ? nullptr : pids()[0];
    v.push_back(p ? p->config().kP : 0); v.push_back(p ? p->config().kI : 0); v.push_back(p ? p->config().kD : 0);
    v.push_back(p ? p->p() : 0); v.push_back(p ? p->d() : 0); v.push_back(p ? p->output() : 0);
    v.push_back(g_stat);
    const int m = std::min(n, (int)v.size());
    for (int i = 0; i < m; i++) o[i] = v[i];
    return (int)v.size();
}

const char *sim_telemetry()
{
    g_teleStr.clear();
    for (auto &kv : g_tele) { g_teleStr += kv.first; g_teleStr += "="; g_teleStr += kv.second; g_teleStr += "\n"; }
    return g_teleStr.c_str();
}

const char *sim_events() { g_eventsOut = g_events; g_events.clear(); return g_eventsOut.c_str(); }

}  // extern "C"
