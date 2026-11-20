// ============================================================
//  GlitchTuning.h — shared line-following tuning (GLITCHED, Team 190)
//  Used by "TerminalTests" (AUTO) and "Line Bench" (TELEOP), so the bench
//  tests exactly the speeds and timings the AUTO runs with.
//  Change numbers HERE, not in the OpMode files.
// ============================================================
#pragma once
#include "NRL.h"

namespace glitch {

// ── Overall speed ─────────────────────────────────────────────────────────
inline constexpr float SPEED_SCALE = 0.8f;   // 20 Nov: "reduce the overall speed"

// ── Park ──────────────────────────────────────────────────────────────────
// Park: followToBox() brakes with the sensor bar just inside the PARK square;
// then the SDK's park(ms) drives straight on at tuning.parkSpeed for PARK_MS
// and stops. NO ENCODERS (20 Nov).
// Programming guide 4.2: park(ms) "start small (300-500) — about 250 mm takes
// the robot to the middle of a 500 mm box" (at the default parkSpeed 0.25).
// So 400 ms at 0.25; here at SPEED_SCALE: slower speed, longer time, same
// distance. Tune on the mat: all drive wheels must be on the black square ->
// wheels short of the box: raise PARK_MS; robot past it: lower.
inline constexpr float    PARK_SPEED      = 0.25f * SPEED_SCALE;
inline constexpr uint16_t PARK_MS          = (uint16_t)(400 / SPEED_SCALE);

// ── Line-following tuning ──────────────────────────────────────────────────
// The SDK defaults ARE the Start to Park values (0.35 / 0.25 / 60 ms / 0.25 /
// 0.45 / 0.30). Here they are scaled by SPEED_SCALE.
inline uint16_t slowerMs(uint16_t v) { return (uint16_t)(v / SPEED_SCALE + 0.5f); }
inline uint16_t shorterMs(uint16_t v) { return (uint16_t)(v * SPEED_SCALE + 0.5f); }

inline LineTuning testTuning() {
    LineTuning t;                              // SDK defaults
    const float k = SPEED_SCALE;

    // Speeds
    t.followSpeed   *= k;   // 0.35 -> 0.28
    t.junctionSpeed *= k;   // 0.25 -> 0.20
    t.pivotSpeed    *= k;   // 0.25 -> 0.20
    // Turn / spin / search speeds are NOT scaled (Start to Park values: 0.45,
    // 0.30, 0.35). Run 1 of TerminalTests (20 Nov) at 0.36 / 0.24: at C4 the
    // robot turned back the way it came. Slow spins stop short of the line
    // (seen at C1/C2 in Start to Park too), the turn times out, and the
    // search can catch the old line. See the direction check in loop().
    t.startSpeed    *= k;   // 0.35 -> 0.28  (leaving the START box)
    t.parkSpeed     = PARK_SPEED;   // park(ms) speed (SDK 0.25 x SPEED_SCALE)

    // Distances driven "for X ms" — same distance at the lower speed.
    t.pivotMs       = slowerMs(t.pivotMs);       // 60 -> 75: roll before a turn
    t.straightMs    = slowerMs(t.straightMs);
    t.uTurnBackupMs = slowerMs(t.uTurnBackupMs);
    t.parkMs        = slowerMs(t.parkMs);

    // Time limits — a slower robot needs longer.
    t.crossMaxMs     = slowerMs(t.crossMaxMs);     // 600 -> 750: crossing a tape
    t.boxConfirmMs   = slowerMs(t.boxConfirmMs);   // 150 -> 188: must outlast a tape crossing
    t.turnMinMs      = slowerMs(t.turnMinMs);
    t.uTurnMinMs     = slowerMs(t.uTurnMinMs);
    t.leaveLineMs    = slowerMs(t.leaveLineMs);
    t.turnTimeoutMs  = slowerMs(t.turnTimeoutMs);  // 3000 -> 3750
    t.squareMaxMs    = slowerMs(t.squareMaxMs);
    t.startTimeoutMs = slowerMs(t.startTimeoutMs); // 4000 -> 5000

    // Junction brake pulse: less speed = less momentum to cancel. Same brake
    // speed, shorter pulse, so the stop point (and the turn) stays about the
    // same. (The turn brake pulse is unchanged: turn speeds are unchanged.)
    t.brakeMs     = shorterMs(t.brakeMs);          // 80 -> 64
    return t;
}
}  // namespace glitch
