# Findings from the SIL simulator (interpretation)

This page is engineering interpretation, written by hand. Every claim points at runs in
[VALIDATION_REPORT.md](VALIDATION_REPORT.md) that anyone can reproduce with `python3 main.py run ...`.
It is **simulation evidence only**. Items marked *depends on* rest on an unmeasured parameter.

Firmware under test = the latest `AutoPark.cpp` given to the team, called "candidate" below (it is not in the repo yet).
"Repo V3" = `RobotFirmware/opmodes/AutoPark.cpp` at commit 8005f0d.

## Headline

| | Park Run | P1-D1 Run |
|---|---|---|
| candidate, Monte Carlo (60 randomised robots) | 60/60 parked | 58/60 parked |
| repo V3, Monte Carlo | 47/60 | 5/60 |
| candidate, 22 fault scenarios x 3 seeds (all seeds park) | 17/22 | 15/22 |
| repo V3, 22 fault scenarios x 3 seeds | 14/22 | 0/22 |

The candidate fixes the two big V3 problems: V3's start-box exit (see F7) and V3 missing the P1 branch (V3 fails P1-D1
even in ideal conditions, matching the real-robot bug found on 2026-11-19).

## F1. The corner pivot point is the single point of failure (most important)

**What happens.** After ADJUST the robot STOPs, and while stopping it **coasts**. If the middle of the chassis ends up
more than about 4-5 cm past the corner, then after the 90 deg turn the new line is outside the part of the sensor bar
that works. Channel 7 is dead, so the live channels span about +35 to -25 mm (at the assumed 10 mm pitch). The firmware then:

1. SWEEPs ±25 deg, finds nothing, and **advances to the next plan step anyway** (event `SWEEP -> STOP [next step] why=no line after turn`);
2. SQUARE / CENTRE rotate looking for tape and can **lock onto the line the robot came in on**;
3. resumes FOLLOW **up to 45 deg off the grid** (seen at 134.5 deg, should be 90).

From there the step counter and the robot's real position disagree and the run fails.

**Evidence.**
- MC P1-D1 seed 1030: ADJUST stops at x = 1.983 m and the robot coasts to 2.032-2.049 (C1 is at 2.000). Then `no line after turn`.
  It steps through "to P1" and "P1 end" while it is still at y = 0.16-0.28 m (P1 is at 0.50) and FAULTs later.
  The same happens in seed 1055.
- Scenario 13 (slower motor response): after the turn, CENTRE swings to 167 deg, FOLLOW drives back west along the
  bottom line towards START, and `off course` FAULTs after about 1 m.
- Scenarios 4 and 22 end the same way.

**Why it matters for the real robot.** This matches what you reported on the real robot: "on the first turn it always
overshoots". C1 is the first corner, so the learned coast (`adjCoast`) has no data yet and `COAST_GUESS_M` (2 cm) is used.

**Depends on:** `tau_coast_s` and `sensor_pitch_m`. Both are unmeasured. A shorter coast or a wider bar removes most
of these failures.

**Suggested direction** (not applied: the firmware was not changed for the simulator):
- After `no line after turn`, **do not advance the step**. Back up a few cm along the new heading, or creep sideways,
  and look again before giving up.
- Refuse to enter FOLLOW when the gyro says the robot is more than about 15 deg off the grid.
- Use a larger first-corner coast guess, or measure the coast once (below) and put it in `COAST_GUESS_M`.

**Measure on the robot (2 minutes):** run Min Drive or any test at ROLL_SPEED 0.22. Mark the floor at the moment the
motors are cut. Measure how far the robot rolls. Do it 3 times and send the numbers. That one number calibrates F1.

## F2. One dead encoder is not detected

Distance = the average of both encoders. If one encoder stops, every measured move reads half its real length.
ADJUST then drives about twice as far and C1 is overshot by 6-8 cm, which leads into F1.
The `enc off` check (`ENC_STALL_MS`) only trips when **neither** side counts.

Evidence: scenarios 4 and 5 (Park Run 0/3 parks). Scenario 6 (**both** dead) parks 3/3: the tape-only mode works.

Suggested direction: while driving straight, if one side counts and the other has not counted for about 300 ms, either
use the working side alone or switch to `encOk = false` (tape-only), which is proven by scenario 6.

## F3. A blink at the start-box exit ends the run

A single 50 ms all-white blink while leaving START gives `line not at box exit`, then a search, then FAULT.
Evidence: scenario 18, seed 101.

Real dropouts are usually one or two channels, not all eight, so this is a moderate risk.

Suggested direction: require the "no line at exit" condition for a few ticks, not one.

## F4. "edge not found" after a long coast turns far too early

In scenario 22 (slippery mat) the robot coasts 13 cm past C1, so LOCATE cannot find the line end.
The fallback ADJUST then backs up and the turn happens **39 cm before** the corner.

Suggested direction: when LOCATE fails, re-measure from the original detection point instead of from where the robot
ended up, or creep backwards until the tape is seen again.

## F5. The spin limit applies per turn, not per corner

Header: "nothing ever spins more than target + 30 deg". That is true for each turn, but two chained TAPE TURNs at the same
spot added up to 242 deg in place (scenario 22, seed 102). This is low risk; it only happens after F1 or F4.

## F6. Steer PID gains are not the bottleneck

Across Kp 0.10-0.40 and Kd 0-0.16, every cell parked 8/8 and the completion time stayed at 36-37 s.

- Higher Kp lowers tracking error (RMS about 780 at Kp 0.10, about 450 at Kp 0.40).
- Kd = 0 raises oscillation at high Kp (3.2 swings/m vs about 2.2).
- The baseline Kp 0.22 / Kd 0.08 sits in the good region.

Likely reason the gains matter little: the firmware limits steering to ±0.25 and 0.025 per tick.
**Recommendation: leave the gains alone**; time is better spent on F1.

## F7. Repo V3 start: the SDK `leaveBox()` exit condition is fragile

V3 Park Run fails 13/60 runs, every one as LEFT FIELD.

The cause is the right side being 5% faster, which matches the measured 6 cm/m veer. The line drifts under ch6, and
`leaveBox()` waits for ch6 to read white, so it never finishes and drives blind until its 4 s timeout.
It is still 11/60 if the SDK holds heading with the gyro (`sdk_leavebox_gyro_hold=1`; the header does not say which).
Roughly 1 cm of sideways offset is enough. The candidate's own LEAVE state does not have this problem.

## F8. Code never exercised

`CORRECT` is unreachable while `MAX_CORRECTIONS = 0`. Other transitions that never fired in about 250 runs are listed in
the report, section 7. They are untested paths, not proven wrong.

## What to measure next (in order of value)

1. **Coast distance** from ROLL_SPEED 0.22 (F1).
2. **Sensor pitch**: centre-to-centre distance of the IR channels. It sets how far off the line can be after a turn.
3. **Wheelbase** (front-to-rear axle). It is only used for the "all wheels in PARK" check.
4. One filmed C1 turn: where the chassis middle ends up relative to the corner.

## SIM2 firmware changes (2026-11-20), tested in this simulator

Three small changes went into `AutoPark.cpp`; search for `SIM2` in the file to see each one.

1. **ADJUST trim (F1).** After the ADJUST stop, the firmware checks whether the encoders show it more than **2.5 cm** off
   the turning spot. If so, it makes one slow correction move (TRIM_SPEED 0.17) before turning.
   This fixes the coast overshoot, scenario 13.
   A 1.5 cm threshold was tried first and was rejected: it hid errors from `learnCorner`, which lost 3-4 runs in 40 under 5x sensor noise.
2. **One-dead-encoder check (F2).** During the straight-only moves (LEAVE, ADJUST, BACKUP, PARK IN), if one side
   counts 40 or more pulses while the other counts 3 or fewer, the firmware switches to `encOk = false`, the tape-only mode.
3. **LEAVE blink (F3).** If 6 or more channels read dark again in phase 1, the robot is still over the box,
   so LEAVE goes back to phase 0.

A fourth change, "re-square before following if more than 20 deg off the grid", was tested and **removed**: it changed nothing.

Comparison on the same seeds. Previous = the version before SIM2.

| | previous | SIM2 |
|---|---|---|
| Park Run, 200 randomised robots | 200/200 | 200/200 |
| P1-D1, 200 randomised robots | 192/200 (45 recoveries) | 195/200 (26 recoveries) |
| Park Run, one encoder dead (L / R, 10 seeds) | 0 / 0 | 10 / 10 |
| Park Run, slow motor response | 0/10 | 10/10 |
| P1-D1, slow motor response | 0/10 | 9/10 |
| Park Run / P1-D1, intermittent line loss | 8 / 6 of 10 | 9 / 8 of 10 |
| any of the 22 scenarios worse | - | **none** |
| nominal Park Run time | 38.2 s | 38.4 s |

Not fixed: F4 (LOCATE fallback after a very long coast; scenario 22 now ends in a safe FAULT stop instead of wandering off),
and P1-D1 under heavy gyro drift (0.5 deg/s; fails before and after).
