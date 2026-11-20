# GLITCHED AutoPark - software-in-the-loop simulator

A digital twin of Pixel on the NRL 2026 autonomous arena. It runs **our real `AutoPark.cpp`**: the file is compiled
unchanged and its own state machine, constants and PID make every decision. The simulator supplies the world around it:
arena, physics, IR sensor, encoders, gyro and faults.

> Simulation evidence is **not** hardware validation. Several robot values are not measured yet; they are marked
> `ASSUMPTION` in `config/*.json`. Results: [reports/VALIDATION_REPORT.md](reports/VALIDATION_REPORT.md) and
> [reports/FINDINGS.md](reports/FINDINGS.md).

## Setup on macOS

```bash
xcode-select --install            # C++ compiler (clang), once; skip if already installed
cd Glitch/simulation
python3 -m pip install -r requirements.txt     # numpy, matplotlib, pytest
python3 main.py test              # builds the simulator automatically, runs all tests
python3 main.py view              # live viewer
```

Python 3.9+ works. On Linux, use g++ or clang the same way. Nothing in `RobotFirmware/` is modified.

## Commands

| command | what it does |
|---|---|
| `python3 main.py view` | live 2D viewer. Use `--opmode "P1-D1 Run"` or `--scenario 13` to change the run |
| `python3 main.py run --scenario 4 --seed 2 --log` | one run, prints the event log + verdict, writes `logs/` CSV |
| `python3 main.py suite` | the 22 fault scenarios x 3 seeds (`--opmode "P1-D1 Run"`) |
| `python3 main.py montecarlo --n 100` | randomised robots: success rate, times, errors |
| `python3 main.py sweep` | steer-PID Kp x Kd sweep -> `reports/pid_sweep.png` |
| `python3 main.py audit` | stand-in SDK vs real headers + source change check |
| `python3 main.py report --compare baseline` | everything -> `reports/VALIDATION_REPORT.md` |
| `python3 main.py test` | pytest: audit, ideal, 22 faults (parks + safe), parking, PID |

**Which firmware?** By default, `RobotFirmware/opmodes/AutoPark.cpp`. To test another copy:
`python3 main.py --firmware ~/Downloads/AutoPark.cpp suite` (or set `GLITCH_FIRMWARE=...`).
`--compare baseline` adds the repo file as a second column in the report.

### Viewer controls

- **Run control:** Start/Pause, Step (one 10 ms tick), Reset (fresh power-on).
- **Speed:** 0.25x / 1x / 8x buttons, plus a speed slider.
- **Faults:** the Sensor, Encoder and Gyro fault boxes take effect **live**, mid-run.
- **Choosing a run:** `<` `>` cycle the 22 scenarios ("nominal" = the measured robot); the radio switches between Park Run and P1-D1 Run.

On screen:
- **Arena:** the robot's wheels, the 8 IR spots, the path, and the result banner.
- **Telemetry:** the firmware's own telemetry (state, why, leg, d cm, turns, jx), motor commands, PWM, encoder pulses,
  heading, line mask, position and error, PID gains and output, and the fault reason.

Each finished run writes `logs/<time>_*.csv` (every tick: state, motor commands, sensors), `*.events.log`
(every state change, step, turn, fault, result) and `*.json`.

## How it works

```
simulation/
  core/sdk/NRL.h       stand-in for the NRL SDK v1.5.0 public API (same declarations as the real headers)
  core/sim_core.cpp    SDK behaviour from the header docs + physics + sensors + faults + C API
  core/build.py        compiles sim_core.cpp + an UNCHANGED AutoPark.cpp into a shared library
  simulator/engine.py  ctypes bridge; every run loads a fresh copy (= power-on, no state carried over)
  simulator/arena.py   tape geometry from config/arena.json
  simulator/evaluator.py  judges runs from the simulated truth (wheels in PARK, route, turns, spins)
  simulator/scenarios.py  22 scenarios + Monte Carlo
  simulator/pid_sweep.py  steer-PID sweep (simulator-side gain override, never in the source)
  simulator/audit.py   header consistency, constants, plans, state transitions, source manifest
  simulator/viewer.py  matplotlib live viewer
  simulator/report.py  VALIDATION_REPORT.md
  config/              robot.json, sensor.json, arena.json, scenarios.json, montecarlo.json, experimental.json
  tests/               pytest
```

Every 10 ms tick follows the order documented in the SDK headers:

1. IR array read and processing (`HexaLineArray` rules: normalise, dark >= 250, centroid, lost after 1200 ms);
2. queued actions (`runAction`), then `loop()`;
3. `TankDrive` ramp;
4. physics.

Physics models skid-steer kinematics, per-side start thresholds (your Min Drive numbers), speed and response,
coast and slip. Encoder pulses go into AutoPark's own interrupt handlers on pins 37/36, including its 50 us glitch filter.

**What is not simulated:** the SDK's own line follower (`followToJunction`, etc.) is prebuilt ESP32 code. That only
affects the fallback OpModes "Park Run Tape" and "First Junction". The SDK `LineTuning.pid` (0.25 / 0.06) runs only
inside it, so it is documented but not swept.

## Updating after you measure the robot

Edit `value` in `config/robot.json` / `config/sensor.json`, and change `source` from `ASSUMPTION` to `measured`.
The most valuable values to measure:

1. `tau_coast_s`: measure the roll distance after the motors are cut at ROLL_SPEED 0.22.
2. `sensor_pitch_m`: IR channel spacing.
3. `wheelbase_m`: front-to-rear axle spacing.

Then re-run `python3 main.py report`.

## When the firmware or SDK changes

`config/source_manifest.json` stores SHA-256 hashes of `AutoPark.cpp`, the SDK headers and the simulator core.
`test_source_manifest_unchanged` fails when any of them change, because the report no longer describes the code.
To clear it: re-run `python3 main.py report`, review it, then run `python3 main.py audit --accept`.
`test_standin_matches_real_header` fails if a new SDK changes a struct, enum or constant that the stand-in mirrors.

## Rules this simulator follows

- `AutoPark.cpp` is never copied, edited or patched; the controller parameters are the source's.
- Experimental gains live only in `config/experimental.json` and are applied by the simulator at run time.
- A failed run stays failed. Tests that fail are firmware findings and are not skipped or marked "expected".
- No route is hard-coded: the firmware only ever sees the IR array, the encoders and the gyro.
