#!/usr/bin/env python3
"""GLITCHED AutoPark SIL simulator - command line.

  python3 main.py view                              live viewer (Park Run, nominal robot)
  python3 main.py view --opmode "P1-D1 Run" --scenario 13
  python3 main.py run --opmode "Park Run" --scenario 4 --seed 3 --log
  python3 main.py suite                             22 fault scenarios x 3 seeds
  python3 main.py montecarlo --n 100
  python3 main.py sweep                             steer-PID Kp x Kd sweep + plot
  python3 main.py audit [--accept]                  header/source consistency + manifest
  python3 main.py report [--compare baseline]       everything -> reports/VALIDATION_REPORT.md
  python3 main.py test                              pytest suite

Firmware under test: --firmware PATH, or env GLITCH_FIRMWARE, else RobotFirmware/opmodes/AutoPark.cpp.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def _fw(a):
    if a.firmware:
        os.environ["GLITCH_FIRMWARE"] = os.path.abspath(a.firmware)
    from simulator import scenarios as S
    return S.default_firmware()


def cmd_run(a):
    from simulator import engine, scenarios as S
    from simulator.logs import save_run
    fw = _fw(a)
    if a.scenario:
        d = S.load_scenarios()
        sc = [s for s in d["scenarios"] if s["id"] == a.scenario][0]
        spec = S.scenario_spec(sc, d["bases"], a.seed, a.opmode, fw, keep_trace=True)
    else:
        spec = {"name": a.opmode, "opmode": a.opmode, "firmware": fw, "seed": a.seed, "keep_trace": True}
    r = engine.run(spec)
    print(r["events"])
    e = r["eval"]
    print(f"VERDICT {S.verdict(e)}: {e['reason']}")
    print(json.dumps({k: e[k] for k in ("time_s", "final_pose_mm_deg", "wheels_inside", "lateral_err_mm", "heading_err_deg",
                                        "turns_log", "recoveries", "max_line_error", "oscillations_per_m") if k in e}, default=str))
    if a.log:
        print("log:", save_run(r, r["trace"]))
    return 0 if e.get("passed") else 1


def cmd_view(a):
    from simulator.viewer import Viewer
    v = Viewer(firmware=_fw(a), opmode=a.opmode, scenario=a.scenario, headless=bool(a.png))
    if a.png:
        v.run_headless(a.seconds, a.png)
        print(a.png, v.result_text)
    else:
        v.show()


def cmd_suite(a):
    from simulator import report
    rows, _ = report.fault_suite(_fw(a), a.opmode, a.seeds)
    for s in rows:
        print(f"{s['id']:2d} {s['name']:42s} {s['verdict']:9s} {s['pass']}/{s['safe']}/{s['fail']}  {s['why'][:150]}")
    bad = [s for s in rows if s["verdict"] != "PASS"]
    print(f"\n{len(rows) - len(bad)}/{len(rows)} scenarios PASS on every seed")
    return 0 if not bad else 1


def cmd_mc(a):
    from simulator import engine, scenarios as S
    res = engine.run_many(S.mc_specs(a.n, a.opmode, _fw(a)), progress=None)
    print(json.dumps(S.summarise(res), indent=1))


def cmd_sweep(a):
    from simulator import pid_sweep, report
    sw = pid_sweep.sweep(_fw(a), a.opmode)
    os.makedirs(report.REPORT_DIR, exist_ok=True)
    pid_sweep.save(sw, os.path.join(report.REPORT_DIR, "pid_sweep.json"))
    print(pid_sweep.plot(sw, os.path.join(report.REPORT_DIR, "pid_sweep.png")))


def cmd_audit(a):
    from simulator import audit as A
    ok = True
    print("Stand-in SDK vs real headers:")
    for item, good, detail in A.header_consistency():
        ok &= good
        print(f"  [{'ok' if good else 'MISMATCH'}] {item}: {detail}")
    if a.accept:
        A.accept_manifest(a.note or "")
        print("manifest accepted")
    print("Source manifest:")
    for f, st in A.check_manifest():
        ok &= st == "ok"
        print(f"  [{st}] {f}")
    if not ok:
        print("\nSomething changed. Re-run `python3 main.py report`, check it, then `python3 main.py audit --accept`.")
    return 0 if ok else 1


def cmd_report(a):
    from simulator import report
    out = report.build(_fw(a), mc_runs=a.n, seeds=a.seeds, compare=a.compare)
    print(out)


def cmd_test(a):
    import pytest
    return pytest.main([os.path.join(HERE, "tests"), "-q"] + a.rest)


def cmd_build(a):
    from simulator import engine
    print(engine.library_for(_fw(a)))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--firmware", help="path of the AutoPark.cpp to test")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("run"); s.add_argument("--opmode", default="Park Run"); s.add_argument("--scenario", type=int); s.add_argument("--seed", type=int, default=1); s.add_argument("--log", action="store_true"); s.set_defaults(f=cmd_run)
    s = sub.add_parser("view"); s.add_argument("--opmode", default="Park Run"); s.add_argument("--scenario", type=int); s.add_argument("--png"); s.add_argument("--seconds", type=float, default=40); s.set_defaults(f=cmd_view)
    s = sub.add_parser("suite"); s.add_argument("--opmode", default="Park Run"); s.add_argument("--seeds", type=int, default=3); s.set_defaults(f=cmd_suite)
    s = sub.add_parser("montecarlo"); s.add_argument("--opmode", default="Park Run"); s.add_argument("--n", type=int, default=100); s.set_defaults(f=cmd_mc)
    s = sub.add_parser("sweep"); s.add_argument("--opmode", default="Park Run"); s.set_defaults(f=cmd_sweep)
    s = sub.add_parser("audit"); s.add_argument("--accept", action="store_true"); s.add_argument("--note"); s.set_defaults(f=cmd_audit)
    s = sub.add_parser("report"); s.add_argument("--n", type=int, default=60); s.add_argument("--seeds", type=int, default=3); s.add_argument("--compare", help="second firmware for comparison, e.g. 'baseline' or a path"); s.set_defaults(f=cmd_report)
    s = sub.add_parser("test"); s.add_argument("rest", nargs="*"); s.set_defaults(f=cmd_test)
    s = sub.add_parser("build"); s.set_defaults(f=cmd_build)
    a = p.parse_args()
    sys.exit(a.f(a) or 0)


if __name__ == "__main__":
    main()
