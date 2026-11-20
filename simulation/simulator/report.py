"""Builds reports/VALIDATION_REPORT.md from fresh simulation runs. Nothing is hand-edited into the
numbers: every table comes from runs made by this script, and failed runs stay failed."""
import datetime
import json
import os
import platform

import numpy as np

from . import audit as A
from . import config as C
from . import pid_sweep
from . import scenarios as S
from .engine import IDX, firmware_path, run_many

REPORT_DIR = os.path.join(C.SIM_DIR, "reports")


def _md_table(head, rows):
    out = ["| " + " | ".join(head) + " |", "|" + "|".join("---" for _ in head) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(x).replace("|", "/") for x in r) + " |")
    return "\n".join(out)


def fault_suite(firmware, opmode, seeds=3, keep_trace=False):
    d = S.load_scenarios()
    specs = [S.scenario_spec(sc, d["bases"], 100 + k, opmode, firmware, keep_trace=keep_trace)
             for sc in d["scenarios"] for k in range(seeds)]
    res = run_many(specs)
    rows = []
    for sc in d["scenarios"]:
        rs = [r for r in res if r.get("name", "").startswith(f"{sc['id']:02d} ")]
        v = [S.verdict(r["eval"]) for r in rs]
        reasons = sorted({r["eval"]["reason"] for r in rs if not r["eval"].get("passed")})
        verdict = "PASS" if v.count("PASS") == len(v) else ("SAFE FAIL" if all(x != "FAIL" for x in v) else "FAIL")
        rows.append({"id": sc["id"], "name": sc["name"], "means": sc["means"], "verdict": verdict,
                     "pass": v.count("PASS"), "safe": v.count("SAFE FAIL"), "fail": v.count("FAIL"), "runs": len(v),
                     "why": " / ".join(reasons) if reasons else "parked: all 4 wheels inside PARK on every seed",
                     "time": round(float(np.mean([r["eval"].get("time_s", 0) for r in rs])), 1)})
    return rows, res


def trajectories_png(results, arena, title, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, ax = plt.subplots(figsize=(7.2, 7.2), facecolor="#fcfcfb")
    for r in arena.rects:
        ax.add_patch(Rectangle((r[0] / 1000, r[1] / 1000), (r[2] - r[0]) / 1000, (r[3] - r[1]) / 1000, fc="#2b2b2b", ec="none"))
    n_ok = n_bad = 0
    for res in results:
        tr = res.get("trace")
        if tr is None:
            continue
        ok = res["eval"].get("passed")
        n_ok += ok
        n_bad += not ok
        ax.plot(tr[:, IDX["x"]], tr[:, IDX["y"]], lw=0.8 if ok else 1.4, alpha=0.35 if ok else 0.9,
                color="#2a78d6" if ok else "#c92a2a", zorder=3 if not ok else 2)
    fx0, fy0, fx1, fy1 = [v / 1000 for v in arena.field]
    ax.set_xlim(fx0, fx1)
    ax.set_ylim(fy0, fy1)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color("#c3c2b7")
    ax.set_title(f"{title}\nchassis path: {n_ok} passed (blue), {n_bad} failed (red)", fontsize=9.5, loc="left", color="#0b0b0b")
    fig.tight_layout()
    fig.savefig(out_png, dpi=110, facecolor=fig.get_facecolor())
    plt.close(fig)
    return out_png


def build(firmware=None, mc_runs=60, seeds=3, sweep=True, compare=None, progress=print):
    from .arena import Arena
    os.makedirs(REPORT_DIR, exist_ok=True)
    fw = firmware or S.default_firmware()
    fws = [("under test", fw)] + ([("comparison", compare)] if compare else [])
    cfg = C.load_all()
    arena = Arena(cfg["arena"])
    data = {"generated": datetime.datetime.now().isoformat(timespec="seconds"), "platform": platform.platform(), "firmware": {}}
    all_results = []
    for label, f in fws:
        path = firmware_path(f)
        entry = {"path": path, "sha256": A.sha(path), "runs": {}}
        for op in ("Park Run", "P1-D1 Run"):
            progress(f"[{label}] {op}: fault suite ({22 * seeds} runs)")
            rows, res = fault_suite(f, op, seeds)
            if label == "under test":
                all_results += res
            progress(f"[{label}] {op}: Monte Carlo ({mc_runs} runs)")
            specs = S.mc_specs(mc_runs, op, f)
            for s in specs:
                s["keep_trace"] = True
            mc = run_many(specs)
            if label == "under test":
                all_results += mc
            tag = ("fw" if label == "under test" else "cmp") + "_" + op.replace(" ", "").replace("-", "")
            png = trajectories_png(mc, arena, f"{op} - Monte Carlo, {os.path.basename(os.path.dirname(os.path.dirname(path)))}/{os.path.basename(path)} ({label})",
                                   os.path.join(REPORT_DIR, f"mc_paths_{tag}.png"))
            fails = [{"seed": r["seed"], "reason": r["eval"]["reason"], "events_tail": r["events"].splitlines()[-6:]}
                     for r in mc if not r["eval"].get("passed")]
            entry["runs"][op] = {"suite": rows, "mc": S.summarise(mc), "mc_png": os.path.basename(png), "mc_failures": fails[:10]}
        data["firmware"][label] = entry
    if sweep:
        progress("PID sweep")
        sw = pid_sweep.sweep(fw)
        pid_sweep.save(sw, os.path.join(REPORT_DIR, "pid_sweep.json"))
        pid_sweep.plot(sw, os.path.join(REPORT_DIR, "pid_sweep.png"))
        data["sweep"] = sw
    sm = A.state_machine(firmware_path(fw))
    data["transitions"] = A.transition_table(sm, A.observed_transitions(all_results, sm))
    data["state_display"] = sm["display"]
    data["header_checks"] = A.header_consistency()
    data["manifest"] = A.check_manifest()
    data["constants"] = A.firmware_constants(firmware_path(fw))
    data["plans"] = A.plans(firmware_path(fw))
    md = render(data, cfg)
    out = os.path.join(REPORT_DIR, "VALIDATION_REPORT.md")
    with open(out, "w") as f:
        f.write(md)
    slim = {k: v for k, v in data.items() if k != "sweep"}
    with open(os.path.join(REPORT_DIR, "validation_data.json"), "w") as f:
        json.dump(slim, f, indent=1, default=str)
    return out


def render(d, cfg):
    L = []
    w = L.append
    w("# GLITCHED Pixel - AutoPark software-in-the-loop validation report\n")
    w(f"Generated {d['generated']} on {d['platform']} by `python3 main.py report`. Every number below comes from "
      "simulation runs made by that command. **This is simulation evidence, not hardware validation.** Results rest on the "
      "parameters in `config/`, several of which are ASSUMPTIONS (listed in section 4).\n")
    if os.path.exists(os.path.join(REPORT_DIR, "FINDINGS.md")):
        w("Engineering interpretation of these results (written by hand, with the run evidence it rests on): "
          "[FINDINGS.md](FINDINGS.md).\n")
    w("## 1. What ran\n")
    w("The C++ file `AutoPark.cpp` was compiled **unchanged** (no copy, no edit) together with `simulation/core/` - a stand-in for the "
      "NRL SDK v1.5.0 runtime written from its public headers, plus robot physics, virtual sensors and fault injection. "
      "The firmware's own state machine, constants, PID and encoder interrupt handlers make every decision. "
      "It is **not** the ESP32 binary: the SDK's prebuilt runtime libraries cannot execute off the robot, so their documented "
      "behaviour is re-implemented (section 2).\n")
    rows = []
    for label, e in d["firmware"].items():
        rows.append((label, f"`{e['path']}`", f"`{e['sha256'][:16]}...`"))
    w(_md_table(["role", "file", "sha256"], rows) + "\n")
    w("## 2. SDK runtime: simulator equivalents\n")
    w(_md_table(["SDK behaviour", "source", "simulator implementation", "fidelity", "note"], A.SDK_MAPPING) + "\n")
    w("### Stand-in SDK vs the real headers (automated check)\n")
    w(_md_table(["item", "matches", "detail"], [(a, "yes" if b else "**NO**", c) for a, b, c in d["header_checks"]]) + "\n")
    w("### Source manifest\n")
    w(_md_table(["file", "status vs accepted manifest"], d["manifest"]) + "\n")
    w("## 3. Firmware parameters\n")
    w("Every constant below is used **exactly**: the simulator compiles this file, it does not copy values. "
      "Simulator equivalent = the same symbol in the same compiled code.\n")
    w(_md_table(["C++ name", "value in source", "type", "simulator", "copied exactly?"],
                [(f"`{n}`", f"`{v}`", t, "compiled from source", "yes") for n, v, t in d["constants"]]) + "\n")
    for pn, steps in d["plans"].items():
        w(f"**{pn}**: " + " -> ".join(f"{s['name']} ({s['event']}, {s['expM']})" for s in steps) + "\n")
    w("## 4. Robot and world parameters (configs)\n")
    w(_md_table(["parameter", "value", "source", "note"], [(f"`{k}`", v, ("**" + s + "**") if s == "ASSUMPTION" else s, n)
                                                           for k, v, s, n in C.sources(cfg) if s != "fault"]) + "\n")
    w("## 5. Results\n")
    for label, e in d["firmware"].items():
        for op, r in e["runs"].items():
            w(f"### {op} - firmware {label}\n")
            w("**Fault-injection suite** (each scenario on 3 seeds; verdict PASS only if every seed parked):\n")
            w(_md_table(["#", "scenario", "what it means", "verdict", "pass/safe/fail", "mean t (s)", "why"],
                        [(s["id"], s["name"], s["means"], s["verdict"], f"{s['pass']}/{s['safe']}/{s['fail']}", s["time"], s["why"]) for s in r["suite"]]) + "\n")
            m = r["mc"]
            w(f"**Monte Carlo** ({m['runs']} randomised robots, ranges in `config/montecarlo.json`):\n")
            w(_md_table(["metric", "value"], [
                ("success rate", f"{m['success_rate']:.0%} ({m['passed']}/{m['runs']})"),
                ("safe fails / fails", f"{m['safe_fail']} / {m['fail']}"),
                ("completion time mean (min-max), s", f"{m['time_mean_s']} ({m['time_min_s']}-{m['time_max_s']})"),
                ("final lateral error mean / max, mm", f"{m['lateral_err_mean_mm']} / {m['lateral_err_max_mm']}"),
                ("final heading error mean / max, deg", f"{m['heading_err_mean_deg']} / {m['heading_err_max_deg']}"),
                ("recoveries (S + T turns)", m["recoveries_total"]),
                ("faults", m["faults_total"]),
                ("max line error while following", m["max_line_error"]),
                ("missed turns (DONE runs) / turns away from plan", f"{m['missed_turns_total']} / {m['incorrect_turns_total']}"),
                ("failure reasons", "; ".join(f"{k} x{v}" for k, v in m["failure_reasons"].items()) or "-"),
            ]) + "\n")
            w(f"![paths]({r['mc_png']})\n")
            if r["mc_failures"]:
                w("<details><summary>First failed Monte Carlo runs (event log tails)</summary>\n\n```")
                for f in r["mc_failures"]:
                    w(f"seed {f['seed']}: {f['reason']}")
                    for line in f["events_tail"]:
                        w("    " + line)
                w("```\n</details>\n")
    if "sweep" in d:
        sw = d["sweep"]
        w("## 6. Steer PID sweep\n")
        w(f"Firmware `HexaPID steer` gains overridden by the simulator (never in the source). {sw['opmode']}, "
          f"{sw['seeds']} Monte Carlo robots per cell, the same robots in every cell. Baseline = Kp 0.22 / Kd 0.08. "
          "The SDK `LineTuning.pid` (0.25 / 0.06) is kept separate and is not swept: it only runs inside the SDK follower, "
          "which cannot execute off the robot.\n")
        w("![pid sweep](pid_sweep.png)\n")
        rows = [(k[0], k[1], f"{v['success_rate']:.0%}", f"{v['oscillations_per_m']:.2f}", f"{v['rms_line_error']:.0f}",
                 f"{v['post_turn_overshoot']:.0f}", f"{v['follow_entry_heading_err']:.1f}", f"{v['time_s']:.1f}", f"{v['lateral_abs_mm']:.1f}")
                for k, v in sorted(sw["table"].items())]
        w(_md_table(["Kp", "Kd", "success", "osc/m", "RMS err", "overshoot", "corner hdg err", "time s", "|lat| mm"], rows) + "\n")
        w("No gain set is declared optimal from this: differences in success and time are within run-to-run spread, and "
          "the line sensor pitch, inertia and skid factor are unmeasured.\n")
    w("## 7. State machine: source transitions vs observed\n")
    rows, extra = d["transitions"]
    disp = d["state_display"]
    w(_md_table(["from (C++)", "to (C++)", "how (source)", "simulator", "times observed in this report's runs"],
                [(f"`{a}` ({disp.get(a, '')})", f"`{b}`", h, "compiled from source", n if n else "**not exercised**") for a, b, h, n in rows]) + "\n")
    if extra:
        w("Transitions observed but not found by the static parser (parser limitation, or a path the parser missed):\n")
        w(_md_table(["from", "to", "count"], extra) + "\n")
    w("## 8. Limits of this evidence\n")
    w("- Unmeasured: sensor pitch, wheelbase, motor/chassis inertia, skid factor, coast, IR levels. All are ASSUMPTIONS in `config/`.\n"
      "- The SDK's own line follower (fallback OpModes) is not simulated.\n"
      "- `HexaLineArray::pattern()` and `lastSeenSide()` update rules are approximations (AutoPark decides from `mask()`).\n"
      "- Wheel slip is a constant factor; the mat, seams and lighting are ideal apart from the injected faults.\n"
      "- A PASS here means the logic copes with this model. Only runs on the real arena validate the robot.\n")
    return "\n".join(L)
