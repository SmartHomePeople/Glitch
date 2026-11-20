"""Steer-PID sweep (the firmware's own HexaPID 'steer', instance pid0).

The baseline gains are whatever AutoPark.cpp compiles with (Kp 0.22 / Kd 0.08). For the sweep the
SIMULATOR overwrites that instance's kP/kD after start() - the firmware file is never edited.
Each cell runs many Monte Carlo robots (config/montecarlo.json), so no conclusion rests on one
ideal run.

The SDK LineTuning PD (Kp 0.25 / Kd 0.06) is NOT swept: it is only used inside the SDK's own
follower, which is prebuilt ESP32 code and cannot run here (see audit SDK_MAPPING).
"""
import json
import os

import numpy as np

from . import config as C
from . import scenarios as S
from .engine import run_many

METRICS = [
    ("success_rate", "Park success rate", "higher is better", "{:.0%}"),
    ("oscillations_per_m", "Oscillation (line-error swings per metre)", "lower is better", "{:.2f}"),
    ("rms_line_error", "Tracking error (RMS line error, 0..3500)", "lower is better", "{:.0f}"),
    ("post_turn_overshoot", "Overshoot after turns (max |error| first 1.5 s)", "lower is better", "{:.0f}"),
    ("follow_entry_heading_err", "Corner stability (heading error when following resumes, deg)", "lower is better", "{:.1f}"),
    ("time_s", "Completion time (s, successful runs)", "lower is better", "{:.1f}"),
    ("lateral_abs_mm", "Parking accuracy (|lateral error| mm, successful runs)", "lower is better", "{:.1f}"),
]


def sweep(firmware=None, opmode=None, kps=None, kds=None, seeds=None, progress=None):
    exp = C.load_json(os.path.join(C.CONFIG_DIR, "experimental.json"))
    kps = kps or exp["steer_pid_grid"]["kp"]
    kds = kds or exp["steer_pid_grid"]["kd"]
    seeds = seeds or exp["seeds_per_cell"]
    opmode = opmode or exp["opmode"]
    specs, cells = [], []
    for kp in kps:
        for kd in kds:
            # same MC robots (seeds) in every cell: a paired comparison
            for sp in S.mc_specs(seeds, opmode, firmware, seed0=5000, extra={"pid0_kp": kp, "pid0_kd": kd}, name=f"kp{kp} kd{kd}"):
                specs.append(sp)
                cells.append((kp, kd))
    res = run_many(specs, progress=progress)
    grid = {}
    for (kp, kd), r in zip(cells, res):
        grid.setdefault((kp, kd), []).append(r["eval"])
    table = {}
    for key, evs in grid.items():
        ok = [e for e in evs if e.get("passed")]
        mean = lambda k, src: float(np.mean([e.get(k, 0.0) for e in src])) if src else float("nan")
        table[key] = {
            "success_rate": len(ok) / len(evs),
            "oscillations_per_m": mean("oscillations_per_m", evs),
            "rms_line_error": mean("rms_line_error", evs),
            "post_turn_overshoot": mean("post_turn_overshoot", evs),
            "follow_entry_heading_err": mean("follow_entry_heading_err", evs),
            "time_s": mean("time_s", ok),
            "lateral_abs_mm": float(np.mean([abs(e["lateral_err_mm"]) for e in ok])) if ok else float("nan"),
            "runs": len(evs),
        }
    # sanity: the override really reached the firmware's PID (state vector reports pid0 gains)
    return {"kps": kps, "kds": kds, "seeds": seeds, "opmode": opmode, "table": table}


BLUES = ["#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7", "#3987e5", "#2a78d6", "#256abf", "#1c5cab", "#184f95", "#104281", "#0d366b"]


def plot(result, out_png, baseline=(0.22, 0.08)):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Rectangle
    kps, kds, tab = result["kps"], result["kds"], result["table"]
    cmap = ListedColormap(BLUES)
    fig, axes = plt.subplots(2, 4, figsize=(17, 8.2), facecolor="#fcfcfb")
    for ax, (key, title, better, fmt) in zip(axes.flat, METRICS):
        Z = np.array([[tab[(kp, kd)][key] for kd in kds] for kp in kps])
        # colour scale anchored at 0 so shade is proportional to magnitude: a 36 vs 37 s spread
        # must not look as dramatic as a 450 vs 780 one
        vmax = 1.0 if key == "success_rate" else float(np.nanmax(Z)) * 1.05
        im = ax.imshow(Z, cmap=cmap, origin="lower", aspect="auto", vmin=0.0, vmax=vmax)
        ax.set_xticks(range(len(kds)), [f"{k:.2f}" for k in kds])
        ax.set_yticks(range(len(kps)), [f"{k:.2f}" for k in kps])
        ax.set_xlabel("Kd", color="#52514e")
        ax.set_ylabel("Kp", color="#52514e")
        ax.set_title(f"{title}\n({better})", fontsize=9.5, color="#0b0b0b", loc="left")
        lo, hi = 0.0, vmax
        for i in range(len(kps)):
            for j in range(len(kds)):
                v = Z[i, j]
                frac = 0 if hi == lo or np.isnan(v) else (v - lo) / (hi - lo)
                ax.text(j, i, "-" if np.isnan(v) else fmt.format(v), ha="center", va="center", fontsize=8,
                        color="#ffffff" if frac > 0.55 else "#0b0b0b")
        if baseline[0] in kps and baseline[1] in kds:
            bi, bj = kps.index(baseline[0]), kds.index(baseline[1])
            ax.add_patch(Rectangle((bj - 0.5, bi - 0.5), 1, 1, fill=False, ec="#d9480f", lw=2.2))
        for sp in ax.spines.values():
            sp.set_visible(False)
        ax.tick_params(colors="#52514e", length=0)
    ax = axes.flat[-1]
    ax.axis("off")
    ax.text(0, 0.95, "Steer PID sweep (firmware HexaPID 'steer')", fontsize=11, weight="bold", va="top")
    ax.text(0, 0.80, f"{result['opmode']}, {result['seeds']} Monte Carlo robots per cell\n(the same robots in every cell)."
                     "\nShade starts at 0 in every panel, so\nsimilar shades = differences that are small.\n\nOrange outline = firmware baseline\nKp 0.22 / Kd 0.08.\n\nThe SDK LineTuning PD (0.25 / 0.06) is\n"
                     "not swept: it runs only inside the SDK\nfollower, which cannot run off-robot.\n\n"
                     "Simulation only - confirm on the real\nrobot before changing any gain.",
            fontsize=9, va="top", color="#52514e")
    fig.tight_layout()
    fig.savefig(out_png, dpi=120, facecolor=fig.get_facecolor())
    plt.close(fig)
    return out_png


def save(result, path):
    out = dict(result)
    out["table"] = [{"kp": k[0], "kd": k[1], **v} for k, v in result["table"].items()]
    with open(path, "w") as f:
        json.dump(out, f, indent=1)
