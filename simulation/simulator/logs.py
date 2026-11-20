"""Run logs: per-tick CSV (every motor command, sensor reading and state), the event log
(every state transition, step, turn, fault, final result) and a JSON summary."""
import csv
import json
import os
import time

from . import config as C
from .engine import IDX
from .evaluator import state_per_row

LOG_DIR = os.path.join(C.SIM_DIR, "logs")

CSV_COLS = ["t", "state", "x", "y", "th_deg", "cmdL", "cmdR", "pwmL", "pwmR", "vL", "vR", "pulsesL", "pulsesR",
            "heading_cw", "mask", "position", "error", "on_line", "pattern"] + [f"n{i}" for i in range(8)] + ["pidP", "pidD", "pidOut"]


def save_run(res, trace, tag=None, log_dir=LOG_DIR):
    os.makedirs(log_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    base = os.path.join(log_dir, f"{stamp}_{(tag or res.get('name', 'run')).replace(' ', '_').replace('/', '-')}")
    states = state_per_row(trace, res.get("tele_log", []))
    with open(base + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_COLS)
        for row, st in zip(trace, states):
            out = []
            for c in CSV_COLS:
                if c == "state":
                    out.append(st)
                elif c == "mask":
                    out.append(format(int(row[IDX["mask"]]), "08b")[::-1])   # ch0 first
                else:
                    v = row[IDX[c]]
                    out.append(round(float(v), 4))
            w.writerow(out)
    with open(base + ".events.log", "w") as f:
        f.write(res.get("events", ""))
        f.write(f"RESULT {res['status']}: {res['eval'].get('reason', '')}\n")
    summ = {k: v for k, v in res.items() if k not in ("trace", "events", "tele_log", "params")}
    summ["params_used"] = res.get("params")
    with open(base + ".json", "w") as f:
        json.dump(summ, f, indent=1, default=str)
    return base
