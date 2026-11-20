"""Load the JSON configs and flatten them into the key/value parameters the C++ core reads.

Every parameter carries its source (measured / derived / header / firmware / ASSUMPTION / fault),
so reports can show which results rest on unmeasured values.
"""
import copy
import json
import os

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(SIM_DIR, "config")
REPO_DIR = os.path.dirname(SIM_DIR)


def _load(name):
    with open(os.path.join(CONFIG_DIR, name)) as f:
        return json.load(f)


def load_all():
    return {"robot": _load("robot.json"), "sensor": _load("sensor.json"), "arena": _load("arena.json")}


def flat_params(cfg):
    """{key: value} for the C++ core, from robot.json + sensor.json (+ derived keys)."""
    out = {}
    for section in ("robot", "sensor"):
        for block in ("params", "simulation"):
            for k, v in cfg[section].get(block, {}).items():
                out[k] = float(v["value"])
    out["track_eff_m"] = out["track_width_m"] * out["skid_factor"]
    a = cfg["arena"]
    out["start_x_m"] = a["start_pose"]["x"] / 1000.0
    out["start_y_m"] = a["start_pose"]["y"] / 1000.0
    out["start_heading_deg"] = a["start_pose"]["heading_deg"]
    fx0, fy0, fx1, fy1 = a["field"]["rect"]
    out.update(field_x0_m=fx0 / 1000, field_y0_m=fy0 / 1000, field_x1_m=fx1 / 1000, field_y1_m=fy1 / 1000)
    return out


def sources(cfg):
    """[(key, value, source, note)] for the audit report."""
    rows = []
    for section in ("robot", "sensor"):
        for block in ("params", "simulation"):
            for k, v in cfg[section].get(block, {}).items():
                rows.append((k, v["value"], v["source"], v.get("note", "")))
    return rows


def apply_overrides(params, overrides):
    """Overrides: {key: value} sets, {key: {"mul": f}} scales, {key: {"add": d}} offsets."""
    p = dict(params)
    for k, v in (overrides or {}).items():
        if isinstance(v, dict):
            if "mul" in v:
                p[k] = p.get(k, 0.0) * v["mul"]
            if "add" in v:
                p[k] = p.get(k, 0.0) + v["add"]
        else:
            p[k] = float(v)
    if "track_width_m" in (overrides or {}) or "skid_factor" in (overrides or {}):
        p["track_eff_m"] = p["track_width_m"] * p["skid_factor"]
    return p


def load_json(path):
    with open(path) as f:
        return json.load(f)


def deep(x):
    return copy.deepcopy(x)
