"""Judge one run from the physical truth (simulated pose) - never from what the firmware claims.

A run PASSES only if: the firmware reached DONE, every wheel contact point is inside the PARK
zone, the robot is stopped, it never left the planned route, it took no turn away from a planned
turn point, missed none, and did not spin excessively.
"""
import math
import re

import numpy as np

from .arena import dist_to_polyline

TURN_STATES = ("GYRO TURN", "TAPE TURN", "TURN", "TURN BACK")
OFF_ROUTE_MM = 250.0        # chassis middle further than this from the planned path = off route
TURN_MATCH_MM = 300.0       # a turn must start within this of a planned turn point


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def planned_turn_points(arena, opmode):
    c = arena.corners
    if "D1" in opmode:
        p1, d1 = arena.terminals["P1"], arena.terminals["D1"]
        return [("C1", c["C1"]), ("to P1", p1["attach"]), ("P1 end", p1["end"]), ("P1 back", p1["attach"]),
                ("to D1", d1["attach"]), ("D1 end", d1["end"]), ("D1 back", d1["attach"]),
                ("C2", c["C2"]), ("C3", c["C3"]), ("C4", c["C4"])]
    return [("C1", c["C1"]), ("C2", c["C2"]), ("C3", c["C3"]), ("C4", c["C4"])]


EVENT_RE = re.compile(r"^([\d.]+) STATE (.*?) -> (.*?) \[(.*?)\] why=(.*?) @\(([-\d.]+),([-\d.]+),([-\d.]+)deg\)")


def parse_events(text):
    ev = []
    for line in text.splitlines():
        m = EVENT_RE.match(line)
        if m:
            ev.append({"t": float(m.group(1)) - 1.0, "from": m.group(2), "to": m.group(3), "leg": m.group(4),
                       "why": m.group(5), "x": float(m.group(6)), "y": float(m.group(7)), "th": float(m.group(8))})
    return ev


def state_per_row(trace, tele_log):
    """State name for each trace row (from the telemetry change log)."""
    if trace is None or len(trace) == 0:
        return []
    times = [t for t, _ in tele_log]
    names = [d.get("state", "") for _, d in tele_log]
    idx = np.searchsorted(np.array(times), trace[:, 0], side="right") - 1
    return [names[i] if i >= 0 else "" for i in idx]


def evaluate(res, trace, arena, params):
    from .engine import IDX
    e = {}
    status = res["status"]
    tele = res.get("telemetry", {})
    e["status"] = status
    e["why"] = tele.get("why", "")
    e["turns_log"] = tele.get("turns", "")
    e["final_leg"] = tele.get("leg", "")
    if trace is None or len(trace) == 0:
        e.update(passed=False, reason="no trace")
        return e
    last = trace[-1]
    x, y, th = last[IDX["x"]] * 1000, last[IDX["y"]] * 1000, last[IDX["th_deg"]]
    e["time_s"] = float(last[IDX["t"]])
    e["final_pose_mm_deg"] = (round(float(x), 1), round(float(y), 1), round(float(wrap180(th)), 1))

    # ---- parking: all four wheel contact points inside PARK -------------------------------
    px0, py0, px1, py1 = arena.boxes["PARK"]
    cx, cy = arena.park_centre()
    hw = params["track_width_m"] * 500.0
    hl = params["wheelbase_m"] * 500.0
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    wheels = [(x + a * c - b * s, y + a * s + b * c) for a in (hl, -hl) for b in (hw, -hw)]
    inside = [px0 <= wx <= px1 and py0 <= wy <= py1 for wx, wy in wheels]
    e["wheels_inside"] = int(sum(inside))
    e["parked_inside"] = all(inside)
    e["lateral_err_mm"] = round(float(y - cy), 1)          # PARK is entered heading east: lateral = y
    e["longitudinal_err_mm"] = round(float(x - cx), 1)
    e["heading_err_deg"] = round(float(wrap180(th - 0.0)), 2)
    # "stopped": motors commanded off at the end AND the chassis has come to rest (2 s settle in the core)
    e["motors_off"] = bool(last[IDX["pwmL"]] == 0 and last[IDX["pwmR"]] == 0)
    e["stopped"] = bool(e["motors_off"] and abs(last[IDX["vL"]]) < 0.02 and abs(last[IDX["vR"]]) < 0.02)

    # ---- route / turns ------------------------------------------------------------------
    route = arena.route_polyline(res["opmode"])
    xs, ys = trace[:, IDX["x"]] * 1000, trace[:, IDX["y"]] * 1000
    sub = slice(None, None, 5)
    dev = [dist_to_polyline(a, b, route) for a, b in zip(xs[sub], ys[sub])]
    e["max_route_dev_mm"] = round(float(max(dev)), 1) if dev else 0.0
    e["off_route"] = e["max_route_dev_mm"] > OFF_ROUTE_MM

    ev = parse_events(res.get("events", ""))
    points = planned_turn_points(arena, res["opmode"])
    turns = [v for v in ev if v["to"] in TURN_STATES and v["from"] not in TURN_STATES]
    used = [False] * len(points)
    wrong, repeats = [], 0
    for t in turns:
        tx, ty = t["x"] * 1000, t["y"] * 1000
        near = [i for i, (_, (qx, qy)) in enumerate(points) if math.hypot(tx - qx, ty - qy) <= TURN_MATCH_MM]
        free = [i for i in near if not used[i]]
        if free:
            used[free[0]] = True          # planned points are consumed in route order
        elif near:
            repeats += 1                  # a second turn at a point already turned at (re-try)
        else:
            d, bi = min((math.hypot(tx - qx, ty - qy), i) for i, (_, (qx, qy)) in enumerate(points))
            wrong.append({"t": round(t["t"], 2), "at_mm": (round(tx), round(ty)), "nearest": points[bi][0], "dist_mm": round(d)})
    e["repeat_turns"] = repeats
    e["turns_taken"] = len(turns)
    e["incorrect_turns"] = wrong
    # a planned turn point is "missed" if no turn started near it (only up to the step reached)
    e["missed_turns"] = [points[i][0] for i in range(len(points)) if not used[i]]
    # rotation: total (info) and the longest CONTINUOUS rotation one way (spin detection).
    th_all = np.unwrap(np.radians(trace[:, IDX["th_deg"]])) * 180 / math.pi
    e["total_rotation_deg"] = round(float(np.sum(np.abs(np.diff(th_all)))), 1)
    # spin = rotation while the chassis stays within 10 cm of where the rotation started
    longest, a_i = 0.0, 0
    for i in range(1, len(th_all)):
        if math.hypot(xs[i] - xs[a_i], ys[i] - ys[a_i]) > 100.0:
            a_i = i
        longest = max(longest, abs(th_all[i] - th_all[a_i]))
    e["longest_spin_deg"] = round(float(longest), 1)
    max_single = 180.0 if "D1" in res["opmode"] else 90.0
    e["spin_limit_deg"] = max_single + 120.0       # turn + search window + margin
    e["excess_rotation"] = e["longest_spin_deg"] > e["spin_limit_deg"]

    # ---- states, recoveries, faults -----------------------------------------------------
    entries = {}
    for v in ev:
        entries[v["to"]] = entries.get(v["to"], 0) + 1
    e["state_entries"] = entries
    tl = e["turns_log"] or ""
    e["recoveries"] = tl.count("S") + tl.count("T")
    whys = [v["why"] for v in ev if v["why"] not in ("", "-")]
    e["fallback_reasons"] = sorted(set(whys))
    e["faults"] = 1 if status == "FAULT" else 0

    # ---- line following quality (FOLLOW state only) -------------------------------------
    st = state_per_row(trace, res.get("tele_log", []))
    fol = np.array([s == "FOLLOW" for s in st]) & (trace[:, IDX["on_line"]] > 0)
    err = trace[:, IDX["error"]]
    if fol.any():
        ef = err[fol]
        e["max_line_error"] = int(np.max(np.abs(ef)))
        e["rms_line_error"] = round(float(np.sqrt(np.mean(ef ** 2))), 1)
        # oscillation: sign changes beyond a +/-300 hysteresis, per metre followed
        sgn, flips = 0, 0
        for v in ef:
            ns = 1 if v > 300 else (-1 if v < -300 else sgn)
            if sgn != 0 and ns != sgn:
                flips += 1
            sgn = ns
        v_av = 0.5 * (np.abs(trace[:, IDX["vL"]]) + np.abs(trace[:, IDX["vR"]]))
        dist = float(np.sum(v_av[fol])) * 0.01
        e["follow_m"] = round(dist, 2)
        e["oscillations_per_m"] = round(flips / dist, 2) if dist > 0.1 else 0.0
    else:
        e.update(max_line_error=0, rms_line_error=0.0, follow_m=0.0, oscillations_per_m=0.0)
    # corner stability: for every FOLLOW leg that starts after a turn, the largest line error in its
    # first 1.5 s (overshoot / settling) and the heading error vs the grid when it starts
    starts = [i for i in range(1, len(st)) if st[i] == "FOLLOW" and st[i - 1] != "FOLLOW"]
    over, herr = [], []
    for i in starts[1:]:
        w = slice(i, min(len(st), i + 150))
        ok = trace[w, IDX["on_line"]] > 0
        if ok.any():
            over.append(float(np.max(np.abs(err[w][ok]))))
        h = trace[i, IDX["th_deg"]]
        herr.append(abs(wrap180(h - round(h / 90.0) * 90.0)))
    e["post_turn_overshoot"] = round(float(np.mean(over)), 1) if over else 0.0
    e["follow_entry_heading_err"] = round(float(np.mean(herr)), 2) if herr else 0.0

    # ---- verdict ------------------------------------------------------------------------
    reasons = []
    if status != "DONE":
        reasons.append(f"run ended {status}" + (f" ({e['why']})" if e["why"] not in ("", "-") else ""))
    if status == "DONE" and not e["parked_inside"]:
        reasons.append(f"DONE but only {e['wheels_inside']}/4 wheels inside PARK (pose {e['final_pose_mm_deg']})")
    if not e["motors_off"]:
        reasons.append("motors still driven at the end")
    elif not e["stopped"]:
        reasons.append("still rolling 2 s after the end")
    if e["off_route"]:
        reasons.append(f"left the route by {e['max_route_dev_mm']:.0f} mm")
    if wrong:
        reasons.append(f"{len(wrong)} turn(s) away from any planned turn point")
    if status == "DONE" and e["missed_turns"]:
        reasons.append("missed turn point(s): " + ",".join(e["missed_turns"]))
    if e["excess_rotation"]:
        reasons.append(f"spun {e['longest_spin_deg']:.0f} deg in one go (limit {e['spin_limit_deg']:.0f})")
    e["passed"] = not reasons
    # a FAULT stop with the robot stopped, in the field and on the route is a SAFE failure
    e["safe_stop"] = status == "FAULT" and e["stopped"] and not e["off_route"] and not wrong
    e["reason"] = "parked: all 4 wheels inside PARK" if e["passed"] else "; ".join(reasons)
    return e
