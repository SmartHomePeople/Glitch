"""Scenario + Monte Carlo spec builders and summaries."""
import os
import random
import statistics as stats

from . import config as C

CANDIDATE_ENV = "GLITCH_FIRMWARE"   # optional: path of an AutoPark.cpp to test instead of the repo's


def default_firmware():
    return os.environ.get(CANDIDATE_ENV) or "baseline"


def load_scenarios():
    return C.load_json(os.path.join(C.CONFIG_DIR, "scenarios.json"))


def scenario_spec(sc, bases, seed, opmode="Park Run", firmware=None, keep_trace=False):
    ov = dict(bases[sc["base"]])
    ov.update(sc["set"])
    return {"name": f"{sc['id']:02d} {sc['name']}", "scenario_id": sc["id"], "opmode": opmode,
            "firmware": firmware or default_firmware(), "seed": seed, "overrides": ov,
            "park_shift_mm": sc.get("park_shift_mm", 0.0), "keep_trace": keep_trace}


def verdict(e):
    if e.get("passed"):
        return "PASS"
    if e.get("safe_stop"):
        return "SAFE FAIL"
    return "FAIL"


def mc_overrides(rng, mc, base=None):
    nominal, _ = None, None
    params = C.flat_params(C.load_all())
    ov = dict(base or {})
    for k, r in mc["uniform"].items():
        cur = ov.get(k, params.get(k, 0.0))
        if isinstance(cur, dict):
            cur = params.get(k, 0.0)
        if "mul" in r:
            ov[k] = cur * rng.uniform(*r["mul"])
        elif "add" in r:
            ov[k] = cur + rng.uniform(*r["add"])
        else:
            ov[k] = rng.uniform(*r["lohi"])
    return ov


def mc_specs(n, opmode="Park Run", firmware=None, seed0=1000, extra=None, name="MC"):
    mc = C.load_json(os.path.join(C.CONFIG_DIR, "montecarlo.json"))
    specs = []
    for i in range(n):
        rng = random.Random(seed0 + i)
        ov = mc_overrides(rng, mc)
        if extra:
            ov.update(extra)
        specs.append({"name": f"{name} {i}", "opmode": opmode, "firmware": firmware or default_firmware(),
                      "seed": seed0 + i, "overrides": ov})
    return specs


def summarise(results):
    """Monte Carlo statistics required by the spec."""
    ev = [r["eval"] for r in results]
    n = len(ev)
    ok = [e for e in ev if e.get("passed")]
    times = [e["time_s"] for e in ok]
    lat = [abs(e["lateral_err_mm"]) for e in ok]
    hdg = [abs(e["heading_err_deg"]) for e in ok]

    def m(v):
        return round(stats.mean(v), 2) if v else None

    out = {
        "runs": n,
        "passed": len(ok),
        "success_rate": round(len(ok) / n, 3) if n else 0,
        "safe_fail": sum(1 for e in ev if not e.get("passed") and e.get("safe_stop")),
        "fail": sum(1 for e in ev if not e.get("passed") and not e.get("safe_stop")),
        "time_mean_s": m(times), "time_min_s": min(times) if times else None, "time_max_s": max(times) if times else None,
        "lateral_err_mean_mm": m(lat), "lateral_err_max_mm": max(lat) if lat else None,
        "heading_err_mean_deg": m(hdg), "heading_err_max_deg": max(hdg) if hdg else None,
        "recoveries_total": sum(e.get("recoveries", 0) for e in ev),
        "faults_total": sum(e.get("faults", 0) for e in ev),
        "max_line_error": max((e.get("max_line_error", 0) for e in ev), default=0),
        "missed_turns_total": sum(len(e.get("missed_turns", [])) for e in ev if e.get("status") == "DONE"),
        "incorrect_turns_total": sum(len(e.get("incorrect_turns", [])) for e in ev),
        "failure_reasons": {},
    }
    for e in ev:
        if not e.get("passed"):
            key = e.get("reason", "?").split(";")[0][:90]
            out["failure_reasons"][key] = out["failure_reasons"].get(key, 0) + 1
    return out
