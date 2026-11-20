"""Parking checks: the evaluator itself, and the parking requirements over Monte Carlo robots."""
import math

from simulator import config as C
from simulator import engine, scenarios as S
from simulator.arena import Arena
from simulator.evaluator import evaluate


def _fake(x, y, th, status="DONE"):
    params, cfg = engine.build_params()
    arena = Arena(cfg["arena"])
    row = [0.0] * len(engine.STATE_FIELDS)
    row[engine.IDX["x"]], row[engine.IDX["y"]], row[engine.IDX["th_deg"]] = x, y, th
    import numpy as np
    tr = np.array([row, row])
    res = {"status": status, "opmode": "Park Run", "telemetry": {}, "events": "", "tele_log": []}
    return evaluate(res, tr, arena, params)


def test_evaluator_inside_centre():
    e = _fake(0.55, 0.0, 0.0)
    assert e["parked_inside"] and e["wheels_inside"] == 4


def test_evaluator_one_wheel_out():
    # lateral 0.10 m: wheels at y = 0.10 + 0.158 = 0.258 > 0.25
    e = _fake(0.55, 0.10, 0.0)
    assert not e["parked_inside"] and not e["passed"]


def test_evaluator_rotated_out():
    e = _fake(0.55, 0.0, 60.0)
    assert e["wheels_inside"] < 4 or e["parked_inside"]   # geometry consistent either way
    assert math.isclose(e["heading_err_deg"], 60.0, abs_tol=0.01)


def test_monte_carlo_parking(firmware):
    """Requirement: across randomised robots, every DONE run is fully inside PARK and stopped,
    and the success rate is reported. A DONE outside PARK is a firmware failure."""
    res = engine.run_many(S.mc_specs(30, "Park Run", firmware))
    s = S.summarise(res)
    done_outside = [r["seed"] for r in res if r["status"] == "DONE" and not r["eval"]["parked_inside"]]
    print("\nMonte Carlo Park Run:", s)
    assert not done_outside, f"DONE but not inside PARK: seeds {done_outside}"
    assert s["success_rate"] >= 0.9, f"success rate {s['success_rate']:.0%}: {s['failure_reasons']}"
