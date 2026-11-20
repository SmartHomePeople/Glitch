"""Ideal-condition behaviour and determinism."""
import pytest

from simulator import engine, scenarios as S

D = S.load_scenarios()
IDEAL = D["scenarios"][0]


@pytest.mark.parametrize("seed", [100, 101, 102])
@pytest.mark.parametrize("opmode", ["Park Run", "P1-D1 Run"])
def test_ideal_parks(firmware, opmode, seed):
    r = engine.run(S.scenario_spec(IDEAL, D["bases"], seed, opmode, firmware))
    e = r["eval"]
    assert e["passed"], f"{opmode} seed {seed}: {e['reason']}\n" + r["events"][-1500:]


def test_every_planned_step_reached(firmware):
    r = engine.run(S.scenario_spec(IDEAL, D["bases"], 100, "Park Run", firmware))
    legs = [l.split(" STEP ")[1].split(" exp")[0] for l in r["events"].splitlines() if " STEP " in l]
    assert legs == ["C1", "C2", "C3", "C4", "BOX"], legs


def test_deterministic(firmware):
    a = engine.run({"opmode": "Park Run", "firmware": firmware, "seed": 7})
    b = engine.run({"opmode": "Park Run", "firmware": firmware, "seed": 7})
    assert a["eval"]["final_pose_mm_deg"] == b["eval"]["final_pose_mm_deg"]
    assert a["events"] == b["events"]


def test_runs_are_isolated(firmware):
    """A run must start from power-on state even after another run in the same process."""
    engine.run({"opmode": "P1-D1 Run", "firmware": firmware, "seed": 3})
    a = engine.run({"opmode": "Park Run", "firmware": firmware, "seed": 7})
    b = engine.run({"opmode": "Park Run", "firmware": firmware, "seed": 7})
    assert a["events"] == b["events"]
