"""The 22 required scenarios. Two requirements per scenario:
  parks - every seed parks with all 4 wheels inside PARK (competition requirement)
  safe  - no seed leaves the field, keeps driving after the end, or spins in place
A failing test here is a finding about the FIRMWARE under these conditions, not a test bug.
Failures are reported, never skipped or marked expected."""
import pytest

from simulator import engine, scenarios as S

D = S.load_scenarios()
SEEDS = (100, 101, 102)
_cache = {}


def _runs(firmware, sc):
    key = (firmware, sc["id"])
    if key not in _cache:
        specs = [S.scenario_spec(sc, D["bases"], s, "Park Run", firmware) for s in SEEDS]
        _cache[key] = engine.run_many(specs, workers=1)
    return _cache[key]


@pytest.mark.parametrize("sc", D["scenarios"], ids=lambda s: f"{s['id']:02d}-{s['name']}")
def test_parks(firmware, sc):
    bad = [(r["seed"], r["eval"]["reason"]) for r in _runs(firmware, sc) if not r["eval"]["passed"]]
    assert not bad, f"{sc['name']} ({sc['means']}): " + "; ".join(f"seed {s}: {why}" for s, why in bad)


@pytest.mark.parametrize("sc", D["scenarios"], ids=lambda s: f"{s['id']:02d}-{s['name']}")
def test_safe(firmware, sc):
    problems = []
    for r in _runs(firmware, sc):
        e = r["eval"]
        if r["status"] in ("LEFT FIELD", "TIMEOUT", "CRASH"):
            problems.append(f"seed {r['seed']}: {r['status']}")
        if not e.get("motors_off", True):
            problems.append(f"seed {r['seed']}: motors still driven after the end")
        if e.get("excess_rotation"):
            problems.append(f"seed {r['seed']}: spun {e['longest_spin_deg']} deg in place")
        if r["status"] == "STUCK":
            problems.append(f"seed {r['seed']}: stuck without DONE/FAULT")
    assert not problems, "; ".join(problems)
