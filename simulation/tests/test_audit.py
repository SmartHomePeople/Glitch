"""Source-to-simulator consistency. These fail when the SDK headers or the firmware change."""
import pytest

from simulator import audit as A
from simulator import engine


@pytest.mark.parametrize("item,ok,detail", A.header_consistency())
def test_standin_matches_real_header(item, ok, detail):
    assert ok, f"stand-in SDK out of date for {item}: {detail}. Update simulation/core/sdk/NRL.h + sim_core.cpp."


def test_source_manifest_unchanged():
    changed = [(f, s) for f, s in A.check_manifest() if s != "ok"]
    assert not changed, ("Sources changed since the simulator evidence was accepted: "
                         f"{changed}. Re-run `python3 main.py report`, review it, then `python3 main.py audit --accept`.")


def test_firmware_compiles_and_registers_opmodes(firmware):
    sim = engine.Sim(engine.library_for(firmware))
    names = [n for n, _ in sim.opmodes()]
    for required in ("Park Run", "P1-D1 Run", "First Junction"):
        assert required in names, f"{required} not registered; found {names}"


def test_plan_parsed(firmware):
    plans = A.plans(engine.firmware_path(firmware))
    assert [s["name"] for s in plans["PARK_PLAN"]] == ["C1", "C2", "C3", "C4", "BOX"]


def test_every_state_has_a_screen_name(firmware):
    sm = A.state_machine(engine.firmware_path(firmware))
    missing = [s for s in sm["states"] if s not in sm["display"]]
    assert not missing, f"states without stateName(): {missing}"
