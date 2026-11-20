"""PID handling: the baseline gains come from the source, overrides are simulator-only and isolated."""
from simulator import audit as A
from simulator import engine


def _gains(firmware, overrides=None):
    r = engine.run({"opmode": "Park Run", "firmware": firmware, "seed": 1, "overrides": overrides or {}, "keep_trace": True})
    t = r["trace"][-1]
    return round(t[engine.IDX["kP"]], 4), round(t[engine.IDX["kI"]], 4), round(t[engine.IDX["kD"]], 4)


def test_baseline_gains_are_the_sources(firmware):
    src = [v for n, v, _ in A.firmware_constants(engine.firmware_path(firmware)) if n == "HexaPID steer"][0]
    kp, ki, kd = _gains(firmware)
    assert f"{kp:g}" in src and f"{kd:g}" in src and ki == 0.0, (src, kp, ki, kd)


def test_override_reaches_firmware_pid(firmware):
    assert _gains(firmware, {"pid0_kp": 0.4, "pid0_kd": 0.12})[0::2] == (0.4, 0.12)


def test_override_does_not_leak(firmware):
    base = _gains(firmware)
    _gains(firmware, {"pid0_kp": 0.4, "pid0_kd": 0.12})
    assert _gains(firmware) == base


def test_sdk_linetuning_pid_kept_separate(firmware):
    rows = dict((n, v) for n, v, _ in A.firmware_constants(engine.firmware_path(firmware)))
    assert "0.25" in rows["LineTuning.pid"] and "0.06" in rows["LineTuning.pid"]
