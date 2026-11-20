"""ctypes bridge to the C++ core (unchanged AutoPark.cpp + stand-in SDK + physics).

Fresh firmware state for every run: AutoPark.cpp keeps its state in file-level statics (learned
trims, plan index...). Each Sim() loads its OWN copy of the shared library, so every run starts
from a power-on state, exactly like switching the Hub on. Nothing is carried between runs.
"""
import concurrent.futures as cf
import ctypes
import hashlib
import multiprocessing as mp
import os
import shutil
import sys
import tempfile
import uuid

import numpy as np

from . import config as C
from .arena import Arena

sys.path.insert(0, os.path.join(C.SIM_DIR, "core"))
import build as _build  # noqa: E402

STATE_FIELDS = (["t", "x", "y", "th_deg", "vL", "vR", "cmdL", "cmdR", "pwmL", "pwmR", "pulsesL", "pulsesR",
                 "heading_cw", "mask", "position", "error", "on_line", "pattern"]
                + [f"n{i}" for i in range(8)]
                + [f"s{i}{a}" for i in range(8) for a in "xy"]
                + ["kP", "kI", "kD", "pidP", "pidD", "pidOut", "status"])
IDX = {k: i for i, k in enumerate(STATE_FIELDS)}
STATUS = {0: "RUNNING", 1: "DONE", 2: "FAULT", 3: "TIMEOUT", 4: "LEFT FIELD", 5: "STUCK", 6: "NO OPMODE"}
PATTERNS = ["LOST", "LINE", "LEFT_BRANCH", "RIGHT_BRANCH", "TERMINAL"]

BASELINE_SRC = os.path.join(C.REPO_DIR, "RobotFirmware", "opmodes", "AutoPark.cpp")


def firmware_path(fw):
    if fw in (None, "", "baseline"):
        return BASELINE_SRC
    return os.path.abspath(fw)


def library_for(fw):
    """Build (cached) the core around the given AutoPark.cpp and return the library path."""
    src = firmware_path(fw)
    tag = "baseline" if src == BASELINE_SRC else "fw_" + hashlib.sha1(src.encode()).hexdigest()[:10]
    return _build.build(src, os.path.join(C.SIM_DIR, "build", tag), quiet=True)


class Sim:
    def __init__(self, lib_path):
        tmp = os.path.join(tempfile.gettempdir(), "glitchsim_" + uuid.uuid4().hex + os.path.splitext(lib_path)[1])
        shutil.copyfile(lib_path, tmp)
        self.lib = ctypes.CDLL(tmp)
        try:
            os.unlink(tmp)      # already mapped; the file is no longer needed
        except OSError:
            pass
        L = self.lib
        L.sim_set_param.argtypes = [ctypes.c_char_p, ctypes.c_double]
        L.sim_arena_rect.argtypes = [ctypes.c_double] * 4
        L.sim_start.argtypes = [ctypes.c_char_p]
        L.sim_start.restype = ctypes.c_int
        L.sim_step.argtypes = [ctypes.c_int]
        L.sim_step.restype = ctypes.c_int
        L.sim_get_state.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int]
        L.sim_get_state.restype = ctypes.c_int
        for f in ("sim_telemetry", "sim_events", "sim_list_opmodes"):
            getattr(L, f).restype = ctypes.c_char_p
        self._buf = (ctypes.c_double * 128)()

    def opmodes(self):
        out = []
        for line in self.lib.sim_list_opmodes().decode().splitlines():
            n, t = line.rsplit("|", 1)
            out.append((n, t))
        return out

    def configure(self, params, arena):
        for k, v in params.items():
            self.lib.sim_set_param(k.encode(), float(v))
        self.lib.sim_arena_clear()
        for r in arena.rects:
            self.lib.sim_arena_rect(*r)

    def start(self, opmode):
        return self.lib.sim_start(opmode.encode())

    def step(self, n=1):
        return self.lib.sim_step(n)

    def state(self):
        n = self.lib.sim_get_state(self._buf, 128)
        return np.frombuffer(self._buf, dtype=np.float64, count=n).copy()

    def telemetry(self):
        d = {}
        for line in self.lib.sim_telemetry().decode(errors="replace").splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                d[k] = v
        return d

    def events(self):
        return self.lib.sim_events().decode(errors="replace")

    def stop(self):
        self.lib.sim_stop()


def build_params(overrides=None, cfg=None):
    cfg = cfg or C.load_all()
    return C.apply_overrides(C.flat_params(cfg), overrides), cfg


def run(spec):
    """One complete run. spec keys: opmode, firmware, seed, overrides, park_shift_mm, keep_trace, name.
    Returns a dict with the raw outcome, the event log, the trace (optional) and the evaluation."""
    from .evaluator import evaluate
    params, cfg = build_params(spec.get("overrides"))
    params["seed"] = spec.get("seed", 1)
    arena = Arena(cfg["arena"], spec.get("park_shift_mm", 0.0))
    sim = Sim(library_for(spec.get("firmware")))
    sim.configure(params, arena)
    opmode = spec.get("opmode", "Park Run")
    if sim.start(opmode) != 0:
        return {"name": spec.get("name", opmode), "status": "NO OPMODE", "events": "", "eval": {"pass": False, "reason": "opmode not found"}}
    rows, tele_log = [], []
    status = 0
    last_tele = {}
    while status == 0:
        status = sim.step(1)
        st = sim.state()
        rows.append(st)
        tl = sim.telemetry()
        if tl.get("state") != last_tele.get("state") or tl.get("leg") != last_tele.get("leg"):
            tele_log.append((st[0], dict(tl)))
        last_tele = tl
    trace = np.array(rows)
    events = sim.events()
    res = {"name": spec.get("name", opmode), "opmode": opmode, "firmware": firmware_path(spec.get("firmware")),
           "seed": params["seed"], "status": STATUS.get(status, str(status)), "telemetry": last_tele,
           "events": events, "params": params, "park_rect": arena.boxes["PARK"], "tele_log": tele_log}
    res["eval"] = evaluate(res, trace, arena, params)
    if spec.get("keep_trace"):
        res["trace"] = trace
    return res


def _worker(spec):
    try:
        return run(spec)
    except Exception as e:  # a crash is a result too, never hidden
        import traceback
        return {"name": spec.get("name"), "status": "CRASH", "events": traceback.format_exc(),
                "eval": {"pass": False, "reason": "simulator crash: " + repr(e)}}


def run_many(specs, workers=None, progress=None):
    """Run specs in parallel worker processes. Order of results = order of specs."""
    # build once up front (workers would race on the compiler otherwise)
    for fw in {s.get("firmware") for s in specs}:
        library_for(fw)
    workers = workers or max(1, min(len(specs), (os.cpu_count() or 2)))
    out = [None] * len(specs)
    if workers == 1:
        for i, s in enumerate(specs):
            out[i] = _worker(s)
            if progress:
                progress(i + 1, len(specs))
        return out
    ctx = mp.get_context("spawn")
    with cf.ProcessPoolExecutor(max_workers=workers, mp_context=ctx) as ex:
        futs = {ex.submit(_worker, s): i for i, s in enumerate(specs)}
        done = 0
        for f in cf.as_completed(futs):
            out[futs[f]] = f.result()
            done += 1
            if progress:
                progress(done, len(specs))
    return out
