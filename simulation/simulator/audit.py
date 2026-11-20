"""Source audit: what the simulator runs vs what the robot runs.

1. Firmware constants  - parsed from AutoPark.cpp. They are not copied anywhere: the simulator
   compiles that file unchanged, so every constant is used exactly.
2. Stand-in SDK        - checked against the REAL SDK headers (struct field order + defaults,
   enum order, constants). A header change that the stand-in does not follow fails the test.
3. State machine       - states, display names and transitions parsed from AutoPark.cpp, then
   matched against transitions actually observed in simulated runs.
4. Source manifest     - SHA-256 of AutoPark.cpp and every header the stand-in mirrors. Any change
   = the evidence in the report is stale until the suite is re-run and the manifest accepted.
"""
import hashlib
import json
import os
import re

from . import config as C

REPO = C.REPO_DIR
SDK_NRL = os.path.join(REPO, "RobotFirmware", "lib", "HexaNRL", "src")
HEADERS = {
    "HexaPID.h": os.path.join(SDK_NRL, "HexaPID.h"),
    "LineTuning.h": os.path.join(SDK_NRL, "LineTuning.h"),
    "LineActions.h": os.path.join(SDK_NRL, "LineActions.h"),
    "TankDrive.h": os.path.join(SDK_NRL, "TankDrive.h"),
    "NRLOpMode.h": os.path.join(SDK_NRL, "NRLOpMode.h"),
    "NRLAction.h": os.path.join(SDK_NRL, "NRLAction.h"),
    "NRLComms.h": os.path.join(SDK_NRL, "NRLComms.h"),
    "BoardPins.h": os.path.join(SDK_NRL, "BoardPins.h"),
    "OpModeRegistry.h": os.path.join(SDK_NRL, "OpModeRegistry.h"),
    "HexaLineArray.h": os.path.join(REPO, "lib", "HexaLineArray", "src", "HexaLineArray.h"),
    "HexaDCMotor.h": os.path.join(REPO, "lib", "HexaServos", "src", "HexaDCMotor.h"),
}
STANDIN = os.path.join(C.SIM_DIR, "core", "sdk", "NRL.h")
CORE = os.path.join(C.SIM_DIR, "core", "sim_core.cpp")
MANIFEST = os.path.join(C.CONFIG_DIR, "source_manifest.json")
BASELINE = os.path.join(REPO, "RobotFirmware", "opmodes", "AutoPark.cpp")


def sha(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def strip_comments(src):
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


# ------------------------------------------------------------------ C++ mini-parsers
def _norm_val(v):
    if v is None:
        return None
    v = v.strip()
    if v.startswith("{"):
        return re.sub(r"\s+", "", v)
    v2 = re.sub(r"(?<=[\d.])[fF]\b", "", v)
    try:
        return float(v2)
    except ValueError:
        return {"true": 1.0, "false": 0.0}.get(v2, v2)


def parse_struct(src, name):
    """[(field, type, default)] in declaration order."""
    s = strip_comments(src)
    m = re.search(r"struct\s+" + name + r"\s*\{", s)
    if not m:
        return None
    i, depth = m.end(), 1
    while depth:
        depth += {"{": 1, "}": -1}.get(s[i], 0)
        i += 1
    body = s[m.end():i - 1]
    fields = []
    for stmt in body.split(";"):
        stmt = " ".join(stmt.split())
        if not stmt or "(" in stmt.split("=")[0]:
            continue
        mm = re.match(r"^((?:const\s+)?[\w:<>]+)\s+(.*)$", stmt)
        if not mm:
            continue
        typ, rest = mm.group(1), mm.group(2)
        # split declarators on top-level commas
        parts, depth, cur = [], 0, ""
        for ch in rest:
            depth += {"{": 1, "}": -1}.get(ch, 0)
            if ch == "," and depth == 0:
                parts.append(cur)
                cur = ""
            else:
                cur += ch
        parts.append(cur)
        for p in parts:
            p = p.strip()
            dm = re.match(r"^(\w+)(\[\d+\])?\s*(?:=\s*(.+)|(\{.*\}))?$", p)
            if dm:
                fields.append((dm.group(1), typ + (dm.group(2) or ""), _norm_val(dm.group(3) or dm.group(4))))
    return fields


def parse_enum(src, name):
    s = strip_comments(src)
    m = re.search(r"enum\s+class\s+" + name + r"\s*(?::\s*\w+)?\s*\{(.*?)\}", s, re.S)
    if not m:
        return None
    return [x.split("=")[0].strip() for x in m.group(1).split(",") if x.strip()]


def parse_constexpr(src, name):
    m = re.search(r"constexpr\s+[\w:]+\s+" + name + r"\s*=\s*([^;]+);", strip_comments(src))
    return _norm_val(m.group(1)) if m else None


# ------------------------------------------------------------------ 1. stand-in vs real headers
def header_consistency():
    """[(item, ok, detail)]"""
    real = {k: open(p).read() for k, p in HEADERS.items() if os.path.exists(p)}
    stand = open(STANDIN).read()
    rows = []

    def cmp_struct(name, header, order_matters=True, subset=False):
        r, s = parse_struct(real[header], name), parse_struct(stand, name)
        if r is None:
            rows.append((f"struct {name}", False, f"not found in {header}"))
            return
        rmap = {f: (t, d) for f, t, d in r}
        smap = {f: (t, d) for f, t, d in s}
        problems = []
        for f, (t, d) in smap.items():
            if f not in rmap:
                problems.append(f"{f}: not in the real header")
            elif rmap[f][1] != d and not (isinstance(d, str) and isinstance(rmap[f][1], str)):
                problems.append(f"{f}: default {d} vs real {rmap[f][1]}")
        if not subset:
            for f in rmap:
                if f not in smap:
                    problems.append(f"{f}: missing in stand-in")
        if order_matters:
            ro = [f for f, _, _ in r if f in smap]
            so = [f for f, _, _ in s if f in rmap]
            if ro != so:
                problems.append(f"field order differs: real {ro} vs stand-in {so}")
        rows.append((f"struct {name} ({header})", not problems, "; ".join(problems) or f"{len(s)} fields match"))

    cmp_struct("HexaPIDConfig", "HexaPID.h")
    cmp_struct("HexaLineArrayConfig", "HexaLineArray.h")
    cmp_struct("HexaDCMotorConfig", "HexaDCMotor.h")
    cmp_struct("LineTuning", "LineTuning.h", order_matters=False, subset=True)
    for en, h in (("LineFault", "LineActions.h"), ("LinePattern", "HexaLineArray.h"), ("LineSide", "LineActions.h"),
                  ("OpModeType", "OpModeRegistry.h")):
        r, s = parse_enum(real.get(h, ""), en), parse_enum(stand, en)
        rows.append((f"enum {en} ({h})", r == s, f"real {r} vs stand-in {s}" if r != s else f"{len(r)} values in order"))
    for const, h in (("MOTOR_DEADBAND", "HexaDCMotor.h"), ("DIGITAL_D1", "BoardPins.h"), ("DIGITAL_D2", "BoardPins.h"),
                     ("DIGITAL_D3", "BoardPins.h")):
        r = parse_constexpr(real[h], const)
        s = parse_constexpr(stand, const)
        if r is None:      # BoardPins may use a different style
            m = re.search(const + r"\s*=\s*(\d+)", real[h])
            r = float(m.group(1)) if m else None
        if s is None:
            m = re.search(const + r"\s*=\s*(\d+)", stand)
            s = float(m.group(1)) if m else None
        rows.append((f"{const} ({h})", r == s, f"real {r} vs stand-in {s}"))
    m_real = re.search(r"_maxDelta\s*=\s*([\d.]+)f?", real["TankDrive.h"])
    m_st = re.search(r"_maxDelta\s*=\s*([\d.]+)f?", stand)
    ok = bool(m_real and m_st and float(m_real.group(1)) == float(m_st.group(1)))
    rows.append(("TankDrive default ramp _maxDelta (TankDrive.h)", ok, f"real {m_real and m_real.group(1)} vs stand-in {m_st and m_st.group(1)}"))
    # the documented tick order the core implements
    ok = "runs your actions FIRST" in real["NRLOpMode.h"] or "actions FIRST" in real["NRLOpMode.h"]
    rows.append(("tick order: actions then loop() (NRLOpMode.h)", ok, "documented in NRLOpMode.h" if ok else "wording not found - re-check sim_core oneTick()"))
    return rows


# ------------------------------------------------------------------ 2. firmware constants
def firmware_constants(path=BASELINE):
    src = open(path).read()
    s = strip_comments(src)
    rows = []
    for m in re.finditer(r"constexpr\s+([\w:]+)\s+(\w+)\s*=\s*([^;]+);", s):
        rows.append((m.group(2), " ".join(m.group(3).split()), m.group(1)))
    for m in re.finditer(r"static\s+HexaPID\s+(\w+)\s*\{\s*HexaPIDConfig\s*\{([^}]*)\}", s):
        rows.append((f"HexaPID {m.group(1)}", "{" + " ".join(m.group(2).split()) + "}", "HexaPIDConfig"))
    for m in re.finditer(r"\bt\.(\w+(?:\.\w+)?)\s*=\s*([^;]+);", s):
        rows.append((f"LineTuning.{m.group(1)}", " ".join(m.group(2).split()), "LineTuning"))
    for m in re.finditer(r"static\s+HexaDCMotor\s+(\w+)\s*\{\s*\{(.*?)\}\s*\}", s, re.S):
        rows.append((f"HexaDCMotor {m.group(1)}", "{" + " ".join(m.group(2).split()) + "}", "HexaDCMotorConfig"))
    return rows


def plans(path=BASELINE):
    s = strip_comments(open(path).read())
    out = {}
    for m in re.finditer(r"static\s+const\s+Step\s+(\w+)\[\]\s*=\s*\{(.*?)\n\};", s, re.S):
        steps = re.findall(r'\{"([^"]+)",\s*Ev::(\w+),\s*([^,]+),\s*([-+]?\d+),\s*([\d.]+)f?', m.group(2))
        out[m.group(1)] = [{"name": a, "event": b, "expM": " ".join(c.split()), "turnDir": int(d), "turnDeg": float(e)} for a, b, c, d, e in steps]
    return out


# ------------------------------------------------------------------ 3. state machine
def _bodies(s):
    """{function name: body} for file-level static functions."""
    out = {}
    for m in re.finditer(r"\nstatic\s+[\w:<>*&\s]+?\b(\w+)\s*\(([^;{)]*)\)\s*\{", s):
        i, depth = m.end(), 1
        while depth and i < len(s):
            depth += {"{": 1, "}": -1}.get(s[i], 0)
            i += 1
        out[m.group(1)] = s[m.end():i - 1]
    return out


def state_machine(path=BASELINE):
    src = open(path).read()
    s = strip_comments(src)
    states = parse_enum(s, "St") or []
    names = dict(re.findall(r"case\s+St::(\w+):\s*return\s+\"([^\"]*)\"", s))
    bodies = _bodies(s)
    def direct(b):
        t = {x: "direct" for x in re.findall(r"\benter\(\s*St::(\w+)\s*\)", b)}
        for x in re.findall(r"\bstopThen\(\s*St::(\w+)\s*\)", b):
            t.setdefault(x, "after STOP")
            t.setdefault("STOP", "direct")
        if re.search(r"\bfault\(", b):
            t.setdefault("FAULT", "direct")
        return t
    # helper closure: which states a helper can lead to
    helpers = {fn: set(direct(b)) for fn, b in bodies.items()}
    for _ in range(4):
        for fn, b in bodies.items():
            for other in list(helpers):
                if other != fn and re.search(r"\b" + other + r"\s*\(", b):
                    helpers[fn] |= helpers[other]
    rt = bodies.get("runTick", "")
    trans = []
    blocks = re.split(r"\bcase\s+St::(\w+)\s*:", rt)
    for i in range(1, len(blocks), 2):
        frm, body = blocks[i], blocks[i + 1]
        tg = direct(body)
        for fn, targets in helpers.items():
            if fn not in ("runTick", "enter", "stopThen", "fault") and targets and re.search(r"\b" + fn + r"\s*\(", body):
                for t in targets:
                    tg.setdefault(t, f"via {fn}()")
        if frm == "STOP":
            tg.setdefault("(stored target)", "enter(next) - the state passed to stopThen()")
        for t, how in sorted(tg.items()):
            trans.append((frm, t, how))
    return {"states": states, "display": names, "transitions": trans}


def observed_transitions(results, sm):
    """Count observed (from, to) enum pairs. A 'stopThen(St::X)' hop Y -> STOP -> X is also
    credited to (Y, X) so it can be matched against the static 'after STOP' transitions."""
    rev = {v: k for k, v in sm["display"].items()}
    from .evaluator import parse_events
    cnt = {}
    for r in results:
        prev_from = None
        for e in parse_events(r.get("events", "")):
            a, b = rev.get(e["from"]), rev.get(e["to"])
            if not (a and b):
                continue
            cnt[(a, b)] = cnt.get((a, b), 0) + 1
            if a == "STOP" and prev_from:
                cnt[(prev_from, b)] = cnt.get((prev_from, b), 0) + 1
            prev_from = a if b == "STOP" else None
    return cnt


def transition_table(sm, observed):
    rows = []
    for frm, to, how in sm["transitions"]:
        n = observed.get((frm, to), 0)
        if to == "(stored target)":
            n = sum(c for (a, _), c in observed.items() if a == frm)
        rows.append((frm, to, how, n))
    static = {(f, t) for f, t, _ in sm["transitions"]}
    extra = [(f, t, n) for (f, t), n in observed.items() if (f, t) not in static and f != "STOP"]
    return rows, extra


# ------------------------------------------------------------------ 4. manifest
def current_manifest():
    files = {"RobotFirmware/opmodes/AutoPark.cpp": BASELINE}
    files.update({f"sdk/{k}": v for k, v in HEADERS.items()})
    files["simulation/core/sdk/NRL.h"] = STANDIN
    files["simulation/core/sim_core.cpp"] = CORE
    return {k: sha(v) for k, v in files.items() if os.path.exists(v)}


def check_manifest():
    """[(file, status)] where status is 'ok', 'CHANGED', 'NEW' or 'MISSING'."""
    if not os.path.exists(MANIFEST):
        return [("(manifest)", "MISSING - run: python3 main.py audit --accept")]
    old = json.load(open(MANIFEST))["sha256"]
    cur = current_manifest()
    out = []
    for k in sorted(set(old) | set(cur)):
        if k not in cur:
            out.append((k, "MISSING"))
        elif k not in old:
            out.append((k, "NEW"))
        else:
            out.append((k, "ok" if old[k] == cur[k] else "CHANGED"))
    return out


def accept_manifest(note=""):
    with open(MANIFEST, "w") as f:
        json.dump({"_doc": "SHA-256 of the sources the simulator evidence is based on. If any changes, re-run the suite "
                           "(python3 main.py suite) and accept with: python3 main.py audit --accept",
                   "note": note, "sha256": current_manifest()}, f, indent=1)


# ------------------------------------------------------------------ 5. SDK runtime mapping (what the stand-in does)
SDK_MAPPING = [
    ("HexaLineArray::tick() normalisation", "HexaLineArray.h", "(raw-calMin)/(calMax-calMin)*1000, clamped 0..1000", "exact (header formula)", ""),
    ("HexaLineArray dark mask", "HexaLineArray.h", "norm >= darkThreshold (250)", "exact", ""),
    ("HexaLineArray dead channel", "HexaLineArray.h", "span < minSpan (300) -> always 0", "exact", ""),
    ("HexaLineArray::position()", "HexaLineArray.h", "sum(norm*i*1000)/sum(norm) over channels >= weightThreshold", "exact", ""),
    ("position() during dropout", "HexaLineArray.h", "0 or 7000 on the last-seen side; -1 after lostStopMs (1200 ms)", "exact", ""),
    ("lastSeenSide()", "HexaLineArray.h", "set from the sign of every non-zero on-line error; never reset to 0", "approximated", "ASSUMPTION: the exact update rule is in the .a"),
    ("HexaLineArray::pattern()", "HexaLineArray.h", "zones ch0-1 / ch3-4 / ch6-7, TERMINAL >= 7 dark", "approximated", "ASSUMPTION: middle = ch3 OR ch4; both edges -> LEFT_BRANCH. AutoPark's own logic reads mask(), not pattern(), for its decisions"),
    ("IR analog front end", "-", "white/black raw levels + Gaussian noise + spot coverage of tape", "model", "ASSUMPTION values in sensor.json"),
    ("HexaDCMotor::setSpeed()", "HexaDCMotor.h", "|cmd| <= 10 -> 0; minDrive remaps 1..255 onto minDrive..255; 0 = coast", "approximated", "ASSUMPTION: the remap is linear"),
    ("TankDrive::drive()/tick", "TankDrive.h", "L=(f+t)*scale, R=(f-t)*scale; ramp _maxDelta per tick; output*255", "exact / approx", "ASSUMPTION: output*255 rounded"),
    ("TankDrive::stop()", "TankDrive.h", "command and output to 0 at once, no ramp", "exact", ""),
    ("HexaPID::update()", "HexaPID.h", "dt from micros capped at maxDtMs; D filter dAlpha; no D on first update; iMax + anti-windup", "exact (header doc)", ""),
    ("NRLOpMode tick", "NRLOpMode.h", "line tick -> actions FIRST -> loop() -> TankDrive::tickAll -> physics", "exact (header doc)", ""),
    ("runAction()", "NRLOpMode.h", "queued; starts on the next tick; max 8", "exact (header doc)", ""),
    ("sequential()", "NRLAction.h", "next child starts on the next tick", "approximated", "ASSUMPTION (seq_chain=1 chains in the same tick)"),
    ("LineActions::leaveBox()", "LineActions.h + LineTuning.h", "startSpeed straight until ch3|ch4 dark and ch0,1,6,7 white for boxLeaveTicks; NO_LINE after startTimeoutMs", "approximated", "ASSUMPTION: no gyro heading hold unless sdk_leavebox_gyro_hold=1. Used only by the repo V3 Park Run"),
    ("LineActions builder (followToJunction etc.)", "LineActions.h", "NOT simulated", "not simulated", "the SDK follower lives in the prebuilt .a; affects only the fallback OpModes 'Park Run Tape' and 'First Junction'"),
    ("LineTuning.pid (SDK follower PD 0.25/0.06)", "LineTuning.h", "NOT simulated", "not simulated", "used only inside the SDK follower (.a). Park Run / P1-D1 Run steer with their own HexaPID 'steer'"),
    ("NRLComms heading", "NRLComms.h", "true heading CW+ + bias, drift, noise, scale; zeroHeading() reference", "model", "noise/drift values are ASSUMPTIONS"),
    ("Encoders", "AutoPark.cpp ISR", "pulses delivered to AutoPark's own leftIsr/rightIsr (with its 50 us glitch filter) on pins 37/36", "exact (firmware ISR runs)", "channel A only: a 'wrong direction' fault is not observable and is not simulated"),
]
