"""Live 2D viewer (matplotlib - pygame is not required).

    python3 main.py view                       # Park Run, nominal robot
    python3 main.py view --opmode "P1-D1 Run" --scenario 13

Controls: Start/Pause, Step (one 10 ms tick), Reset (fresh power-on), Slow / Normal / Fast + speed
slider, fault toggles (applied LIVE to the running sim), scenario arrows, opmode selector.
Everything the robot does comes from the compiled AutoPark.cpp; the viewer only draws.
"""
import math
import os
import time

import matplotlib
import numpy as np

from . import config as C
from . import scenarios as S
from .arena import Arena
from .engine import IDX, STATUS, Sim, build_params, library_for
from .evaluator import evaluate
from .logs import save_run

FAULT_PACKS = {
    "Sensor faults": {"raw_noise": 200.0, "line_dropout_per_s": 1.0, "line_dropout_ms": 50.0},
    "Encoder faults": {"enc_dead_l": 1.0},
    "Gyro faults": {"gyro_drift_dps": 0.5, "gyro_noise_deg": 0.3},
}
INK = "#1b1b1f"
MAT = "#f4f1ea"
ACCENT = "#d9480f"


class Viewer:
    def __init__(self, firmware=None, opmode="Park Run", scenario=None, headless=False):
        self.firmware = firmware or S.default_firmware()
        self.lib = library_for(self.firmware)
        self.opmode = opmode
        self.sc_data = S.load_scenarios()
        self.sc_idx = None if scenario is None else [s["id"] for s in self.sc_data["scenarios"]].index(int(scenario))
        self.speed = 1.0
        self.running = False
        self.faults_on = {k: False for k in FAULT_PACKS}
        self.headless = headless
        if headless:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        self.plt = plt
        self._build_figure()
        self.reset()

    # ------------------------------------------------------------------ sim control
    def _overrides(self):
        if self.sc_idx is None:
            return {}, 0.0
        sc = self.sc_data["scenarios"][self.sc_idx]
        ov = dict(self.sc_data["bases"][sc["base"]])
        ov.update(sc["set"])
        return ov, sc.get("park_shift_mm", 0.0)

    def reset(self, *_):
        ov, shift = self._overrides()
        self.params, cfg = build_params(ov)
        self.params["seed"] = 1
        self.arena = Arena(cfg["arena"], shift)
        self.sim = Sim(self.lib)                      # fresh copy = fresh firmware statics (power-on)
        self.sim.configure(self.params, self.arena)
        for name, on in self.faults_on.items():
            if on:
                for k, v in FAULT_PACKS[name].items():
                    self.sim.lib.sim_set_param(k.encode(), v)
        self.sim.start(self.opmode)
        self.rows, self.tele_log, self.status, self.events = [], [], 0, ""
        self.tele = {}
        self.result_text = ""
        self._draw_arena()
        self._advance(0)
        self._redraw()

    def _advance(self, n):
        for _ in range(n):
            if self.status != 0:
                break
            self.status = self.sim.step(1)
            st = self.sim.state()
            self.rows.append(st)
            tl = self.sim.telemetry()
            if tl.get("state") != self.tele.get("state") or tl.get("leg") != self.tele.get("leg"):
                self.tele_log.append((st[0], dict(tl)))
            self.tele = tl
        if not self.rows:
            self.rows.append(self.sim.state())
        new = self.sim.events()
        if new:
            self.events += new
        if self.status != 0 and not self.result_text:
            res = {"name": self.opmode, "opmode": self.opmode, "status": STATUS[self.status], "telemetry": self.tele,
                   "events": self.events, "tele_log": self.tele_log, "params": self.params}
            trace = np.array(self.rows)
            res["eval"] = evaluate(res, trace, self.arena, self.params)
            v = S.verdict(res["eval"])
            self.result_text = f"{v}: {res['eval']['reason']}"
            self.log_base = save_run(res, trace, tag=f"view_{self.opmode}")
            self.running = False

    def toggle(self, *_):
        self.running = not self.running
        self.btn_run.label.set_text("Pause" if self.running else "Start")

    def step_once(self, *_):
        self.running = False
        self.btn_run.label.set_text("Start")
        self._advance(1)
        self._redraw()

    def set_speed(self, s):
        self.speed = s
        self.sl_speed.set_val(math.log10(s))

    def _on_slider(self, v):
        self.speed = 10 ** v

    def _on_fault(self, label):
        self.faults_on[label] = not self.faults_on[label]
        pack = FAULT_PACKS[label]
        for k, v in pack.items():   # live: the core reads parameters every tick
            val = v if self.faults_on[label] else self.params.get(k, 0.0)
            self.sim.lib.sim_set_param(k.encode(), float(val))
        self.events += f"{self.rows[-1][0] + 1:.2f} FAULT INJECTION {label} {'ON' if self.faults_on[label] else 'OFF'}\n"

    def _scenario(self, d):
        n = len(self.sc_data["scenarios"])
        self.sc_idx = (0 if self.sc_idx is None else (self.sc_idx + d) % n) if d else None
        self._update_sc_label()
        self.reset()

    def _update_sc_label(self):
        if self.sc_idx is None:
            self.sc_label.set_text("nominal robot (no scenario)")
        else:
            sc = self.sc_data["scenarios"][self.sc_idx]
            self.sc_label.set_text(f"#{sc['id']} {sc['name']}")

    def _on_opmode(self, label):
        self.opmode = label
        self.reset()

    def _tick(self):
        if self.running:
            self._advance(max(1, int(round(self.speed * 3.3))))   # ~30 fps x 3.3 ticks = real time at 1x
            self._redraw()

    # ------------------------------------------------------------------ drawing
    def _build_figure(self):
        plt = self.plt
        from matplotlib.widgets import Button, CheckButtons, RadioButtons, Slider
        plt.rcParams.update({"font.size": 9, "font.family": "DejaVu Sans"})
        self.fig = plt.figure(figsize=(15, 9), facecolor="white")
        self.fig.canvas.manager.set_window_title("GLITCHED - AutoPark SIL simulator") if hasattr(self.fig.canvas, "manager") and self.fig.canvas.manager else None
        self.ax = self.fig.add_axes([0.01, 0.15, 0.56, 0.83])
        self.ax_txt = self.fig.add_axes([0.585, 0.40, 0.405, 0.58])
        self.ax_txt.axis("off")
        self.ax_bar = self.fig.add_axes([0.60, 0.20, 0.17, 0.15])
        self.ax_err = self.fig.add_axes([0.81, 0.20, 0.18, 0.15])
        bx = lambda x, w: self.fig.add_axes([x, 0.035, w, 0.05])
        self.btn_run = Button(bx(0.01, 0.07), "Start")
        self.btn_step = Button(bx(0.085, 0.06), "Step")
        self.btn_reset = Button(bx(0.15, 0.06), "Reset")
        self.btn_slow = Button(bx(0.22, 0.05), "0.25x")
        self.btn_norm = Button(bx(0.275, 0.05), "1x")
        self.btn_fast = Button(bx(0.33, 0.05), "8x")
        self.sl_speed = Slider(self.fig.add_axes([0.43, 0.05, 0.12, 0.025]), "speed", -1.0, 1.3, valinit=0.0)
        self.sl_speed.valtext.set_visible(False)
        self.chk = CheckButtons(self.fig.add_axes([0.59, 0.01, 0.12, 0.12]), list(FAULT_PACKS), [False] * len(FAULT_PACKS))
        self.radio = RadioButtons(self.fig.add_axes([0.72, 0.03, 0.09, 0.08]), ["Park Run", "P1-D1 Run"],
                                  active=0 if self.opmode == "Park Run" else 1)
        self.btn_prev = Button(self.fig.add_axes([0.82, 0.065, 0.03, 0.04]), "<")
        self.btn_next = Button(self.fig.add_axes([0.955, 0.065, 0.03, 0.04]), ">")
        self.btn_nom = Button(self.fig.add_axes([0.855, 0.065, 0.095, 0.04]), "nominal")
        self.sc_label = self.fig.text(0.82, 0.035, "", fontsize=8)
        self._update_sc_label()
        self.btn_run.on_clicked(self.toggle)
        self.btn_step.on_clicked(self.step_once)
        self.btn_reset.on_clicked(self.reset)
        self.btn_slow.on_clicked(lambda e: self.set_speed(0.25))
        self.btn_norm.on_clicked(lambda e: self.set_speed(1.0))
        self.btn_fast.on_clicked(lambda e: self.set_speed(8.0))
        self.sl_speed.on_changed(self._on_slider)
        self.chk.on_clicked(self._on_fault)
        self.radio.on_clicked(self._on_opmode)
        self.btn_prev.on_clicked(lambda e: self._scenario(-1))
        self.btn_next.on_clicked(lambda e: self._scenario(+1))
        self.btn_nom.on_clicked(lambda e: self._scenario(0))
        self.timer = self.fig.canvas.new_timer(interval=33)
        self.timer.add_callback(self._tick)

    def _draw_arena(self):
        from matplotlib.patches import Rectangle, Polygon
        ax = self.ax
        ax.clear()
        fx0, fy0, fx1, fy1 = [v / 1000 for v in self.arena.field]
        ax.add_patch(Rectangle((fx0, fy0), fx1 - fx0, fy1 - fy0, fc=MAT, ec="#999", lw=1))
        for b, col in (("START", "#2b8a3e"), ("PARK", "#1971c2")):
            x0, y0, x1, y1 = [v / 1000 for v in self.arena.boxes[b]]
            ax.text((x0 + x1) / 2, y0 - 0.07, b, ha="center", va="top", color=col, fontsize=9, weight="bold")
        for r in self.arena.rects:
            x0, y0, x1, y1 = [v / 1000 for v in r]
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=INK, ec="none"))
        for n, (x, y) in self.arena.corners.items():
            ax.text(x / 1000 + (0.06 if x > 0 else -0.06), y / 1000 + (0.06 if y > 0 else -0.06), n,
                    ha="center", va="center", fontsize=8, color="#555")
        for n, t in self.arena.terminals.items():
            (ax_, ay_), (ex, ey) = t["attach"], t["end"]
            L = math.hypot(ex - ax_, ey - ay_) or 1.0
            lx, ly = (ex + (ex - ax_) / L * 110) / 1000, (ey + (ey - ay_) / L * 110) / 1000   # just beyond the end bar
            ax.text(lx, ly, n, ha="center", va="center", fontsize=8, weight="bold",
                    color="#a61e4d" if t["kind"] == "pickup" else "#5f3dc4")
        ax.set_xlim(fx0, fx1)
        ax.set_ylim(fy0, fy1)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        self.trail, = ax.plot([], [], color=ACCENT, lw=1.0, alpha=0.8)
        self.body = Polygon(np.zeros((4, 2)), closed=True, fc="#ced4da", ec=INK, lw=1, alpha=0.9)
        ax.add_patch(self.body)
        self.wheels = [Polygon(np.zeros((4, 2)), closed=True, fc="#343a40", ec="none") for _ in range(4)]
        for w in self.wheels:
            ax.add_patch(w)
        self.arrow, = ax.plot([], [], color=ACCENT, lw=2)
        self.dots = ax.scatter(np.zeros(8), np.zeros(8), s=18, c="white", edgecolors=INK, linewidths=0.6, zorder=5)
        self.banner = ax.text(0.5, 0.985, "", transform=ax.transAxes, ha="center", va="top", fontsize=10, weight="bold")

    def _poly(self, x, y, th, pts):
        c, s = math.cos(th), math.sin(th)
        return np.array([[x + a * c - b * s, y + a * s + b * c] for a, b in pts])

    def _redraw(self):
        r = self.rows[-1]
        P = self.params
        x, y, th = r[IDX["x"]], r[IDX["y"]], math.radians(r[IDX["th_deg"]])
        tr = np.array(self.rows)
        self.trail.set_data(tr[:, IDX["x"]], tr[:, IDX["y"]])
        hw, hl, fwd = P["track_width_m"] / 2, P["wheelbase_m"] / 2, P["sensor_fwd_m"]
        self.body.set_xy(self._poly(x, y, th, [(-hl - 0.05, -hw + 0.02), (fwd + 0.01, -hw + 0.02), (fwd + 0.01, hw - 0.02), (-hl - 0.05, hw - 0.02)]))
        rw = P["wheel_radius_m"]
        for w, (a, b) in zip(self.wheels, [(hl, hw), (hl, -hw), (-hl, hw), (-hl, -hw)]):
            w.set_xy(self._poly(x, y, th, [(a - rw, b - 0.015), (a + rw, b - 0.015), (a + rw, b + 0.015), (a - rw, b + 0.015)]))
        self.arrow.set_data([x, x + 0.18 * math.cos(th)], [y, y + 0.18 * math.sin(th)])
        sxy = np.array([[r[IDX[f"s{i}x"]], r[IDX[f"s{i}y"]]] for i in range(8)])
        norms = np.array([r[IDX[f"n{i}"]] for i in range(8)]) / 1000.0
        self.dots.set_offsets(sxy)
        self.dots.set_facecolors([(1 - v * 0.9, 0.35 + 0.3 * (1 - v), 0.1, 1) if v >= 0.25 else (1, 1, 1, 1) for v in norms])

        # sensor bars
        self.ax_bar.clear()
        self.ax_bar.bar(range(8), norms * 1000, color=[ACCENT if v >= 0.25 else "#adb5bd" for v in norms])
        self.ax_bar.axhline(250, color="#555", lw=0.6, ls="--")
        self.ax_bar.set_ylim(0, 1050)
        self.ax_bar.set_xticks(range(8))
        self.ax_bar.set_title("IR channels (normalised, dark >= 250)", fontsize=8)
        # line error history (last 10 s)
        self.ax_err.clear()
        t = tr[:, IDX["t"]]
        m = t >= t[-1] - 10
        self.ax_err.plot(t[m], tr[m, IDX["error"]], color=INK, lw=0.8)
        self.ax_err.plot(t[m], tr[m, IDX["pidOut"]] * 3500 / 0.25, color=ACCENT, lw=0.8, alpha=0.7)
        self.ax_err.set_ylim(-3700, 3700)
        self.ax_err.set_title("line error (black), steer PID out (orange, scaled)", fontsize=8)

        T = self.tele
        sc = "nominal" if self.sc_idx is None else self.sc_data["scenarios"][self.sc_idx]["name"]
        enc_m = 0.5 * (r[IDX["pulsesL"]] + r[IDX["pulsesR"]]) / 1300.0
        hd = r[IDX["heading_cw"]]
        grid = (round(-hd / 90.0) * 90) % 360
        lines = [
            ("firmware", os.path.basename(os.path.dirname(os.path.dirname(os.path.dirname(self._fw_label())))) + "/" + os.path.basename(self._fw_label())),
            ("opmode / scenario", f"{self.opmode}  |  {sc}"),
            ("sim time", f"{r[IDX['t']]:6.2f} s    speed x{self.speed:.2f}    status {STATUS[int(r[IDX['status']])]}"),
            ("", ""),
            ("state", T.get("state", "")),
            ("why", T.get("why", "")),
            ("step (leg)", T.get("leg", "")),
            ("leg distance", f"{T.get('d cm', '')} cm (firmware, encoders)"),
            ("turns log", T.get("turns", "")),
            ("junctions", T.get("jx", "")),
            ("", ""),
            ("motor cmd L / R", f"{r[IDX['cmdL']]:+.3f} / {r[IDX['cmdR']]:+.3f}   PWM {int(r[IDX['pwmL']]):+4d} / {int(r[IDX['pwmR']]):+4d}"),
            ("encoder pulses L / R", f"{int(r[IDX['pulsesL']])} / {int(r[IDX['pulsesR']])}   ({enc_m:.2f} m at CPM 1300)"),
            ("heading (true, CW+)", f"{hd:+7.1f} deg   nearest grid dir {grid:.0f} deg (CCW from east)"),
            ("pose (sim truth)", f"x {x:.3f}  y {y:.3f}  th {math.degrees(th) % 360:.1f} deg"),
            ("line mask ch0..7", format(int(r[IDX['mask']]), "08b")[::-1] + f"   pattern {['LOST', 'LINE', 'LEFT_BR', 'RIGHT_BR', 'TERMINAL'][int(r[IDX['pattern']])]}"),
            ("line position / error", f"{int(r[IDX['position']])} / {int(r[IDX['error']])}"),
            ("steer PID kP kI kD", f"{r[IDX['kP']]:.3f} {r[IDX['kI']]:.3f} {r[IDX['kD']]:.3f}   P {r[IDX['pidP']]:+.3f} D {r[IDX['pidD']]:+.3f} out {r[IDX['pidOut']]:+.3f}"),
            ("faults injected", ", ".join(k for k, v in self.faults_on.items() if v) or "none"),
            ("fault reason", T.get("why", "") if T.get("state") == "FAULT" else "-"),
        ]
        self.ax_txt.clear()
        self.ax_txt.axis("off")
        for i, (k, v) in enumerate(lines):
            yy = 1.0 - i * 0.047
            self.ax_txt.text(0.0, yy, k, fontsize=8.5, color="#666", va="top", family="DejaVu Sans")
            self.ax_txt.text(0.33, yy, str(v), fontsize=8.5, color=INK, va="top", family="DejaVu Sans Mono",
                             weight="bold" if k == "state" else "normal")
        col = {"DONE": "#2b8a3e", "FAULT": "#c92a2a"}.get(T.get("state", ""), INK)
        self.banner.set_text(self.result_text or T.get("state", ""))
        self.banner.set_color("#2b8a3e" if self.result_text.startswith("PASS") else ("#c92a2a" if self.result_text else col))
        self.fig.canvas.draw_idle()

    def _fw_label(self):
        from .engine import firmware_path
        return firmware_path(self.firmware)

    def show(self):
        self.timer.start()
        self.plt.show()

    def run_headless(self, seconds, png):
        self.running = True
        self._advance(int(seconds * 100))
        self._redraw()
        self.fig.savefig(png, dpi=110)
        return png
