"""Arena geometry from config/arena.json: black tape rectangles (mm), zones and the route polylines
used by the evaluator. Nothing here steers the robot - the firmware only ever sees the IR array."""
import math


class Arena:
    def __init__(self, cfg, park_shift_mm=0.0):
        self.cfg = cfg
        tw = cfg["tape_width"]
        self.tw = tw
        self.rects = []          # black tape, mm: (x0, y0, x1, y1)
        s = cfg["loop"]["size"]
        x0, y0 = cfg["loop"]["x0"], cfg["loop"]["y0"]
        self.corners = {k: tuple(v) for k, v in cfg["corners"].items()}
        self.boxes = {k: list(v["rect"]) for k, v in cfg["boxes"].items()}
        if park_shift_mm:
            b = self.boxes["PARK"]
            self.boxes["PARK"] = [b[0] + park_shift_mm, b[1], b[2] + park_shift_mm, b[3]]
        for b in self.boxes.values():
            self.rects.append(tuple(b))
        # bottom line pieces (y = y0); PARK shift moves the C4-PARK piece's end with the box
        for a, b in cfg["bottom_line"]["segments"]:
            if park_shift_mm and min(a, b) <= x0 + 1:
                b = self.boxes["PARK"][0]
            self._seg((a, y0), (b, y0))
        self._seg((x0 + s, y0), (x0 + s, y0 + s))      # right side
        self._seg((x0 + s, y0 + s), (x0, y0 + s))      # top
        self._seg((x0, y0 + s), (x0, y0))              # left
        self.terminals = {}
        bar = cfg["terminal_end_bar"]
        for t in cfg["terminals"]:
            ax, ay = t["attach"]
            dx, dy = t["dir"]
            ex, ey = ax + dx * t["length"], ay + dy * t["length"]
            self._seg((ax, ay), (ex, ey))
            px, py = -dy, dx
            self._seg((ex - px * bar / 2, ey - py * bar / 2), (ex + px * bar / 2, ey + py * bar / 2))
            self.terminals[t["name"]] = {"attach": (ax, ay), "end": (ex, ey), "kind": t["kind"]}
        self.field = cfg["field"]["rect"]

    def _seg(self, a, b):
        h = self.tw / 2
        self.rects.append((min(a[0], b[0]) - h, min(a[1], b[1]) - h, max(a[0], b[0]) + h, max(a[1], b[1]) + h))

    def route_polyline(self, opmode):
        """Centre-line path (mm) the robot is meant to drive, for off-route detection."""
        c = self.corners
        start = ((self.boxes["START"][0] + self.boxes["START"][2]) / 2, 0)
        park = ((self.boxes["PARK"][0] + self.boxes["PARK"][2]) / 2, 0)
        pts = [start, c["C1"]]
        if "D1" in opmode:
            p1, d1 = self.terminals["P1"], self.terminals["D1"]
            pts += [p1["attach"], p1["end"], p1["attach"], d1["attach"], d1["end"], d1["attach"]]
        pts += [c["C2"], c["C3"], c["C4"], park]
        return pts

    def park_centre(self):
        b = self.boxes["PARK"]
        return ((b[0] + b[2]) / 2, (b[1] + b[3]) / 2)


def dist_to_polyline(x, y, pts):
    best = 1e18
    for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best
