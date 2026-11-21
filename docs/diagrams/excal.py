"""Tiny helpers for writing Excalidraw scenes from Python.

Render with:  excalidraw render <file>.excalidraw --out <file>.png --scale 2
"""
import json
import math
import random

RED, GREEN, BLUE, INK, GREY = "#e03131", "#2f9e44", "#1971c2", "#1e1e1e", "#868e96"
ORANGE, PURPLE, YELLOW_BG, BLUE_BG, GREY_BG, GREEN_BG, RED_BG = (
    "#f08c00", "#9c36b5", "#fff3bf", "#d0ebff", "#e9ecef", "#d3f9d8", "#ffe3e3")


class Scene:
    def __init__(self, seed=7):
        self.els = []
        self._id = 0
        self.rng = random.Random(seed)

    # ── primitives ──────────────────────────────────────────────────────────
    def _base(self, t, x, y, w, h, frame=None, **kw):
        self._id += 1
        e = dict(id=f"{t[0]}{self._id}", type=t, x=x, y=y, width=w, height=h, angle=0,
                 strokeColor=INK, backgroundColor="transparent", fillStyle="solid",
                 strokeWidth=2, strokeStyle="solid", roughness=1, opacity=100, groupIds=[],
                 frameId=frame, roundness=None, seed=self.rng.randint(1, 2**31), version=1,
                 versionNonce=self.rng.randint(1, 2**31), isDeleted=False, boundElements=[],
                 updated=1, link=None, locked=False)
        e.update(kw)
        self.els.append(e)
        return e

    def frame(self, x, y, w, h, name):
        return self._base("frame", x, y, w, h, None, name=name, roughness=0, strokeWidth=1)["id"]

    def rect(self, x, y, w, h, frame, **kw):
        kw.setdefault("roundness", {"type": 3})
        return self._base("rectangle", x, y, w, h, frame, **kw)

    def ellipse(self, x, y, w, h, frame, **kw):
        return self._base("ellipse", x, y, w, h, frame, **kw)

    def diamond(self, x, y, w, h, frame, **kw):
        return self._base("diamond", x, y, w, h, frame, **kw)

    @staticmethod
    def _measure(s, size, family):
        lines = s.split("\n")
        k = 0.6 if family == 3 else 0.55
        return max(len(l) for l in lines) * size * k, len(lines) * size * 1.25

    def text(self, x, y, s, frame, size=18, color=INK, align="left", family=5, **kw):
        w, h = self._measure(s, size, family)
        if align == "center":
            x -= w / 2
        return self._base("text", x, y, w, h, frame, text=s, originalText=s, fontSize=size,
                          fontFamily=family, textAlign=align, verticalAlign="top",
                          containerId=None, lineHeight=1.25, autoResize=True,
                          strokeColor=color, **kw)

    def mono(self, x, y, s, frame, size=16, color=INK, **kw):
        return self.text(x, y, s, frame, size=size, color=color, family=3, **kw)

    def box(self, x, y, w, h, label, frame, size=16, family=5, color=INK, shape="rectangle", **kw):
        """Shape with centred, bound label text (stays attached when edited)."""
        make = {"rectangle": self.rect, "ellipse": self.ellipse, "diamond": self.diamond}[shape]
        kw.setdefault("strokeColor", color)
        r = make(x, y, w, h, frame, **kw)
        tw, th = self._measure(label, size, family)
        t = self._base("text", x + (w - tw) / 2, y + (h - th) / 2, tw, th, frame, text=label,
                       originalText=label, fontSize=size, fontFamily=family, textAlign="center",
                       verticalAlign="middle", containerId=r["id"], lineHeight=1.25,
                       autoResize=True, strokeColor=color)
        r["boundElements"].append({"type": "text", "id": t["id"]})
        return r

    def line(self, pts, frame, arrow_end=None, arrow_start=None, kind="arrow", **kw):
        x0, y0 = pts[0]
        rel = [[px - x0, py - y0] for px, py in pts]
        xs, ys = [p[0] for p in rel], [p[1] for p in rel]
        return self._base(kind, x0, y0, max(xs) - min(xs), max(ys) - min(ys), frame, points=rel,
                          lastCommittedPoint=None, startBinding=None, endBinding=None,
                          startArrowhead=arrow_start, endArrowhead=arrow_end, **kw)

    def arrow(self, p, q, frame, color=INK, width=2, head="arrow", both=False, **kw):
        return self.line([p, q], frame, arrow_end=head, arrow_start=head if both else None,
                         strokeColor=color, strokeWidth=width, **kw)

    def elbow(self, pts, frame, color=INK, width=2, **kw):
        return self.line(pts, frame, arrow_end="arrow", strokeColor=color, strokeWidth=width, **kw)

    # ── coordinate-frame helpers ────────────────────────────────────────────
    def axis(self, o, d, label, color, frame, length=130, off=(8, -12)):
        end = (o[0] + d[0] * length, o[1] + d[1] * length)
        self.arrow(o, end, frame, color=color, width=3, roughness=0)
        self.text(end[0] + off[0], end[1] + off[1], label, frame, size=20, color=color)

    def out_of_page(self, c, label, color, frame, r=11, off=(16, -10)):
        self.ellipse(c[0] - r, c[1] - r, 2 * r, 2 * r, frame, strokeColor=color, strokeWidth=3, roughness=0)
        self.ellipse(c[0] - 3, c[1] - 3, 6, 6, frame, strokeColor=color, backgroundColor=color, roughness=0)
        if label:
            self.text(c[0] + off[0], c[1] + off[1], label, frame, size=20, color=color)

    def arc(self, cx, cy, r, a0, a1, frame, color, n=24, head=True, width=2, **kw):
        """Arc in math angles (CCW, y up on screen) → screen coordinates."""
        pts = [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
                cy - r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]
        return self.line(pts, frame, arrow_end="arrow" if head else None, strokeColor=color,
                         strokeWidth=width, roundness={"type": 2}, **kw)

    def save(self, path):
        # Draw-order keys (fractional indexing), required by Excalidraw+:
        # "b" + two base-62 digits sorts in creation order, up to 3844 elements.
        digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
        assert len(self.els) <= 62 * 62, "too many elements for the index scheme"
        for i, e in enumerate(self.els):
            e["index"] = "b" + digits[i // 62] + digits[i % 62]
        scene = dict(type="excalidraw", version=2, source="https://excalidraw.com", elements=self.els,
                     appState=dict(viewBackgroundColor="#ffffff", gridSize=None, exportBackground=True),
                     files={})
        with open(path, "w") as f:
            json.dump(scene, f, indent=1)
        print(f"{path}: {len(self.els)} elements")
