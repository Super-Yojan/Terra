"""Generate app-wireframes.excalidraw: TerraPhone UI, Tesla-app style (dark, rover-first).

    python3 docs/diagrams/gen_app_wireframes.py docs/diagrams/app-wireframes.excalidraw

Patterns from the Tesla app (Mobbin): name + status + hero vehicle + icon row on
home, then one context card and a plain list; the vehicle itself as the control
surface (Controls, Flat Tire); a hero above a bottom sheet with one big value and
< > steppers (Climate); slider card with split buttons (Charging); full-bleed map
with a route sheet and one blue send button (trip); red text for destructive
actions. The previous light version is gen_app_wireframes_light.py.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from excal import *  # noqa: E402,F403

S = Scene(seed=31)
W, H = 360, 800
GAP_X, GAP_Y = 230, 170
PAD = 22

BG, CARD, LINE = "#17181c", "#25272d", "#3a3c43"
TXT, SUB = "#f5f5f7", "#9a9ca3"
ACC, OK, STOP, WARN = "#3e6ae1", "#3ecf6e", "#e82127", "#f5a524"
BODY, WHEEL = "#34363d", "#0b0c0e"


def flat(**kw):
    kw.setdefault("roughness", 0)
    kw.setdefault("strokeWidth", 1)
    return kw


class Phone:
    def __init__(self, col, row, name):
        self.x0 = 40 + col * (W + GAP_X)
        self.y0 = 130 + row * (H + GAP_Y)
        self.f = S.frame(self.x0, self.y0, W, H, name)
        S.rect(self.x0, self.y0, W, H, self.f, **flat(backgroundColor=BG, strokeColor="#000000", strokeWidth=3,
                                                      roundness={"type": 3, "value": 44}))
        self.t(28, 16, "9:41", 13)
        self.t(W - 74, 16, "▮▮ ◠ ▭", 12, SUB)
        self.y = self.y0 + 52

    # primitives in phone coordinates ---------------------------------------
    def t(self, x, y, s, size=15, color=TXT, family=2, align="left", abs_y=False):
        return S.text(self.x0 + x, (y if abs_y else self.y0 + y), s, self.f, size=size, color=color,
                      family=family, align=align)

    def tr(self, y, s, size=14, color=SUB, family=2, inset=PAD):
        w, _ = S._measure(s, size, family)
        S.text(self.x0 + W - inset - w, y, s, self.f, size=size, color=color, family=family)

    def card(self, x, y, w, h, **kw):
        kw = flat(backgroundColor=CARD, strokeColor=CARD, roundness={"type": 3, "value": 12}, **kw)
        return S.rect(self.x0 + x, y, w, h, self.f, **kw)

    def nav(self, title, sub=None, sub_color=OK, right=None):
        self.t(PAD, 50, "‹", 26)
        self.t(W / 2, 52, title, 18, align="center")
        if sub:
            self.t(W / 2, 76, sub, 12, sub_color, align="center")
        if right:
            self.tr(self.y0 + 54, right, 15, TXT)
        self.y = self.y0 + (100 if sub else 90)

    def row(self, icon, title, sub=None, value=None, color=TXT):
        y = self.y
        self.t(PAD, y + 8, icon, 18, SUB, abs_y=True)
        S.text(self.x0 + PAD + 34, y + 6, title, self.f, size=17, family=2, color=color)
        if sub:
            S.text(self.x0 + PAD + 34, y + 30, sub, self.f, size=12, family=2, color=SUB)
        if value:
            self.tr(y + 9, value, 14, SUB, inset=PAD + 22)
        S.text(self.x0 + W - PAD - 8, y + 6, "›", self.f, size=18, family=2, color=SUB)
        self.y += 56 if sub else 46

    def split(self, y, left, right, h=44, x=PAD, w=None, color=SUB):
        w = w or W - 2 * PAD
        S.line([(self.x0 + x, y), (self.x0 + x + w, y)], self.f, kind="line", **flat(strokeColor=LINE))
        S.line([(self.x0 + x + w / 2, y), (self.x0 + x + w / 2, y + h)], self.f, kind="line",
               **flat(strokeColor=LINE))
        self.t(x + w / 4, y + 13, left, 13, color, align="center", abs_y=True)
        self.t(x + 3 * w / 4, y + 13, right, 13, color, align="center", abs_y=True)

    def slider(self, y, frac, x=PAD + 14, w=None, color=OK):
        w = w or W - 2 * PAD - 28
        x0 = self.x0 + x
        S.line([(x0, y), (x0 + w, y)], self.f, kind="line", **flat(strokeColor=LINE, strokeWidth=4))
        S.line([(x0, y), (x0 + w * frac, y)], self.f, kind="line", **flat(strokeColor=color, strokeWidth=4))
        S.ellipse(x0 + w * frac - 9, y - 9, 18, 18, self.f, **flat(backgroundColor=TXT, strokeColor=TXT))

    def stepper(self, y, value, x=PAD + 14, w=None, h=40):
        w = w or W - 2 * PAD - 28
        S.rect(self.x0 + x, y, w, h, self.f, **flat(backgroundColor="#31333a", strokeColor="#31333a",
                                                     roundness={"type": 3, "value": 8}))
        self.t(x + 16, y + 9, "‹", 20, SUB, abs_y=True)
        self.t(x + w - 26, y + 9, "›", 20, SUB, abs_y=True)
        self.t(x + w / 2, y + 11, value, 15, TXT, align="center", abs_y=True)

    def button(self, y, label, color=ACC, h=48):
        S.box(self.x0 + PAD, y, W - 2 * PAD, h, label, self.f, size=15, family=2, color=TXT,
              **flat(backgroundColor=color, strokeColor=color, roundness={"type": 3, "value": 8}))

    def icon_row(self, y, items):
        """items: (glyph, label or None, color)"""
        n = len(items)
        for i, (glyph, label, color) in enumerate(items):
            cx = W / 2 + (i - (n - 1) / 2) * (W - 2 * PAD) / n
            self.t(cx, y, glyph, 22, color, align="center", abs_y=True)
            if label:
                self.t(cx, y + 32, label, 11, SUB, align="center", abs_y=True)

    def sheet(self, y):
        S.rect(self.x0, y, W, self.y0 + H - y, self.f, **flat(backgroundColor="#1e2025", strokeColor="#1e2025",
                                                                roundness={"type": 3, "value": 20}))
        S.line([(self.x0 + W / 2 - 20, y + 10), (self.x0 + W / 2 + 20, y + 10)], self.f, kind="line",
               **flat(strokeColor=LINE, strokeWidth=4))


def rover_top(p, cx, cy, s=1.0, phone=True, labels=None):
    """Top-down rover (front up). labels: dict corner -> (big, small)."""
    bw, bl = 150 * s, 230 * s
    x, y = p.x0 + cx - bw / 2, cy - bl / 2
    S.rect(x, y, bw, bl, p.f, **flat(backgroundColor=BODY, strokeColor="#4a4d55", strokeWidth=2,
                                     roundness={"type": 3, "value": 26 * s}))
    S.rect(x + 18 * s, y + 40 * s, bw - 36 * s, bl - 80 * s, p.f,
           **flat(backgroundColor="#2b2d33", strokeColor="#2b2d33", roundness={"type": 3, "value": 14 * s}))
    for sx in (0, 1):
        for sy in (0, 1):
            S.rect(x - 24 * s + sx * (bw + 4 * s), y + 22 * s + sy * (bl - 98 * s), 20 * s, 54 * s, p.f,
                   **flat(backgroundColor=WHEEL, strokeColor="#000000", roundness={"type": 3, "value": 6}))
    if phone:
        S.rect(p.x0 + cx - 34 * s, y - 6 * s, 68 * s, 12 * s, p.f,
               **flat(backgroundColor="#5b7fd6", strokeColor="#8aa6ec", roundness={"type": 3, "value": 3}))
    for corner, (big, small) in (labels or {}).items():
        left = "l" in corner
        top = "t" in corner
        ly = y + (30 if top else bl - 74) * s
        if left:
            p.t(PAD - 6, ly, big, 15, TXT, abs_y=True)
            p.t(PAD - 6, ly + 20, small, 11, SUB, abs_y=True)
        else:
            p.tr(ly, big, 15, TXT, inset=PAD - 6)
            p.tr(ly + 20, small, 11, SUB, inset=PAD - 6)
    return x, y, bw, bl


def rover_side(p, cx, cy):
    """Three-quarter-ish hero: side view of the rover with the phone up front."""
    x, y = p.x0 + cx - 120, cy - 40
    S.rect(x, y, 240, 62, p.f, **flat(backgroundColor=BODY, strokeColor="#4a4d55", strokeWidth=2,
                                      roundness={"type": 3, "value": 18}))
    S.rect(x + 16, y + 10, 208, 18, p.f, **flat(backgroundColor="#2b2d33", strokeColor="#2b2d33",
                                                roundness={"type": 3, "value": 8}))
    for wx in (x + 22, x + 160):
        S.ellipse(wx, y + 34, 58, 58, p.f, **flat(backgroundColor=WHEEL, strokeColor="#000000", strokeWidth=2))
        S.ellipse(wx + 20, y + 54, 18, 18, p.f, **flat(backgroundColor="#3a3c43", strokeColor="#3a3c43"))
    S.rect(x + 214, y - 44, 12, 48, p.f, **flat(backgroundColor="#5b7fd6", strokeColor="#8aa6ec",
                                                roundness={"type": 3, "value": 3}))
    S.line([(x - 40, y + 94), (x + 280, y + 94)], p.f, kind="line", **flat(strokeColor="#2c2e34", strokeWidth=2))
    S.line([(x + 40, y + 98), (x + 240, y + 104)], p.f, kind="line", **flat(strokeColor=OK, strokeWidth=2))


def caption(p, s):
    S.text(p.x0, p.y0 + H + 16, s, None, size=14, color=GREY)


def note(p, y, s):
    x = p.x0 + W + 14
    S.arrow((x + 10, y + 8), (p.x0 + W - 4, y + 8), p.f, color=PURPLE, width=1)
    S.text(x + 16, y, s, None, size=13, color=PURPLE)


S.text(40, 20, "TerraPhone · Tesla-style UI · wireframes", None, size=34)
S.text(40, 66, "Dark and rover-first: the rover is the control surface. One context card, then a plain list. "
               "E-stop on every screen.", None, size=17, color=GREY)

# ───────────────────────── 1 · Home ─────────────────────────
p = Phone(0, 0, "1 · Home")
p.t(PAD, 54, "Rover 0  ⌄", 26)
p.tr(p.y0 + 60, "☰", 20, TXT)
p.t(PAD, 92, "● Ready", 14, OK)
p.t(PAD + 70, 92, "VIO tracked · base connected", 13, SUB)
rover_side(p, W / 2, p.y0 + 214)
p.icon_row(p.y0 + 310, [("▶", None, TXT), ("◎", None, TXT), ("⌖", None, TXT), ("STOP", None, STOP)])
y = p.y0 + 360
p.card(PAD, y, W - 2 * PAD, 150)
S.text(p.x0 + PAD + 14, y + 14, "Task 9 · Running", p.f, size=14, family=2, color=TXT)
S.text(p.x0 + PAD + 14, y + 36, "DWA v1 · 2.3 m to goal · 41 s", p.f, size=12, family=2, color=SUB)
p.slider(y + 72, 0.62)
S.text(p.x0 + PAD + 14, y + 86, "Timeout in 1 min 20 s", p.f, size=11, family=2, color=SUB)
p.split(y + 106, "Pause", "Clear task")
p.y = y + 166
p.row("✥", "Drive", "Joystick · max 0.5 m/s")
p.row("◈", "Autonomy", "DWA · straight line planner")
p.row("➤", "Go to", "Set a goal on the map")
p.row("⚙", "Rover", "Motors, phone mount, base station")
caption(p, "Tesla home: name ⌄ + status, hero vehicle, icon row,\n"
           "one context card (here: the task), then a plain list.")
note(p, p.y0 + 312, "▶ start · ◎ hold · ⌖ locate\n· STOP = e-stop")
note(p, p.y0 + 420, "card = whatever is active:\ntask, or 'Needs attention'")

# ───────────────────────── 2 · Drive ─────────────────────────
p = Phone(1, 0, "2 · Drive")
p.nav("Drive", "Teleop · motors connected")
rover_top(p, W / 2, p.y0 + 290, 0.95,
          labels={"tl": ("+0.42", "left effort"), "tr": ("+0.38", "right effort"),
                  "bl": ("0.31 m/s", "measured"), "br": ("0.10 rad/s", "turn")})
p.sheet(p.y0 + 470)
p.t(W / 2, 492, "Max speed", 12, SUB, align="center")
p.t(W / 2 - 112, 518, "‹", 30, SUB)
p.t(W / 2, 510, "0.50 m/s", 40, TXT, align="center")
p.t(W / 2 + 100, 518, "›", 30, SUB)
p.t(PAD + 6, 520, "▶", 20, TXT)
p.t(PAD, 548, "Start", 11, SUB)
p.tr(p.y0 + 520, "◎", 20, TXT, inset=PAD + 6)
p.tr(p.y0 + 548, "Hold", 11, SUB)
S.ellipse(p.x0 + W / 2 - 62, p.y0 + 580, 124, 124, p.f, **flat(backgroundColor=CARD, strokeColor=LINE))
S.ellipse(p.x0 + W / 2 - 22, p.y0 + 620, 44, 44, p.f, **flat(backgroundColor="#4a4d55", strokeColor="#4a4d55"))
p.button(p.y0 + 724, "E-STOP", color=STOP, h=50)
caption(p, "Tesla Controls / Flat Tire: values at each wheel.\nClimate: hero over a sheet with one big value and ‹ ›.")
note(p, p.y0 + 230, "live wheel efforts and\nmeasured twist on the rover")

# ───────────────────────── 3 · Autonomy ─────────────────────────
p = Phone(2, 0, "3 · Autonomy")
p.nav("Autonomy", "Running · DWA v1")
y = p.y
p.card(PAD, y, W - 2 * PAD, 268)
S.text(p.x0 + PAD + 14, y + 14, "Controller: DWA · v1", p.f, size=14, family=2, color=TXT)
S.text(p.x0 + PAD + 14, y + 36, "Dynamic window · local goals", p.f, size=12, family=2, color=SUB)
S.text(p.x0 + PAD + 14, y + 64, "Horizon", p.f, size=12, family=2, color=SUB)
p.tr(y + 64, "2.0 s", 12, TXT, inset=PAD + 14)
p.slider(y + 92, 0.2)
S.text(p.x0 + PAD + 14, y + 112, "Speed weight", p.f, size=12, family=2, color=SUB)
p.tr(y + 112, "0.5", 12, TXT, inset=PAD + 14)
p.slider(y + 140, 0.25)
S.text(p.x0 + PAD + 14, y + 162, "Samples", p.f, size=12, family=2, color=SUB)
p.stepper(y + 180, "20")
p.split(y + 228, "Reset values", "Apply", color=SUB)
p.t(3 * (W - 2 * PAD) / 4 + PAD, 0, "", 1)
p.y = y + 288
p.row("◈", "Controller", None, "DWA")
p.row("⤳", "Planner", None, "Straight line")
p.row("▦", "Mapper", None, "Grid 0.1 m")
p.row("◷", "Timeout & stuck", "Set by the controller", None)
p.row("↺", "Return to goal when pushed", None, "On")
caption(p, "Tesla Charging: one card with sliders, a stepper\nand split buttons, then the related rows.\n"
           "Sliders and stepper come from ParamSpec.")
note(p, p.y0 + 230, "applies mid-drive as a task\nupdate (same id, goal kept)")

# ───────────────────────── 4 · Go to ─────────────────────────
p = Phone(0, 1, "4 · Go to")
S.rect(p.x0 + 3, p.y0 + 3, W - 6, 470, p.f, **flat(backgroundColor="#1f3b34", strokeColor="#1f3b34",
                                                   roundness={"type": 3, "value": 40}))
for i in range(5):
    S.line([(p.x0 + 3, p.y0 + 90 + i * 80), (p.x0 + W - 3, p.y0 + 60 + i * 86)], p.f, kind="line",
           **flat(strokeColor="#2d5148", strokeWidth=2))
S.line([(p.x0 + 90, p.y0 + 380), (p.x0 + 150, p.y0 + 300), (p.x0 + 230, p.y0 + 250), (p.x0 + 270, p.y0 + 170)],
       p.f, kind="line", **flat(strokeColor=ACC, strokeWidth=5, roundness={"type": 2}))
p.t(78, 368, "▲", 22, "#ff5a4f")
S.ellipse(p.x0 + 262, p.y0 + 156, 18, 18, p.f, **flat(backgroundColor=TXT, strokeColor=ACC, strokeWidth=4))
S.ellipse(p.x0 + 237, p.y0 + 131, 68, 68, p.f, **flat(backgroundColor="transparent", strokeColor=TXT,
                                                       strokeStyle="dashed"))
p.t(220, 112, "tolerance 0.3 m", 11, TXT)
p.t(PAD, 50, "‹", 26)
p.sheet(p.y0 + 470)
for i, (glyph, title, sub) in enumerate([("●", "Rover", "x 0.0 · y 0.0 · tracked"),
                                         ("⌖", "Goal", "x 3.0 · y 1.0  ·  or GPS 51.50, −0.12")]):
    yy = p.y0 + 496 + i * 58
    p.t(PAD, yy + 10, glyph, 16, ACC if i == 0 else SUB, abs_y=True)
    S.rect(p.x0 + PAD + 30, yy, W - 2 * PAD - 30, 48, p.f, **flat(backgroundColor=CARD, strokeColor=CARD,
                                                                   roundness={"type": 3, "value": 8}))
    S.text(p.x0 + PAD + 42, yy + 6, title, p.f, size=14, family=2, color=TXT)
    S.text(p.x0 + PAD + 42, yy + 26, sub, p.f, size=11, family=2, color=SUB)
yy = p.y0 + 620
S.box(p.x0 + PAD, yy, 150, 32, "Controller: DWA  ›", p.f, size=12, family=2, color=TXT,
      **flat(backgroundColor=CARD, strokeColor=LINE, roundness={"type": 3, "value": 8}))
S.box(p.x0 + PAD + 160, yy, W - 2 * PAD - 160, 32, "Planner: straight  ›", p.f, size=12, family=2, color=TXT,
      **flat(backgroundColor=CARD, strokeColor=LINE, roundness={"type": 3, "value": 8}))
p.button(p.y0 + 670, "Send to Rover · DWA · 3.2 m")
p.t(W / 2, 734, "Cancel", 14, TXT, align="center")
caption(p, "Tesla trip: full-bleed map, route sheet, one blue send\nbutton. Sends a Task (where + how) to the pipeline.")
note(p, p.y0 + 632, "how = controller chip;\nGPS needs a GPS anchor")

# ───────────────────────── 5 · Phone mount ─────────────────────────
p = Phone(1, 1, "5 · Phone mount")
p.nav("Phone mount", "Check needed: forward axis", WARN, right="Save")
x, y, bw, bl = rover_top(p, W / 2, p.y0 + 300, 0.95,
                         labels={"tl": ("0.50 m", "track"), "tr": ("0.23 m", "camera height"),
                                 "bl": ("0.00 m", "offset fwd"), "br": ("Landscape", "orientation")})
S.arrow((p.x0 + W / 2, y + bl / 2 + 30), (p.x0 + W / 2, y + 40), p.f, color=STOP, width=3, roughness=0)
p.t(W / 2 + 8, y - p.y0 + 70, "camera", 11, STOP)
S.arrow((p.x0 + W / 2 - 34, y + bl / 2 + 30), (p.x0 + W / 2 - 34, y + 44), p.f, color=OK, width=3, roughness=0,
        strokeStyle="dashed")
p.t(W / 2 - 92, y - p.y0 + 96, "rover +X", 11, OK)
p.sheet(p.y0 + 470)
p.t(PAD, 492, "Point the rover ahead. The red arrow should\nmatch the green one.", 13, SUB)
y = p.y0 + 540
S.rect(p.x0 + PAD, y, W - 2 * PAD, 44, p.f, **flat(backgroundColor=CARD, strokeColor=CARD,
                                                   roundness={"type": 3, "value": 8}))
for i, lbl in enumerate(["Landscape", "Portrait", "Custom"]):
    seg_w = (W - 2 * PAD) / 3
    if i == 0:
        S.rect(p.x0 + PAD + 3, y + 3, seg_w - 6, 38, p.f, **flat(backgroundColor="#3a3c43", strokeColor="#3a3c43",
                                                                 roundness={"type": 3, "value": 6}))
    p.t(PAD + seg_w * (i + 0.5), y + 13 - p.y0, lbl, 13, TXT if i == 0 else SUB, align="center")
p.y = y + 64
p.row("↻", "Rotate 180°", "Fixes a flipped forward axis")
p.row("⌗", "Advanced", "Roll · pitch · yaw · offset")
p.button(p.y0 + 724, "Confirm mount", color=ACC, h=48)
caption(p, "Tesla Flat Tire: values at each corner of the vehicle.\nClimate: segmented control at the bottom.")
note(p, p.y0 + 300, "live check catches\nthe roll-sign bug")

# ───────────────────────── 6 · Rover settings ─────────────────────────
p = Phone(2, 1, "6 · Rover")
p.nav("Rover")
p.row("◉", "Motors", "Rover2 · connected over Bluetooth")
p.row("☁", "Base station", "Fleet router · connected")
p.row("▭", "Phone mount", "Check needed", None, color=WARN)
p.row("⇢", "Vehicle limits", "0.8 m/s wheels · 1.0 m/s² accel")
p.row("≋", "Velocity loop", "kp 0.3 / 0.1 · skid 2.4")
p.row("ⓘ", "About this rover", "terra-av 0.1.0 · rover 0")
y = p.y + 6
S.line([(p.x0 + PAD, y), (p.x0 + W - PAD, y)], p.f, kind="line", **flat(strokeColor=LINE))
p.t(PAD, y - p.y0 + 18, "BASE STATION", 11, SUB)
y += 42
for i, (title, sub, on) in enumerate([("Fleet router", "tls/argos…:7448 · user terra", True),
                                      ("Local router", "tcp/192.168.1.165:7447 · just router", False)]):
    yy = y + i * 64
    S.rect(p.x0 + PAD, yy, W - 2 * PAD, 56, p.f, **flat(backgroundColor=CARD, strokeColor=CARD,
                                                         roundness={"type": 3, "value": 10}))
    S.text(p.x0 + PAD + 14, yy + 8, title, p.f, size=15, family=2, color=TXT)
    S.text(p.x0 + PAD + 14, yy + 30, sub, p.f, size=11, family=2, color=SUB)
    tx = p.x0 + W - PAD - 58
    S.rect(tx, yy + 16, 44, 24, p.f, **flat(backgroundColor=ACC if on else "#4a4d55",
                                            strokeColor=ACC if on else "#4a4d55", roundness={"type": 3, "value": 12}))
    S.ellipse(tx + (22 if on else 2), yy + 18, 20, 20, p.f, **flat(backgroundColor=TXT, strokeColor=TXT))
p.t(W / 2, y - p.y0 + 150, "Router password", 14, ACC, align="center")
p.t(W / 2, H - 70, "Forget this rover", 14, STOP, align="center")
caption(p, "Tesla Settings / Schedules: icon rows with subtitles,\ncards with blue toggles, red text for destructive actions.")
note(p, p.y0 + 470, "one router on at a time")

S.save(sys.argv[1] if len(sys.argv) > 1 else "app-wireframes.excalidraw")
