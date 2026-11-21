"""Generate app-wireframes-light.excalidraw: TerraPhone configuration UI (low fidelity).

    python3 docs/diagrams/gen_app_wireframes.py docs/diagrams/app-wireframes-light.excalidraw

References (Mobbin): Polestar / Rivian home tiles, Eight Sleep / Philips Hue
grouped settings, ElevenLabs model cards + sliders, Mercedes "last changes
sent", Tesla top-down vehicle, Glow / Starlink network settings, Oura /
Phantom pairing, Starlink slide-to-confirm.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from excal import *  # noqa: E402,F403

S = Scene(seed=21)
W, H = 360, 860          # phone screen
GAP_X, GAP_Y = 230, 160
PAD = 20
LINE = "#ced4da"
FILL = "#f8f9fa"


class Phone:
    def __init__(self, col, row, name, title, back=None, action=None):
        self.x0 = 40 + col * (W + GAP_X)
        self.y0 = 120 + row * (H + GAP_Y)
        self.f = S.frame(self.x0, self.y0, W, H, name)
        S.rect(self.x0, self.y0, W, H, self.f, strokeColor=INK, strokeWidth=2, roughness=0,
               roundness={"type": 3, "value": 40})
        S.text(self.x0 + 28, self.y0 + 14, "9:41", self.f, size=13, family=2)
        S.text(self.x0 + W - 70, self.y0 + 14, "▮▮ ◠ ▭", self.f, size=12, family=2, color=GREY)
        y = self.y0 + 50
        if back:
            S.text(self.x0 + PAD, y + 4, "‹ " + back, self.f, size=15, family=2, color=BLUE)
        if action:
            self.right(y + 4, action, size=15, color=BLUE)
        self.y = y + 34
        if title:
            S.text(self.x0 + PAD, self.y, title, self.f, size=26, family=2)
            self.y += 46

    # layout helpers -------------------------------------------------------
    def right(self, y, s, size=15, color=GREY, family=2):
        w, _ = S._measure(s, size, family)
        S.text(self.x0 + W - PAD - w, y, s, self.f, size=size, color=color, family=family)

    def gap(self, h=10):
        self.y += h

    def header(self, s):
        self.y += 8
        S.text(self.x0 + PAD + 4, self.y, s.upper(), self.f, size=11, family=2, color=GREY)
        self.y += 20

    def group(self, rows, row_h=38):
        """rows: (label, value, chevron, color)"""
        h = row_h * len(rows)
        S.rect(self.x0 + PAD, self.y, W - 2 * PAD, h, self.f, backgroundColor=FILL,
               strokeColor=LINE, strokeWidth=1, roughness=0, roundness={"type": 3, "value": 12})
        for i, row in enumerate(rows):
            label, value, chev, color = (list(row) + [None, False, INK])[:4]
            yy = self.y + i * row_h
            if i:
                S.line([(self.x0 + PAD + 14, yy), (self.x0 + W - PAD, yy)], self.f, kind="line",
                       strokeColor=LINE, strokeWidth=1, roughness=0)
            S.text(self.x0 + PAD + 14, yy + 10, label, self.f, size=15, family=2, color=color or INK)
            tail = (value or "") + ("  ›" if chev else "")
            if tail:
                w, _ = S._measure(tail, 14, 2)
                S.text(self.x0 + W - PAD - 14 - w, yy + 11, tail, self.f, size=14, family=2, color=GREY)
        self.y += h + 6

    def note(self, s, color=GREY, size=12):
        S.text(self.x0 + PAD + 4, self.y, s, self.f, size=size, family=2, color=color)
        self.y += 16 * (s.count("\n") + 1) + 6

    def button(self, s, bg=INK, fg="#ffffff", h=48, outline=False):
        S.box(self.x0 + PAD, self.y, W - 2 * PAD, h, s, self.f, size=16, family=2, color=fg,
              backgroundColor="transparent" if outline else bg, strokeColor=bg, roughness=0,
              roundness={"type": 3, "value": 24})
        self.y += h + 10

    def slider(self, label, value, frac, lo="", hi="", steppers=False):
        x = self.x0 + PAD
        S.text(x, self.y, label, self.f, size=14, family=2)
        self.right(self.y, value, size=14, color=INK, family=3)
        y = self.y + 30
        x0, x1 = (x + 26, self.x0 + W - PAD - 26) if steppers else (x, self.x0 + W - PAD)
        if steppers:
            S.text(x, y - 11, "−", self.f, size=20, family=2)
            S.text(x1 + 12, y - 11, "+", self.f, size=20, family=2)
        S.line([(x0, y), (x1, y)], self.f, kind="line", strokeColor=LINE, strokeWidth=4, roughness=0)
        k = x0 + (x1 - x0) * frac
        S.line([(x0, y), (k, y)], self.f, kind="line", strokeColor=BLUE, strokeWidth=4, roughness=0)
        S.ellipse(k - 9, y - 9, 18, 18, self.f, backgroundColor="#ffffff", strokeColor=BLUE, roughness=0)
        if lo or hi:
            S.text(x0, y + 12, lo, self.f, size=11, family=2, color=GREY)
            w, _ = S._measure(hi, 11, 2)
            S.text(x1 - w, y + 12, hi, self.f, size=11, family=2, color=GREY)
        self.y += 70


def note_beside(p, y, s, color=PURPLE):
    """Design note to the right of a phone, with a pointer."""
    x = p.x0 + W + 14
    S.arrow((x + 10, y + 8), (p.x0 + W - 6, y + 8), p.f, color=color, width=1, head="arrow")
    S.text(x + 16, y, s, None, size=13, family=5, color=color)


S.text(40, 20, "TerraPhone · configuration UI · wireframes", None, size=34)
S.text(40, 66, "Low fidelity. Mobbin references noted under each screen. E-stop is reachable from every screen.",
       None, size=17, color=GREY)

# ───────────────────────── 1 · Home ─────────────────────────
p = Phone(0, 0, "1 · Home", None, action="⚙")
S.text(p.x0 + PAD, p.y - 4, "Rover 0", p.f, size=30, family=2)
S.text(p.x0 + PAD, p.y + 36, "● Ready to drive", p.f, size=15, family=2, color=GREEN)
S.text(p.x0 + W - 110, p.y - 30, "0", p.f, size=120, family=2, color="#dee2e6")
p.y += 80
tw, th = (W - 2 * PAD - 12) / 2, 96
tiles = [("Motors", "Rover2 · BLE", GREEN_BG, GREEN), ("Base station", "Cloud · connected", GREEN_BG, GREEN),
         ("Tracking", "VIO tracked", GREEN_BG, GREEN), ("Task", "Needs attention\nE-stop", "#fff4e6", ORANGE)]
for i, (name, state, bg, col) in enumerate(tiles):
    tx = p.x0 + PAD + (i % 2) * (tw + 12)
    ty = p.y + (i // 2) * (th + 12)
    S.rect(tx, ty, tw, th, p.f, backgroundColor=bg, strokeColor=col, strokeWidth=1, roughness=0,
           roundness={"type": 3, "value": 14})
    S.text(tx + 12, ty + 12, name, p.f, size=15, family=2)
    S.text(tx + 12, ty + 40, state, p.f, size=13, family=2, color=col)
    S.text(tx + tw - 22, ty + 10, "›", p.f, size=16, family=2, color=GREY)
p.y += 2 * th + 30
S.ellipse(p.x0 + W / 2 - 70, p.y, 140, 140, p.f, backgroundColor=FILL, strokeColor=LINE, roughness=0)
S.ellipse(p.x0 + W / 2 - 22, p.y + 48, 44, 44, p.f, backgroundColor=GREY_BG, strokeColor=GREY, roughness=0)
S.text(p.x0 + W / 2 - 30, p.y + 146, "joystick", p.f, size=12, family=2, color=GREY)
p.y += 172
S.box(p.x0 + PAD, p.y, 90, 48, "Start", p.f, size=16, family=2, color="#ffffff", backgroundColor=BLUE,
      strokeColor=BLUE, roughness=0, roundness={"type": 3, "value": 24})
S.box(p.x0 + PAD + 102, p.y, W - 2 * PAD - 102, 48, "E-STOP", p.f, size=18, family=2, color="#ffffff",
      backgroundColor=RED, strokeColor=RED, roughness=0, roundness={"type": 3, "value": 24})
S.text(p.x0, p.y0 + H + 14, "Polestar / Rivian: big identity + status tiles.\nEach tile opens its settings section.",
       None, size=14, color=GREY)
note_beside(p, p.y0 + 290, "tile = one-line state;\ncolour = OK / attention")

# ───────────────────────── 2 · Settings ─────────────────────────
p = Phone(1, 0, "2 · Settings", "Settings", back="Home")
S.rect(p.x0 + PAD, p.y, W - 2 * PAD, 52, p.f, backgroundColor="#fff4e6", strokeColor=ORANGE, strokeWidth=1,
       roughness=0, roundness={"type": 3, "value": 12})
S.text(p.x0 + PAD + 14, p.y + 8, "3 unapplied changes", p.f, size=15, family=2, color=ORANGE)
S.text(p.x0 + PAD + 14, p.y + 28, "Last applied 2 min ago", p.f, size=12, family=2, color=GREY)
S.text(p.x0 + W - PAD - 60, p.y + 16, "Apply", p.f, size=15, family=2, color=BLUE)
p.y += 62
p.header("Rover")
p.group([("Name", "Rover 0", True), ("Motors", "Rover2 · connected", True)])
p.header("Driving")
p.group([("Vehicle limits", "0.8 m/s", True), ("Velocity loop", None, True), ("Phone mount", "check ⚠", True, None)])
p.header("Autonomy")
p.group([("Controller", "teleop", True), ("Planner", "straight_line", True), ("Mapper", "grid 0.1 m", True)])
p.header("Base station")
p.group([("Router", "Cloud", True), ("Status", "Connected")])
p.header("Advanced")
p.group([("Reset to defaults", None, False, RED)])
S.text(p.x0, p.y0 + H + 14, "Eight Sleep / Philips Hue: grouped list,\nvalue on the right. Mercedes: 'last changes\nsent' banner + one Apply.", None, size=14, color=GREY)
note_beside(p, p.y0 + 150, "Apply rebuilds the pipeline\n(only while stopped)")

# ───────────────────────── 3 · Controller ─────────────────────────
p = Phone(2, 0, "3 · Controller", "Controller", back="Settings", action="Save")
cw = 200
for i, (name, desc, tag, sel) in enumerate([("DWA", "Avoids obstacles\nwhile following", "Local goals", True),
                                            ("Drive pose", "Steer onto the\nnext pose", "Local goals", False)]):
    cx = p.x0 + PAD + i * (cw + 12)
    S.rect(cx, p.y, cw, 130, p.f, backgroundColor=FILL, strokeColor=INK if sel else LINE,
           strokeWidth=2 if sel else 1, roughness=0, roundness={"type": 3, "value": 14})
    S.text(cx + 14, p.y + 12, "v1", p.f, size=11, family=3, color=GREY)
    if sel:
        S.text(cx + cw - 30, p.y + 8, "✓", p.f, size=18, family=2)
    S.text(cx + 14, p.y + 34, name, p.f, size=18, family=2)
    S.text(cx + 14, p.y + 60, desc, p.f, size=13, family=2, color=GREY)
    S.box(cx + 14, p.y + 98, 96, 22, tag, p.f, size=11, family=2, color=GREY, backgroundColor="#ffffff",
          strokeColor=LINE, roughness=0, roundness={"type": 3, "value": 11})
p.y += 150
p.header("Parameters  (from ParamSpec)")
p.slider("Horizon", "2.0 s", 0.2, "0.1 s", "10 s")
p.slider("Samples", "20", 0.35, "1", "50")
p.slider("Clearance weight", "1.0", 0.5, "0", "2")
p.slider("Speed weight", "0.5", 0.25, "0", "2")
p.note("Help text for the selected slider comes from ParamSpec.help.")
p.gap(10)
p.button("Reset values", bg=INK, fg=INK, outline=True)
S.text(p.x0, p.y0 + H + 14, "ElevenLabs: model cards (scroll sideways) +\nlabelled sliders + Reset / Save. Generated\nfrom Capabilities: no per-controller UI code.", None, size=14, color=GREY)
note_beside(p, p.y0 + 330, "range = min / max\nunit + value shown live")

# ───────────────────────── 4 · Phone mount ─────────────────────────
p = Phone(0, 1, "4 · Phone mount", "Phone mount", back="Settings", action="Save")
# top-down rover sketch with the phone at the front
rx, ry, rw, rh = p.x0 + 100, p.y + 10, 160, 210
S.rect(rx, ry, rw, rh, p.f, backgroundColor=FILL, strokeColor=INK, roughness=0, roundness={"type": 3, "value": 18})
for sx in (-1, 1):
    for sy in (0, 1):
        S.rect(rx + (rw if sx > 0 else -22), ry + 24 + sy * 128, 22, 50, p.f, backgroundColor=INK,
               strokeColor=INK, roughness=0, roundness={"type": 3, "value": 6})
S.rect(rx + rw / 2 - 40, ry - 8, 80, 18, p.f, backgroundColor=BLUE_BG, strokeColor=BLUE, roughness=0,
       roundness={"type": 3, "value": 4})
S.text(rx + rw / 2 - 20, ry - 30, "phone", p.f, size=12, family=2, color=BLUE)
S.arrow((rx + rw / 2, ry + rh / 2), (rx + rw / 2, ry + 40), p.f, color=RED, width=3, roughness=0)
S.text(rx + rw / 2 + 8, ry + 50, "+X forward", p.f, size=12, family=2, color=RED)
S.text(rx + rw / 2 - 52, ry + rh - 26, "drag phone to set offset", p.f, size=11, family=2, color=GREY)
p.y += 240
p.header("Orientation")
p.group([("Landscape, camera forward", "✓"), ("Portrait, camera forward", None)])
p.header("Live check")
S.rect(p.x0 + PAD, p.y, W - 2 * PAD, 86, p.f, backgroundColor=GREEN_BG, strokeColor=GREEN, strokeWidth=1,
       roughness=0, roundness={"type": 3, "value": 12})
S.arrow((p.x0 + 70, p.y + 62), (p.x0 + 70, p.y + 18), p.f, color=GREEN, width=3, roughness=0)
S.text(p.x0 + 100, p.y + 14, "Camera forward = rover +X", p.f, size=14, family=2, color=GREEN)
S.text(p.x0 + 100, p.y + 38, "Point the rover ahead and\nconfirm the arrow matches.", p.f, size=12, family=2,
       color=GREY)
p.y += 96
p.group([("Advanced: roll / pitch / yaw / offset", None, True)])
S.text(p.x0, p.y0 + H + 14, "Tesla: top-down vehicle you interact with.\nBabbel mic test: live feedback while adjusting\n(catches the roll-sign bug).", None, size=14, color=GREY)
note_beside(p, p.y0 + 450, "presets first;\nEuler angles hidden\nunder Advanced")

# ───────────────────────── 5 · Base station ─────────────────────────
p = Phone(1, 1, "5 · Base station", "Base station", back="Settings", action="Save")
p.group([("Fleet router (cloud)", "✓"), ("Local router (development)", None)])
p.note("Local: run `just router` on your Mac;\nthe phone must be on the same Wi-Fi.")
p.header("Connection")
for label, val in [("Endpoint", "tls/argos…sslip.io:7448"), ("Rover ID", "0"), ("User", "terra"),
                   ("Password", "••••••••")]:
    S.text(p.x0 + PAD + 4, p.y, label, p.f, size=12, family=2, color=GREY)
    S.rect(p.x0 + PAD, p.y + 18, W - 2 * PAD, 40, p.f, strokeColor=LINE, strokeWidth=1, roughness=0,
           roundness={"type": 3, "value": 10})
    S.text(p.x0 + PAD + 12, p.y + 29, val, p.f, size=14, family=3)
    p.y += 62
p.note("Password is stored in the Keychain, never in the config file.")
p.header("Status")
p.group([("Router", "● Connected"), ("Sent / dropped", "1 204 / 3"), ("Last task", "rejected (no registry)")])
S.rect(p.x0 + PAD, p.y + 4, W - 2 * PAD, 52, p.f, backgroundColor="#fff9db", strokeColor=ORANGE, strokeWidth=1,
       roughness=0, roundness={"type": 3, "value": 12})
S.text(p.x0 + PAD + 12, p.y + 12, "⚠ Local router: allow Local Network\naccess when iOS asks.", p.f, size=12,
       family=2, color=ORANGE)
S.text(p.x0, p.y0 + H + 14, "Glow: two-option picker with a hint.\nStarlink: form + help text.\nNordVPN: warning box.", None, size=14, color=GREY)
note_beside(p, p.y0 + 140, "choosing 'Local' fills\nendpoint with this Mac's IP")

# ───────────────────────── 6 · Pairing ─────────────────────────
p = Phone(2, 1, "6 · Pair rover", None, back="Cancel")
mini = (H - 140) / 3
steps = [("Looking for rovers…", "Turn on the rover-pi.\nHold USR to open pairing.", "◌"),
         ("Rover2 found", "Connect      ·      Not my rover", "◉"),
         ("Confirm code 482 913", "Matches the rover's screen?\nConfirm   ·   Cancel", "✓")]
for i, (title, sub, glyph) in enumerate(steps):
    yy = p.y + i * (mini + 6)
    S.rect(p.x0 + PAD, yy, W - 2 * PAD, mini - 6, p.f, backgroundColor=FILL, strokeColor=LINE, strokeWidth=1,
           roughness=0, roundness={"type": 3, "value": 14})
    S.ellipse(p.x0 + W / 2 - 30, yy + 18, 60, 60, p.f, backgroundColor=BLUE_BG, strokeColor=BLUE, roughness=0)
    S.text(p.x0 + W / 2 - 9, yy + 34, glyph, p.f, size=22, family=2, color=BLUE)
    S.text(p.x0 + W / 2, yy + 94, title, p.f, size=17, family=2, align="center")
    S.text(p.x0 + W / 2, yy + 122, sub, p.f, size=13, family=2, color=GREY, align="center")
    S.text(p.x0 + PAD + 10, yy + 10, f"{i + 1}", p.f, size=13, family=2, color=GREY)
    if i < 2:
        S.arrow((p.x0 + W / 2, yy + mini - 18), (p.x0 + W / 2, yy + mini + 6), p.f, color=GREY, width=1)
S.text(p.x0, p.y0 + H + 14, "Phantom: 'looking for devices'. Oura: 'Connect\n/ not mine'. Hatch: hardware hint. Code\ncheck matches rover-pi pairing.py.", None, size=14, color=GREY)

# navigation arrows between screens
nav = PURPLE
S.text(40 + W + 10, 120 + 40, "tap ⚙ / tile →", None, size=13, color=nav)

S.save(sys.argv[1] if len(sys.argv) > 1 else "app-wireframes-light.excalidraw")
