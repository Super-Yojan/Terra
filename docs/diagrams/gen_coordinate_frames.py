"""Generate docs/diagrams/coordinate-frames.excalidraw (Terra frames + vehicle model)."""
import json, math, random, sys

random.seed(7)
RED, GREEN, BLUE, INK, GREY, ORANGE, PURPLE = "#e03131", "#2f9e44", "#1971c2", "#1e1e1e", "#868e96", "#f08c00", "#9c36b5"
els = []
_id = 0


def nid(p="e"):
    global _id
    _id += 1
    return f"{p}{_id}"


def base(t, x, y, w, h, frame=None, **kw):
    e = dict(id=nid(t[0]), type=t, x=x, y=y, width=w, height=h, angle=0, strokeColor=INK,
             backgroundColor="transparent", fillStyle="solid", strokeWidth=2, strokeStyle="solid",
             roughness=1, opacity=100, groupIds=[], frameId=frame, roundness=None,
             seed=random.randint(1, 2**31), version=1, versionNonce=random.randint(1, 2**31),
             isDeleted=False, boundElements=[], updated=1, link=None, locked=False)
    e.update(kw)
    els.append(e)
    return e


def rect(x, y, w, h, frame, **kw):
    kw.setdefault("roundness", {"type": 3})
    return base("rectangle", x, y, w, h, frame, **kw)


def ellipse(x, y, w, h, frame, **kw):
    return base("ellipse", x, y, w, h, frame, **kw)


def text(x, y, s, frame, size=18, color=INK, align="left", family=5, **kw):
    lines = s.split("\n")
    w = max(len(l) for l in lines) * size * 0.55
    h = len(lines) * size * 1.25
    if align == "center":
        x -= w / 2
    return base("text", x, y, w, h, frame, text=s, originalText=s, fontSize=size, fontFamily=family,
                textAlign=align, verticalAlign="top", containerId=None, lineHeight=1.25,
                autoResize=True, strokeColor=color, **kw)


def mono(x, y, s, frame, size=16, color=INK, **kw):
    return text(x, y, s, frame, size=size, color=color, family=3, **kw)


def line(pts, frame, arrow_end=None, arrow_start=None, kind="arrow", **kw):
    x0, y0 = pts[0]
    rel = [[px - x0, py - y0] for px, py in pts]
    xs, ys = [p[0] for p in rel], [p[1] for p in rel]
    return base(kind, x0, y0, max(xs) - min(xs), max(ys) - min(ys), frame, points=rel,
                lastCommittedPoint=None, startBinding=None, endBinding=None,
                startArrowhead=arrow_start, endArrowhead=arrow_end, **kw)


def arrow(p, q, frame, color=INK, width=2, head="arrow", both=False, **kw):
    return line([p, q], frame, arrow_end=head, arrow_start=head if both else None,
                strokeColor=color, strokeWidth=width, **kw)


def axis(o, d, label, color, frame, length=130, off=(8, -12)):
    end = (o[0] + d[0] * length, o[1] + d[1] * length)
    arrow(o, end, frame, color=color, width=3, roughness=0)
    text(end[0] + off[0], end[1] + off[1], label, frame, size=20, color=color)


def out_of_page(c, label, color, frame, r=11, off=(16, -10)):
    ellipse(c[0] - r, c[1] - r, 2 * r, 2 * r, frame, strokeColor=color, strokeWidth=3, roughness=0)
    ellipse(c[0] - 3, c[1] - 3, 6, 6, frame, strokeColor=color, backgroundColor=color, roughness=0)
    text(c[0] + off[0], c[1] + off[1], label, frame, size=20, color=color)


def arc(cx, cy, r, a0, a1, frame, color, n=24, head=True, **kw):
    """Arc in math angles (CCW, y up on screen) -> screen coords."""
    pts = [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
            cy - r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]
    return line(pts, frame, arrow_end="arrow" if head else None, strokeColor=color,
                strokeWidth=kw.pop("width", 2), roundness={"type": 2}, **kw)


def frame(x, y, w, h, name):
    f = base("frame", x, y, w, h, None, name=name, roughness=0, strokeWidth=1)
    return f["id"]


W, H, GAP = 760, 640, 60

# ───────────────────────── Panel 1: body frame, top view ─────────────────────────
F = frame(0, 0, W, H, "1 · Body frame (top view)")
text(30, 24, "Body frame  ·  FLU  ·  right-handed  ·  top view", F, size=26)
text(30, 60, "+X forward   +Y left   +Z up (out of page)", F, size=18, color=GREY)
cx, cy = 330, 340
bw, bl = 200, 290          # body width (Y), length (X) in px; track 0.5 m ≈ 250 px between wheel centres
# chassis (X points right on the page)
rect(cx - bl / 2, cy - bw / 2, bl, bw, F, backgroundColor="#e9ecef", fillStyle="hachure")
# wheels: left side is +Y (top of page), right side is −Y (bottom)
for sx in (-1, 1):
    for sy in (-1, 1):
        rect(cx + sx * 95 - 35, cy + sy * 125 - 14, 70, 28, F, backgroundColor="#343a40", strokeColor="#343a40")
text(cx - 30, cy - 175, "left side", F, size=15, color=GREY)
text(cx - 32, cy + 147, "right side", F, size=15, color=GREY)
# phone on the front edge
rect(cx + bl / 2 - 18, cy - 45, 22, 90, F, backgroundColor="#a5d8ff", strokeColor=BLUE)
text(cx + bl / 2 + 10, cy - 70, "phone\n(rear cam →)", F, size=14, color=BLUE)
# axes at body origin (centre of the wheelbase, on the ground)
axis((cx, cy), (1, 0), "+X  forward", RED, F, length=230, off=(-60, 14))
axis((cx, cy), (0, -1), "+Y  left", GREEN, F, length=110, off=(-40, -34))
out_of_page((cx, cy), "", BLUE, F)
text(cx - 40, cy + 14, "+Z", F, size=16, color=BLUE)
# yaw arc
arc(cx, cy, 70, -40, 200, F, PURPLE, width=2)
text(cx + 60, cy - 235, "+yaw / +w : CCW from above = turn left", F, size=15, color=PURPLE)
# track dimension
tx = cx - bl / 2 - 40
arrow((tx, cy - 125), (tx, cy + 125), F, color=GREY, both=True, width=1)
text(tx - 70, cy - 10, "track\n0.5 m", F, size=15, color=GREY)
# v arrow
arrow((cx + 160, cy + 60), (cx + 260, cy + 60), F, color=ORANGE, width=3)
text(cx + 175, cy + 68, "v (m/s)", F, size=16, color=ORANGE)
text(30, H - 70, "Origin: rover centre on the ground.  Pose2.to_body / to_world convert to & from the world frame.\n"
                 "Source: crates/terra-av/src/geometry.rs:107, sense.rs:22", F, size=14, color=GREY)

# ───────────────────────── Panel 2: side view + mount ─────────────────────────
ox = W + GAP
F = frame(ox, 0, W, H, "2 · Side view & phone mount")
text(ox + 30, 24, "Side view  ·  phone mount", F, size=26)
text(ox + 30, 60, "looking from the rover's right side: +X right, +Z up, +Y into the page", F, size=18, color=GREY)
gy = 470
line([(ox + 40, gy), (ox + W - 40, gy)], F, kind="line", strokeColor=GREY, strokeWidth=2)
text(ox + W - 110, gy + 8, "ground", F, size=15, color=GREY)
gx = ox + 300
rect(gx - 150, gy - 110, 300, 70, F, backgroundColor="#e9ecef", fillStyle="hachure")
for sx in (-1, 1):
    ellipse(gx + sx * 95 - 38, gy - 76, 76, 76, F, backgroundColor="#343a40", strokeColor="#343a40")
# phone standing on the front, landscape, screen facing the rover
px, ph = gx + 150, gy - 230
rect(px, ph, 18, 110, F, backgroundColor="#a5d8ff", strokeColor=BLUE)
ellipse(px + 12, ph + 8, 12, 12, F, backgroundColor=INK, roughness=0)
arrow((px + 26, ph + 14), (px + 120, ph + 14), F, color=BLUE, width=2, strokeStyle="dashed")
text(px + 40, ph - 18, "lens = cam −Z", F, size=14, color=BLUE)
# body axes
axis((gx, gy), (1, 0), "+X", RED, F, length=360, off=(6, -26))
axis((gx, gy), (0, -1), "+Z", BLUE, F, length=320, off=(8, -10))
ellipse(gx - 11, gy - 11, 22, 22, F, strokeColor=GREEN, strokeWidth=3, roughness=0)
line([(gx - 7, gy - 7), (gx + 7, gy + 7)], F, kind="line", strokeColor=GREEN, strokeWidth=3, roughness=0)
line([(gx - 7, gy + 7), (gx + 7, gy - 7)], F, kind="line", strokeColor=GREEN, strokeWidth=3, roughness=0)
text(gx - 110, gy + 14, "+Y into page ⊗", F, size=16, color=GREEN)
# height dimension
dx = px + 60
arrow((dx, gy), (dx, ph + 14), F, color=GREY, both=True, width=1)
text(dx + 8, gy - 120, "0.23 m", F, size=15, color=GREY)
mono(ox + 30, H - 105, "[mount]  roll = -90   pitch = 0   yaw = 0\n"
                       "         offset = [0.0, 0.0, 0.23]   # camera in body frame (m)", F, size=15)
text(ox + 30, H - 135, "offset x = 0 puts the camera above the origin; set x to the real forward distance", F, size=14, color=ORANGE)
text(ox + 30, H - 55, "camera_to_body = Rz(yaw)·Ry(pitch)·Rx(roll)   (geometry.rs:59, config.rs:244)", F, size=14, color=GREY)

# ───────────────────────── Panel 3: ARKit frames → body ─────────────────────────
oy = H + GAP
F = frame(0, oy, W, H, "3 · ARKit camera → body")
text(30, oy + 24, "ARKit camera frame → body frame", F, size=26)
text(30, oy + 60, "ARKit is Y-up (OpenGL style). This is the only Y-up frame in Terra.", F, size=18, color=GREY)
# phone, landscape, seen from behind the screen (as the rover sees it)
fx, fy = 70, oy + 150
rect(fx, fy, 260, 140, F, backgroundColor="#d0ebff", strokeColor=BLUE)
text(fx + 12, fy + 10, "phone screen\n(landscape)", F, size=16, color=BLUE)
co = (fx + 130, fy + 70)
axis(co, (1, 0), "+X right", RED, F, length=210, off=(-30, 10))
axis(co, (0, -1), "+Y up", GREEN, F, length=120, off=(8, -6))
out_of_page((fx + 230, fy + 115), "+Z toward viewer\n(out of screen)", BLUE, F, off=(-60, 18))
text(fx - 30, fy + 195, "Camera axes (PhoneSensors.swift:109): +X right, +Y up, −Z forward (out of the lens)", F, size=15)
# mapping table
mono(40, oy + 370, "Needed mapping (lens looking along rover +X):\n"
                   "  body +X  =  −cam Z     (lens → forward)\n"
                   "  body +Y  =  −cam X     (cam right → rover right)\n"
                   "  body +Z  =  +cam Y     (cam up → up)\n\n"
                   "  camera_to_body = [[ 0, 0,-1],\n"
                   "                    [-1, 0, 0],\n"
                   "                    [ 0, 1, 0]]  → roll=+90, pitch=0, yaw=−90", F, size=15)
mono(40, oy + 545, "⚠ current config roll=-90, yaw=0 maps\n  lens → body −Y and cam-up → body −Z", F, size=15, color=RED)
text(430, oy + 545, "Depth points are in the camera frame;\nreplay export shows images as RDF\n(right, down, forward).", F, size=14, color=GREY)

# ───────────────────────── Panel 4: vehicle model ─────────────────────────
F = frame(ox, oy, W, H, "4 · Vehicle model (DiffDrive)")
text(ox + 30, oy + 24, "Vehicle model  ·  DiffDrive (unicycle)", F, size=26)
text(ox + 30, oy + 60, "world frame: X/Y on the ground, Z up, yaw from +X (CCW)", F, size=18, color=GREY)
wo = (ox + 70, oy + 420)
axis(wo, (1, 0), "X_w", RED, F, length=260, off=(4, 6))
axis(wo, (0, -1), "Y_w", GREEN, F, length=300, off=(8, -6))
# pose at P, arc to P'
R = 150
c = (ox + 190, oy + 210)               # ICR (instantaneous centre of rotation), on the left of the rover
th0, th1 = -90, -10                    # angle of rover position around the ICR
P = (c[0] + R * math.cos(math.radians(th0)), c[1] - R * math.sin(math.radians(th0)))
P2 = (c[0] + R * math.cos(math.radians(th1)), c[1] - R * math.sin(math.radians(th1)))
arc(c[0], c[1], R, th0, th1, F, ORANGE, width=3, strokeStyle="dashed")
ellipse(c[0] - 4, c[1] - 4, 8, 8, F, backgroundColor=INK, roughness=0)
text(c[0] - 40, c[1] - 30, "ICR", F, size=15)
arrow(c, P, F, color=GREY, width=1, head=None)
text(c[0] + 6, c[1] + 50, "r = v / w", F, size=15, color=GREY)
for (q, ang, lab) in [(P, 0, "pose (x, y, θ)"), (P2, 80, "pose after dt")]:
    a = math.radians(ang)
    rect(q[0] - 22, q[1] - 14, 44, 28, F, backgroundColor="#e9ecef", angle=-a)
    arrow(q, (q[0] + 70 * math.cos(a), q[1] - 70 * math.sin(a)), F, color=RED, width=2, roughness=0)
    text(q[0] + 20 if ang == 0 else q[0] - 125, q[1] + 18 if ang == 0 else q[1] - 62, lab, F, size=15)
text(P[0] + 75, P[1] - 12, "θ", F, size=16, color=PURPLE)
# equations
mono(ox + 420, oy + 110,
     "Twist (body):  v  m/s, w  rad/s (+left)\n\n"
     "step(pose, twist, dt):\n"
     " w≈0: x += v·cosθ·dt\n"
     "      y += v·sinθ·dt\n"
     " else r = v/w\n"
     "  x' = x + r(sin(θ+w·dt) − sinθ)\n"
     "  y' = y − r(cos(θ+w·dt) − cosθ)\n"
     "  θ' = θ + w·dt  (not wrapped)", F, size=14)
mono(ox + 30, oy + 470,
     "Wheel sides:  v_L = v − w·track/2     v_R = v + w·track/2\n"
     "clamp:  fastest = |v| + |w|·track/2 ≤ max_wheel_speed (0.8)\n"
     "        else scale v, w by max/fastest  (keeps turn radius)\n"
     "Limits: max_wheel_speed, max_accel (v̇), max_yaw_accel (ẇ)", F, size=14)
text(ox + 30, H + oy - 50, "Source: crates/terra-av/src/vehicle.rs   ·   step() and limits() are still todo!()", F, size=14, color=GREY)

scene = dict(type="excalidraw", version=2, source="https://excalidraw.com", elements=els,
             appState=dict(viewBackgroundColor="#ffffff", gridSize=None, exportBackground=True), files={})
# Draw-order keys (fractional indexing), required by Excalidraw+.
_digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
for _i, _e in enumerate(els):
    _e["index"] = "b" + _digits[_i // 62] + _digits[_i % 62]
json.dump(scene, open(sys.argv[1], "w"), indent=1)
print(len(els), "elements")
