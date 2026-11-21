"""Generate docs/diagrams/coordinate-transforms.excalidraw (math & intuition of coordinate transforms)."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from excal import (
    BLUE, BLUE_BG, GREEN, GREEN_BG, GREY, GREY_BG, INK, ORANGE, PURPLE, RED, RED_BG,
    YELLOW_BG, Scene,
)

s = Scene(seed=11)
W, H, GAP = 820, 680, 60

# ═══════════════════════════════════════════════════════════════════════════
# Panel 1: What a coordinate transformation actually is (2D)
# ═══════════════════════════════════════════════════════════════════════════
F = s.frame(0, 0, W, H, "1 · What a coordinate transform is")
s.text(30, 22, "1 · Same point, two coordinate frames", F, size=26)
s.text(30, 58, "A transform doesn't move the point — it translates its address from frame B to frame W.", F, size=16, color=GREY)

# World frame W
wo = (85, 410)
s.axis(wo, (1, 0), "+X_W", RED, F, length=295, off=(6, -12))
s.axis(wo, (0, -1), "+Y_W", GREEN, F, length=260, off=(-22, -30))
s.text(wo[0] - 42, wo[1] + 10, "W (0,0)", F, size=14, color=GREY)

# Rover body frame B at t = (tx, ty), rotated by theta = 35 deg CCW
bo = (240, 320)
th = math.radians(35)
c, sn = math.cos(th), math.sin(th)
# Rover body box centred at bo
s.rect(bo[0] - 32, bo[1] - 20, 64, 40, F, backgroundColor=GREY_BG, strokeColor=INK, angle=-th)
# Body axes
s.axis(bo, (c, -sn), "+X_B", RED, F, length=120, off=(6, -16))
s.axis(bo, (-sn, -c), "+Y_B", GREEN, F, length=105, off=(-34, -22))
# Translation vector t from W to B
s.arrow(wo, bo, F, color=ORANGE, width=2, strokeStyle="dashed")
s.text(140, 372, "t = (x, y)", F, size=15, color=ORANGE)
# Angle arc theta
s.line([bo, (bo[0] + 75, bo[1])], F, kind="line", strokeColor=GREY, strokeWidth=1, strokeStyle="dashed")
s.arc(bo[0], bo[1], 52, 0, 35, F, PURPLE, width=2)
s.text(bo[0] + 58, bo[1] - 24, "θ", F, size=17, color=PURPLE)

# Physical point P in body frame: p^B = (85, 55) px
pbx, pby = 85, 55
px = bo[0] + pbx * c - pby * sn
py = bo[1] - (pbx * sn + pby * c)
s.ellipse(px - 6, py - 6, 12, 12, F, backgroundColor=BLUE, strokeColor=BLUE, roughness=0)
s.text(px - 22, py - 34, "P (same point!)", F, size=15, color=BLUE)
# Vector from B to P
s.arrow(bo, (px, py), F, color=BLUE, width=2)
s.text(bo[0] + 8, bo[1] - 66, "p^B", F, size=15, color=BLUE)
# Vector from W to P
s.arrow(wo, (px, py), F, color=PURPLE, width=2, strokeStyle="dashed")
s.text(132, 268, "p^W", F, size=15, color=PURPLE)

# Equations card on right
s.rect(455, 115, 340, 270, F, backgroundColor=BLUE_BG, strokeColor=BLUE, strokeWidth=1)
s.text(472, 128, "Body → World  (to_world)", F, size=18, color=BLUE)
s.mono(472, 162, "p^W = R(θ) · p^B + t", F, size=18, color=INK)
s.text(472, 198, "Step 1: rotate p^B to align\n        with world axes  → R(θ)·p^B\nStep 2: shift by rover\n        position t       → + t", F, size=15, color=INK)

s.rect(455, 405, 340, 225, F, backgroundColor=YELLOW_BG, strokeColor=ORANGE, strokeWidth=1)
s.text(472, 418, "World → Body  (to_body)", F, size=18, color=ORANGE)
s.mono(472, 452, "p^B = R(θ)^T · (p^W − t)", F, size=17, color=INK)
s.text(472, 488, "Undo in REVERSE order:\nStep 1: subtract t first\nStep 2: rotate back by −θ\n        using R^T (= R⁻¹)", F, size=15, color=INK)

s.text(30, H - 55, "In Terra: Pose2::to_world and Pose2::to_body in crates/terra-av/src/geometry.rs", F, size=14, color=GREY)

# ═══════════════════════════════════════════════════════════════════════════
# Panel 2: Why a rotation matrix works (the Columns Trick)
# ═══════════════════════════════════════════════════════════════════════════
ox = W + GAP
F = s.frame(ox, 0, W, H, "2 · Why rotation matrices work")
s.text(ox + 30, 22, "2 · The Columns Trick: what R actually stores", F, size=26)
s.text(ox + 30, 58, "The columns of R_{W←B} are literally where B's unit axes point, written in frame W.", F, size=16, color=GREY)

# Unit circle showing rotated basis vectors
uc = (ox + 185, 295)
rad = 98
s.ellipse(uc[0] - rad, uc[1] - rad, 2 * rad, 2 * rad, F, strokeColor=GREY, strokeWidth=1, strokeStyle="dashed")
s.axis(uc, (1, 0), "X_W", GREY, F, length=130, off=(6, -12))
s.axis(uc, (0, -1), "Y_W", GREY, F, length=130, off=(-16, -28))

th2 = math.radians(35)
c2, s2 = math.cos(th2), math.sin(th2)
# x_B unit vector = (cos θ, sin θ)
xend = (uc[0] + rad * c2, uc[1] - rad * s2)
s.arrow(uc, xend, F, color=RED, width=4, roughness=0)
s.text(xend[0] + 8, xend[1] - 24, "x̂_B =\n(cosθ, sinθ)", F, size=14, color=RED)
# y_B unit vector = (-sin θ, cos θ)
yend = (uc[0] - rad * s2, uc[1] - rad * c2)
s.arrow(uc, yend, F, color=GREEN, width=4, roughness=0)
s.text(ox + 22, yend[1] - 18, "ŷ_B =\n(−sinθ, cosθ)", F, size=14, color=GREEN)
# angle arc
s.arc(uc[0], uc[1], 45, 0, 35, F, PURPLE, width=2)
s.text(uc[0] + 50, uc[1] - 20, "θ", F, size=16, color=PURPLE)

# Matrix breakdown card
s.rect(ox + 410, 120, 380, 240, F, backgroundColor=GREY_BG, strokeColor=INK, strokeWidth=1)
s.text(ox + 430, 134, "Build R by stacking those two vectors:", F, size=16, color=INK)
s.mono(ox + 440, 172, "         col 1      col 2\n"
                      "        (x̂_B)      (ŷ_B)\n"
                      "R(θ) = [ cosθ      −sinθ ]\n"
                      "       [ sinθ       cosθ ]", F, size=16, color=INK)
s.text(ox + 430, 278, "Multiply R · p^B:", F, size=15, color=GREY)
s.mono(ox + 430, 304, "R · [px] = px · x̂_B + py · ŷ_B\n    [py]", F, size=15, color=BLUE)

# Bottom intuition boxes
s.rect(ox + 30, 445, 365, 175, F, backgroundColor=GREEN_BG, strokeColor=GREEN, strokeWidth=1)
s.text(ox + 48, 458, "Geometric meaning of R · p^B", F, size=17, color=GREEN)
s.text(ox + 48, 490, "\"Walk px steps along the rover's\n forward axis x̂_B, then py steps\n along its left axis ŷ_B.\"\n\nSame rule in 3D: R is 3×3 with\ncolumns [ x̂_B | ŷ_B | ẑ_B ]!", F, size=15, color=INK)

s.rect(ox + 415, 445, 375, 175, F, backgroundColor=RED_BG, strokeColor=RED, strokeWidth=1)
s.text(ox + 433, 458, "Why R⁻¹ = R^T (free inverse!)", F, size=17, color=RED)
s.text(ox + 433, 490, "The columns have length 1 and are\nperpendicular (x̂·ŷ = 0, x̂·x̂ = 1).\nSo R^T · R = I.\n\nEach row of R^T is a dot product:\n  px^B = x̂_B · v,   py^B = ŷ_B · v", F, size=15, color=INK)

s.text(ox + 30, H - 45, "Never memorize signs — just ask \"where do +X, +Y, +Z go?\" and write them as columns.", F, size=14, color=GREY)

# ═══════════════════════════════════════════════════════════════════════════
# Panel 3: Chaining frames in Terra (Subscript cancellation)
# ═══════════════════════════════════════════════════════════════════════════
oy = H + GAP
F = s.frame(0, oy, W, H, "3 · Chaining frames (Terra VIO)")
s.text(30, oy + 22, "3 · Chaining transforms: read right-to-left", F, size=26)
s.text(30, oy + 58, "Multiply rotations to hop across frames; matching adjacent subscripts cancel out.", F, size=16, color=GREY)

# 4 frame boxes in a horizontal chain
by = oy + 140
bw, bh = 148, 84
bxs = [28, 234, 440, 646]
labels = [
    ("Body  B\n+X fwd, +Y left\n+Z up", GREEN_BG, GREEN),
    ("Camera  C\n+X right, +Y up\n−Z lens fwd", BLUE_BG, BLUE),
    ("ARKit World  A\nY-up world\n(from ARKit)", YELLOW_BG, ORANGE),
    ("Terra World  W\nZ-up world\n+X fwd, +Y left", GREY_BG, PURPLE),
]
for bx, (lab, bg, col) in zip(bxs, labels):
    s.box(bx, by, bw, bh, lab, F, size=14, color=col, backgroundColor=bg, strokeWidth=2)

# Arrows between boxes
arrows = [
    (bxs[0] + bw, bxs[1], "R_{C←B}\nmount^T"),
    (bxs[1] + bw, bxs[2], "R_{A←C}\nvio.rot"),
    (bxs[2] + bw, bxs[3], "R_{W←A}\nY→Z up"),
]
for x0, x1, lab in arrows:
    ym = by + bh / 2
    s.arrow((x0 + 4, ym), (x1 - 4, ym), F, color=INK, width=2)
    s.text((x0 + x1) / 2, by - 36, lab, F, size=12, color=INK, align="center")

# Subscript cancellation callout
s.rect(30, oy + 245, 760, 115, F, backgroundColor=BLUE_BG, strokeColor=BLUE, strokeWidth=1)
s.text(50, oy + 258, "The Subscript Cancellation Rule (apply right-to-left to a vector v^B):", F, size=16, color=BLUE)
s.mono(50, oy + 292, "v^W  =  R_{W←A}  ·  R_{A←C}  ·  R_{C←B}  ·  v^B", F, size=18, color=INK)
s.text(50, oy + 325, "             W ← A      A ← C      C ← B     (B)      ⇒  A cancels, C cancels, B cancels → leaves W!", F, size=14, color=PURPLE)

# Concrete 3x3 matrices for Terra
s.rect(30, oy + 382, 368, 240, F, backgroundColor=GREY_BG, strokeColor=INK, strokeWidth=1)
s.text(48, oy + 396, "mount.camera_to_body  (R_{B←C})", F, size=16, color=GREEN)
s.mono(48, oy + 428, "          camX  camY  camZ\n"
                     "R_{B←C} = [  0     0    -1 ]  ← body +X\n"
                     "          [ -1     0     0 ]  ← body +Y\n"
                     "          [  0     1     0 ]  ← body +Z", F, size=14, color=INK)
s.text(48, oy + 530, "Check columns:\n• cam +X (right) → body −Y (right)\n• cam +Y (up)    → body +Z (up)\n• cam −Z (lens)  → body +X (fwd) ✓", F, size=14, color=INK)

s.rect(418, oy + 382, 372, 240, F, backgroundColor=GREY_BG, strokeColor=INK, strokeWidth=1)
s.text(436, oy + 396, "ARKit World → Terra World  (R_{W←A})", F, size=16, color=PURPLE)
s.mono(436, oy + 428, "          arkX  arkY  arkZ\n"
                      "R_{W←A} = [  0     0    -1 ]  x_W = -z_A\n"
                      "          [ -1     0     0 ]  y_W = -x_A\n"
                      "          [  0     1     0 ]  z_W = +y_A", F, size=14, color=INK)
s.text(436, oy + 530, "Check det = +1 (right-handed):\n• ARKit +Y (up)     → Terra +Z (up)\n• ARKit −Z (fwd)    → Terra +X (fwd)\n• ARKit −X (left)   → Terra +Y (left) ✓", F, size=14, color=INK)

s.text(30, oy + H - 40, "In code, you don't even have to multiply matrices — just call .apply() three times in order!", F, size=14, color=GREY)

# ═══════════════════════════════════════════════════════════════════════════
# Panel 4: Worked Example — Vio::update (+30 deg turn in place)
# ═══════════════════════════════════════════════════════════════════════════
F = s.frame(ox, oy, W, H, "4 · Worked example: Vio::update")
s.text(ox + 30, oy + 22, "4 · Worked example: turning +30° left in place", F, size=26)
s.text(ox + 30, oy + 58, "Camera sits 0.25 m ahead of the pivot. Why subtracting R·offset keeps body at (0, 0).", F, size=16, color=GREY)

# Top-down diagram of in-place turn
piv = (ox + 150, oy + 305)
r_px = 135
# Offset circle r = 0.25 m
s.ellipse(piv[0] - r_px, piv[1] - r_px, 2 * r_px, 2 * r_px, F, strokeColor=GREY, strokeWidth=1, strokeStyle="dashed")
s.axis(piv, (1, 0), "+X_W", RED, F, length=175, off=(6, -10))
s.axis(piv, (0, -1), "+Y_W", GREEN, F, length=165, off=(-16, -28))

# Rover rotated +30 deg CCW
ang = math.radians(30)
ca, sa = math.cos(ang), math.sin(ang)
s.rect(piv[0] - 45, piv[1] - 28, 90, 56, F, backgroundColor=GREY_BG, strokeColor=INK, angle=-ang)
# Body origin dot
s.ellipse(piv[0] - 5, piv[1] - 5, 10, 10, F, backgroundColor=GREEN, strokeColor=GREEN, roughness=0)
s.text(piv[0] - 92, piv[1] + 14, "body B\n(0, 0)", F, size=14, color=GREEN)

# Camera position on the circle at 30 deg
cam = (piv[0] + r_px * ca, piv[1] - r_px * sa)
s.ellipse(cam[0] - 6, cam[1] - 6, 12, 12, F, backgroundColor=BLUE, strokeColor=BLUE, roughness=0)
s.text(cam[0] - 10, cam[1] - 48, "cam C\n(0.217, 0.125)", F, size=14, color=BLUE)

# Vector R_{W←B} · offset from B to C
s.arrow(piv, cam, F, color=ORANGE, width=3)
s.text(piv[0] + 10, piv[1] - 74, "R · offset", F, size=14, color=ORANGE)
# Arc showing +30 deg yaw
s.arc(piv[0], piv[1], 70, 0, 30, F, PURPLE, width=2)
s.text(piv[0] + 76, piv[1] - 32, "+30°", F, size=15, color=PURPLE)

# Step-by-step math cards on right
s.rect(ox + 385, oy + 115, 405, 165, F, backgroundColor=GREEN_BG, strokeColor=GREEN, strokeWidth=1)
s.text(ox + 402, oy + 126, "Step A · Get heading (yaw)", F, size=17, color=GREEN)
s.mono(ox + 402, oy + 156, "fwd_B = (1, 0, 0)\n"
                           "fwd_W = R_{W←A} · vio.rot · mount^T · fwd_B\n"
                           "      = (0.866, 0.500, 0)\n\n"
                           "yaw   = atan2(0.500, 0.866) = +30° ✓", F, size=14, color=INK)

s.rect(ox + 385, oy + 298, 405, 195, F, backgroundColor=BLUE_BG, strokeColor=BLUE, strokeWidth=1)
s.text(ox + 402, oy + 309, "Step B · Get body position (x, y)", F, size=17, color=BLUE)
s.mono(ox + 402, oy + 339, "cam_W    = R_{W←A} · vio.position\n"
                           "         = (0.217, 0.125, 0.230)\n"
                           "offset_W = R_{W←B} · (0.25, 0, 0.23)\n"
                           "         = (0.217, 0.125, 0.230)\n\n"
                           "body_W   = cam_W − offset_W\n"
                           "         = (0.000, 0.000, 0.000) ✓", F, size=14, color=INK)

s.rect(ox + 30, oy + 512, 760, 118, F, backgroundColor=YELLOW_BG, strokeColor=ORANGE, strokeWidth=1)
s.text(ox + 48, oy + 524, "Step C · Get body twist (v, w) from consecutive frames (dt = t − t_prev)", F, size=16, color=ORANGE)
s.mono(ox + 48, oy + 554, "v = (dx·cos(yaw) + dy·sin(yaw)) / dt    // project world velocity onto body +X\n"
                          "w = wrap_angle(yaw − yaw_prev) / dt     // shortest angular diff in (−π, π]", F, size=14, color=INK)
s.text(ox + 48, oy + 602, "If !vio.tracked → return None and clear prev (never invent a pose when tracking is lost).", F, size=14, color=RED)

out = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parent / "coordinate-transforms.excalidraw")
s.save(out)
