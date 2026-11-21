"""Generate docs/diagrams/all-frames.svg: one unified academic-style 3D diagram of all 4 Terra frames."""
import math
from pathlib import Path

W_SVG, H_SVG = 1120, 680

# 3D right-handed Terra world (X forward, Y left, Z up) -> 2D screen projection.
# Chosen so +X points down-right, +Y points up-right (90 deg CCW from +X), +Z points straight up.
AZ = math.radians(38)
EL = math.radians(24)
SCALE = 155.0
OX, OY = 315.0, 425.0


def proj(p):
    x, y, z = p
    # Right-handed (x, y, z) -> iso-style screen coords:
    # +X goes down-right, +Y goes up-right (CCW from +X when viewed from +Z), +Z goes up.
    sx = OX + (x * math.cos(AZ) + y * math.sin(AZ)) * SCALE
    sy = OY + (x * math.sin(AZ) * math.sin(EL) - y * math.cos(AZ) * math.sin(EL) - z * math.cos(EL)) * SCALE
    return (sx, sy)


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def mul(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def rot_z(p, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (p[0] * c - p[1] * s, p[0] * s + p[1] * c, p[2])


def pts_str(pts):
    return " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)


def fmt_label(main, sub_txt="", sup_txt="", color="#111", x=0, y=0, size=18, anchor="middle", weight="normal", italic=True):
    style = "italic" if italic else "normal"
    s = f'<text x="{x:.1f}" y="{y:.1f}" fill="{color}" font-family="Georgia, \'Times New Roman\', serif" font-size="{size}" font-style="{style}" font-weight="{weight}" text-anchor="{anchor}">'
    s += main
    if sub_txt:
        s += f'<tspan dy="4" font-size="{int(size*0.72)}" font-style="italic">{sub_txt}</tspan>'
    if sup_txt:
        s += f'<tspan dy="-8" font-size="{int(size*0.72)}" font-style="normal">{sup_txt}</tspan>'
    s += "</text>"
    return s


# Colors (academic palette matching the reference)
COL_W = "#1d4ed8"   # Terra World W (blue)
COL_A = "#7c3aed"   # ARKit World A (purple)
COL_B = "#b91c1c"   # Rover Body B (crimson)
COL_C = "#15803d"   # Camera C (forest green)
COL_OFF = "#c2410c" # Offset / connecting vector (burnt orange)
COL_INK = "#111827"
COL_MUTED = "#4b5563"

elems = []

# Defs: crisp arrowheads per color
markers = []
for name, col in [
    ("w", COL_W),
    ("a", COL_A),
    ("b", COL_B),
    ("c", COL_C),
    ("off", COL_OFF),
    ("ink", COL_INK),
    ("muted", COL_MUTED),
]:
    markers.append(
        f'<marker id="arr-{name}" viewBox="0 0 10 10" refX="8.5" refY="5" '
        f'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
        f'<path d="M 0 1.2 L 9.5 5 L 0 8.8 z" fill="{col}"/></marker>'
    )


def arrow2d(p0, p1, col, mname, width=2.2, dash=""):
    d_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<line x1="{p0[0]:.1f}" y1="{p0[1]:.1f}" x2="{p1[0]:.1f}" y2="{p1[1]:.1f}" '
        f'stroke="{col}" stroke-width="{width}"{d_attr} marker-end="url(#arr-{mname})"/>'
    )


def line2d(p0, p1, col, width=1.4, dash=""):
    d_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<line x1="{p0[0]:.1f}" y1="{p0[1]:.1f}" x2="{p1[0]:.1f}" y2="{p1[1]:.1f}" '
        f'stroke="{col}" stroke-width="{width}"{d_attr}/>'
    )


def poly3d(pts3, fill="#f3f4f6", stroke="#111827", width=1.4, opacity=1.0):
    pts2 = [proj(p) for p in pts3]
    return (
        f'<polygon points="{pts_str(pts2)}" fill="{fill}" fill-opacity="{opacity}" '
        f'stroke="{stroke}" stroke-width="{width}" stroke-linejoin="round"/>'
    )


def arc3d(center, r, a0, a1, z_plane, col, mname, width=1.8, dash="", n=24):
    pts = []
    for i in range(n + 1):
        a = a0 + (a1 - a0) * i / n
        pts.append(proj((center[0] + r * math.cos(a), center[1] + r * math.sin(a), z_plane)))
    d = "M " + " L ".join(f"{x:.1f} {y:.1f}" for x, y in pts)
    d_attr = f' stroke-dasharray="{dash}"' if dash else ""
    mark = f' marker-end="url(#arr-{mname})"' if mname else ""
    return f'<path d="{d}" fill="none" stroke="{col}" stroke-width="{width}"{d_attr}{mark}/>'


# ── 0. Subtle ground reference grid in Terra World W (z = 0) ────────────────
for gx in [-0.4, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5]:
    elems.append(line2d(proj((gx, -0.6, 0)), proj((gx, 1.4, 0)), "#e5e7eb", 1.0))
for gy in [-0.5, 0.0, 0.5, 1.0]:
    elems.append(line2d(proj((-0.4, gy, 0)), proj((2.5, gy, 0)), "#e5e7eb", 1.0))

# ── 1. Frame A (ARKit World, Y-up) — shown at left (-1.10, 0.20, 0) ─────────
# Placed right beside Frame W with a clear curved transform arrow R_{W<-A}
# so both world origins and their axis correspondence are unmistakably readable.
A_O = (-1.10, 0.20, 0.0)
L_A = 0.72
pA = proj(A_O)
# ARKit axes drawn in the SAME physical orientation as Terra W:
# +Y_A = up (+Z_W), -Z_A = forward (+X_W), +X_A = right (-Y_W)
pA_y = proj(add(A_O, (0, 0, L_A)))         # +Y_A (up)
pA_nz = proj(add(A_O, (L_A, 0, 0)))        # -Z_A (forward)
pA_z = proj(add(A_O, (-L_A * 0.55, 0, 0))) # +Z_A (back, dashed)
pA_x = proj(add(A_O, (0, -L_A, 0)))        # +X_A (right)

elems.append(f'<circle cx="{pA[0]:.1f}" cy="{pA[1]:.1f}" r="3.8" fill="{COL_A}"/>')
elems.append(arrow2d(pA, pA_y, COL_A, "a", 2.3))
elems.append(arrow2d(pA, pA_nz, COL_A, "a", 2.3))
elems.append(arrow2d(pA, pA_x, COL_A, "a", 2.3))
elems.append(arrow2d(pA, pA_z, COL_A, "a", 1.4, dash="4,3"))

elems.append(fmt_label("Y", "A", "", COL_A, pA_y[0] - 4, pA_y[1] - 12, 18))
elems.append(fmt_label("−Z", "A", "", COL_A, pA_nz[0] - 8, pA_nz[1] - 14, 17))
elems.append(fmt_label("X", "A", "", COL_A, pA_x[0] - 20, pA_x[1] + 16, 18))
elems.append(fmt_label("+Z", "A", "", COL_A, pA_z[0] - 18, pA_z[1] - 6, 14))
elems.append(fmt_label("A", "", "", COL_A, pA[0] - 20, pA[1] - 10, 19, weight="bold"))
elems.append(
    f'<text x="{pA[0] + 6:.1f}" y="{pA[1] + 76:.1f}" fill="{COL_A}" font-family="Georgia, serif" '
    f'font-size="13" font-style="normal" text-anchor="middle">ARKit World (Y-up)</text>'
)

# ── 2. Frame W (Terra World, Z-up) at (0, 0, 0) ─────────────────────────────
W_O = (0.0, 0.0, 0.0)
L_W = 0.72
pW = proj(W_O)
pW_x = proj((L_W, 0, 0))
pW_y = proj((0, L_W, 0))
pW_z = proj((0, 0, L_W))

elems.append(f'<circle cx="{pW[0]:.1f}" cy="{pW[1]:.1f}" r="4.0" fill="{COL_W}"/>')
elems.append(arrow2d(pW, pW_x, COL_W, "w", 2.5))
elems.append(arrow2d(pW, pW_y, COL_W, "w", 2.5))
elems.append(arrow2d(pW, pW_z, COL_W, "w", 2.5))

elems.append(fmt_label("X", "W", "", COL_W, pW_x[0] - 4, pW_x[1] + 22, 18))
elems.append(fmt_label("Y", "W", "", COL_W, pW_y[0] + 16, pW_y[1] - 8, 18))
elems.append(fmt_label("Z", "W", "", COL_W, pW_z[0], pW_z[1] - 12, 18))
elems.append(fmt_label("W", "", "", COL_W, pW[0] - 20, pW[1] + 18, 19, weight="bold"))
elems.append(
    f'<text x="{pW[0] - 8:.1f}" y="{pW[1] + 44:.1f}" fill="{COL_W}" font-family="Georgia, serif" '
    f'font-size="13" font-style="normal" text-anchor="middle">Terra World (Z-up)</text>'
)

# Curved transform arrow between A and W: R_{W<-A}
a_top = proj(add(A_O, (0.14, 0.0, 0.76)))
w_top = proj(add(W_O, (-0.14, 0.0, 0.76)))
mid_aw = ((a_top[0] + w_top[0]) / 2, min(a_top[1], w_top[1]) - 34)
elems.append(
    f'<path d="M {a_top[0]:.1f} {a_top[1]:.1f} Q {mid_aw[0]:.1f} {mid_aw[1]:.1f} {w_top[0]:.1f} {w_top[1]:.1f}" '
    f'fill="none" stroke="{COL_INK}" stroke-width="1.6" stroke-dasharray="4,3" marker-end="url(#arr-ink)"/>'
)
elems.append(fmt_label("R", "W←A", "", COL_INK, mid_aw[0], mid_aw[1] + 2, 15))

# ── 3. Rover Body Frame B at (1.75, 0.35, 0.0), rotated by yaw = +28 deg ────
B_O = (1.75, 0.35, 0.0)
YAW = math.radians(28)


def b_to_w(p):
    return add(B_O, rot_z(p, YAW))


# Rover chassis box (0.60m x 0.44m, raised slightly above ground)
# Scale rover slightly up for crisp readability (length 0.90, width 0.64, deck z=0.14..0.24)
RL, RW = 0.45, 0.30
Z0, Z1 = 0.10, 0.22
corners_bot = [b_to_w((sx * RL, sy * RW, Z0)) for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]
corners_top = [b_to_w((sx * RL, sy * RW, Z1)) for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]

# Subtle shadow / footprint on ground plane z=0
foot = [b_to_w((sx * RL, sy * RW, 0.0)) for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]
elems.append(poly3d(foot, fill="#e5e7eb", stroke="#9ca3af", width=1.0, opacity=0.45))

# Visible side faces of rover body + top face
elems.append(poly3d([corners_bot[0], corners_bot[1], corners_top[1], corners_top[0]], fill="#e5e7eb", stroke="#374151", width=1.3))
elems.append(poly3d([corners_bot[1], corners_bot[2], corners_top[2], corners_top[1]], fill="#d1d5db", stroke="#374151", width=1.3))
elems.append(poly3d(corners_top, fill="#f3f4f6", stroke="#111827", width=1.5, opacity=0.92))

# 4 wheel blocks (subtle dark accents on sides)
for wx in (-0.28, 0.28):
    for wy in (-RW - 0.04, RW + 0.04):
        wc = proj(b_to_w((wx, wy, 0.11)))
        elems.append(f'<ellipse cx="{wc[0]:.1f}" cy="{wc[1]:.1f}" rx="11" ry="8" fill="#374151" fill-opacity="0.85" stroke="#111827" stroke-width="1.2"/>')

# Phone mast + landscape iPhone at front of rover: offset = (0.42, 0, 0.76) visually representing (0.25, 0, 0.23) m
C_LOCAL = (0.42, 0.0, 0.76)
C_O = b_to_w(C_LOCAL)
mast_bot = proj(b_to_w((C_LOCAL[0], 0.0, Z1)))
mast_top = proj(b_to_w((C_LOCAL[0], 0.0, C_LOCAL[2] - 0.12)))
elems.append(line2d(mast_bot, mast_top, "#374151", 4.0))

# Landscape iPhone slab centred at C_LOCAL (screen facing -X_B, rear lens facing +X_B)
ph_pts = [
    b_to_w((C_LOCAL[0], -0.18, C_LOCAL[2] - 0.12)),
    b_to_w((C_LOCAL[0],  0.18, C_LOCAL[2] - 0.12)),
    b_to_w((C_LOCAL[0],  0.18, C_LOCAL[2] + 0.12)),
    b_to_w((C_LOCAL[0], -0.18, C_LOCAL[2] + 0.12)),
]
elems.append(poly3d(ph_pts, fill="#dbeafe", stroke="#1e40af", width=1.5, opacity=0.92))
elems.append(
    f'<text x="{proj(b_to_w((C_LOCAL[0], 0.20, C_LOCAL[2] + 0.17)))[0] + 6:.1f}" '
    f'y="{proj(b_to_w((C_LOCAL[0], 0.20, C_LOCAL[2] + 0.17)))[1]:.1f}" '
    f'fill="#1e40af" font-family="Georgia, serif" font-size="12" text-anchor="start">iPhone (rear lens →)</text>'
)

# ── 4. Connecting vectors (the closed triangle of Vio::update) ──────────────
pB = proj(B_O)
pC = proj(C_O)

# Ground translation vector p_B^W from W to B
elems.append(arrow2d(pW, pB, COL_W, "w", 1.8, dash="6,4"))
mid_wb = (pW[0] * 0.38 + pB[0] * 0.62, pW[1] * 0.38 + pB[1] * 0.62)
elems.append(fmt_label("p", "B", "W", COL_W, mid_wb[0] - 8, mid_wb[1] + 24, 16))

# ARKit measured camera vector p_C^W (from W to C)
elems.append(arrow2d(pW, pC, COL_A, "a", 1.6, dash="5,4"))
mid_wc = (pW[0] * 0.46 + pC[0] * 0.54, pW[1] * 0.46 + pC[1] * 0.54)
elems.append(fmt_label("p", "C", "W", COL_A, mid_wc[0] + 14, mid_wc[1] - 34, 16))
elems.append(
    f'<text x="{mid_wc[0] + 14:.1f}" y="{mid_wc[1] - 17:.1f}" fill="{COL_A}" font-family="Georgia, serif" '
    f'font-size="12" font-style="italic" text-anchor="middle">(= R_W←A · vio.pos)</text>'
)

# Offset vector R_{W<-B} * d_{BC} from B to C
elems.append(arrow2d(pB, pC, COL_OFF, "off", 2.0, dash="4,3"))
mid_bc = ((pB[0] + pC[0]) / 2, (pB[1] + pC[1]) / 2)
elems.append(
    f'<text x="{mid_bc[0] + 48:.1f}" y="{mid_bc[1] + 12:.1f}" fill="{COL_OFF}" font-family="Georgia, serif" '
    f'font-size="14" font-style="italic" text-anchor="start">R<tspan dy="3" font-size="10">W←B</tspan><tspan dy="-3"> · d</tspan><tspan dy="3" font-size="10">BC</tspan></text>'
)

# ── 5. Body Frame B triad at B_O (crimson) ──────────────────────────────────
L_B = 0.78
pB_x = proj(b_to_w((L_B, 0, 0)))
pB_y = proj(b_to_w((0, L_B, 0)))
pB_z = proj(b_to_w((0, 0, L_B)))

# Reference dashed line along +X_W from B_O + yaw arc psi
pB_xref = proj(add(B_O, (0.68, 0, 0)))
elems.append(line2d(pB, pB_xref, COL_MUTED, 1.2, dash="4,3"))
elems.append(arc3d(B_O, 0.48, 0.0, YAW, 0.0, COL_B, "b", 1.7))
p_psi = proj(add(B_O, (0.62 * math.cos(YAW * 0.45), 0.62 * math.sin(YAW * 0.45), 0.0)))
elems.append(fmt_label("ψ (yaw)", "", "", COL_B, p_psi[0] + 14, p_psi[1] + 24, 14))

elems.append(f'<circle cx="{pB[0]:.1f}" cy="{pB[1]:.1f}" r="4.0" fill="{COL_B}"/>')
elems.append(arrow2d(pB, pB_x, COL_B, "b", 2.5))
elems.append(arrow2d(pB, pB_y, COL_B, "b", 2.5))
elems.append(arrow2d(pB, pB_z, COL_B, "b", 2.5))

elems.append(fmt_label("X", "B", "", COL_B, pB_x[0] + 20, pB_x[1] + 6, 18))
elems.append(fmt_label("Y", "B", "", COL_B, pB_y[0] - 18, pB_y[1] - 8, 18))
elems.append(fmt_label("Z", "B", "", COL_B, pB_z[0] - 10, pB_z[1] - 12, 18))
elems.append(fmt_label("B", "", "", COL_B, pB[0] - 20, pB[1] + 18, 19, weight="bold"))
elems.append(
    f'<text x="{pB[0] - 20:.1f}" y="{pB[1] + 40:.1f}" fill="{COL_B}" font-family="Georgia, serif" '
    f'font-size="13" text-anchor="middle">Rover Body (FLU)</text>'
)

# ── 6. Camera Frame C triad at C_O (forest green) ───────────────────────────
L_C = 0.62
# Camera axes in body frame: -Z_C = +X_B, +X_C = -Y_B, +Y_C = +Z_B
pC_nz = proj(b_to_w(add(C_LOCAL, (L_C, 0, 0))))     # -Z_C (lens forward = +X_B)
pC_x = proj(b_to_w(add(C_LOCAL, (0, -L_C, 0))))     # +X_C (camera right = -Y_B)
pC_y = proj(b_to_w(add(C_LOCAL, (0, 0, L_C))))      # +Y_C (camera up = +Z_B)
pC_z = proj(b_to_w(add(C_LOCAL, (-L_C * 0.45, 0, 0))))  # +Z_C (into phone, dashed)

elems.append(f'<circle cx="{pC[0]:.1f}" cy="{pC[1]:.1f}" r="4.0" fill="{COL_C}"/>')
elems.append(arrow2d(pC, pC_nz, COL_C, "c", 2.5))
elems.append(arrow2d(pC, pC_x, COL_C, "c", 2.5))
elems.append(arrow2d(pC, pC_y, COL_C, "c", 2.5))
elems.append(arrow2d(pC, pC_z, COL_C, "c", 1.4, dash="4,3"))

elems.append(fmt_label("−Z", "C", "", COL_C, pC_nz[0] + 24, pC_nz[1] + 6, 18))
elems.append(fmt_label("X", "C", "", COL_C, pC_x[0] + 16, pC_x[1] + 14, 18))
elems.append(fmt_label("Y", "C", "", COL_C, pC_y[0] + 4, pC_y[1] - 12, 18))
elems.append(fmt_label("+Z", "C", "", COL_C, pC_z[0] + 8, pC_z[1] - 12, 13))
elems.append(fmt_label("C", "", "", COL_C, pC[0] + 16, pC[1] - 10, 18, weight="bold"))

# ── 7. Compact summary box (top-right) ──────────────────────────────────────
bx, by, bw, bh = 748, 34, 340, 215
elems.append(
    f'<rect x="{bx}" y="{by}" width="{bw}" height="{bh}" rx="6" fill="#fafafa" stroke="#d1d5db" stroke-width="1.2"/>'
)
summary_lines = [
    ("Frame Summary", COL_INK, 15, "bold", "normal"),
    ("A  ARKit World:  +Y up, −Z fwd, +X right", COL_A, 13, "normal", "normal"),
    ("W  Terra World:  +Z up, +X fwd, +Y left", COL_W, 13, "normal", "normal"),
    ("B  Rover Body:   at pivot, yaw ψ CCW (+left)", COL_B, 13, "normal", "normal"),
    ("C  Camera:       at lens, offset d_BC = [0.25, 0, 0.23] m", COL_C, 13, "normal", "normal"),
    ("", COL_INK, 6, "normal", "normal"),
    ("Transform Chain (Vio::update)", COL_INK, 14, "bold", "normal"),
    ("R_W←B  =  R_W←A · R_A←C · (R_B←C)ᵀ", COL_INK, 13, "normal", "italic"),
    ("p_B^W  =  R_W←A · p_C^A  −  R_W←B · d_BC", COL_OFF, 13, "normal", "italic"),
]
cy_t = by + 24
for txt, col, sz, wt, st in summary_lines:
    if txt:
        elems.append(
            f'<text x="{bx + 14}" y="{cy_t}" fill="{col}" font-family="Georgia, \'Times New Roman\', serif" '
            f'font-size="{sz}" font-weight="{wt}" font-style="{st}">{txt}</text>'
        )
    cy_t += sz + 9

# ── 8. Academic Fig. 1 caption at bottom ────────────────────────────────────
elems.append(line2d((40, 565), (W_SVG - 40, 565), "#e5e7eb", 1.0))
caption = [
    'Fig. 1.  Terra VIO System Coordinate Definition.  All four frames shown in a single 3D view: ARKit World A (Y-up, purple),',
    'Terra World W (Z-up, blue, related by R_W←A: (x,y,z)_A → (−z,−x,y)_W), Rover Body B (FLU, crimson, translated by p_B^W and',
    'rotated by yaw ψ CCW about +Z_W), and Camera C (green, mounted at offset d_BC = [0.25, 0, 0.23]ᵀ m with lens −Z_C along +X_B).',
    'The dashed triangle shows why Vio::update recovers the body origin via  p_B^W = R_W←A · p_C^A − R_W←B · d_BC.',
]
for i, line in enumerate(caption):
    elems.append(
        f'<text x="44" y="{590 + i * 21}" fill="{COL_INK}" font-family="Georgia, \'Times New Roman\', serif" '
        f'font-size="14.5">{line}</text>'
    )

svg = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W_SVG} {H_SVG}" width="{W_SVG}" height="{H_SVG}">
  <defs>
    {"".join(markers)}
  </defs>
  <rect width="{W_SVG}" height="{H_SVG}" fill="#ffffff"/>
  {"".join(elems)}
</svg>
'''

out_svg = Path(__file__).resolve().parent / "all-frames.svg"
out_svg.write_text(svg)
print("wrote", out_svg)
