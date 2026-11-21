import { readFileSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import * as k from "/Users/yojan/.agents/skills/anatomy/kit/iso-kit.mjs";
import * as G from "/Users/yojan/.agents/skills/anatomy/kit/lathe.mjs";
import { kitScript } from "/Users/yojan/.agents/skills/anatomy/scripts/inline-kit.mjs";

const HERE = dirname(fileURLToPath(import.meta.url));
const KIT_DIR = "/Users/yojan/.agents/skills/anatomy/kit";
const theme = process.argv.includes("--light") ? "light" : "dark";

const WIDTH = 660;
const HEIGHT = 430;

// ------------------------------------------------------------------ world layout
// Terra right-handed world (wx forward, wy left, wz up) maps to iso-kit [wx, -wy, wz].
// That makes +X_w point down-right, +Y_w point up-right (90 deg CCW from +X_w),
// and +Z_w point straight up.

const PLATE = { x: -148, y: -88, w: 296, d: 176, r: 8 };
const PLATE_H = 14;
const FOOT_H = 6;

const TABLE = [0, 6]; // rotary table centre [x, y] on surface plate
const TABLE_HOUSING_R = 74;
const TABLE_R = 64;
const TABLE_BASE_Z = 0;
const TABLE_HOUSING_TOP = 9;
const TABLE_TOP = 15;

// Real Terra parameters (VehicleSetup.swift / terra-av.toml)
const OFFSET_M = 0.25; // camera forward offset (m)
const HEIGHT_M = 0.23; // camera height above ground (m)
const OFFSET_U = 30; // world units on the platter (matches calibration ring)

const WORLD_O = [-108, -48]; // fixed World (W / A) reference pillar
const DIAL_O = [110, -48]; // fixed yaw dial gauge
const DIAL_R = 20;
const DIAL_Z = 44;
const DIAL_H = 9;
const DIAL_TOP = DIAL_Z + DIAL_H;

const VERNIER_O = [0, TABLE[1] + TABLE_HOUSING_R]; // fixed front-left vernier index
const GEAR_O = [74, 48]; // fixed front-right worm-drive gearbox & handwheel

const P = k.fitProjection(
  [
    ...k.boxCorners(PLATE, -PLATE_H - FOOT_H, 0),
    [WORLD_O[0] - 18, WORLD_O[1] - 32, 74],
    [DIAL_O[0] + DIAL_R + 8, DIAL_O[1] - DIAL_R, DIAL_TOP + 4],
    [TABLE[0], TABLE[1], 88],
    [GEAR_O[0] + 24, GEAR_O[1] + 18, 22],
  ],
  WIDTH,
  HEIGHT,
  { pad: 26, azimuth: 42, elevation: 30 },
);

const at = (point) => k.iso(point, P);
const S = (paths, style) => k.solidSvg(paths, style);
const L = (d, style) => k.lineSvg(d, style);
const D = (points, style) => k.dotsSvg(points, style);
const byDepth = (points) =>
  [...points].sort((a, b) => k.depthOf([a[0], a[1], 0], P) - k.depthOf([b[0], b[1], 0], P));

// ------------------------------------------------------------------ shared triad & rover renderer
// Serialised into the page so build-time SVG and live frames use identical geometry.

function latheScript() {
  const src = readFileSync(join(KIT_DIR, "lathe.mjs"), "utf8")
    .replace(/^import .*$/gm, "")
    .replace(/^export /gm, "");
  return `const _lathe = (() => {\n${src}\nreturn { frameAlong, solidOf, sphereOf, arcOf };\n})();\n`;
}

function makeDynamicBuilder(kit, lathe, proj) {
  const {
    iso,
    depthOf,
    roundedPlan,
    extrude,
    cylinder,
    ring,
    radialTicks,
    lineOnTop,
    solidSvg,
    lineSvg,
    dotsSvg,
    pathOf,
  } = kit;
  const { frameAlong, solidOf, sphereOf, arcOf } = lathe;

  const isoAt = (p) => iso(p, proj);
  const depth = (p) => depthOf(p, proj);

  // Convert rover body-frame (bx forward, by left, bz up from platter top)
  // into iso-kit world coordinates at heading `yaw` (rad, + = CCW / left).
  const toWorld = (bx, by, bz, yaw, cx, cy, z0) => {
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    const wx = bx * c - by * s;
    const wy = bx * s + by * c;
    return [cx + wx, cy - wy, z0 + bz];
  };

  // Convert rover body-frame direction vector (dx, dy, dz) into iso-kit unit vector.
  const dirWorld = (dx, dy, dz, yaw) => {
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    const wx = dx * c - dy * s;
    const wy = dx * s + dy * c;
    return [wx, -wy, dz];
  };

  // Rotate a local body-frame rounded rectangle plan into iso-kit XY at `yaw`.
  const bodyPlan = (rect, yaw, cx, cy, steps = 5) => {
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    return roundedPlan(rect, steps).map(([bx, by]) => [
      cx + (bx * c - by * s),
      cy - (bx * s + by * c),
    ]);
  };

  // Machined turned axis rod: separate slender shaft + distinct conical tip
  // so lathe's hull never bridges the step between shaft and cone.
  const axisRodItems = (origin, dir, length, rRod, rHub, style, keyBias = 0) => {
    const F = frameAlong(origin, dir);
    const tip = 6.0;
    const collar = rHub + 1.6;
    const shaftEnd = length - tip;
    const shaft = solidSvg(
      solidOf(
        [
          [rHub * 0.78, rRod],
          [shaftEnd, rRod],
        ],
        F,
        proj,
        { steps: 18, rims: "all" },
      ),
      style,
    );
    const col = solidSvg(
      solidOf(
        [
          [rHub * 0.72, rRod * 1.5],
          [collar, rRod * 1.5],
        ],
        F,
        proj,
        { steps: 18, rims: "all" },
      ),
      { tone: "mid" },
    );
    const head = solidSvg(
      solidOf(
        [
          [shaftEnd, rRod * 2.25],
          [length, 0],
        ],
        F,
        proj,
        { steps: 20, rims: "all" },
      ),
      style,
    );
    const pCol = [
      origin[0] + dir[0] * ((rHub * 0.72 + collar) * 0.5),
      origin[1] + dir[1] * ((rHub * 0.72 + collar) * 0.5),
      origin[2] + dir[2] * ((rHub * 0.72 + collar) * 0.5),
    ];
    const pShaft = [
      origin[0] + dir[0] * ((collar + shaftEnd) * 0.5),
      origin[1] + dir[1] * ((collar + shaftEnd) * 0.5),
      origin[2] + dir[2] * ((collar + shaftEnd) * 0.5),
    ];
    const pHead = [
      origin[0] + dir[0] * (shaftEnd + tip * 0.45),
      origin[1] + dir[1] * (shaftEnd + tip * 0.45),
      origin[2] + dir[2] * (shaftEnd + tip * 0.45),
    ];
    return [
      { k: depth(pCol) + keyBias, svg: col },
      { k: depth(pShaft) + keyBias, svg: shaft },
      { k: depth(pHead) + keyBias, svg: head },
    ];
  };

  // Draw a complete 3-axis machined triad (hub sphere + 3 orthogonal rods), depth-sorted.
  const triadItems = (origin, axes, rHub, rRod, keyBias = 0) => {
    const items = [];
    items.push({
      k: depth(origin) + keyBias,
      svg: solidSvg(sphereOf(origin, rHub, proj, { steps: 28 }), { tone: "mid", crease: "none" }),
    });
    for (const ax of axes) {
      items.push(...axisRodItems(origin, ax.dir, ax.len, rRod, rHub, ax.style, keyBias));
    }
    return items;
  };

  // Wheel cylinder along the rover's left-right axle (+Y_b).
  const wheelItem = (bx, by, sideSign, yaw, cx, cy, z0) => {
    const r = 8.4;
    const w = 6.2;
    const axleDir = dirWorld(0, sideSign, 0, yaw);
    const inner = toWorld(bx, by - sideSign * (w * 0.5 + 3.2), r, yaw, cx, cy, z0);
    const F = frameAlong(inner, axleDir);
    // Axle stub + tyre body + recessed hub cap
    const tyreProfile = [
      [0, 2.2],
      [3.2, 2.2],
      [3.2, r - 0.8],
      [4.0, r],
      [3.2 + w - 0.8, r],
      [3.2 + w, r - 0.8],
      [3.2 + w, 4.2],
      [3.2 + w + 0.9, 4.2],
      [3.2 + w + 0.9, 0],
    ];
    const paths = solidOf(tyreProfile, F, proj, { steps: 28 });
    const treadA = arcOf(F, 3.2 + w * 0.35, r, proj, { steps: 20 });
    const treadB = arcOf(F, 3.2 + w * 0.65, r, proj, { steps: 20 });
    const centre = toWorld(bx, by, r, yaw, cx, cy, z0);
    return {
      k: depth(centre),
      svg: solidSvg(paths, {
        tone: "lo",
        inner: lineSvg(treadA + treadB, { tone: "faint" }),
      }),
    };
  };

  // Render the rotating turntable platter, degree ring, Terra rover, B triad, and C triad.
  function renderTurntable(yaw, table, tableBaseZ, tableTopZ, tableR, offsetU) {
    const [cx, cy] = table;
    const out = [];

    // 1. Turntable platter disc
    out.push(
      solidSvg(cylinder(cx, cy, tableR, tableBaseZ, tableTopZ - tableBaseZ, proj, 72, 0.7), {
        tone: "mid",
        crease: "lo",
      }),
    );

    // 2. Rotating degree ticks around platter rim (72 ticks = every 5 deg, major every 15 deg)
    const tickMinor = [];
    const tickMajor = [];
    for (let i = 0; i < 72; i++) {
      const a = yaw + (i * Math.PI * 2) / 72;
      const major = i % 3 === 0;
      const r0 = tableR - (major ? 4.8 : 2.6);
      const r1 = tableR;
      const p0 = [cx + Math.cos(a) * r0, cy - Math.sin(a) * r0];
      const p1 = [cx + Math.cos(a) * r1, cy - Math.sin(a) * r1];
      const seg = lineOnTop(p0, p1, tableTopZ, proj);
      if (major) tickMajor.push(seg);
      else tickMinor.push(seg);
    }
    out.push(lineSvg(tickMinor.join(""), { tone: "lo" }));
    out.push(lineSvg(tickMajor.join(""), { tone: "mid" }));

    // 3. Concentric engraved rings on platter: outer track ring + 0.25m camera offset ring
    out.push(lineSvg(ring(cx, cy, tableR - 6.8, tableTopZ, proj, 64), { tone: "faint" }));
    out.push(lineSvg(ring(cx, cy, offsetU, tableTopZ, proj, 56), { tone: "lo" }));

    // 4. Engraved crosshairs through pivot ending on the offset ring
    const chA = toWorld(-offsetU, 0, 0, yaw, cx, cy, tableTopZ);
    const chB = toWorld(offsetU, 0, 0, yaw, cx, cy, tableTopZ);
    const chC = toWorld(0, -offsetU, 0, yaw, cx, cy, tableTopZ);
    const chD = toWorld(0, offsetU, 0, yaw, cx, cy, tableTopZ);
    out.push(
      lineSvg(
        pathOf([isoAt(chA), isoAt(chB)]) + pathOf([isoAt(chC), isoAt(chD)]),
        { tone: "faint" },
      ),
    );

    // 5. Lit tick mark at 0 deg on the platter (forward reference on the ring)
    const zeroA = toWorld(tableR - 6.8, 0, 0, yaw, cx, cy, tableTopZ);
    const zeroB = toWorld(tableR, 0, 0, yaw, cx, cy, tableTopZ);
    out.push(lineSvg(pathOf([isoAt(zeroA), isoAt(zeroB)]), { tone: "lit" }));

    // 6. Rover + triads, depth-sorted in 3 layers (undercarriage/wheels -> deck/modules -> hubs/triads)
    const lowerItems = [];
    // 4 wheels (FL, FR, RL, RR)
    for (const bx of [-17, 17]) {
      for (const side of [-1, 1]) {
        lowerItems.push(wheelItem(bx, side * 23, side, yaw, cx, cy, tableTopZ));
      }
    }
    // Lower chassis tray
    const skidPlan = bodyPlan({ x: -27, y: -17, w: 54, d: 34, r: 3 }, yaw, cx, cy, 4);
    lowerItems.push({
      k: depth(toWorld(0, 0, 7, yaw, cx, cy, tableTopZ)),
      svg: solidSvg(extrude(skidPlan, tableTopZ + 4.5, 4.5, proj, 0.4), { tone: "lo" }),
    });
    // Front & rear bumper bars
    for (const bx of [-29, 27]) {
      const bPlan = bodyPlan({ x: bx, y: -15, w: 2.6, d: 30, r: 0.8 }, yaw, cx, cy, 2);
      lowerItems.push({
        k: depth(toWorld(bx + 1.3, 0, 8, yaw, cx, cy, tableTopZ)),
        svg: solidSvg(extrude(bPlan, tableTopZ + 6, 3.2, proj, 0.3), { tone: "mid" }),
      });
    }
    lowerItems.sort((a, b) => a.k - b.k);
    for (const item of lowerItems) out.push(item.svg);

    // Main anodized rover deck plate (sits above wheels/skid)
    const deckZ0 = tableTopZ + 9;
    const deckH = 5;
    const deckTop = deckZ0 + deckH;
    const deckPlan = bodyPlan({ x: -29, y: -19, w: 58, d: 38, r: 4 }, yaw, cx, cy, 5);
    out.push(solidSvg(extrude(deckPlan, deckZ0, deckH, proj, 0.6), { tone: "hi", crease: "lo" }));
    const deckGroove = bodyPlan({ x: -25.5, y: -15.5, w: 51, d: 31, r: 2.5 }, yaw, cx, cy, 4);
    out.push(
      lineSvg(pathOf(deckGroove.map(([x, y]) => isoAt([x, y, deckTop])), true), { tone: "faint" }),
    );
    const deckScrews = [
      [-24, -14],
      [-24, 14],
      [24, -14],
      [24, 14],
    ].map(([bx, by]) => isoAt(toWorld(bx, by, 0, yaw, cx, cy, deckTop)));
    out.push(dotsSvg(deckScrews, { size: 0.45, tone: "mid" }));

    // Upper rover assemblies (battery + Fusion HAT, Body triad B, Phone mast + iPhone + Camera triad C)
    const upper = [];

    // Rear electronics stack (battery pack + Fusion HAT board)
    const rearMid = toWorld(-16, 0, 6, yaw, cx, cy, deckTop);
    const battPlan = bodyPlan({ x: -24, y: -12, w: 16, d: 24, r: 2 }, yaw, cx, cy, 3);
    const hatPlan = bodyPlan({ x: -22.5, y: -10.5, w: 13, d: 21, r: 1.5 }, yaw, cx, cy, 3);
    const chipA = bodyPlan({ x: -19, y: -6, w: 5, d: 5, r: 0.6 }, yaw, cx, cy, 2);
    const chipB = bodyPlan({ x: -19, y: 2, w: 4, d: 6, r: 0.6 }, yaw, cx, cy, 2);
    const hatDots = [
      [-20.5, -8.5],
      [-20.5, 8.5],
      [-11.5, -8.5],
      [-11.5, 8.5],
    ].map(([bx, by]) => isoAt(toWorld(bx, by, 9.5, yaw, cx, cy, deckTop)));
    upper.push({
      k: depth(rearMid),
      svg:
        solidSvg(extrude(battPlan, deckTop, 6.5, proj, 0.5), { tone: "mid" }) +
        solidSvg(extrude(hatPlan, deckTop + 6.5, 3, proj, 0.4), { tone: "hi", crease: "lo" }) +
        solidSvg(extrude(chipA, deckTop + 9.5, 1.4, proj), { tone: "lo" }) +
        solidSvg(extrude(chipB, deckTop + 9.5, 1.4, proj), { tone: "lo" }) +
        dotsSvg(hatDots, { size: 0.4, tone: "mid" }),
    });

    // Body frame (B) pedestal + triad at pivot (bx = 0, by = 0)
    const bBase = toWorld(0, 0, 0, yaw, cx, cy, deckTop);
    const bHub = toWorld(0, 0, 9.5, yaw, cx, cy, deckTop);
    upper.push({
      k: depth(toWorld(0, 0, 2.5, yaw, cx, cy, deckTop)),
      svg:
        solidSvg(cylinder(bBase[0], bBase[1], 5.6, deckTop, 3.2, proj, 28, 0.4), { tone: "mid" }) +
        solidSvg(cylinder(bBase[0], bBase[1], 3.0, deckTop + 3.2, 3.8, proj, 20), {
          tone: "mid",
          crease: "none",
        }),
    });
    const bAxes = [
      { dir: dirWorld(1, 0, 0, yaw), len: 21, style: { tone: "hi", lit: true } }, // +X_b forward (lit, clears mast)
      { dir: dirWorld(0, 1, 0, yaw), len: 22, style: { tone: "hi" } }, // +Y_b left
      { dir: [0, 0, 1], len: 24, style: { tone: "hi" } }, // +Z_b up
    ];
    upper.push(...triadItems(bHub, bAxes, 3.8, 1.15, 0.2));

    // Phone mast + offset pointer pin at bx = offsetU (0.25m ahead of body centre)
    const mastX = offsetU - 4;
    const mastPos = toWorld(mastX, 0, 0, yaw, cx, cy, deckTop);
    const mastTopZ = deckTop + 17;
    upper.push({
      k: depth(toWorld(mastX, 0, 8, yaw, cx, cy, deckTop)),
      svg:
        solidSvg(cylinder(mastPos[0], mastPos[1], 4.8, deckTop, 3.0, proj, 24, 0.3), { tone: "mid" }) +
        solidSvg(cylinder(mastPos[0], mastPos[1], 2.2, deckTop + 3.0, 14, proj, 20), {
          tone: "mid",
          crease: "none",
        }),
    });

    // Phone cradle + landscape iPhone at bx = offsetU (camera face looking along +X_b)
    const phoneX = offsetU;
    const cradlePlan = bodyPlan({ x: mastX - 2.5, y: -7, w: 6.5, d: 14, r: 1.4 }, yaw, cx, cy, 3);
    const phonePlan = bodyPlan({ x: phoneX - 2.2, y: -13, w: 4.4, d: 26, r: 1.4 }, yaw, cx, cy, 3);
    const bumpPlan = bodyPlan({ x: phoneX + 2.0, y: -4.2, w: 1.8, d: 8.4, r: 0.8 }, yaw, cx, cy, 2);
    const lensOrigin = toWorld(phoneX + 3.6, 0, 26.5, yaw, cx, cy, deckTop);
    const lensF = frameAlong(lensOrigin, dirWorld(1, 0, 0, yaw));
    const lensRing = solidOf(
      [
        [0, 2.9],
        [1.8, 2.9],
        [1.8, 0],
      ],
      lensF,
      proj,
      { steps: 20 },
    );
    upper.push({
      k: depth(toWorld(phoneX, 0, 24, yaw, cx, cy, deckTop)),
      svg:
        solidSvg(extrude(cradlePlan, mastTopZ - 1, 5.5, proj, 0.4), { tone: "mid" }) +
        solidSvg(extrude(phonePlan, mastTopZ + 1, 14.5, proj, 0.5), { tone: "hi", crease: "lo" }) +
        solidSvg(extrude(bumpPlan, mastTopZ + 5.5, 8.2, proj, 0.3), { tone: "mid" }) +
        solidSvg(lensRing, { tone: "lo" }),
    });

    // Camera frame (C) triad seated on the lens boss:
    // -Z_c out of lens (+X_b, lit), +X_c right (-Y_b), +Y_c up (+Z_b)
    const cHub = toWorld(phoneX + 7.2, 0, 26.5, yaw, cx, cy, deckTop);
    const cAxes = [
      { dir: dirWorld(1, 0, 0, yaw), len: 23, style: { tone: "hi", lit: true } }, // -Z_c lens forward (lit)
      { dir: dirWorld(0, -1, 0, yaw), len: 19, style: { tone: "mid" } }, // +X_c camera right (-Y_b)
      { dir: [0, 0, 1], len: 19, style: { tone: "mid" } }, // +Y_c camera up (+Z_b)
    ];
    upper.push(...triadItems(cHub, cAxes, 2.9, 0.95, 0.4));

    upper.sort((a, b) => a.k - b.k);
    for (const item of upper) out.push(item.svg);

    return out.join("");
  }

  function needlePath(yaw, dial, dialR, dialTop) {
    const [dx, dy] = dial;
    // 0 yaw points along +X_w; +yaw turns CCW (toward -y in iso-kit)
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    return lineOnTop(
      [dx - c * 4.2, dy + s * 4.2],
      [dx + c * (dialR - 2.4), dy - s * (dialR - 2.4)],
      dialTop + 0.2,
      proj,
    );
  }

  return { renderTurntable, needlePath, triadItems };
}

const dyn = makeDynamicBuilder(k, G, P);

// ------------------------------------------------------------------ static bench parts

function surfacePlate() {
  const out = [`<path class="iso-halo" d="${k.haloOf(PLATE, -PLATE_H - FOOT_H, PLATE_H, P)}"/>`];
  // 4 stepped levelling feet
  for (const [x, y] of byDepth(k.corners(PLATE, 22))) {
    out.push(S(k.cylinder(x, y, 8.2, -PLATE_H - FOOT_H, 2.0, P, 28), { tone: "lo" }));
    out.push(S(k.cylinder(x, y, 6.0, -PLATE_H - FOOT_H + 2.0, FOOT_H - 2.0, P, 28), { tone: "mid" }));
  }
  // Heavy bevelled surface plate
  out.push(S(k.slabOf(PLATE, -PLATE_H, PLATE_H, P, 8, 1.8), { tone: "lo" }));
  // Perimeter oil groove
  out.push(L(k.planOutline(k.insetPlan(PLATE, 5.5), 0, P), { tone: "faint" }));
  // 4 corner socket-head cap screws
  const screws = k.corners(PLATE, 11);
  out.push(L(screws.map(([x, y]) => k.ring(x, y, 2.4, 0, P, 16)).join(""), { tone: "lo" }));
  out.push(D(screws.map(([x, y]) => at([x, y, 0])), { size: 0.55 }));

  // Front ruler reference strip (clear of the vernier block)
  const strip = { x: -92, y: PLATE.y + PLATE.d - 14.5, w: 68, d: 6.8, r: 1 };
  out.push(S(k.slabOf(strip, 0, 0.7, P, 3), { tone: "mid", crease: "none" }));
  const ticks = k.topTicks(strip.x + 4, strip.x + strip.w - 4, 4, 5, strip.y + 0.8, 0.7, [2.0, 3.8], P);
  out.push(L(ticks.minor, { tone: "lo" }), L(ticks.major, { tone: "mid" }));
  out.push(
    D(
      [at([strip.x + 2, strip.y + 3.4, 0.7]), at([strip.x + strip.w - 2, strip.y + 3.4, 0.7])],
      { size: 0.45 },
    ),
  );

  // Specification nameplate (front-left) with 3 free rule lines
  const badge = { x: -136, y: 44, w: 40, d: 24, r: 2 };
  out.push(S(k.slabOf(badge, 0, 0.8, P, 3), { tone: "mid", crease: "none" }));
  out.push(
    D(
      k.corners(badge, 3).map(([x, y]) => at([x, y, 0.8])),
      { size: 0.4, tone: "lo" },
    ),
  );
  for (const [dy, len] of [
    [6.5, 24],
    [12, 18],
    [17.5, 22],
  ]) {
    out.push(
      L(k.lineOnTop([badge.x + 6, badge.y + dy], [badge.x + 6 + len, badge.y + dy], 0.8, P), {
        tone: "lo",
        free: true,
      }),
    );
  }
  return out.join("");
}

function worldFramePillar() {
  const [wx, wy] = WORLD_O;
  const out = [];
  // Square flange bolted to plate
  const base = { x: wx - 13, y: wy - 13, w: 26, d: 26, r: 3 };
  out.push(S(k.slabOf(base, 0, 5, P, 4, 0.6), { tone: "mid" }));
  out.push(
    D(
      k.corners(base, 4).map(([x, y]) => at([x, y, 5])),
      { size: 0.45, tone: "lo" },
    ),
  );
  // Pedestal collar + column + top shoulder
  out.push(S(k.cylinder(wx, wy, 7.2, 5, 4, P, 28, 0.4), { tone: "mid" }));
  out.push(S(k.cylinder(wx, wy, 4.0, 9, 22, P, 24), { tone: "mid", crease: "none" }));
  out.push(L(k.sideArc(wx, wy, 4.0, 20, P, 16), { tone: "faint" }));
  out.push(S(k.cylinder(wx, wy, 6.2, 31, 3.5, P, 28, 0.4), { tone: "mid" }));

  // Engraved "W" on the front corner of the base plate (only label in drawing)
  out.push(
    k.faceTextSvg(k.topMatrix([wx + 8, wy + 8.5], 5.1, P, "x"), "W", {
      size: 4.0,
      tone: "mid",
      anchor: "middle",
    }),
  );

  // Fixed World frame (W / A) orthogonal triad:
  // +X_w (-Z_a) = [1, 0, 0], +Y_w (-X_a) = [0, -1, 0], +Z_w (+Y_a) = [0, 0, 1]
  const hub = [wx, wy, 39];
  const axes = [
    { dir: [1, 0, 0], len: 26, style: { tone: "hi" } },
    { dir: [0, -1, 0], len: 25, style: { tone: "hi" } },
    { dir: [0, 0, 1], len: 26, style: { tone: "hi" } },
  ];
  const items = dyn.triadItems(hub, axes, 4.3, 1.25, 0);
  items.sort((a, b) => a.k - b.k);
  for (const item of items) out.push(item.svg);

  return out.join("");
}

function yawDial() {
  const [dx, dy] = DIAL_O;
  const out = [];
  // Mounting block + vertical post + collar
  const blk = { x: dx - 12, y: dy - 12, w: 24, d: 24, r: 3 };
  out.push(S(k.slabOf(blk, 0, 7, P, 4, 0.6), { tone: "mid" }));
  out.push(
    D(
      k.corners(blk, 4).map(([x, y]) => at([x, y, 7])),
      { size: 0.45, tone: "lo" },
    ),
  );
  out.push(S(k.cylinder(dx, dy, 5.5, 7, 4, P, 24, 0.4), { tone: "mid" }));
  out.push(S(k.cylinder(dx, dy, 3.2, 11, DIAL_Z - 11, P, 24), { tone: "mid", crease: "none" }));
  out.push(L(k.sideArc(dx, dy, 3.2, 26, P, 14), { tone: "faint" }));

  // Dial gauge bezel + fine knurl + 36 ticks (every 10 deg, major every 30 deg)
  out.push(S(k.cylinder(dx, dy, DIAL_R, DIAL_Z, DIAL_H, P, 64, 0.8), { tone: "hi", crease: "lo" }));
  out.push(L(k.knurl(dx, dy, DIAL_R, DIAL_TOP - 3.2, DIAL_TOP, 72, P, 0.3), { tone: "lo" }));
  out.push(L(k.sideArc(dx, dy, DIAL_R, DIAL_TOP - 3.2, P, 36), { tone: "faint" }));
  out.push(L(k.ring(dx, dy, DIAL_R - 2.2, DIAL_TOP, P, 64), { tone: "lo" }));
  const ticks = k.radialTicks(dx, dy, DIAL_R - 3.0, 36, 3, DIAL_TOP, [1.4, 3.0], P);
  out.push(L(ticks.minor, { tone: "lo" }), L(ticks.major, { tone: "mid" }));
  // Zero mark pointing along +X_w
  out.push(
    L(k.lineOnTop([dx + DIAL_R - 6.2, dy], [dx + DIAL_R - 2.6, dy], DIAL_TOP, P), { tone: "hi" }),
  );
  out.push(L(k.ring(dx, dy, 1.8, DIAL_TOP, P, 16), { tone: "mid" }));
  return out.join("");
}

function tableHousing() {
  const [cx, cy] = TABLE;
  const out = [];
  // Outer bolted bearing housing
  out.push(
    S(k.cylinder(cx, cy, TABLE_HOUSING_R, TABLE_BASE_Z, 5.5, P, 72, 0.8), {
      tone: "lo",
      crease: "lo",
    }),
  );
  // 8 flange bolts around the outer rim
  const bolts = [];
  for (let i = 0; i < 8; i++) {
    const a = (i * Math.PI * 2) / 8 + Math.PI / 8;
    bolts.push(at([cx + Math.cos(a) * (TABLE_HOUSING_R - 3.4), cy + Math.sin(a) * (TABLE_HOUSING_R - 3.4), 5.5]));
  }
  out.push(D(bolts, { size: 0.5, tone: "mid" }));
  // Stepped inner bearing race
  out.push(
    S(k.cylinder(cx, cy, TABLE_R + 4.5, 5.5, TABLE_HOUSING_TOP - 5.5, P, 72, 0.5), {
      tone: "mid",
    }),
  );
  return out.join("");
}

function frontFixtures() {
  const out = [];

  // 1. Fixed vernier index block at the front-left of the turntable
  const [vx, vy] = VERNIER_O;
  const vBlock = { x: vx - 12, y: vy - 4, w: 24, d: 13, r: 2.2 };
  out.push(S(k.slabOf(vBlock, 0, 15.5, P, 4, 0.6), { tone: "mid" }));
  out.push(
    D(
      [at([vx - 7, vy + 4, 15.5]), at([vx + 7, vy + 4, 15.5])],
      { size: 0.45, tone: "lo" },
    ),
  );
  // Vernier zero witness line edge-to-edge across the block top
  out.push(L(k.lineOnTop([vx, vy - 4], [vx, vy + 9], 15.5, P), { tone: "lit" }));

  // 2. Worm-drive gearbox + knurled azimuth handwheel at front-right
  const [gx, gy] = GEAR_O;
  const gBox = { x: gx - 11, y: gy - 11, w: 22, d: 22, r: 2.5 };
  out.push(S(k.slabOf(gBox, 0, 14, P, 4, 0.6), { tone: "mid" }));
  out.push(
    D(
      k.corners(gBox, 3.8).map(([x, y]) => at([x, y, 14])),
      { size: 0.45, tone: "lo" },
    ),
  );
  // Horizontal worm shaft + knurled handwheel facing +X (down-right)
  const shaftF = G.frameAlong([gx + 11, gy, 7.5], [1, 0, 0]);
  out.push(
    S(
      G.solidOf(
        [
          [0, 3.4],
          [3.5, 3.4],
          [3.5, 2.0],
          [11, 2.0],
          [11, 8.2],
          [16, 8.2],
          [16, 0],
        ],
        shaftF,
        P,
        { steps: 32 },
      ),
      { tone: "hi", crease: "lo" },
    ),
  );
  // Knurl ribs on the handwheel rim
  const ribs = G.ribsOf(shaftF, 11.2, 15.8, 8.2, 28, P, { fade: [0.08, 0.45], seams: true });
  out.push(k.fadedSvg(ribs, { tone: "lo" }));
  // Tiny crank handle pin on the handwheel face
  const crankF = G.frameAlong([gx + 27, gy, 12.2], [1, 0, 0]);
  out.push(
    S(
      G.solidOf(
        [
          [0, 1.2],
          [5.2, 1.2],
          [5.2, 1.9],
          [7.5, 1.9],
          [7.5, 0],
        ],
        crankF,
        P,
        { steps: 16 },
      ),
      { tone: "mid" },
    ),
  );

  return out.join("");
}

// ------------------------------------------------------------------ assemble SVG & live script

const START_YAW = 0.42; // ~24 deg CCW so offset & frames read clearly at rest

const svg = k.figureSvg({
  width: WIDTH,
  height: HEIGHT,
  label:
    "A calibration rotary table showing Terra's VIO coordinate frames. A fixed World frame pillar (W, Z-up; ARKit A, Y-up) stands at the back left, a yaw dial gauge at the back right, and a graduated turntable in the centre carries the Terra rover with its Body frame triad (B) at the pivot and Camera frame triad (C) mounted 0.25 metres forward on the iPhone lens. Rotating the turntable by yaw swings the camera along the 0.25 m calibration ring while the body origin stays fixed at the pivot.",
  body: [
    surfacePlate(),
    worldFramePillar(),
    yawDial(),
    `<path data-part="needle" class="iso-line" data-tone="lit" data-free="start" d="${dyn.needlePath(START_YAW, DIAL_O, DIAL_R, DIAL_TOP)}" style="stroke-width:1.15"/>`,
    tableHousing(),
    `<g data-part="turntable">${dyn.renderTurntable(START_YAW, TABLE, TABLE_HOUSING_TOP, TABLE_TOP, TABLE_R, OFFSET_U)}</g>`,
    frontFixtures(),
  ],
});

const live = `
const P = ${JSON.stringify(P)};
const TABLE = ${JSON.stringify(TABLE)};
const TABLE_HOUSING_TOP = ${TABLE_HOUSING_TOP};
const TABLE_TOP = ${TABLE_TOP};
const TABLE_R = ${TABLE_R};
const OFFSET_U = ${OFFSET_U};
const OFFSET_M = ${OFFSET_M};
const HEIGHT_M = ${HEIGHT_M};
const DIAL_O = ${JSON.stringify(DIAL_O)};
const DIAL_R = ${DIAL_R};
const DIAL_TOP = ${DIAL_TOP};
const makeDynamicBuilder = ${makeDynamicBuilder.toString()};
const kitApi = { iso, depthOf, roundedPlan, extrude, cylinder, ring, radialTicks, lineOnTop, solidSvg, lineSvg, dotsSvg, pathOf };
const dyn = makeDynamicBuilder(kitApi, _lathe, P);

const stage = document.querySelector(".iso-stage");
const svg = stage.querySelector("svg");
const turntable = svg.querySelector('[data-part="turntable"]');
const needle = svg.querySelector('[data-part="needle"]');
const readout = document.querySelector("[data-readout]");
const still = matchMedia("(prefers-reduced-motion: reduce)");

const LIMIT = Math.PI * 0.75; // -135 deg .. +135 deg
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const fmt = (v) => (Math.abs(v) < 0.005 ? "0.00" : (v >= 0 ? "+" : "") + v.toFixed(2));

let yaw = ${START_YAW};
let target = yaw;
let lastRendered = yaw;
let frame = 0, last = 0, clock = 0, touring = !still.matches, visible = false, idleTimer = 0;

const tourStops = [0, Math.PI / 6, Math.PI / 2, -Math.PI / 4];
const tourAt = (t) => tourStops[Math.floor(t / 2.8) % tourStops.length];

function draw() {
  if (Math.abs(yaw - lastRendered) > 1e-4) {
    turntable.innerHTML = dyn.renderTurntable(yaw, TABLE, TABLE_HOUSING_TOP, TABLE_TOP, TABLE_R, OFFSET_U);
    needle.setAttribute("d", dyn.needlePath(yaw, DIAL_O, DIAL_R, DIAL_TOP));
    lastRendered = yaw;
  }
  const deg = Math.round((yaw * 180) / Math.PI);
  // Real Terra VIO math: body centre fixed at (0, 0) during in-place turn;
  // camera in Terra W = (r cos yaw, r sin yaw, h);
  // camera in ARKit A = (-wy, wz, -wx) = (-r sin yaw, h, -r cos yaw).
  const camWx = OFFSET_M * Math.cos(yaw);
  const camWy = OFFSET_M * Math.sin(yaw);
  const vioAx = -camWy;
  const vioAz = -camWx;
  const text = "yaw " + (deg >= 0 ? "+" : "") + deg + "° · cam (" + fmt(camWx) + ", " + fmt(camWy) + ") · body (0, 0)";
  if (readout.textContent !== text) readout.textContent = text;
  stage.setAttribute("aria-valuenow", String(deg));
  stage.setAttribute("aria-valuetext", text);
}

function tick(now) {
  const dt = last ? Math.min((now - last) / 1000, 1 / 30) : 1 / 60;
  last = now;
  const calm = still.matches;
  if (touring) { clock += dt; target = tourAt(clock); }
  yaw = calm ? target : yaw + (target - yaw) * (1 - Math.exp(-dt / 0.24));
  if (Math.abs(target - yaw) < 0.001) yaw = target;
  draw();
  const settled = !touring && yaw === target;
  frame = visible && !settled ? requestAnimationFrame(tick) : 0;
}

const run = () => { if (!frame && visible) { last = 0; frame = requestAnimationFrame(tick); } };
const steer = (next) => {
  touring = false;
  target = clamp(next, -LIMIT, LIMIT);
  clearTimeout(idleTimer);
  idleTimer = setTimeout(() => { touring = !still.matches; clock = 0; run(); }, 4000);
  run();
};

new IntersectionObserver((entries) => {
  visible = entries[entries.length - 1].isIntersecting;
  if (visible) run();
}, { rootMargin: "120px 0px" }).observe(stage);

if (still.matches) touring = false;

stage.addEventListener("pointermove", (event) => {
  const box = svg.getBoundingClientRect();
  const share = clamp(((event.clientX - box.left) / box.width - 0.15) / 0.7, 0, 1);
  steer((0.5 - share) * 2 * LIMIT);
});
stage.addEventListener("pointerleave", () => steer(target));
stage.addEventListener("keydown", (event) => {
  const step = Math.PI / 12; // 15 deg
  let next = null;
  if (event.key === "ArrowLeft" || event.key === "ArrowUp") next = target + step;
  if (event.key === "ArrowRight" || event.key === "ArrowDown") next = target - step;
  if (event.key === "Home" || event.key === "0") next = 0;
  if (next === null) return;
  event.preventDefault();
  steer(next);
});

draw();
`;

const body = k.plateHtml({
  fig: "Fig 1",
  title: "Terra VIO coordinate frames",
  hint: "Hover or ←/→ to turn",
  readout: "yaw +24° · cam (+0.23, +0.10) · body (0, 0)",
  keys: [
    { mark: "lit", label: "Forward heading (+X_b / −Z_c lens axis), dial needle, and vernier index" },
    { mark: "raised", label: "Rover body frame (B) at pivot, Camera frame (C) at +0.25 m offset, fixed World frame (W)" },
  ],
  caption:
    "A calibration rotary table for Terra's VIO frames. ARKit reports camera pose in Y-up world A, remapped to Z-up world W via (x, y, z)_A → (−z, −x, y)_W. Turning left (+yaw, CCW from above) swings the lens and Camera triad C along the 0.25 m offset ring while subtracting R·offset keeps the Body origin B fixed on the pivot.",
  body: `<div class="iso-stage" tabindex="0" role="slider" aria-label="Terra VIO coordinate frames: arrow keys rotate the rover yaw" aria-valuemin="-135" aria-valuemax="135" aria-valuenow="24">${svg}</div>`,
});

const outName = theme === "light" ? "coordinate-frames-light.html" : "coordinate-frames.html";
writeFileSync(
  join(HERE, outName),
  k.pageHtml({
    title: "Terra VIO coordinate frames",
    theme,
    body,
    script: kitScript() + "\n" + latheScript() + "\n" + live,
  }),
);
console.log("wrote", join(HERE, outName));
