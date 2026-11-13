//! Coarse long-range memory of the rolling map, for goals beyond its edge.
//!
//! The local occupancy map is a rover-centred window (30 m at 0.1 m), so a far
//! goal is clamped to the window edge and anything the rover saw more than half
//! a window behind is forgotten: it can drive back into a dead end it already
//! left. [`LongRangeMemory`] keeps a 0.5 m grid over 100 m, filled from each
//! local snapshot, and [`LongRangeMemory::subgoal`] runs a coarse 8-connected
//! A* to the far goal and returns where that path leaves the local window.
//! The fine Hybrid A* route then drives to that point instead of to the
//! straight-line clamp.
use std::cmp::Reverse;
use std::collections::BinaryHeap;
use terra_mapping::MapSnapshot;
use terra_mapping::OCCUPIED;

/// Coarse cell side, metres.
const MEMORY_RESOLUTION: f64 = 0.5;
/// Coarse cells per side (100 m at 0.5 m).
const MEMORY_CELLS: usize = 200;
/// Share of a coarse cell's fine cells that must be occupied to block it.
const BLOCKED_SHARE: f64 = 0.15;
/// Share of a coarse cell's fine cells that must be observed to update it.
const OBSERVED_SHARE: f64 = 0.3;
/// Cost multiple for crossing unknown coarse cells.
const UNKNOWN_COST: u32 = 3;
/// A snapshot centre this far from the last one is a local frame jump, metres.
const FRAME_JUMP_M: f64 = 2.;
/// Coarse search expansion cap (the grid has 40,000 cells).
const MAX_EXPANSIONS: usize = 40_000;

const UNKNOWN: u8 = 0;
const FREE: u8 = 1;
const BLOCKED: u8 = 2;

/// See the module docs.
#[derive(Clone, Debug)]
pub struct LongRangeMemory {
    origin_x: f64,
    origin_y: f64,
    cells: Vec<u8>,
    last_center: Option<(f64, f64)>,
}

impl Default for LongRangeMemory {
    fn default() -> Self {
        Self {
            origin_x: -(MEMORY_CELLS as f64) * MEMORY_RESOLUTION / 2.,
            origin_y: -(MEMORY_CELLS as f64) * MEMORY_RESOLUTION / 2.,
            cells: vec![UNKNOWN; MEMORY_CELLS * MEMORY_CELLS],
            last_center: None,
        }
    }
}

impl LongRangeMemory {
    /// Forget everything.
    pub fn clear(&mut self) {
        *self = Self::default();
    }

    fn index(&self, c: i64, r: i64) -> Option<usize> {
        let n = MEMORY_CELLS as i64;
        (c >= 0 && r >= 0 && c < n && r < n).then(|| (r * n + c) as usize)
    }

    fn cell_of(&self, x: f64, y: f64) -> (i64, i64) {
        (
            ((x - self.origin_x) / MEMORY_RESOLUTION).floor() as i64,
            ((y - self.origin_y) / MEMORY_RESOLUTION).floor() as i64,
        )
    }

    fn centre_of(&self, c: i64, r: i64) -> (f64, f64) {
        (
            self.origin_x + (c as f64 + 0.5) * MEMORY_RESOLUTION,
            self.origin_y + (r as f64 + 0.5) * MEMORY_RESOLUTION,
        )
    }

    /// Shift the memory window by whole cells so `(x, y)` is near its centre.
    fn recenter(&mut self, x: f64, y: f64) {
        let half = MEMORY_CELLS as f64 * MEMORY_RESOLUTION / 2.;
        let dc = ((x - half - self.origin_x) / MEMORY_RESOLUTION).round() as i64;
        let dr = ((y - half - self.origin_y) / MEMORY_RESOLUTION).round() as i64;
        if dc == 0 && dr == 0 {
            return;
        }
        let n = MEMORY_CELLS as i64;
        let mut cells = vec![UNKNOWN; self.cells.len()];
        for r in 0..n {
            for c in 0..n {
                if let Some(from) = self.index(c + dc, r + dr) {
                    cells[(r * n + c) as usize] = self.cells[from];
                }
            }
        }
        self.cells = cells;
        self.origin_x += dc as f64 * MEMORY_RESOLUTION;
        self.origin_y += dr as f64 * MEMORY_RESOLUTION;
    }

    /// Fold a local snapshot in. Coarse cells it covers well are overwritten
    /// by what it shows now; the rest keep their memory.
    pub fn absorb(&mut self, map: &MapSnapshot) {
        let (w, h) = (map.width as usize, map.height as usize);
        if !map.valid() {
            return;
        }
        let cx = map.origin_x + w as f64 * map.resolution / 2.;
        let cy = map.origin_y + h as f64 * map.resolution / 2.;
        if !cx.is_finite() || !cy.is_finite() {
            return;
        }
        if self
            .last_center
            .replace((cx, cy))
            .is_some_and(|(px, py)| (cx - px).hypot(cy - py) > FRAME_JUMP_M)
        {
            self.clear();
            self.last_center = Some((cx, cy));
        }
        self.recenter(cx, cy);
        let n = MEMORY_CELLS;
        let mut total = vec![0u32; n * n];
        let mut seen = vec![0u32; n * n];
        let mut hit = vec![0u32; n * n];
        for row in 0..h {
            for col in 0..w {
                let (x, y) = map.centre(col, row);
                let (c, r) = self.cell_of(x, y);
                let Some(i) = self.index(c, r) else { continue };
                total[i] += 1;
                let v = map.occupancy[row * w + col];
                if v >= 0 {
                    seen[i] += 1;
                }
                if v >= OCCUPIED {
                    hit[i] += 1;
                }
            }
        }
        // Only coarse cells fully inside the snapshot are rewritten.
        let full = (MEMORY_RESOLUTION / map.resolution).round().powi(2).max(1.) as u32;
        for i in 0..n * n {
            if total[i] < full || (seen[i] as f64) < OBSERVED_SHARE * total[i] as f64 {
                continue;
            }
            self.cells[i] = if hit[i] as f64 >= BLOCKED_SHARE * total[i] as f64 {
                BLOCKED
            } else {
                FREE
            };
        }
    }

    /// Whether the remembered cell at world `(x, y)` is blocked.
    pub fn blocked_at(&self, x: f64, y: f64) -> bool {
        let (c, r) = self.cell_of(x, y);
        self.index(c, r).is_some_and(|i| self.cells[i] == BLOCKED)
    }

    /// Where a coarse path from `(sx, sy)` to `(gx, gy)` last lies inside
    /// `window` shrunk by `inset` metres. `None` when the goal is inside that
    /// area already, or memory has no path (the caller falls back to clamping).
    ///
    /// Blocked cells are grown by `radius` metres (the rover's body).
    pub fn subgoal(
        &self,
        window: &MapSnapshot,
        (sx, sy): (f64, f64),
        (gx, gy): (f64, f64),
        inset: f64,
        radius: f64,
    ) -> Option<(f64, f64)> {
        let span_x = window.width as f64 * window.resolution;
        let span_y = window.height as f64 * window.resolution;
        let inset = inset.max(0.).min(span_x / 2.).min(span_y / 2.);
        let inside = |x: f64, y: f64| {
            x >= window.origin_x + inset
                && x <= window.origin_x + span_x - inset
                && y >= window.origin_y + inset
                && y <= window.origin_y + span_y - inset
        };
        if inside(gx, gy) || ![sx, sy, gx, gy].iter().all(|v| v.is_finite()) {
            return None;
        }
        let n = MEMORY_CELLS as i64;
        let start = self.cell_of(sx, sy);
        self.index(start.0, start.1)?;
        // A goal beyond memory is aimed at the memory cell nearest to it.
        let goal = self.cell_of(gx, gy);
        let goal = (goal.0.clamp(0, n - 1), goal.1.clamp(0, n - 1));

        let grow = (radius.max(0.) / MEMORY_RESOLUTION).ceil() as i64;
        let mut blocked = vec![false; self.cells.len()];
        for r in 0..n {
            for c in 0..n {
                if self.cells[(r * n + c) as usize] != BLOCKED {
                    continue;
                }
                for dr in -grow..=grow {
                    for dc in -grow..=grow {
                        if let Some(i) = self.index(c + dc, r + dr) {
                            blocked[i] = true;
                        }
                    }
                }
            }
        }
        // The rover's own cell never blocks its start.
        for dr in -grow..=grow {
            for dc in -grow..=grow {
                if let Some(i) = self.index(start.0 + dc, start.1 + dr) {
                    blocked[i] = false;
                }
            }
        }

        // Octile A*, costs in tenths of a cell.
        let h = |c: i64, r: i64| {
            let (dx, dy) = ((c - goal.0).abs(), (r - goal.1).abs());
            (10 * dx.max(dy) + 4 * dx.min(dy)) as u32
        };
        let size = self.cells.len();
        let mut cost = vec![u32::MAX; size];
        let mut parent = vec![usize::MAX; size];
        let s = self.index(start.0, start.1)?;
        let g = self.index(goal.0, goal.1)?;
        cost[s] = 0;
        let mut open = BinaryHeap::from([Reverse((h(start.0, start.1), s))]);
        let mut expansions = 0;
        let mut reached = false;
        while let Some(Reverse((_, i))) = open.pop() {
            if i == g {
                reached = true;
                break;
            }
            expansions += 1;
            if expansions > MAX_EXPANSIONS {
                break;
            }
            let (c, r) = ((i % MEMORY_CELLS) as i64, (i / MEMORY_CELLS) as i64);
            for (dc, dr) in [
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ] {
                let Some(j) = self.index(c + dc, r + dr) else {
                    continue;
                };
                if blocked[j] {
                    continue;
                }
                let step = if dc != 0 && dr != 0 { 14 } else { 10 };
                let step = if self.cells[j] == UNKNOWN {
                    step * UNKNOWN_COST
                } else {
                    step
                };
                let next = cost[i] + step;
                if next < cost[j] {
                    cost[j] = next;
                    parent[j] = i;
                    open.push(Reverse((next + h(c + dc, r + dr), j)));
                }
            }
        }
        if !reached {
            return None;
        }
        // Walk back from the goal to the first path cell inside the window.
        let mut i = g;
        loop {
            let (c, r) = ((i % MEMORY_CELLS) as i64, (i / MEMORY_CELLS) as i64);
            let (x, y) = self.centre_of(c, r);
            if inside(x, y) && !window_occupied(window, x, y) {
                return Some((x, y));
            }
            if i == s {
                return None;
            }
            i = parent[i];
        }
    }
}

fn window_occupied(map: &MapSnapshot, x: f64, y: f64) -> bool {
    let c = ((x - map.origin_x) / map.resolution).floor();
    let r = ((y - map.origin_y) / map.resolution).floor();
    if c < 0. || r < 0. || c >= map.width as f64 || r >= map.height as f64 {
        return false;
    }
    map.occupancy
        .get(r as usize * map.width as usize + c as usize)
        .is_some_and(|v| *v >= OCCUPIED)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 30 m, 0.1 m window centred on `(cx, cy)`, filled by `cell(x, y)`.
    fn window(cx: f64, cy: f64, cell: impl Fn(f64, f64) -> i8) -> MapSnapshot {
        let (n, res) = (300u32, 0.1);
        let (ox, oy) = (cx - 15., cy - 15.);
        let mut occupancy = Vec::with_capacity((n * n) as usize);
        for r in 0..n {
            for c in 0..n {
                let x = ox + (c as f64 + 0.5) * res;
                let y = oy + (r as f64 + 0.5) * res;
                occupancy.push(cell(x, y));
            }
        }
        MapSnapshot {
            width: n,
            height: n,
            resolution: res,
            origin_x: ox,
            origin_y: oy,
            occupancy,
        }
    }

    fn wall(x: f64, y: f64) -> i8 {
        if (11.8..12.2).contains(&x) && y.abs() < 14. {
            100
        } else {
            0
        }
    }

    #[test]
    fn a_goal_inside_the_window_needs_no_subgoal() {
        let mut memory = LongRangeMemory::default();
        let map = window(0., 0., |_, _| 0);
        memory.absorb(&map);
        assert_eq!(memory.subgoal(&map, (0., 0.), (5., 5.), 1., 0.4), None);
    }

    #[test]
    fn a_remembered_wall_behind_the_window_edge_is_routed_around() {
        let mut memory = LongRangeMemory::default();
        // The rover saw a wall 12 m ahead, then drove 10 m back (small steps)
        // so the wall is now outside its 30 m window.
        for step in 0..=20 {
            let x = -(step as f64) * 0.5;
            memory.absorb(&window(x, 0., wall));
        }
        let now = window(-10., 0., |x, y| if wall(x, y) > 0 { 100 } else { 0 });
        assert!(memory.blocked_at(12., 0.));
        let (sx, sy) = memory
            .subgoal(&now, (-10., 0.), (40., 0.), 1., 0.4)
            .expect("a way round the wall");
        // The straight clamp would aim at (4, 0), into the wall; memory aims
        // toward one of its ends.
        assert!(sy.abs() > 5., "({sx}, {sy})");
        assert!(sx <= 5.);
    }

    #[test]
    fn an_unknown_far_goal_is_approached_through_unknown_space() {
        let mut memory = LongRangeMemory::default();
        let map = window(0., 0., |_, _| 0);
        memory.absorb(&map);
        let (x, y) = memory.subgoal(&map, (0., 0.), (60., 0.), 1., 0.4).unwrap();
        assert!(x > 13. && y.abs() < 1., "({x}, {y})");
    }

    #[test]
    fn a_local_frame_jump_forgets_memory() {
        let mut memory = LongRangeMemory::default();
        memory.absorb(&window(0., 0., wall));
        assert!(memory.blocked_at(12., 0.));
        memory.absorb(&window(5., 0., |_, _| -1));
        assert!(!memory.blocked_at(12., 0.));
    }
}
