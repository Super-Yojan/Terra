//! Fuse several rovers' occupancy grids into one world-frame grid.
//!
//! Starting point for the map-merge work in Terra #95 (umbrella) and
//! Zorvane #8 / #13 / #14. It assumes the rover-to-world transform of every
//! grid is already known: in Zorvane that is the spawn pose (Zorvane #13);
//! on real rovers it must come from GPS/GeoAnchor alignment or from map
//! registration, which this crate does not do yet.
//!
//! Cells use the wire format Terra publishes on `map/occupancy`: row-major,
//! rows increasing in +Y, `-1` unknown, `0..=100` occupancy percent, and
//! [`OCCUPIED_THRESHOLD`] as the occupied cutoff (same as ARGOS).
//!
//! Two fusion rules are provided:
//! - [`FusionRule::Priority`]: occupied beats free, free beats unknown
//!   (the Zorvane #14 rule). Conservative: one rover's false obstacle wins.
//! - [`FusionRule::LogOdds`]: sums each rover's evidence in log-odds, so two
//!   rovers that disagree average out instead of the obstacle always winning.
//!
//! ```
//! use terra_map_merge::{FusionRule, Grid, Pose2, merge};
//!
//! let mut a = Grid::unknown(4, 4, 0.5, 0.0, 0.0);
//! a.set(1, 1, 100);
//! let b = Grid::filled(4, 4, 0.5, 0.0, 0.0, 0);
//! let fused = merge(&[(&a, Pose2::IDENTITY), (&b, Pose2::IDENTITY)], 0.5, FusionRule::Priority).unwrap();
//! assert_eq!(fused.at_world(0.75, 0.75), Some(100));
//! assert_eq!(fused.at_world(1.75, 1.75), Some(0));
//! ```

/// A cell at or above this value is occupied. Matches ARGOS `OCCUPIED_THRESHOLD`.
pub const OCCUPIED_THRESHOLD: i8 = 65;
/// Unknown cell value.
pub const UNKNOWN: i8 = -1;
/// Largest fused grid accepted, cells (a 100 m × 100 m area at 0.1 m).
pub const MAX_CELLS: u64 = 1_000_000;

/// Why a merge was refused.
#[derive(Clone, Debug, PartialEq)]
pub enum MergeError {
    /// No input grids.
    Empty,
    /// Resolution is not finite and positive.
    BadResolution,
    /// An input grid's cell count does not match width × height, or a cell is out of range.
    BadGrid(usize),
    /// A transform is not finite.
    BadPose(usize),
    /// The fused grid would exceed [`MAX_CELLS`].
    TooLarge,
}

impl std::fmt::Display for MergeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Empty => write!(f, "no grids to merge"),
            Self::BadResolution => write!(f, "resolution must be finite and positive"),
            Self::BadGrid(i) => write!(f, "grid {i} has invalid geometry or cells"),
            Self::BadPose(i) => write!(f, "pose {i} is not finite"),
            Self::TooLarge => write!(f, "fused grid exceeds {MAX_CELLS} cells"),
        }
    }
}
impl std::error::Error for MergeError {}

/// Display class of one cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CellClass {
    Unknown,
    Free,
    Occupied,
}

/// Class of a raw cell value.
pub fn classify(cell: i8) -> CellClass {
    if cell < 0 {
        CellClass::Unknown
    } else if cell >= OCCUPIED_THRESHOLD {
        CellClass::Occupied
    } else {
        CellClass::Free
    }
}

/// Rigid 2D transform from a rover's map frame into the shared world frame:
/// `world = R(yaw) * local + (x, y)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    /// Radians, counter-clockwise.
    pub yaw: f64,
}

impl Pose2 {
    pub const IDENTITY: Self = Self {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
    };

    pub fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    /// Local point to world.
    pub fn apply(&self, lx: f64, ly: f64) -> (f64, f64) {
        let (s, c) = self.yaw.sin_cos();
        (c * lx - s * ly + self.x, s * lx + c * ly + self.y)
    }

    /// World point to local.
    pub fn inverse_apply(&self, wx: f64, wy: f64) -> (f64, f64) {
        let (s, c) = self.yaw.sin_cos();
        let (dx, dy) = (wx - self.x, wy - self.y);
        (c * dx + s * dy, -s * dx + c * dy)
    }

    fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.yaw.is_finite()
    }
}

/// Row-major occupancy grid, axis-aligned in its own frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Grid {
    pub width: u32,
    pub height: u32,
    /// Metres per cell.
    pub resolution: f64,
    /// Lower-left corner, metres.
    pub origin_x: f64,
    pub origin_y: f64,
    /// `-1` unknown, `0..=100` occupancy.
    pub cells: Vec<i8>,
}

impl Grid {
    /// All-unknown grid.
    pub fn unknown(width: u32, height: u32, resolution: f64, origin_x: f64, origin_y: f64) -> Self {
        Self::filled(width, height, resolution, origin_x, origin_y, UNKNOWN)
    }

    /// Grid with every cell set to `value`.
    pub fn filled(
        width: u32,
        height: u32,
        resolution: f64,
        origin_x: f64,
        origin_y: f64,
        value: i8,
    ) -> Self {
        Self {
            width,
            height,
            resolution,
            origin_x,
            origin_y,
            cells: vec![value; width as usize * height as usize],
        }
    }

    /// Set cell (col, row). Out-of-range indices are ignored.
    pub fn set(&mut self, col: u32, row: u32, value: i8) {
        if col < self.width && row < self.height {
            self.cells[(row * self.width + col) as usize] = value;
        }
    }

    /// Cell (col, row).
    pub fn get(&self, col: u32, row: u32) -> Option<i8> {
        (col < self.width && row < self.height)
            .then(|| self.cells[(row * self.width + col) as usize])
    }

    /// Cell containing a point in this grid's own frame.
    pub fn at_world(&self, x: f64, y: f64) -> Option<i8> {
        let col = ((x - self.origin_x) / self.resolution).floor();
        let row = ((y - self.origin_y) / self.resolution).floor();
        if col < 0.0 || row < 0.0 || col >= f64::from(self.width) || row >= f64::from(self.height) {
            return None;
        }
        self.get(col as u32, row as u32)
    }

    /// Centre of cell (col, row) in this grid's own frame.
    pub fn cell_center(&self, col: u32, row: u32) -> (f64, f64) {
        (
            self.origin_x + (f64::from(col) + 0.5) * self.resolution,
            self.origin_y + (f64::from(row) + 0.5) * self.resolution,
        )
    }

    /// Number of non-unknown cells.
    pub fn known_cells(&self) -> usize {
        self.cells.iter().filter(|&&c| c >= 0).count()
    }

    fn is_valid(&self) -> bool {
        self.resolution.is_finite()
            && self.resolution > 0.0
            && self.origin_x.is_finite()
            && self.origin_y.is_finite()
            && self.cells.len() as u64 == u64::from(self.width) * u64::from(self.height)
            && self.cells.iter().all(|c| (-1..=100).contains(c))
    }
}

/// How overlapping evidence is combined.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FusionRule {
    /// Occupied beats free, free beats unknown. Output is 100, 0 or -1.
    Priority,
    /// Sum of per-rover log-odds, clamped, output as 0..=100.
    LogOdds,
}

/// Largest |log-odds| one cell contributes or the fused cell holds (≈ 0.5 %…99.5 %).
const LOG_ODDS_CLAMP: f64 = 5.3;

fn log_odds(cell: i8) -> f64 {
    let p = (f64::from(cell) / 100.0).clamp(0.005, 0.995);
    (p / (1.0 - p)).ln().clamp(-LOG_ODDS_CLAMP, LOG_ODDS_CLAMP)
}

fn from_log_odds(l: f64) -> i8 {
    let l = l.clamp(-LOG_ODDS_CLAMP, LOG_ODDS_CLAMP);
    (100.0 / (1.0 + (-l).exp())).round() as i8
}

/// Fuse `grids`, each with its rover-to-world [`Pose2`], into one axis-aligned
/// world grid at `resolution`. Each output cell samples the nearest input cell
/// under its centre (no interpolation, so unknown never bleeds into known).
pub fn merge(
    grids: &[(&Grid, Pose2)],
    resolution: f64,
    rule: FusionRule,
) -> Result<Grid, MergeError> {
    if grids.is_empty() {
        return Err(MergeError::Empty);
    }
    if !resolution.is_finite() || resolution <= 0.0 {
        return Err(MergeError::BadResolution);
    }
    let (mut min_x, mut min_y, mut max_x, mut max_y) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for (i, (grid, pose)) in grids.iter().enumerate() {
        if !grid.is_valid() {
            return Err(MergeError::BadGrid(i));
        }
        if !pose.is_finite() {
            return Err(MergeError::BadPose(i));
        }
        let x1 = grid.origin_x + f64::from(grid.width) * grid.resolution;
        let y1 = grid.origin_y + f64::from(grid.height) * grid.resolution;
        for (lx, ly) in [
            (grid.origin_x, grid.origin_y),
            (x1, grid.origin_y),
            (grid.origin_x, y1),
            (x1, y1),
        ] {
            let (wx, wy) = pose.apply(lx, ly);
            min_x = min_x.min(wx);
            min_y = min_y.min(wy);
            max_x = max_x.max(wx);
            max_y = max_y.max(wy);
        }
    }
    let width = ((max_x - min_x) / resolution - 1e-9).ceil().max(1.0);
    let height = ((max_y - min_y) / resolution - 1e-9).ceil().max(1.0);
    if !width.is_finite() || !height.is_finite() || width * height > MAX_CELLS as f64 {
        return Err(MergeError::TooLarge);
    }
    let mut out = Grid::unknown(width as u32, height as u32, resolution, min_x, min_y);
    for row in 0..out.height {
        for col in 0..out.width {
            let (wx, wy) = out.cell_center(col, row);
            let mut priority = UNKNOWN;
            let mut sum = 0.0;
            let mut seen = false;
            for (grid, pose) in grids {
                let (lx, ly) = pose.inverse_apply(wx, wy);
                let Some(cell) = grid.at_world(lx, ly) else {
                    continue;
                };
                if cell < 0 {
                    continue;
                }
                seen = true;
                sum += log_odds(cell);
                priority = match (classify(priority), classify(cell)) {
                    (_, CellClass::Occupied) | (CellClass::Occupied, _) => 100,
                    _ => 0,
                };
            }
            if seen {
                out.set(
                    col,
                    row,
                    match rule {
                        FusionRule::Priority => priority,
                        FusionRule::LogOdds => from_log_odds(sum),
                    },
                );
            }
        }
    }
    Ok(out)
}

/// How well a fused map matches a reference (for example Zorvane ground truth).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Agreement {
    /// Cells known in both maps.
    pub compared: usize,
    /// Of those, cells with the same class.
    pub matching: usize,
    /// Reference-known cells that the map leaves unknown.
    pub missing: usize,
    /// Reference-free cells the map calls occupied.
    pub false_occupied: usize,
    /// Reference-occupied cells the map calls free.
    pub false_free: usize,
}

impl Agreement {
    /// `matching / compared`, or 0 when nothing overlaps.
    pub fn accuracy(&self) -> f64 {
        if self.compared == 0 {
            0.0
        } else {
            self.matching as f64 / self.compared as f64
        }
    }
    /// Share of reference-known cells the map covers.
    pub fn coverage(&self) -> f64 {
        let total = self.compared + self.missing;
        if total == 0 {
            0.0
        } else {
            self.compared as f64 / total as f64
        }
    }
}

/// Compare `map` against `reference`, sampling `map` at each reference cell
/// centre. Both are assumed to be in the same frame.
pub fn agreement(map: &Grid, reference: &Grid) -> Agreement {
    let mut a = Agreement {
        compared: 0,
        matching: 0,
        missing: 0,
        false_occupied: 0,
        false_free: 0,
    };
    for row in 0..reference.height {
        for col in 0..reference.width {
            let truth = classify(reference.get(col, row).unwrap_or(UNKNOWN));
            if truth == CellClass::Unknown {
                continue;
            }
            let (x, y) = reference.cell_center(col, row);
            let got = classify(map.at_world(x, y).unwrap_or(UNKNOWN));
            match (truth, got) {
                (_, CellClass::Unknown) => a.missing += 1,
                (t, g) => {
                    a.compared += 1;
                    if t == g {
                        a.matching += 1;
                    } else if t == CellClass::Free {
                        a.false_occupied += 1;
                    } else {
                        a.false_free += 1;
                    }
                }
            }
        }
    }
    a
}
