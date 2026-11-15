//! Square world-aligned cell grid shared by the mesh surfaces.

/// Largest accepted grid side, in cells.
const MAX_SIDE: u32 = 400;

/// Geometry of a square grid of `side` × `side` cells, row-major from `origin`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SquareGrid {
    origin_x: f64,
    origin_y: f64,
    pub(crate) resolution: f64,
    side: u32,
}
impl SquareGrid {
    /// Grid of `resolution` cells covering `half_extent` around `(centre_x, centre_y)`.
    /// `None` for nonfinite input, resolution outside `0.01..=10` m, a
    /// nonpositive extent, or more than 400 cells per side.
    pub(crate) fn around(
        centre_x: f64,
        centre_y: f64,
        half_extent: f64,
        resolution: f64,
    ) -> Option<Self> {
        if ![centre_x, centre_y, half_extent, resolution]
            .iter()
            .all(|v| v.is_finite())
            || !(0.01..=10.0).contains(&resolution)
            || half_extent <= 0.0
        {
            return None;
        }
        let side = (2.0 * half_extent / resolution).ceil();
        if side > MAX_SIDE as f64 {
            return None;
        }
        let side = side as u32;
        Some(Self {
            origin_x: centre_x - side as f64 * resolution / 2.0,
            origin_y: centre_y - side as f64 * resolution / 2.0,
            resolution,
            side,
        })
    }
    /// Number of cells.
    pub(crate) fn len(&self) -> usize {
        self.side as usize * self.side as usize
    }
    /// `(col, row)` of the cell containing world `(x, y)`, if inside.
    pub(crate) fn cell(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let col = ((x - self.origin_x) / self.resolution).floor();
        let row = ((y - self.origin_y) / self.resolution).floor();
        (col >= 0.0 && row >= 0.0 && col < self.side as f64 && row < self.side as f64)
            .then_some((col as usize, row as usize))
    }
    /// Row-major index of the cell containing world `(x, y)`, if inside.
    pub(crate) fn index(&self, x: f64, y: f64) -> Option<usize> {
        self.cell(x, y)
            .map(|(col, row)| row * self.side as usize + col)
    }
    /// World centre of the cell at row-major `index`.
    pub(crate) fn centre(&self, index: usize) -> (f64, f64) {
        let side = self.side as usize;
        (
            self.origin_x + ((index % side) as f64 + 0.5) * self.resolution,
            self.origin_y + ((index / side) as f64 + 0.5) * self.resolution,
        )
    }
}
