//! Synthetic occupancy grids on Terra's default 200×200, 0.1 m window.

use base64::Engine;
use terra_mapping::MapSnapshot;

pub const WIDTH: u32 = 200;
pub const HEIGHT: u32 = 200;
pub const RESOLUTION: f64 = 0.1;
pub const ORIGIN: f64 = -10.0;

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct Counts {
    /// Cells with observed occupancy below 65.
    pub free: usize,
    /// Cells with occupancy at least 65.
    pub occupied: usize,
    /// Cells with negative occupancy values.
    pub unknown: usize,
}

pub fn blank() -> MapSnapshot {
    MapSnapshot {
        width: WIDTH,
        height: HEIGHT,
        resolution: RESOLUTION,
        origin_x: ORIGIN,
        origin_y: ORIGIN,
        occupancy: vec![-1; (WIDTH * HEIGHT) as usize],
    }
}

pub fn cell_index(map: &MapSnapshot, col: i32, row: i32) -> Option<usize> {
    if col < 0 || row < 0 || col >= map.width as i32 || row >= map.height as i32 {
        None
    } else {
        Some(row as usize * map.width as usize + col as usize)
    }
}

pub fn world_cell(map: &MapSnapshot, x: f64, y: f64) -> Option<(i32, i32)> {
    if !x.is_finite() || !y.is_finite() {
        return None;
    }
    let col = ((x - map.origin_x) / map.resolution).floor() as i32;
    let row = ((y - map.origin_y) / map.resolution).floor() as i32;
    cell_index(map, col, row)?;
    Some((col, row))
}

pub fn cell_center(map: &MapSnapshot, col: i32, row: i32) -> (f64, f64) {
    (
        map.origin_x + (f64::from(col) + 0.5) * map.resolution,
        map.origin_y + (f64::from(row) + 0.5) * map.resolution,
    )
}

pub fn occupancy_at(map: &MapSnapshot, x: f64, y: f64) -> Option<i8> {
    let (col, row) = world_cell(map, x, y)?;
    Some(map.occupancy[cell_index(map, col, row)?])
}

pub fn fill_rect(map: &mut MapSnapshot, min_x: f64, min_y: f64, max_x: f64, max_y: f64, value: i8) {
    for row in 0..map.height as i32 {
        for col in 0..map.width as i32 {
            let (x, y) = cell_center(map, col, row);
            if x >= min_x && x < max_x && y >= min_y && y < max_y {
                let index = cell_index(map, col, row).expect("in range");
                map.occupancy[index] = value;
            }
        }
    }
}

pub fn fill_disk(map: &mut MapSnapshot, cx: f64, cy: f64, radius: f64, value: i8) {
    for row in 0..map.height as i32 {
        for col in 0..map.width as i32 {
            let (x, y) = cell_center(map, col, row);
            if (x - cx).hypot(y - cy) <= radius {
                map.occupancy[row as usize * map.width as usize + col as usize] = value;
            }
        }
    }
}

pub fn counts(map: &MapSnapshot) -> Counts {
    let mut out = Counts {
        free: 0,
        occupied: 0,
        unknown: 0,
    };
    for value in &map.occupancy {
        if *value < 0 {
            out.unknown += 1;
        } else if *value >= 65 {
            out.occupied += 1;
        } else {
            out.free += 1;
        }
    }
    out
}

/// 1 m symbolic grid. `#` if any child is occupied, `.` if every child is known
/// and free, `?` otherwise, `R` on the rover's metre.
pub fn symbolic_grid(map: &MapSnapshot, pose_x: f64, pose_y: f64) -> String {
    let metres_x = (f64::from(map.width) * map.resolution).round() as i32;
    let metres_y = (f64::from(map.height) * map.resolution).round() as i32;
    let child = (1.0 / map.resolution).round() as i32;
    let mut lines = Vec::with_capacity(metres_y as usize);
    for metre_y in (0..metres_y).rev() {
        let mut line = String::with_capacity(metres_x as usize);
        for metre_x in 0..metres_x {
            let x0 = map.origin_x + f64::from(metre_x);
            let y0 = map.origin_y + f64::from(metre_y);
            let rover_here = pose_x >= x0 && pose_x < x0 + 1.0 && pose_y >= y0 && pose_y < y0 + 1.0;
            let col0 = ((x0 - map.origin_x) / map.resolution).round() as i32;
            let row0 = ((y0 - map.origin_y) / map.resolution).round() as i32;
            let mut occupied = 0;
            let mut free = 0;
            let mut unknown = 0;
            for dy in 0..child {
                for dx in 0..child {
                    if let Some(index) = cell_index(map, col0 + dx, row0 + dy) {
                        let value = map.occupancy[index];
                        if value < 0 {
                            unknown += 1;
                        } else if value >= 65 {
                            occupied += 1;
                        } else {
                            free += 1;
                        }
                    }
                }
            }
            let symbol = if rover_here {
                'R'
            } else if occupied > 0 {
                '#'
            } else if free > 0 && unknown == 0 {
                '.'
            } else {
                '?'
            };
            line.push(symbol);
        }
        lines.push(line);
    }
    lines.join("\n")
}

pub fn segment_hits_occupied(map: &MapSnapshot, from: [f64; 2], to: [f64; 2]) -> bool {
    let distance = (to[0] - from[0]).hypot(to[1] - from[1]);
    let steps = (distance / map.resolution).ceil() as usize;
    (0..=steps).any(|step| {
        let t = step as f64 / steps.max(1) as f64;
        let x = from[0] + (to[0] - from[0]) * t;
        let y = from[1] + (to[1] - from[1]) * t;
        occupancy_at(map, x, y).is_some_and(|value| value >= 65)
    })
}

pub fn near_unknown(map: &MapSnapshot, x: f64, y: f64, radius: f64) -> bool {
    let steps = (radius / map.resolution).ceil() as i32 + 1;
    let Some((col, row)) = world_cell(map, x, y) else {
        return false;
    };
    for dy in -steps..=steps {
        for dx in -steps..=steps {
            let Some(index) = cell_index(map, col + dx, row + dy) else {
                continue;
            };
            if map.occupancy[index] >= 0 {
                continue;
            }
            let (cx, cy) = cell_center(map, col + dx, row + dy);
            if (cx - x).hypot(cy - y) <= radius {
                return true;
            }
        }
    }
    false
}

pub fn encode_occupancy(values: &[i8]) -> String {
    let bytes: Vec<u8> = values.iter().map(|value| *value as u8).collect();
    base64::engine::general_purpose::STANDARD.encode(bytes)
}

pub fn decode_occupancy(text: &str) -> Result<Vec<i8>, String> {
    let bytes = base64::engine::general_purpose::STANDARD
        .decode(text)
        .map_err(|err| err.to_string())?;
    Ok(bytes.into_iter().map(|value| value as i8).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The coarse text grid preserves rover, obstacle, and unknown-space symbols.
    #[test]
    fn symbolic_marks_rover_obstacle_and_unknown() {
        let mut map = blank();
        fill_rect(&mut map, -1.0, -1.0, 2.0, 2.0, 0);
        fill_disk(&mut map, 1.5, 1.5, 0.25, 100);
        let grid = symbolic_grid(&map, 0.2, 0.2);
        let lines: Vec<&str> = grid.lines().collect();
        assert_eq!(lines.len(), 20);
        assert!(lines.iter().any(|line| line.contains('R')));
        assert!(lines.iter().any(|line| line.contains('#')));
        assert!(lines[0].chars().all(|ch| ch == '?'));
    }

    /// Base64 storage preserves signed unknown values as well as observed occupancy.
    #[test]
    fn occupancy_roundtrip_keeps_unknown() {
        let values = vec![-1, 0, 65, 100];
        let decoded = decode_occupancy(&encode_occupancy(&values)).unwrap();
        assert_eq!(decoded, values);
    }
}
