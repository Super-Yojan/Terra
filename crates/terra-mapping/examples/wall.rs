//! Minimal synthetic depth sensor: prints free '.', occupied '#', unknown ' '.
use terra_mapping::{CameraIntrinsics, CameraPose, LocalOccupancyMap, MapConfig};
use terra_types::{Quaternion, Vector3};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = LocalOccupancyMap::new(MapConfig {
        width: 40,
        height: 40,
        resolution: 0.25,
        pixel_stride: 1,
        ..Default::default()
    })?;
    let camera = CameraIntrinsics {
        width: 21,
        height: 1,
        fx: 20.0,
        fy: 20.0,
        cx: 10.0,
        cy: 0.0,
    };
    // Optical right -> world -Y, down -> world -Z, forward -> world +X.
    let pose = CameraPose {
        position: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.5,
        },
        orientation: Quaternion {
            x: -0.5,
            y: 0.5,
            z: -0.5,
            w: 0.5,
        },
    };
    for t in 0..5 {
        map.integrate_depth(t as f64, camera, pose, &[3.0; 21])?;
    }
    let grid = map.snapshot();
    for row in grid.occupancy.chunks(grid.width as usize).rev() {
        println!(
            "{}",
            row.iter()
                .map(|p| match p {
                    -1 => ' ',
                    0..=49 => '.',
                    _ => '#',
                })
                .collect::<String>()
        );
    }
    Ok(())
}
