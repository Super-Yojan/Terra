//! The phone hands Rust ARKit's floor faces; Rust decides which depth returns
//! that floor vetoes and how far the mesh floor can seed the ground policy.
use terra_mobile::{MappingDepthFrame, MobileOccupancyMap, default_occupancy_settings};

fn map() -> std::sync::Arc<MobileOccupancyMap> {
    let mut settings = default_occupancy_settings();
    settings.width = 20;
    settings.height = 20;
    settings.resolution = 0.5;
    settings.pixel_stride = 1;
    MobileOccupancyMap::new(settings).unwrap()
}
/// Floor faces on a 0.1 m lattice, `n` × `n` from `(x0, y0)`, as xyz triples.
fn floor(x0: f32, y0: f32, n: usize, z: f32) -> Vec<f32> {
    (0..n * n)
        .flat_map(|i| {
            [
                x0 + (i % n) as f32 * 0.1 + 0.05,
                y0 + (i / n) as f32 * 0.1 + 0.05,
                z,
            ]
        })
        .collect()
}
/// One level ray along world +X from 0.5 m, returning at 2 m.
fn level_frame(timestamp: f64) -> MappingDepthFrame {
    let half = std::f64::consts::FRAC_PI_4;
    MappingDepthFrame {
        timestamp,
        width: 1,
        height: 1,
        fx: 1.0,
        fy: 1.0,
        cx: 0.0,
        cy: 0.0,
        camera_x: 0.25,
        camera_y: 0.25,
        camera_z: 0.5,
        quaternion_x: 0.0,
        quaternion_y: half.sin(),
        quaternion_z: 0.0,
        quaternion_w: half.cos(),
        depth_metres: vec![2.0],
    }
}

#[test]
fn mesh_floor_vetoes_the_band_floor_ring() {
    for (floor_z, occupied) in [(0.38_f32, false), (0.0, true)] {
        let map = map();
        let cells = map
            .update_floor_surface(0.25, 0.25, 0.5, floor(1.0, -1.0, 20, floor_z))
            .unwrap();
        assert!(cells >= 16, "{cells}");
        for t in 0..4 {
            map.integrate_depth_in_horizon_band(level_frame(t as f64), 10.0, 3.4)
                .unwrap();
        }
        let cell = map.snapshot().unwrap().occupancy[10 * 20 + 14];
        assert!(cell >= 0);
        assert_eq!(cell >= 65, occupied, "floor at {floor_z}");
    }
}

#[test]
fn floor_estimate_seeds_only_from_trusted_floor() {
    let map = map();
    assert!(map.floor_estimate(0.0, 0.0, 2.0).is_none());
    map.update_floor_surface(0.0, 0.0, 0.2, floor(-1.0, -1.0, 20, -0.01))
        .unwrap();
    let estimate = map.floor_estimate(0.0, 0.0, 2.0).unwrap();
    assert!((estimate.height + 0.01).abs() < 1e-6);
    assert!((estimate.area - 4.0).abs() < 0.01, "{}", estimate.area);
    // An empty update turns the veto and the seed off.
    assert_eq!(map.update_floor_surface(0.0, 0.0, 0.2, vec![]).unwrap(), 0);
    assert!(map.floor_estimate(0.0, 0.0, 2.0).is_none());
    map.update_floor_surface(0.0, 0.0, 0.2, floor(-1.0, -1.0, 4, -0.01))
        .unwrap();
    map.clear().unwrap();
    assert!(map.floor_estimate(0.0, 0.0, 2.0).is_none());
}

#[test]
fn malformed_floor_updates_are_rejected() {
    let map = map();
    assert!(
        map.update_floor_surface(0.0, 0.0, 0.5, vec![0.0, 0.0])
            .is_err()
    );
    assert!(
        map.update_floor_surface(f64::NAN, 0.0, 0.5, vec![])
            .is_err()
    );
    assert!(
        map.update_floor_surface(0.0, 0.0, 0.5, vec![0.0; 300_003])
            .is_err()
    );
}

/// The phone's classification view sees each frame with the labels the grid
/// used, plus the mesh floor cells, and only while it asked for them.
#[test]
fn classification_debug_reports_labels_and_floor_cells() {
    let map = map();
    map.update_floor_surface(0.25, 0.25, 0.5, floor(1.0, -1.0, 20, 0.38))
        .unwrap();
    map.integrate_depth_in_horizon_band(level_frame(0.0), 10.0, 3.4)
        .unwrap();
    assert!(map.take_classified_frame(1).is_none(), "off by default");
    map.set_classification_debug(true);
    map.integrate_depth_in_horizon_band(level_frame(1.0), 10.0, 3.4)
        .unwrap();
    let frame = map.take_classified_frame(1).unwrap();
    assert_eq!(frame.classifier, "horizon_band");
    assert_eq!(frame.ground_z, None);
    assert!((frame.mesh_floor_z.unwrap() - 0.38).abs() < 1e-5);
    assert!((frame.low_return_z.unwrap() - 0.5).abs() < 1e-5);
    assert_eq!(frame.labels, vec![6]);
    assert_eq!(frame.counts[6], 1);
    assert_eq!(frame.points.len(), 3);
    assert!((frame.points[0] - 2.25).abs() < 1e-5);
    assert_eq!(frame.camera, vec![0.25, 0.25, 0.5]);
    assert!(!frame.floor_cells.is_empty() && frame.floor_cells.len().is_multiple_of(3));
    assert!(map.take_classified_frame(1).is_none(), "each frame once");
    map.set_classification_debug(false);
    map.integrate_depth_in_horizon_band(level_frame(2.0), 10.0, 3.4)
        .unwrap();
    assert!(map.take_classified_frame(1).is_none());
}

/// The view reports where the classifier puts the ground under the camera, so
/// a wrong mount height shows up against the mesh floor.
#[test]
fn classification_debug_reports_ground_height() {
    let map = map();
    map.set_classification_debug(true);
    map.integrate_depth_with_ground_plane(
        level_frame(0.0),
        terra_mobile::MappingGroundPlane {
            normal_x: 0.0,
            normal_y: 0.0,
            normal_z: 1.0,
            offset: 0.1,
        },
    )
    .unwrap();
    let frame = map.take_classified_frame(1).unwrap();
    assert_eq!(frame.classifier, "ground_plane");
    assert!((frame.ground_z.unwrap() - 0.1).abs() < 1e-9);
    assert_eq!(frame.mesh_floor_z, None);
}
