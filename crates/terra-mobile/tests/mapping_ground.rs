//! Phone-facing ground-plane integration keeps terrain-relative classification
//! separate from the fixed world coordinates used by navigation.
use terra_mobile::{
    MappingDepthFrame, MappingGroundPlane, MobileOccupancyMap, default_occupancy_settings,
};

/// The FFI facade must pass the ground normal and elevation through to mapping:
/// a 20-degree elevated ramp clears its endpoint, while a 30 cm obstacle does not.
#[test]
fn phone_depth_classifies_height_above_the_supporting_slope() {
    let angle = 20_f64.to_radians();
    for (height, occupied) in [(0.0, false), (0.3, true)] {
        let mut settings = default_occupancy_settings();
        settings.width = 20;
        settings.height = 20;
        settings.resolution = 0.5;
        settings.pixel_stride = 1;
        let map = MobileOccupancyMap::new(settings).unwrap();
        let drop = 0.5 - height;
        let ray_angle = angle - (drop / 2_f64).atan();
        let rotation = std::f64::consts::FRAC_PI_2 - ray_angle;
        for timestamp in 0..4 {
            map.integrate_depth_with_ground_plane(
                MappingDepthFrame {
                    timestamp: timestamp as f64,
                    width: 1,
                    height: 1,
                    fx: 1.0,
                    fy: 1.0,
                    cx: 0.0,
                    cy: 0.0,
                    camera_x: 0.25 - angle.sin() * 0.5,
                    camera_y: 0.25,
                    camera_z: 1.0 + angle.cos() * 0.5,
                    quaternion_x: 0.0,
                    quaternion_y: (rotation * 0.5).sin(),
                    quaternion_z: 0.0,
                    quaternion_w: (rotation * 0.5).cos(),
                    depth_metres: vec![(4.0 + drop * drop).sqrt() as f32],
                },
                MappingGroundPlane {
                    normal_x: -angle.sin(),
                    normal_y: 0.0,
                    normal_z: angle.cos(),
                    offset: -angle.sin() * 0.25 + angle.cos(),
                },
            )
            .unwrap();
        }
        let grid = map.snapshot().unwrap();
        assert_eq!((grid.origin_x, grid.origin_y), (-5.0, -5.0));
        let endpoint = grid.occupancy[10 * 20 + 14];
        assert!(endpoint >= 0, "endpoint must be observed");
        assert_eq!(
            endpoint >= 65,
            occupied,
            "wrong terrain-relative classification"
        );
    }
}
