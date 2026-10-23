use terra_autonomy::{ArbiterInput, AutonomyArbiter, Level, LevelRequest};
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, Twist};

fn status_at(timeout: Option<f64>, now: f64, map_time: Option<f64>, pose_time: f64) -> String {
    let mut arbiter = AutonomyArbiter::default();
    if let Some(seconds) = timeout {
        assert!(arbiter.set_map_freshness_timeout(seconds));
    }
    arbiter.set_level(LevelRequest::new(Level::Waypoint, "mode"));
    let map = MapSnapshot {
        width: 10,
        height: 10,
        resolution: 1.,
        origin_x: -5.,
        origin_y: -5.,
        occupancy: vec![0; 100],
    };
    arbiter
        .step(ArbiterInput {
            now,
            pose: Pose::default(),
            pose_time,
            map: Some(&map),
            map_time,
            map_revision: 1,
            measured: Twist::default(),
            healthy: true,
        })
        .status
        .reason
}

#[test]
fn default_and_custom_timeout_use_a_strict_age_boundary() {
    assert_ne!(status_at(None, 1., Some(0.501), 1.), "map_stale");
    assert_eq!(status_at(None, 1., Some(0.5), 1.), "map_stale");
    assert_ne!(status_at(Some(1.), 1., Some(0.25), 1.), "map_stale");
    assert_eq!(status_at(Some(1.), 1., Some(0.), 1.), "map_stale");
    assert_eq!(status_at(Some(0.1), 1., Some(0.8), 1.), "map_stale");
}

#[test]
fn invalid_settings_leave_the_previous_timeout_unchanged() {
    let mut arbiter = AutonomyArbiter::default();
    assert_eq!(arbiter.map_freshness_timeout(), 0.5);
    assert!(arbiter.set_map_freshness_timeout(1.));
    for invalid in [0., -1., f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(!arbiter.set_map_freshness_timeout(invalid));
        assert_eq!(arbiter.map_freshness_timeout(), 1.);
    }
}

#[test]
fn longer_map_timeout_does_not_relax_pose_or_timestamp_validation() {
    assert_eq!(status_at(Some(2.), 1., Some(0.9), 0.), "sensor_unhealthy");
    for timestamp in [None, Some(2.), Some(-1.), Some(f64::NAN)] {
        assert_eq!(status_at(Some(2.), 1., timestamp, 1.), "map_stale");
    }
}
