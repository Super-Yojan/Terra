use terra_robot::{Core, connectivity::ConnectivityPolicySettings};

#[test]
fn delayed_remote_goal_and_reset_cannot_cross_loss_generation() {
    let mut b = Core::new(Default::default(), "run".into()).unwrap();
    let run = b.run_id().to_owned();
    let rev = b.autonomy().authority_revision();
    let goal = serde_json::json!({"frame":"local","x":4,"y":0,"token":"never-seen","run_id":run,"authority_revision":rev});
    assert!(b.remote_action("goal", goal.to_string().as_bytes(), 0.1).is_ok());
    b.lose_link();
    assert!(b.remote_action("goal", goal.to_string().as_bytes(), 0.1).is_err());
    let reset = serde_json::json!({"action":"reset","token":"old-reset","run_id":run,"authority_revision":rev});
    assert!(b.remote_action("safety", reset.to_string().as_bytes(), 0.1).is_err());
    assert!(
        b.remote_action("goal", br#"{"frame":"local","x":1,"y":0,"token":"legacy"}"#, 0.)
            .is_err()
    );
    assert!(b.remote_action("goal", br#"{"cancel":true}"#, 120.).is_ok());
    assert!(b.remote_action("safety", br#"{"action":"stop","token":"stop"}"#, 120.).is_ok());
    let fresh = serde_json::json!({"level":"waypoint","token":"new","run_id":run,"authority_revision":b.autonomy().authority_revision()});
    assert!(b.remote_action("autonomy", fresh.to_string().as_bytes(), 0.5).is_err());
    assert!(
        terra_autonomy::decode_level(
            &b.remote_action("autonomy", fresh.to_string().as_bytes(), 0.1).unwrap()
        )
        .is_some()
    );
}

#[test]
fn display_overrides_are_returned_without_mutating_motion_limits() {
    let mut c = Core::new(Default::default(), "run".into()).unwrap();
    let map_timeout = c.autonomy().map_freshness_timeout();
    let policy: ConnectivityPolicySettings = serde_json::from_str(
        r#"{"gps_display_freshness_seconds":10,"fleet_display_stale_seconds":120}"#,
    )
    .unwrap();
    c.set_connectivity_policy(policy).unwrap();
    let value = serde_json::to_value(c.connectivity()).unwrap();
    assert_eq!(value["gps_display_freshness_seconds"], 10.);
    assert_eq!(value["fleet_display_stale_seconds"], 120.);
    assert_eq!(c.autonomy().map_freshness_timeout(), map_timeout);
}
