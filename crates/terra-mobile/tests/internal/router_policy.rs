use super::*;
#[test]
fn delayed_remote_goal_and_reset_cannot_cross_loss_generation() {
    let c = MobileController::new(default_control_settings()).unwrap();
    let mut b = c.brain.lock().unwrap();
    let run = b.run_id.clone();
    let rev = b.autonomy.authority_revision();
    let goal = serde_json::json!({"frame":"local","x":4,"y":0,"token":"never-seen","run_id":run,"authority_revision":rev});
    assert!(remote_action(&b, "goal", goal.to_string().as_bytes(), 0.1).is_ok());
    c.apply_dashboard_link_loss(&mut b, 1.);
    assert!(remote_action(&b, "goal", goal.to_string().as_bytes(), 0.1).is_err());
    let reset = serde_json::json!({"action":"reset","token":"old-reset","run_id":run,"authority_revision":rev});
    assert!(remote_action(&b, "safety", reset.to_string().as_bytes(), 0.1).is_err());
    assert!(
        remote_action(
            &b,
            "goal",
            br#"{"frame":"local","x":1,"y":0,"token":"legacy"}"#,
            0.
        )
        .is_err()
    );
    assert!(remote_action(&b, "goal", br#"{"cancel":true}"#, 120.).is_ok());
    assert!(remote_action(&b, "safety", br#"{"action":"stop","token":"stop"}"#, 120.).is_ok());
    let fresh = serde_json::json!({"level":"waypoint","token":"new","run_id":run,"authority_revision":b.autonomy.authority_revision()});
    assert!(remote_action(&b, "autonomy", fresh.to_string().as_bytes(), 0.5).is_err());
    assert!(
        terra_autonomy::decode_level(
            &remote_action(&b, "autonomy", fresh.to_string().as_bytes(), 0.1).unwrap()
        )
        .is_some()
    );
}
#[test]
fn display_overrides_are_returned_without_mutating_motion_limits() {
    let c = MobileController::new(default_control_settings()).unwrap();
    c.set_connectivity_policy_settings(
        r#"{"gps_display_freshness_seconds":10,"fleet_display_stale_seconds":120}"#.into(),
    )
    .unwrap();
    let value: serde_json::Value = serde_json::from_str(&c.connectivity_policy_settings()).unwrap();
    assert_eq!(value["gps_display_freshness_seconds"], 10.);
    assert_eq!(value["fleet_display_stale_seconds"], 120.);
    assert_eq!(terra_types::timing::MAP_FRESHNESS_SECONDS, 0.5);
}
