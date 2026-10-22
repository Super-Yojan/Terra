use terra_types::connectivity_policy::{ConnectivityPolicySettings, RouterLossPolicy};
#[test]
fn display_policy_roundtrips_without_changing_motion_freshness() {
    let p = ConnectivityPolicySettings::default();
    assert_eq!(p.gps_display_freshness_seconds, 3.0);
    assert_eq!(p.fleet_display_stale_seconds, 90.0);
    assert_eq!(
        p.autonomous_router_loss,
        RouterLossPolicy::ContinueAcceptedMission
    );
    assert_eq!(
        serde_json::from_str::<ConnectivityPolicySettings>(&serde_json::to_string(&p).unwrap())
            .unwrap(),
        p
    );
    assert!(p.validate().is_ok());
    for value in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let mut invalid = p.clone();
        invalid.gps_display_freshness_seconds = value;
        assert!(invalid.validate().is_err());
        invalid = p.clone();
        invalid.fleet_display_stale_seconds = value;
        assert!(invalid.validate().is_err());
    }
    assert_eq!(terra_types::timing::MAP_FRESHNESS_SECONDS, 0.5);
    assert_eq!(terra_types::timing::POSE_FRESHNESS_SECONDS, 0.5);
    assert_eq!(terra_types::timing::OPERATOR_COMMAND_FRESHNESS_SECONDS, 0.5);
}
