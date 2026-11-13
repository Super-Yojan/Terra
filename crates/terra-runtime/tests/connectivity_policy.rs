use terra_runtime::connectivity::{ConnectivityPolicySettings, RouterLossPolicy};
#[test]
fn connectivity_policy_roundtrips_and_validates() {
    let p = ConnectivityPolicySettings::default();
    let custom = ConnectivityPolicySettings {
        gps_display_freshness_seconds: 7.0,
        fleet_display_stale_seconds: 45.0,
        autonomous_router_loss: RouterLossPolicy::Pause,
    };
    assert_eq!(
        serde_json::from_str::<ConnectivityPolicySettings>(
            &serde_json::to_string(&custom).unwrap()
        )
        .unwrap(),
        custom
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
}
