use serde_json::json;
use terra_observability::ObservationSettings;
#[test]
fn partial_overrides_preserve_other_defaults() {
    let s: ObservationSettings =
        serde_json::from_value(json!({"evidence_sample_seconds":2.0})).unwrap();
    assert!(s.valid());
    assert_eq!(s.evidence_sample_seconds, 2.);
    assert_eq!(s.control_sample_seconds, 0.1);
}
#[test]
fn invalid_sampling_and_retention_are_rejected() {
    for v in [
        json!({"control_sample_seconds":0}),
        json!({"map_sample_seconds":-1}),
        json!({"retention_seconds":0}),
    ] {
        let s: ObservationSettings = serde_json::from_value(v).unwrap();
        assert!(!s.valid());
    }
}
