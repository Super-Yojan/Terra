use terra_autonomy::*;
#[test]
fn strict_requests_and_correlation() {
    assert!(decode_level(br#"{"level":"teleop","token":"switch-1"}"#).is_some());
    let explore =
        decode_level(br#"{"level":"explore","token":"explore-1","budget_minutes":2}"#).unwrap();
    assert_eq!(explore.resolved_budget_seconds(), Some(120.));
    assert!(
        decode_level(br#"{"level":"explore","token":"bad","budget_seconds":1,"budget_minutes":1}"#)
            .is_none()
    );
    assert!(decode_level(br#"{"level":"explore","token":"bad","budget_seconds":0}"#).is_none());
    for b in [
        br#"{"level":"magic","token":"x"}"#.as_slice(),
        br#"{"level":"teleop","token":""}"#,
        br#"{"level":"teleop","token":"x","extra":1}"#,
    ] {
        assert!(decode_level(b).is_none());
    }
    assert!(
        decode_teleop(
            br#"{"linear":1,"angular":0,"operator_session_id":"session-1","sequence":2}"#
        )
        .is_some()
    );
    assert!(decode_teleop(br#"{"linear":true,"angular":0}"#).is_none());
    assert!(decode_safety(br#"{"action":"stop","token":"stop-1"}"#).is_some());
}
