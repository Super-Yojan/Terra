use terra_autonomy::*;
/// Build valid version-one search JSON for mutation by wire-contract tests.
fn request() -> serde_json::Value {
    serde_json::json!({
        "version": 1,
        "run_id": "local",
        "search_id": "search-1",
        "token": "start-1",
        "target_class": "survivor",
        "bounds": {"min_x": -5.0,"min_y": -5.0,"max_x": 5.0,"max_y": 5.0},
        "time_budget_s": 60.0,
    })
}
/// Decode the target-search authority name and a valid version-one, 60-second bounded request.
#[test]
fn search_contract_accepts_named_level_and_bounded_request() {
    assert_eq!(
        decode_level(br#"{"level":"target_search","token":"mode"}"#)
            .unwrap()
            .level,
        Level::TargetSearch
    );
    assert!(decode_search_request(&serde_json::to_vec(&request()).unwrap()).is_some());
}
/// Mutate version, class, budget, token, fields, and bounds; also reject a payload beyond the 2048-byte limit.
#[test]
fn search_contract_rejects_invalid_and_oversized_requests() {
    for (field, value) in [
        ("version", serde_json::json!(2)),
        ("target_class", serde_json::json!("")),
        ("time_budget_s", serde_json::json!(0)),
        ("token", serde_json::json!("bad token")),
        ("unexpected", serde_json::json!(true)),
    ] {
        let mut v = request();
        v[field] = value;
        assert!(
            decode_search_request(&serde_json::to_vec(&v).unwrap()).is_none(),
            "{field}"
        );
    }
    let mut v = request();
    v["bounds"]["max_x"] = serde_json::json!(200);
    assert!(decode_search_request(&serde_json::to_vec(&v).unwrap()).is_none());
    assert!(decode_search_request(&vec![b' '; 2049]).is_none());
}
/// Decode explicit run/search-correlated pause and report-ack payloads while rejecting an unknown restart action.
#[test]
fn actions_and_report_ack_are_session_correlated() {
    assert!(
        decode_search_action(
            br#"{"version":1,"run_id":"local","search_id":"s","token":"p","action":"pause"}"#
        )
        .is_some()
    );
    assert!(
        decode_search_action(
            br#"{"version":1,"run_id":"local","search_id":"s","token":"p","action":"restart"}"#
        )
        .is_none()
    );
    assert!(decode_search_report_ack(br#"{"version":1,"run_id":"local","search_id":"s","report_id":"s:report","token":"ack"}"#).is_some());
}
/// Accept the derived report ID when its 64-character search ID plus report suffix exceeds the ordinary token limit.
#[test]
fn full_length_search_id_has_a_valid_report_ack() {
    let id = "s".repeat(64);
    let v = serde_json::json!({
        "version": 1,
        "run_id": "local",
        "search_id": id,
        "report_id": format!("{}:report",id),
        "token": "ack",
    });
    assert!(decode_search_report_ack(&serde_json::to_vec(&v).unwrap()).is_some());
}
