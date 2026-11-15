//! The phone-side control encoder and layout validator agree with the shared
//! contract fixtures that the rover service's tests also read.
use terra_actuators::MAX_ACTUATORS;
use terra_actuators::protocol::{JSON_LIMIT, PAGINATED_LIMIT, decode_control};

const FIXTURE: &str = include_str!("../../../tests/fixtures/actuators/control_operations.json");

#[test]
fn control_operations_match_the_shared_contract() {
    let fixture: serde_json::Value = serde_json::from_str(FIXTURE).unwrap();
    assert_eq!(fixture["json_limit"], JSON_LIMIT);
    assert_eq!(fixture["paginated_limit"], PAGINATED_LIMIT);
    assert_eq!(fixture["max_actuators"], MAX_ACTUATORS);
    for case in fixture["envelopes"].as_array().unwrap() {
        let envelope = serde_json::json!({
            "schema_version": 1,
            "request_id": 1,
            "operation": case["operation"],
            "payload": case["payload"],
        });
        let accepted = decode_control(envelope.to_string().as_bytes()).is_ok();
        assert_eq!(
            accepted,
            case["rust"].as_bool().unwrap(),
            "{}",
            case["operation"]
        );
    }
}

#[test]
fn relay_drive_accepts_the_shared_actuator_count() {
    let fixture: serde_json::Value = serde_json::from_str(FIXTURE).unwrap();
    let max = fixture["max_actuators"].as_u64().unwrap() as u8;
    let samples = |count: u8| {
        (0..count)
            .map(|id| terra_actuators::protocol::RelaySample {
                id,
                value: 0.0,
                min: -1.0,
                max: 1.0,
            })
            .collect::<Vec<_>>()
    };
    assert!(terra_actuators::protocol::encode_relay_drive(&samples(max)).is_ok());
    assert!(terra_actuators::protocol::encode_relay_drive(&samples(max + 1)).is_err());
}

#[test]
fn layout_validation_matches_the_shared_cases() {
    let fixture: serde_json::Value = serde_json::from_str(include_str!(
        "../../../tests/fixtures/actuators/layout_validation.json"
    ))
    .unwrap();
    let capabilities: terra_actuators::Capabilities =
        serde_json::from_value(fixture["capabilities"].clone()).unwrap();
    for case in fixture["cases"].as_array().unwrap() {
        let layout: terra_actuators::Layout =
            serde_json::from_value(case["layout"].clone()).unwrap();
        let mut codes: Vec<String> = terra_actuators::validate_layout(&layout, &capabilities)
            .err()
            .unwrap_or_default()
            .into_iter()
            .map(|e| e.code)
            .collect();
        codes.sort();
        let mut expected: Vec<String> = serde_json::from_value(case["errors"].clone()).unwrap();
        expected.sort();
        assert_eq!(codes, expected, "{}", case["name"]);
    }
}
