//! The phone-side control encoder agrees with the shared contract fixture that
//! the rover service's tests also read.
use terra_actuators::protocol::{JSON_LIMIT, decode_control};

const FIXTURE: &str = include_str!("../../../tests/fixtures/actuators/control_operations.json");

#[test]
fn control_operations_match_the_shared_contract() {
    let fixture: serde_json::Value = serde_json::from_str(FIXTURE).unwrap();
    assert_eq!(fixture["json_limit"], JSON_LIMIT);
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
