//! The phone-facing WebRTC API does not create hardware or motion intent.
use terra_mobile::{MobileController, default_control_settings};

#[test]
fn webrtc_signaling_without_a_dashboard_does_not_arm_or_drive() {
    let controller = MobileController::new(default_control_settings()).unwrap();
    assert!(controller.take_webrtc_signaling().is_empty());
    let rejected = controller.publish_webrtc_signaling(
        "webrtc/state".into(),
        r#"{"v":1,"session":"11111111-1111-4111-8111-111111111111","state":"closed"}"#.into(),
    );
    assert!(rejected.is_err());
    let motion = controller
        .publish_webrtc_signaling("teleop".into(), r#"{"linear":1,"angular":0.5}"#.into())
        .unwrap_err();
    let _ = motion;
    let effects = controller.fpv_receive(
        "webrtc/request".into(),
        r#"{"v":1,"action":"start","session":"11111111-1111-4111-8111-111111111111","note":"optional"}"#
            .into(),
        0,
    );
    assert!(effects.iter().any(|effect| effect.kind == "create_offer"));
    assert!(effects.iter().all(|effect| {
        effect.topic != "teleop" && effect.topic != "hardware" && effect.topic != "cmd_vel"
    }));
    assert!(
        controller
            .fpv_receive(
                "teleop".into(),
                r#"{"v":1,"linear":1,"angular":0}"#.into(),
                1,
            )
            .is_empty()
    );
    let output = controller.step(1.0).unwrap();
    assert_eq!(output.target_forward, 0.0);
    assert_eq!(output.target_yaw_rate, 0.0);
    assert_eq!(output.left_effort, 0.0);
    assert_eq!(output.right_effort, 0.0);
    assert!(controller.fpv_has_session());
}
