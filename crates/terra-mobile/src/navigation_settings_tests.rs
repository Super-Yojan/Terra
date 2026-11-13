//! Session geometry validation and stale-authority configuration regression scenarios.
use super::*;
fn hardware(b: &mut Brain, output_active: bool) {
    b.runtime
        .ingest(terra_runtime::RuntimeEvent::Hardware {
            status: terra_runtime::HardwareStatus::from_json(
                &serde_json::json!({"ready":true,"output_active":output_active}),
            ),
            received_at: 0.0,
        })
        .unwrap();
}
#[test]
/// Checks the dimensions become the planner's body rectangle (clearance as the
/// margin on every side) and rejects negative clearance or nonfinite dimensions.
fn navigation_dimensions_validate_and_derive_a_conservative_footprint() {
    let mut p = navigation_settings::Parameters::default();
    p.length = 0.8;
    p.width = 0.6;
    p.clearance = 0.1;
    let footprint = p.planner().footprint;
    assert_eq!(
        (footprint.length, footprint.width, footprint.margin),
        (0.8, 0.6, 0.1)
    );
    assert!((footprint.circumradius() - 0.6).abs() < 1e-10);
    assert!(p.valid());
    p.clearance = -0.1;
    assert!(!p.valid());
    p.clearance = 0.1;
    p.length = f64::NAN;
    assert!(!p.valid());
}
#[test]
/// Exercises absent hardware, replayed tokens, a successful geometry update,
/// stale authority in a batch, and active-output rejection.
fn navigation_update_requires_fresh_neutral_idle_hardware_and_current_revisions() {
    let c = MobileController::new(default_control_settings()).unwrap();
    let mut b = c.brain.lock().unwrap();
    b.runtime.step(0.0).unwrap();
    let payload = serde_json::json!({"token":"settings-one","run_id":b.run_id(),"authority_revision":0,"settings_revision":0,"parameters":navigation_settings::Parameters::default()});
    let bytes = serde_json::to_vec(&payload).unwrap();
    navigation_settings::apply(&mut b, &bytes, 0.1);
    assert_eq!(b.navigation().revision, 0);
    hardware(&mut b, false);
    navigation_settings::apply(&mut b, &bytes, 0.1); // rejected tokens cannot be replayed
    assert_eq!(b.navigation().revision, 0);
    let mut p = payload.clone();
    p["token"] = "settings-two".into();
    p["parameters"]["length"] = 0.8.into();
    navigation_settings::apply(&mut b, &serde_json::to_vec(&p).unwrap(), 0.1);
    assert_eq!(b.navigation().revision, 1);
    assert_eq!(b.navigation().parameters.length, 0.8);
    assert_eq!(
        b.autonomy.footprint(),
        b.navigation().parameters.planner().footprint
    );
    let mut stale = payload.clone();
    stale["token"] = "second-in-batch".into();
    stale["settings_revision"] = 1.into();
    navigation_settings::apply(&mut b, &serde_json::to_vec(&stale).unwrap(), 0.1);
    assert_eq!(
        b.navigation().revision,
        1,
        "old authority cannot configure twice in one batch"
    );
    navigation_settings::apply(&mut b, &serde_json::to_vec(&p).unwrap(), 0.1);
    assert_eq!(b.navigation().revision, 1);
    let mut p = payload;
    p["token"] = "settings-three".into();
    p["settings_revision"] = 1.into();
    p["authority_revision"] = 1.into();
    hardware(&mut b, true);
    navigation_settings::apply(&mut b, &serde_json::to_vec(&p).unwrap(), 0.1);
    assert_eq!(b.navigation().revision, 1);
    assert!(
        b.navigation()
            .result
            .as_deref()
            .unwrap()
            .starts_with("rejected")
    );
}
#[test]
/// Applies either a new goal or mode change before settings in the same tick
/// and verifies live authority prevents configuration based on stale cached status.
fn preceding_goal_or_mode_change_cannot_use_an_old_status_snapshot() {
    for change in ["goal", "mode"] {
        let c = MobileController::new(default_control_settings()).unwrap();
        let mut b = c.brain.lock().unwrap();
        let run = b.run_id().to_owned();
        b.runtime.autonomy.set_level(
            terra_autonomy::decode_level(br#"{"level":"waypoint_direct","token":"setup"}"#)
                .unwrap(),
        );
        b.runtime.step(0.0).unwrap();
        hardware(&mut b, false);
        if change == "goal" {
            assert!(b.runtime.autonomy.accept_goal(
                &terra_waypoint::decode_goal(br#"{"frame":"local","x":5,"y":0}"#).unwrap()
            ));
        } else {
            b.runtime.autonomy.set_level(
                terra_autonomy::decode_level(br#"{"level":"teleop","token":"new-level"}"#).unwrap(),
            );
        }
        let r = serde_json::json!({"token":"mixed-settings","run_id":run,"authority_revision":1,"settings_revision":0,"parameters":navigation_settings::Parameters::default()});
        navigation_settings::apply(&mut b, &serde_json::to_vec(&r).unwrap(), 0.1);
        assert_eq!(
            b.navigation().revision,
            0,
            "must reject after preceding {change}"
        );
    }
}
