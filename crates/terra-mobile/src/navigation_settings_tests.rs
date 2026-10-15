use super::*;
#[test]
fn navigation_dimensions_validate_and_derive_a_conservative_footprint() {
    let mut p = navigation_settings::Parameters::default();
    p.length = 0.8;
    p.width = 0.6;
    p.clearance = 0.1;
    assert!((p.planner().radius - 0.6).abs() < 1e-10);
    assert!(p.valid());
    p.clearance = -0.1;
    assert!(!p.valid());
    p.clearance = 0.1;
    p.length = f64::NAN;
    assert!(!p.valid());
}
#[test]
fn navigation_update_requires_fresh_disarmed_idle_hardware_and_current_revisions() {
    let c = MobileController::new(default_control_settings()).unwrap();
    let mut b = c.brain.lock().unwrap();
    b.autonomy.run_id = b.run_id.clone();
    b.autonomy_json=serde_json::json!({"status":{"run_id":b.run_id,"revision":0,"safety":"hold"},"goal":{"state":"idle"}}).to_string();
    let payload = serde_json::json!({"token":"settings-one","run_id":b.run_id,"authority_revision":0,"settings_revision":0,"parameters":navigation_settings::Parameters::default()});
    let bytes = serde_json::to_vec(&payload).unwrap();
    navigation_settings::apply(&mut b, &bytes, 0.1);
    assert_eq!(b.navigation.revision, 0);
    b.hardware_observation = Some((
        serde_json::json!({"ready":true,"armed":false,"arming":false}),
        std::time::Instant::now(),
    ));
    navigation_settings::apply(&mut b, &bytes, 0.1); // rejected tokens cannot be replayed
    assert_eq!(b.navigation.revision, 0);
    let mut p = payload.clone();
    p["token"] = "settings-two".into();
    p["parameters"]["length"] = 0.8.into();
    navigation_settings::apply(&mut b, &serde_json::to_vec(&p).unwrap(), 0.1);
    assert_eq!(b.navigation.revision, 1);
    assert_eq!(b.navigation.parameters.length, 0.8);
    assert!(
        (b.autonomy.planner.config.radius - b.navigation.parameters.planner().radius).abs() < 1e-10
    );
    let mut stale = payload.clone();
    stale["token"] = "second-in-batch".into();
    stale["settings_revision"] = 1.into();
    navigation_settings::apply(&mut b, &serde_json::to_vec(&stale).unwrap(), 0.1);
    assert_eq!(
        b.navigation.revision, 1,
        "old authority cannot configure twice in one batch"
    );
    navigation_settings::apply(&mut b, &serde_json::to_vec(&p).unwrap(), 0.1);
    assert_eq!(b.navigation.revision, 1);
    let mut p = payload;
    p["token"] = "settings-three".into();
    p["settings_revision"] = 1.into();
    p["authority_revision"] = 1.into();
    b.hardware_observation.as_mut().unwrap().0["armed"] = true.into();
    navigation_settings::apply(&mut b, &serde_json::to_vec(&p).unwrap(), 0.1);
    assert_eq!(b.navigation.revision, 1);
    assert!(
        b.navigation
            .result
            .as_deref()
            .unwrap()
            .starts_with("rejected")
    );
}
#[test]
fn preceding_goal_or_mode_change_cannot_use_an_old_status_snapshot() {
    for change in ["goal", "mode"] {
        let c = MobileController::new(default_control_settings()).unwrap();
        let mut b = c.brain.lock().unwrap();
        let run = b.run_id.clone();
        b.autonomy.run_id = run.clone();
        b.autonomy.set_level(
            terra_autonomy::decode_level(br#"{"level":"waypoint_direct","token":"setup"}"#)
                .unwrap(),
        );
        b.autonomy_json=serde_json::json!({"status":{"run_id":run,"revision":1,"safety":"clear"},"goal":{"state":"idle"}}).to_string();
        b.hardware_observation = Some((
            serde_json::json!({"ready":true,"armed":false,"arming":false}),
            std::time::Instant::now(),
        ));
        if change == "goal" {
            assert!(b.autonomy.accept_goal(
                &terra_waypoint::decode_goal(br#"{"frame":"local","x":5,"y":0}"#).unwrap(),
                20_000.
            ));
        } else {
            b.autonomy.set_level(
                terra_autonomy::decode_level(br#"{"level":"teleop","token":"new-level"}"#).unwrap(),
            );
        }
        let r = serde_json::json!({"token":"mixed-settings","run_id":run,"authority_revision":1,"settings_revision":0,"parameters":navigation_settings::Parameters::default()});
        navigation_settings::apply(&mut b, &serde_json::to_vec(&r).unwrap(), 0.1);
        assert_eq!(
            b.navigation.revision, 0,
            "must reject after preceding {change}"
        );
    }
}
