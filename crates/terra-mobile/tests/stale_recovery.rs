//! Recovery from a sensor hold mid-goal (Terra#110): hold while pose data is
//! stale, settle briefly after a long hold, and drop the goal after a timeout.
use terra_mobile::{MobileController, default_control_settings};

fn fresh(c: &MobileController, t: f64) {
    c.push_imu(terra_mobile::ImuReading {
        timestamp: t,
        acceleration_forward: 0.,
        acceleration_left: 0.,
        acceleration_up: 0.,
        gyro_roll: 0.,
        gyro_pitch: 0.,
        gyro_yaw: 0.,
    })
    .unwrap();
    c.push_vio(terra_mobile::VioReading {
        timestamp: t,
        position_x: 0.,
        position_y: 0.,
        position_z: 0.,
        quaternion_x: 0.,
        quaternion_y: 0.,
        quaternion_z: 0.,
        quaternion_w: 1.,
        velocity_x: 0.,
        velocity_y: 0.,
        velocity_z: 0.,
        tracked: true,
    })
    .unwrap();
}
fn status(c: &MobileController) -> serde_json::Value {
    serde_json::from_str(&c.autonomy_status()).unwrap()
}
/// A rover at L2 with a goal 4 m ahead, driving on fresh data from t = 1 s.
fn driving() -> std::sync::Arc<MobileController> {
    let c = MobileController::new(default_control_settings()).unwrap();
    fresh(&c, 1.);
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"waypoint_direct","token":"level"}"#.into(),
        1.,
    )
    .unwrap();
    c.autonomy_request(
        "goal".into(),
        r#"{"frame":"local","x":4,"y":0,"token":"goal"}"#.into(),
        1.,
    )
    .unwrap();
    c.step(1.).unwrap();
    let s = status(&c);
    assert_ne!(s["status"]["reason"], "sensor_unhealthy", "{s}");
    assert_eq!(s["goal"]["state"], "active", "{s}");
    c
}
fn held(c: &MobileController) -> bool {
    status(c)["status"]["reason"] == "sensor_unhealthy"
}
/// Step every 0.1 s from `from` to `to` (inclusive) with no new sensor data.
fn stall(c: &MobileController, from: f64, to: f64) {
    let mut t = from;
    while t <= to + 1e-9 {
        c.step(t).unwrap();
        t += 0.1;
    }
}

#[test]
fn short_hold_resumes_on_the_next_fresh_tick() {
    let c = driving();
    stall(&c, 1.5, 1.7);
    assert!(held(&c));
    fresh(&c, 1.8);
    c.step(1.8).unwrap();
    let s = status(&c);
    assert!(!held(&c), "{s}");
    assert_eq!(s["status"]["sensor_hold"]["held_s"], 0.);
    assert_eq!(s["status"]["sensor_hold"]["settling"], false);
}

#[test]
fn long_hold_waits_out_the_settle_period_then_resumes() {
    let c = driving();
    stall(&c, 1.5, 4.0);
    let s = status(&c);
    assert!(held(&c));
    let hold = &s["status"]["sensor_hold"];
    assert!(hold["cause"].as_str().unwrap().starts_with("stale_"), "{s}");
    assert!((hold["held_s"].as_f64().unwrap() - 2.5).abs() < 1e-6, "{s}");
    assert_eq!(hold["settling"], false);
    // Data is back, but the rover holds for the settle period.
    for t in [4.1, 4.3, 4.5] {
        fresh(&c, t);
        c.step(t).unwrap();
        let s = status(&c);
        assert!(held(&c), "t={t} {s}");
        assert_eq!(s["status"]["sensor_hold"]["settling"], true, "t={t} {s}");
        assert!(
            s["status"]["sensor_hold"]["cause"]
                .as_str()
                .unwrap()
                .starts_with("stale_")
        );
    }
    fresh(&c, 4.6);
    c.step(4.6).unwrap();
    let s = status(&c);
    assert!(!held(&c), "{s}");
    assert_eq!(s["goal"]["state"], "active");
    assert_eq!(s["status"]["sensor_hold"]["settling"], false);
}

#[test]
fn stale_data_during_settling_restarts_the_settle_period() {
    let c = driving();
    stall(&c, 1.5, 3.0);
    fresh(&c, 3.1);
    c.step(3.1).unwrap();
    assert_eq!(status(&c)["status"]["sensor_hold"]["settling"], true);
    // Stale again before the settle period ends.
    stall(&c, 3.5, 3.6);
    assert!(held(&c));
    assert_eq!(status(&c)["status"]["sensor_hold"]["settling"], false);
    for t in [3.7, 4.0] {
        fresh(&c, t);
        c.step(t).unwrap();
        assert!(held(&c), "t={t}");
    }
    fresh(&c, 4.2);
    c.step(4.2).unwrap();
    assert!(!held(&c));
}

#[test]
fn hold_past_the_timeout_drops_the_goal_with_a_reason() {
    let c = driving();
    stall(&c, 1.5, 32.0);
    let s = status(&c);
    assert!(held(&c));
    assert_eq!(s["goal"]["state"], "idle", "{s}");
    assert_eq!(s["status"]["request_reason"], "sensor_hold_timeout");
    assert_eq!(s["status"]["sensor_hold"]["goal_dropped"], true);
    // Fresh data again: no lurch toward the old goal, and the drop stays visible.
    for i in 0..10 {
        let t = 32.1 + i as f64 * 0.1;
        fresh(&c, t);
        c.step(t).unwrap();
    }
    let s = status(&c);
    assert!(!held(&c), "{s}");
    assert_eq!(s["goal"]["state"], "idle");
    assert_eq!(s["status"]["sensor_hold"]["goal_dropped"], true);
    assert_eq!(s["twist"]["linear"], 0.);
}

#[test]
fn hold_shorter_than_the_timeout_keeps_the_goal() {
    let c = driving();
    stall(&c, 1.5, 20.0);
    assert_eq!(status(&c)["goal"]["state"], "active");
    assert_eq!(status(&c)["status"]["sensor_hold"]["goal_dropped"], false);
}
