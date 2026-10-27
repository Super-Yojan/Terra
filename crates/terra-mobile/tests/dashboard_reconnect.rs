//! Loopback router-loss and explicit reconnect regression scenario.
//! Network scenarios require local TCP sockets and use bounded receipt waits.
use std::time::{Duration, Instant};
use terra_mobile::{MobileController, default_control_settings};
use zenoh::Wait;
/// Open a discovery-free loopback router on the supplied endpoint for reconnect scenarios.
fn router(endpoint: &str) -> zenoh::Session {
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"router\"").unwrap();
    config
        .insert_json5(
            "listen/endpoints",
            &serde_json::json!([endpoint]).to_string(),
        )
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    zenoh::open(config).wait().unwrap()
}
#[test]
/// Accepts a routed waypoint, closes the router, observes failure, explicitly clears
/// dashboard intent, then reconnects and verifies the old goal is idle rather than replayed.
fn accepted_goal_does_not_resume_after_router_loss_and_explicit_reconnect() {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let first = router(&endpoint);
    let controller = MobileController::new(default_control_settings()).unwrap();
    controller
        .connect_dashboard(endpoint.clone(), "phone-reconnect".into(), 7)
        .unwrap();
    controller
        .autonomy_request(
            "autonomy".into(),
            r#"{"level":"waypoint","token":"level"}"#.into(),
            1.,
        )
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut now = 1.;
    controller.step(now).unwrap();
    let binding = status(&controller);
    let goal=serde_json::json!({"frame":"local","x":1,"y":0,"token":"old-goal","run_id":binding["status"]["run_id"],"authority_revision":binding["status"]["revision"]}).to_string();
    loop {
        first
            .put("phone-reconnect/7/goal", goal.clone())
            .wait()
            .unwrap();
        now += 0.01;
        controller
            .push_imu(terra_mobile::ImuReading {
                timestamp: now,
                acceleration_forward: 0.,
                acceleration_left: 0.,
                acceleration_up: 0.,
                gyro_roll: 0.,
                gyro_pitch: 0.,
                gyro_yaw: 0.,
            })
            .unwrap();
        controller
            .push_vio(terra_mobile::VioReading {
                timestamp: now,
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
        controller.step(now).unwrap();
        let state: serde_json::Value = serde_json::from_str(&controller.autonomy_status()).unwrap();
        if state["goal"]["token"] == "old-goal" {
            break;
        }
        assert!(Instant::now() < deadline, "goal acknowledgement timed out");
        std::thread::sleep(Duration::from_millis(10));
    }
    first.close().wait().unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while controller.dashboard_status() != "failed" {
        assert!(Instant::now() < deadline, "loss detection timed out");
        std::thread::sleep(Duration::from_millis(20));
    }
    let output = controller.step(now + 0.01).unwrap();
    assert_eq!((output.left_effort, output.right_effort), (0., 0.));
    // The phone lifecycle handles observed loss by detaching and clearing intent.
    controller.disconnect_dashboard().unwrap();
    let _replacement = router(&endpoint);
    controller
        .connect_dashboard(endpoint, "phone-reconnect".into(), 7)
        .unwrap();
    std::thread::sleep(Duration::from_millis(150));
    controller.step(now + 0.02).unwrap();
    let state: serde_json::Value = serde_json::from_str(&controller.autonomy_status()).unwrap();
    assert_eq!(state["goal"]["state"], "idle");
    assert!(state["goal"]["token"].is_null());
    assert_eq!(state["status"]["requested_level"], "teleop");
    assert_eq!(controller.dashboard_status(), "connected");
}

fn fresh_local(c: &MobileController, t: f64) {
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
    c.autonomy_map(
        terra_mobile::OccupancyGrid {
            width: 80,
            height: 80,
            resolution: 0.25,
            origin_x: -10.,
            origin_y: -10.,
            occupancy: vec![0; 6400],
        },
        t,
        1,
    )
    .unwrap();
}
fn status(c: &MobileController) -> serde_json::Value {
    serde_json::from_str(&c.autonomy_status()).unwrap()
}

#[test]
fn automatic_loss_preserves_accepted_mission_and_pause_policy_is_latched() {
    for mode in ["waypoint", "waypoint_direct"] {
        let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
        drop(socket);
        let cloud = router(&endpoint);
        let c = MobileController::new(default_control_settings()).unwrap();
        c.connect_dashboard(endpoint.clone(), "auto-loss".into(), 0)
            .unwrap();
        fresh_local(&c, 1.);
        c.autonomy_request(
            "autonomy".into(),
            format!(r#"{{"level":"{mode}","token":"level"}}"#),
            1.,
        )
        .unwrap();
        c.autonomy_request(
            "goal".into(),
            r#"{"frame":"local","x":4,"y":0,"token":"accepted"}"#.into(),
            1.,
        )
        .unwrap();
        c.step(1.).unwrap();
        assert!(c.accepted_onboard_mission());
        let run = status(&c)["status"]["run_id"].clone();
        let issued_revision = status(&c)["status"]["revision"].clone();
        let held_goal=serde_json::json!({"frame":"local","x":9,"y":0,"token":"never-delivered","run_id":run,"authority_revision":issued_revision}).to_string();
        let held_reset=serde_json::json!({"action":"reset","token":"old-reset","run_id":run,"authority_revision":issued_revision}).to_string();
        cloud.close().wait().unwrap();
        let deadline = Instant::now() + Duration::from_secs(5);
        while c.dashboard_status() != "failed" {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(10));
        }
        // The first native tick sees failed transport before Swift handles it.
        fresh_local(&c, 1.01);
        c.step(1.01).unwrap();
        assert_ne!(status(&c)["status"]["reason"], "sensor_unhealthy");
        c.handle_dashboard_link_loss().unwrap();
        for tick in 1..=1200 {
            let now = 1. + tick as f64 * 0.1;
            fresh_local(&c, now);
            c.step(now).unwrap();
            assert_eq!(status(&c)["goal"]["token"], "accepted");
            assert_ne!(status(&c)["status"]["reason"], "sensor_unhealthy");
        }
        assert_eq!(status(&c)["status"]["run_id"], run);
        let replacement = router(&endpoint);
        c.reconnect_dashboard(endpoint, "auto-loss".into(), 0)
            .unwrap();
        replacement
            .put("auto-loss/0/goal", held_goal)
            .wait()
            .unwrap();
        std::thread::sleep(Duration::from_millis(50));
        fresh_local(&c, 121.05);
        c.step(121.05).unwrap();
        assert_eq!(status(&c)["goal"]["token"], "accepted");
        assert_eq!(status(&c)["status"]["run_id"], run);
        assert!(
            c.set_connectivity_policy_settings(r#"{"gps_display_freshness_seconds":0}"#.into())
                .is_err()
        );
        c.set_connectivity_policy_settings(r#"{"autonomous_router_loss":"pause"}"#.into())
            .unwrap();
        c.handle_dashboard_link_loss().unwrap();
        fresh_local(&c, 121.1);
        let stopped = c.step(121.1).unwrap();
        assert_eq!((stopped.left_effort, stopped.right_effort), (0., 0.));
        assert!(!c.accepted_onboard_mission());
        c.set_connectivity_policy_settings(terra_mobile::default_connectivity_policy_settings())
            .unwrap();
        fresh_local(&c, 121.2);
        c.step(121.2).unwrap();
        assert!(
            !c.accepted_onboard_mission(),
            "policy changes never resume a pause"
        );
        c.autonomy_request(
            "safety".into(),
            r#"{"action":"stop","token":"new-stop"}"#.into(),
            121.3,
        )
        .unwrap();
        replacement
            .put("auto-loss/0/safety", held_reset)
            .wait()
            .unwrap();
        std::thread::sleep(Duration::from_millis(50));
        fresh_local(&c, 121.4);
        c.step(121.4).unwrap();
        assert_eq!(status(&c)["status"]["safety"], "emergency_stop");
        assert_eq!(status(&c)["goal"]["state"], "idle");
        c.disconnect_dashboard().unwrap();
        assert!(!c.accepted_onboard_mission());
    }
}

#[test]
fn router_loss_does_not_mask_local_sensor_and_estop_stops() {
    let c = MobileController::new(default_control_settings()).unwrap();
    fresh_local(&c, 1.);
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"waypoint","token":"mode"}"#.into(),
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
    c.handle_dashboard_link_loss().unwrap();
    let stale = c.step(2.).unwrap();
    assert_eq!((stale.left_effort, stale.right_effort), (0., 0.));
    assert_eq!(status(&c)["status"]["reason"], "sensor_unhealthy");
    fresh_local(&c, 2.1);
    c.autonomy_request(
        "safety".into(),
        r#"{"action":"stop","token":"stop"}"#.into(),
        2.1,
    )
    .unwrap();
    c.step(2.1).unwrap();
    assert_eq!(status(&c)["status"]["safety"], "emergency_stop");
    c.handle_dashboard_link_loss().unwrap();
    fresh_local(&c, 2.2);
    c.step(2.2).unwrap();
    assert!(!c.accepted_onboard_mission());
    assert_eq!(status(&c)["status"]["safety"], "emergency_stop");
}

#[test]
fn exploration_failed_transport_tick_preserves_budget_without_sensor_failure() {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let cloud = router(&endpoint);
    let c = MobileController::new(default_control_settings()).unwrap();
    c.connect_dashboard(endpoint, "explore-loss".into(), 0)
        .unwrap();
    let observe = |t| {
        fresh_local(&c, t);
        let mut cells = vec![-1; 6400];
        for y in 24..56 {
            for x in 24..56 {
                cells[y * 80 + x] = 0;
            }
        }
        c.autonomy_map(
            terra_mobile::OccupancyGrid {
                width: 80,
                height: 80,
                resolution: 0.25,
                origin_x: -10.,
                origin_y: -10.,
                occupancy: cells,
            },
            t,
            2,
        )
        .unwrap();
    };
    observe(1.);
    c.step(1.).unwrap();
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"explore","budget_seconds":2,"token":"explore"}"#.into(),
        1.,
    )
    .unwrap();
    c.step(1.).unwrap();
    assert!(c.accepted_onboard_mission());
    cloud.close().wait().unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while c.dashboard_status() != "failed" {
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(10));
    }
    observe(1.1);
    c.step(1.1).unwrap();
    assert_ne!(status(&c)["exploration"]["end_reason"], "safety_hold");
    assert!(c.accepted_onboard_mission());
    c.handle_dashboard_link_loss().unwrap();
    for tick in 2..=21 {
        let t = 1. + tick as f64 * 0.1;
        observe(t);
        c.step(t).unwrap();
    }
    assert_eq!(status(&c)["exploration"]["end_reason"], "budget_expired");
    assert!(!c.accepted_onboard_mission());
}

/// A one-tick sensor hold stays readable: the status names what went stale
/// and how old the pose was, for a few seconds after the hold clears.
#[test]
fn sensor_hold_reports_its_cause_after_it_clears() {
    let c = MobileController::new(default_control_settings()).unwrap();
    fresh_local(&c, 1.);
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"waypoint","token":"mode"}"#.into(),
        1.,
    )
    .unwrap();
    c.step(1.).unwrap();
    assert!(status(&c)["status"].get("sensor_hold").is_none());
    c.step(1.5).unwrap();
    let held = status(&c)["status"].clone();
    assert_eq!(held["reason"], "sensor_unhealthy");
    let cause = held["sensor_hold"]["cause"].as_str().unwrap().to_string();
    assert!(cause.starts_with("stale_"), "{cause}");
    assert!((held["sensor_hold"]["pose_age_s"].as_f64().unwrap() - 0.5).abs() < 1e-9);
    assert_eq!(held["sensor_hold"]["count"], 1);
    fresh_local(&c, 2.);
    c.step(2.).unwrap();
    let after = status(&c)["status"].clone();
    assert_ne!(after["reason"], "sensor_unhealthy");
    assert_eq!(after["sensor_hold"]["cause"], cause.as_str());
    assert!((after["sensor_hold"]["age_s"].as_f64().unwrap() - 0.5).abs() < 1e-9);
    fresh_local(&c, 12.);
    c.step(12.).unwrap();
    assert!(status(&c)["status"].get("sensor_hold").is_none());
}

/// The per-tick brief keeps the fields the Swift loop reads and drops the
/// L3 planner trace, which made the full status about 30 KB.
#[test]
fn autonomy_brief_is_small_and_matches_the_full_status() {
    let c = MobileController::new(default_control_settings()).unwrap();
    fresh_local(&c, 1.);
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"waypoint","token":"mode"}"#.into(),
        1.,
    )
    .unwrap();
    c.step(1.).unwrap();
    let full = status(&c);
    let brief: serde_json::Value = serde_json::from_str(&c.autonomy_brief()).unwrap();
    for key in [
        "run_id",
        "requested_level",
        "effective_level",
        "safety",
        "reason",
    ] {
        assert_eq!(brief["status"][key], full["status"][key], "{key}");
    }
    assert_eq!(brief["pose_quality"], full["pose_quality"]);
    assert_eq!(brief["goal"]["state"], full["goal"]["state"]);
    assert!(brief["status"].get("planner_trace").is_none());
    assert!(c.autonomy_brief().len() < 1024);
}
