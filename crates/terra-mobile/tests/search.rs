//! Phone search capability and detector-evidence confirmation scenarios.
use terra_mobile::*;
/// Feed stationary tracked IMU/VIO and a 40×40 map at `t` seconds. The map uses
/// 0.25 m cells with a central observed free square and unknown surrounding cells.
fn sensors(c: &MobileController, t: f64) {
    c.push_imu(ImuReading {
        timestamp: t,
        acceleration_forward: 0.,
        acceleration_left: 0.,
        acceleration_up: 0.,
        gyro_roll: 0.,
        gyro_pitch: 0.,
        gyro_yaw: 0.,
    })
    .unwrap();
    c.push_vio(VioReading {
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
    let mut cells = vec![-1; 1600];
    for y in 10..30 {
        for x in 10..30 {
            cells[y * 40 + x] = 0;
        }
    }
    c.autonomy_map(
        OccupancyGrid {
            width: 40,
            height: 40,
            resolution: 0.25,
            origin_x: -5.,
            origin_y: -5.,
            occupancy: cells,
        },
        t,
        1,
    )
    .unwrap();
}
#[test]
/// Checks target search is absent without a detector and invalid detector class metadata is rejected.
fn phone_without_detector_does_not_advertise_l4() {
    let c = MobileController::new(default_control_settings()).unwrap();
    sensors(&c, 1.);
    c.step(1.).unwrap();
    let v: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
    assert!(
        !v["status"]["supported_levels"]
            .as_array()
            .unwrap()
            .contains(&serde_json::json!("target_search"))
    );
    assert!(
        c.register_search_detector(vec!["invalid class".into()], "test".into(), 1.)
            .is_err()
    );
}
#[test]
/// Registers a detector, starts a bounded search, submits three distinct corroborating frames,
/// and checks completion, report frame identities, and zero selected forward motion.
fn mobile_search_replays_confirmation_and_exports_report() {
    let c = MobileController::new(default_control_settings()).unwrap();
    sensors(&c, 1.);
    c.step(1.).unwrap();
    let v: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
    let run = v["status"]["run_id"].as_str().unwrap().to_string();
    c.register_search_detector(vec!["survivor".into()], "sim-test".into(), 1.1)
        .unwrap();
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"target_search","token":"mode"}"#.into(),
        1.1,
    )
    .unwrap();
    sensors(&c, 1.1);
    c.start_target_search(
        MobileSearchRequest {
            run_id: run.clone(),
            search_id: "search".into(),
            token: "start".into(),
            target_class: "survivor".into(),
            min_x: -4.,
            min_y: -4.,
            max_x: 4.,
            max_y: 4.,
            time_budget_s: 60.,
        },
        1.1,
    )
    .unwrap();
    for (frame, t) in [(1, 1.2), (2, 1.5), (3, 1.8)] {
        sensors(&c, t);
        c.register_search_detector(vec!["survivor".into()], "sim-test".into(), t)
            .unwrap();
        c.push_target_observation(
            MobileTargetObservation {
                run_id: run.clone(),
                search_id: "search".into(),
                frame_id: frame,
                target_class: "survivor".into(),
                confidence: 0.9,
                world_x: 2.,
                world_y: 1.,
                received_at: t,
                evidence_id: format!("frame-{frame}"),
            },
            t,
        )
        .unwrap();
    }
    c.step(1.9).unwrap();
    let v: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
    assert_eq!(v["search"]["phase"], "completed");
    assert_eq!(
        v["search"]["report"]["frame_ids"],
        serde_json::json!([1, 2, 3])
    );
    assert_eq!(v["twist"]["linear"], 0.);
}

#[test]
fn offline_search_preserves_identity_and_budget() {
    let c = MobileController::new(default_control_settings()).unwrap();
    sensors(&c, 1.);
    c.step(1.).unwrap();
    let root: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
    let run = root["status"]["run_id"].as_str().unwrap().to_owned();
    c.register_search_detector(vec!["survivor".into()], "offline".into(), 1.)
        .unwrap();
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"target_search","token":"mode"}"#.into(),
        1.,
    )
    .unwrap();
    c.start_target_search(
        MobileSearchRequest {
            run_id: run.clone(),
            search_id: "offline".into(),
            token: "start".into(),
            target_class: "survivor".into(),
            min_x: -4.,
            min_y: -4.,
            max_x: 4.,
            max_y: 4.,
            time_budget_s: 120.,
        },
        1.,
    )
    .unwrap();
    assert!(c.accepted_onboard_mission());
    c.handle_dashboard_link_loss().unwrap();
    for tick in 1..=1200 {
        let t = 1. + tick as f64 * 0.1;
        sensors(&c, t);
        c.register_search_detector(vec!["survivor".into()], "offline".into(), t)
            .unwrap();
        c.step(t).unwrap();
        let root: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
        assert_eq!(root["search"]["run_id"], run);
        assert_ne!(root["search"]["reason"], "sensor_unhealthy");
    }
    let root: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
    assert_ne!(
        root["search"]["phase"], "searching",
        "budget or natural exhaustion ends the search"
    );
    assert!(!c.accepted_onboard_mission());
}

#[test]
fn offline_detector_loss_still_requires_attention() {
    let c = MobileController::new(default_control_settings()).unwrap();
    sensors(&c, 1.);
    c.step(1.).unwrap();
    let root: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
    let run = root["status"]["run_id"].as_str().unwrap().to_owned();
    c.register_search_detector(vec!["survivor".into()], "offline".into(), 1.)
        .unwrap();
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"target_search","token":"mode"}"#.into(),
        1.,
    )
    .unwrap();
    c.start_target_search(
        MobileSearchRequest {
            run_id: run.clone(),
            search_id: "offline".into(),
            token: "start".into(),
            target_class: "survivor".into(),
            min_x: -4.,
            min_y: -4.,
            max_x: 4.,
            max_y: 4.,
            time_budget_s: 120.,
        },
        1.,
    )
    .unwrap();
    assert!(c.accepted_onboard_mission());
    c.handle_dashboard_link_loss().unwrap();
    sensors(&c, 2.);
    c.step(2.).unwrap();
    let root: serde_json::Value = serde_json::from_str(&c.autonomy_status()).unwrap();
    assert_eq!(root["search"]["phase"], "needs_attention");
    assert!(!c.accepted_onboard_mission());
}
