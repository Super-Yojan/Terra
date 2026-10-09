use terra_mobile::*;
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
