//! L5 on the phone facade: Rust builds the query, the phone's model answers, Rust validates.
use terra_mobile::*;

/// Stationary tracked IMU/VIO at the origin and a 10 x 10 m free map with an unknown rim.
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
    for y in 4..36 {
        for x in 4..36 {
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

fn status(c: &MobileController) -> serde_json::Value {
    serde_json::from_str(&c.autonomy_status()).unwrap()
}

#[test]
fn l5_is_offered_only_with_a_phone_model_and_round_trips_a_query() {
    let c = MobileController::new(default_control_settings()).unwrap();
    sensors(&c, 1.);
    c.step(1.).unwrap();
    let levels = status(&c)["status"]["supported_levels"].clone();
    assert!(
        !levels
            .as_array()
            .unwrap()
            .contains(&serde_json::json!("task"))
    );
    let inline = r#"{"level":"task","token":"l5","task":{"task_id":"t1","instruction":"Look around the room."}}"#;
    assert!(
        c.autonomy_request("autonomy".into(), inline.into(), 1.05)
            .is_err()
    );

    c.set_task_model(Some("apple-foundation-models".into()))
        .unwrap();
    // A rejected token is never replayed, so the retry uses a new one.
    c.autonomy_request(
        "autonomy".into(),
        inline.replace("\"l5\"", "\"l5-retry\""),
        1.1,
    )
    .unwrap();
    sensors(&c, 1.2);
    c.step(1.2).unwrap();
    let v = status(&c);
    assert!(
        v["status"]["supported_levels"]
            .as_array()
            .unwrap()
            .contains(&serde_json::json!("task"))
    );
    assert_eq!(v["status"]["requested_level"], "task");
    assert_eq!(v["task"]["phase"], "planning");
    assert_eq!(v["task"]["model"], "apple-foundation-models");
    assert_eq!(v["status"]["task"]["task_id"], "t1");

    let query = c.take_task_query().expect("a query for the phone model");
    assert!(query.prompt.contains("Look around the room."));
    assert!(c.take_task_query().is_none());
    assert!(c.submit_task_response(
        query.query_id,
        r#"{"decision":"go","waypoints":[{"target":"F1"}],"rationale":"see more"}"#.into(),
        1.4,
    ));
    sensors(&c, 1.3);
    c.step(1.3).unwrap();
    let v = status(&c);
    assert_eq!(v["task"]["phase"], "navigating", "{}", v["task"]);
    assert_eq!(v["task"]["plan"][0]["label"], "F1");
    assert_eq!(v["task"]["last_latency_seconds"], 1.4);

    c.autonomy_request(
        "task/action".into(),
        r#"{"task_id":"t1","action":"cancel","token":"stop-task"}"#.into(),
        1.35,
    )
    .unwrap();
    sensors(&c, 1.4);
    c.step(1.4).unwrap();
    let v = status(&c);
    assert_eq!(v["task"]["phase"], "cancelled");
    assert_eq!(v["twist"]["linear"], 0.);
}
