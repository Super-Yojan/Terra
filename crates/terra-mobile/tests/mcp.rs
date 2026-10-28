//! The rover's MCP server on the phone facade: JSON-RPC, the on-phone agent loop, and session control.
use serde_json::{Value, json};
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

fn tick(c: &MobileController, t: f64) {
    sensors(c, t);
    c.step(t).unwrap();
}

fn rpc(c: &MobileController, session: &str, id: u64, method: &str, params: Value) -> Value {
    let reply = c.mcp_handle(
        session.into(),
        json!({"jsonrpc":"2.0","id":id,"method":method,"params":params}).to_string(),
    );
    serde_json::from_str(&reply).unwrap()
}

fn call(c: &MobileController, session: &str, name: &str, arguments: Value) -> Value {
    rpc(
        c,
        session,
        9,
        "tools/call",
        json!({"name":name,"arguments":arguments}),
    )["result"]
        .clone()
}

fn status(c: &MobileController) -> Value {
    serde_json::from_str(&c.autonomy_status()).unwrap()
}

/// A rover at the `task` level running an MCP task, with the on-phone agent attached.
fn agent_task() -> std::sync::Arc<MobileController> {
    let c = MobileController::new(default_control_settings()).unwrap();
    c.set_task_agent(Some("qwen3.5-2b".into()), true).unwrap();
    tick(&c, 1.);
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"task","token":"l5","task":{"task_id":"t1","instruction":"Explore the room.","agent":"mcp"}}"#
            .into(),
        1.05,
    )
    .unwrap();
    tick(&c, 1.1);
    c
}

#[test]
fn speaks_mcp_json_rpc() {
    let c = agent_task();
    let init = rpc(
        &c,
        "http-1",
        1,
        "initialize",
        json!({"protocolVersion":"2025-06-18"}),
    );
    assert_eq!(init["result"]["protocolVersion"], "2025-06-18");
    assert_eq!(init["result"]["serverInfo"]["name"], "terra-rover");
    assert_eq!(
        c.mcp_handle(
            "http-1".into(),
            r#"{"jsonrpc":"2.0","method":"notifications/initialized"}"#.into()
        ),
        ""
    );
    let tools = rpc(&c, "http-1", 2, "tools/list", json!({}))["result"]["tools"].clone();
    let names: Vec<_> = tools
        .as_array()
        .unwrap()
        .iter()
        .map(|t| t["name"].as_str().unwrap())
        .collect();
    for name in ["get_map", "go_to", "request_drive", "drive", "stop"] {
        assert!(names.contains(&name), "{names:?}");
    }
    assert!(
        !names
            .iter()
            .any(|n| n.contains("motor") || *n == "set_autonomy_level")
    );
    assert_eq!(
        rpc(&c, "http-1", 3, "bogus", json!({}))["error"]["code"],
        -32601
    );
    assert_eq!(
        serde_json::from_str::<Value>(&c.mcp_handle("x".into(), "not json".into())).unwrap()["error"]
            ["code"],
        -32700
    );

    let state = call(&c, "http-1", "get_state", json!({}));
    assert_eq!(state["isError"], false);
    let state: Value = serde_json::from_str(state["content"][0]["text"].as_str().unwrap()).unwrap();
    assert_eq!(state["task_phase"], "planning");
    assert_eq!(state["task_agent"], "mcp");
}

#[test]
fn the_on_phone_agent_loop_runs_tools_and_waits_while_the_rover_drives() {
    let c = agent_task();
    let first = c
        .take_agent_prompt()
        .expect("a prompt for the on-phone agent");
    assert!(first.system.contains("<tool_call>"));
    assert!(first.system.contains("go_to"));
    assert!(first.prompt.contains("Explore the room."));
    assert!(c.take_agent_prompt().is_none(), "one prompt at a time");
    assert!(!c.submit_agent_reply(first.prompt_id + 7, "{}".into(), 0.1));

    assert!(c.submit_agent_reply(
        first.prompt_id,
        "<tool_call>{\"name\": \"get_map\", \"arguments\": {}}</tool_call>".into(),
        2.0,
    ));
    let second = c.take_agent_prompt().unwrap();
    assert!(second.prompt.contains("get_map"), "{}", second.prompt);
    assert!(second.prompt.contains("F1"), "{}", second.prompt);

    // `look` attaches the camera frame to the next prompt.
    c.submit_agent_reply(
        second.prompt_id,
        r#"{"name":"look","arguments":{}}"#.into(),
        1.0,
    );
    let third = c.take_agent_prompt().unwrap();
    assert!(third.attach_image);

    c.submit_agent_reply(
        third.prompt_id,
        r#"<tool_call>{"name":"go_to","arguments":{"waypoints":[{"target":"F1"}]}}</tool_call>"#
            .into(),
        1.5,
    );
    tick(&c, 1.2);
    let v = status(&c);
    assert_eq!(v["task"]["phase"], "navigating", "{}", v["task"]);
    assert_eq!(v["task"]["agent"], "mcp");
    // No new prompt while the waypoint plan is running.
    assert!(c.take_agent_prompt().is_none());

    // Another session cannot take over the task the on-phone agent holds.
    let refused = call(
        &c,
        "http-1",
        "go_to",
        json!({"waypoints":[{"target":"F1"}]}),
    );
    assert_eq!(refused["isError"], true);
    assert!(
        refused["content"][0]["text"]
            .as_str()
            .unwrap()
            .contains("another agent")
    );

    // `stop` is always allowed and ends the task.
    assert_eq!(call(&c, "http-1", "stop", json!({}))["isError"], false);
    tick(&c, 1.3);
    let v = status(&c);
    assert_eq!(v["task"]["phase"], "cancelled");
    assert_eq!(v["twist"]["linear"], 0.);
}

#[test]
fn three_replies_without_a_tool_call_fail_the_task() {
    let c = agent_task();
    for _ in 0..3 {
        let p = c.take_agent_prompt().unwrap();
        c.submit_agent_reply(p.prompt_id, "I think I should explore.".into(), 0.5);
    }
    tick(&c, 1.2);
    let v = status(&c);
    assert_eq!(v["task"]["phase"], "failed", "{}", v["task"]);
    assert!(c.take_agent_prompt().is_none());
}

#[test]
fn remote_look_returns_a_recent_camera_frame() {
    let c = agent_task();
    let none = call(&c, "http-1", "look", json!({}));
    assert_eq!(none["isError"], true);
    c.set_look_frame(b"jpeg".to_vec(), 1.1).unwrap();
    let image = call(&c, "http-1", "look", json!({}));
    assert_eq!(image["content"][0]["type"], "image");
    assert_eq!(image["content"][0]["data"], "anBlZw==");
}

#[test]
fn write_tools_refuse_without_an_mcp_task() {
    let c = MobileController::new(default_control_settings()).unwrap();
    tick(&c, 1.);
    let refused = call(
        &c,
        "http-1",
        "go_to",
        json!({"waypoints":[{"target":"F1"}]}),
    );
    assert_eq!(refused["isError"], true);
    assert!(
        refused["content"][0]["text"]
            .as_str()
            .unwrap()
            .starts_with("not_in_control")
    );
    assert!(c.take_agent_prompt().is_none());
}

#[test]
fn confirmed_detections_become_landmarks_the_agent_can_drive_to() {
    let c = agent_task();
    for frame in 0..3 {
        c.push_object_detection(
            MobileObjectDetection {
                class_name: "bucket".into(),
                color: Some("blue".into()),
                confidence: 0.9,
                frame_id: frame,
                received_at: 1.1,
                world_x: 2.,
                world_y: 1.,
            },
            1.1,
        )
        .unwrap();
    }
    let detections = call(&c, "http-1", "get_detections", json!({}));
    assert!(
        detections["content"][0]["text"]
            .as_str()
            .unwrap()
            .contains("bucket")
    );
    let map = call(&c, "http-1", "get_map", json!({}));
    let text = map["content"][0]["text"].as_str().unwrap();
    assert!(text.contains("blue bucket"), "{text}");
    let go = call(
        &c,
        "http-1",
        "go_to",
        json!({"waypoints":[{"target":"L1"}]}),
    );
    assert_eq!(go["isError"], false, "{go}");
    tick(&c, 1.2);
    assert_eq!(status(&c)["task"]["plan"][0]["label"], "L1");
}
