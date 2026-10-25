//! Loopback point capture: nothing is sent until the dashboard asks, then
//! labelled full-resolution frames arrive at the requested rate.
//! Network scenarios require local TCP sockets and use bounded receipt waits.
use std::time::Duration;
use terra_mobile::{
    MappingDepthFrame, MobileController, MobileOccupancyMap, default_control_settings,
    default_occupancy_settings,
};
use terra_types::point_capture::*;
use zenoh::Wait;

fn depth_frame(timestamp: f64) -> MappingDepthFrame {
    // Upright camera 0.5 m up, looking along world +X; 4 × 3 pixels.
    MappingDepthFrame {
        timestamp,
        width: 4,
        height: 3,
        fx: 2.0,
        fy: 2.0,
        cx: 1.5,
        cy: 1.0,
        camera_x: 0.0,
        camera_y: 0.0,
        camera_z: 0.5,
        quaternion_x: -0.5,
        quaternion_y: 0.5,
        quaternion_z: -0.5,
        quaternion_w: 0.5,
        depth_metres: vec![
            3.0,
            3.0,
            f32::NAN,
            3.0,
            2.0,
            2.0,
            2.0,
            2.0,
            1.0,
            1.0,
            1.0,
            1.0,
        ],
    }
}

#[test]
fn capture_sends_labelled_frames_only_on_request() {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let controller = MobileController::new(default_control_settings()).unwrap();
    controller
        .host_dashboard(endpoint.clone(), "capture".into(), 7)
        .unwrap();
    let map = MobileOccupancyMap::new(default_occupancy_settings()).unwrap();
    controller.attach_occupancy_map(map.clone()).unwrap();
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"client\"").unwrap();
    config
        .insert_json5(
            "connect/endpoints",
            &serde_json::json!([endpoint]).to_string(),
        )
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    let dashboard = zenoh::open(config).wait().unwrap();
    let full = dashboard
        .declare_subscriber("capture/7/pointcloud/full")
        .wait()
        .unwrap();
    let thinned = dashboard
        .declare_subscriber("capture/7/pointcloud")
        .wait()
        .unwrap();
    let progress = dashboard
        .declare_subscriber("capture/7/pointcloud/capture")
        .wait()
        .unwrap();
    std::thread::sleep(Duration::from_millis(300));
    let cloud = |sequence: u64| {
        serde_json::json!({"version":1,"sequence":sequence,"frameID":"f","points":[[1.0,2.0,0.0]]})
            .to_string()
    };

    // Idle: depth and the thinned cloud go nowhere.
    let mut t = 1.0;
    for _ in 0..5 {
        map.integrate_depth(depth_frame(t)).unwrap();
        controller.publish_point_cloud(cloud(t as u64)).unwrap();
        controller.step(t).unwrap();
        t += 0.1;
    }
    assert!(
        full.recv_timeout(Duration::from_millis(300))
            .unwrap()
            .is_none()
    );
    assert!(
        thinned
            .recv_timeout(Duration::from_millis(100))
            .unwrap()
            .is_none()
    );

    // Two frames at 2 Hz.
    let request = CaptureRequest {
        duration_seconds: 1.0,
        rate_hz: 2.0,
        ..CaptureRequest::new("cap-1")
    };
    dashboard
        .put(
            "capture/7/pointcloud/request",
            serde_json::to_vec(&request).unwrap(),
        )
        .wait()
        .unwrap();
    std::thread::sleep(Duration::from_millis(200));
    let mut frames = Vec::new();
    let mut thinned_seen = false;
    for i in 0..30 {
        controller.step(t).unwrap();
        map.integrate_depth(depth_frame(t + 0.01)).unwrap();
        controller.publish_point_cloud(cloud(100 + i)).unwrap();
        std::thread::sleep(Duration::from_millis(30));
        while let Ok(Some(sample)) = full.try_recv() {
            frames.push(CaptureFrame::decode(&sample.payload().to_bytes()).unwrap());
        }
        thinned_seen |= thinned.try_recv().unwrap().is_some();
        t += 0.1;
    }
    while let Ok(Some(sample)) = full.recv_timeout(Duration::from_millis(500)) {
        frames.push(CaptureFrame::decode(&sample.payload().to_bytes()).unwrap());
    }
    assert!(thinned_seen, "thinned cloud should flow during a capture");
    assert_eq!(frames.len(), 2);
    let frame = &frames[1];
    assert_eq!(
        (
            frame.rover_id,
            frame.request_id.as_str(),
            frame.index,
            frame.count
        ),
        (7, "cap-1", 1, 2)
    );
    assert_eq!((frame.width, frame.height, frame.points.len()), (4, 3, 12));
    assert!(matches!(
        frame.classifier,
        CaptureClassifier::GroundPlane { .. }
    ));
    assert_eq!(frame.points[2].label, PointLabel::NoReturn);
    // The level row is 0.5 m up: an obstacle at 2 m.
    assert_eq!(frame.points[5].label, PointLabel::Obstacle);
    assert!((frame.points[5].position[0] - 2.0).abs() < 1e-5);

    let mut last = None;
    while let Ok(Some(sample)) = progress.recv_timeout(Duration::from_millis(300)) {
        last =
            Some(serde_json::from_slice::<CaptureProgress>(&sample.payload().to_bytes()).unwrap());
    }
    let last = last.expect("no progress");
    assert_eq!(last.state, CaptureState::Done);
    assert_eq!((last.frames_sent, last.frames_total), (2, 2));

    // Done: the thinned cloud stops again.
    std::thread::sleep(Duration::from_millis(600));
    while thinned.try_recv().unwrap().is_some() {}
    for i in 0..3 {
        controller.publish_point_cloud(cloud(500 + i)).unwrap();
        controller.step(t).unwrap();
        t += 0.1;
    }
    assert!(
        thinned
            .recv_timeout(Duration::from_millis(300))
            .unwrap()
            .is_none()
    );
}

#[test]
fn cancel_stops_a_running_capture() {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let controller = MobileController::new(default_control_settings()).unwrap();
    controller
        .host_dashboard(endpoint.clone(), "capture-cancel".into(), 7)
        .unwrap();
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"client\"").unwrap();
    config
        .insert_json5(
            "connect/endpoints",
            &serde_json::json!([endpoint]).to_string(),
        )
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    let dashboard = zenoh::open(config).wait().unwrap();
    let progress = dashboard
        .declare_subscriber("capture-cancel/7/pointcloud/capture")
        .wait()
        .unwrap();
    std::thread::sleep(Duration::from_millis(300));
    let put = |request: &CaptureRequest| {
        dashboard
            .put(
                "capture-cancel/7/pointcloud/request",
                serde_json::to_vec(request).unwrap(),
            )
            .wait()
            .unwrap();
        std::thread::sleep(Duration::from_millis(200));
    };
    put(&CaptureRequest::new("c"));
    controller.step(1.0).unwrap();
    put(&CaptureRequest {
        cancel: true,
        ..CaptureRequest::new("c")
    });
    controller.step(1.1).unwrap();
    let mut last = None;
    while let Ok(Some(sample)) = progress.recv_timeout(Duration::from_millis(300)) {
        last =
            Some(serde_json::from_slice::<CaptureProgress>(&sample.payload().to_bytes()).unwrap());
    }
    let last = last.unwrap();
    assert_eq!(last.state, CaptureState::Cancelled);
    assert_eq!(last.frames_sent, 0);
}

#[test]
fn live_stream_decimates_renews_and_lapses() {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let controller = MobileController::new(default_control_settings()).unwrap();
    controller
        .host_dashboard(endpoint.clone(), "capture-live".into(), 7)
        .unwrap();
    let map = MobileOccupancyMap::new(default_occupancy_settings()).unwrap();
    controller.attach_occupancy_map(map.clone()).unwrap();
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"client\"").unwrap();
    config
        .insert_json5(
            "connect/endpoints",
            &serde_json::json!([endpoint]).to_string(),
        )
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    let dashboard = zenoh::open(config).wait().unwrap();
    let full = dashboard
        .declare_subscriber("capture-live/7/pointcloud/full")
        .wait()
        .unwrap();
    let thinned = dashboard
        .declare_subscriber("capture-live/7/pointcloud")
        .wait()
        .unwrap();
    let progress = dashboard
        .declare_subscriber("capture-live/7/pointcloud/capture")
        .wait()
        .unwrap();
    std::thread::sleep(Duration::from_millis(300));
    let put = || {
        dashboard
            .put(
                "capture-live/7/pointcloud/request",
                serde_json::to_vec(&CaptureRequest::live("live-1", 5.0, 2, 1.0)).unwrap(),
            )
            .wait()
            .unwrap();
        std::thread::sleep(Duration::from_millis(200));
    };
    let cloud = |sequence: u64| {
        serde_json::json!({"version":1,"sequence":sequence,"frameID":"f","points":[[1.0,2.0,0.0]]})
            .to_string()
    };
    let mut frames = Vec::new();
    let mut thinned_seen = false;
    let mut t = 1.0;
    let mut run = |until: f64, t: &mut f64| {
        while *t < until {
            controller.step(*t).unwrap();
            map.integrate_depth(depth_frame(*t + 0.01)).unwrap();
            controller.publish_point_cloud(cloud(*t as u64)).unwrap();
            std::thread::sleep(Duration::from_millis(20));
            while let Ok(Some(sample)) = full.try_recv() {
                frames.push(CaptureFrame::decode(&sample.payload().to_bytes()).unwrap());
            }
            thinned_seen |= thinned.try_recv().unwrap().is_some();
            *t += 0.1;
        }
    };

    // A 1 s lease, renewed once at 1.6 s, keeps the stream up for about 2.6 s.
    put();
    run(1.6, &mut t);
    put();
    run(4.0, &mut t);
    while let Ok(Some(sample)) = full.recv_timeout(Duration::from_millis(500)) {
        frames.push(CaptureFrame::decode(&sample.payload().to_bytes()).unwrap());
    }
    assert!(
        !thinned_seen,
        "a live stream must not also send the thinned cloud"
    );
    assert!(
        (9..=15).contains(&frames.len()),
        "about 5 Hz for 2.6 s, got {}",
        frames.len()
    );
    assert!(frames.windows(2).all(|w| w[1].index == w[0].index + 1));
    let frame = &frames[0];
    assert_eq!(frame.count, 0);
    // 4 × 3 at stride 2 keeps pixels (0,0) (2,0) (0,2) (2,2).
    assert_eq!((frame.width, frame.height, frame.points.len()), (2, 2, 4));
    assert_eq!(frame.intrinsics, [1.0, 1.0, 0.75, 0.5]);
    assert_eq!(frame.points[1].label, PointLabel::NoReturn);
    assert_ne!(frame.points[0].label, PointLabel::NoReturn);
    // Source pixel (2,2), 1 m down the 0.5 m-high camera's lowest row.
    assert!((frame.points[3].position[0] - 1.0).abs() < 1e-5);

    let mut last = None;
    while let Ok(Some(sample)) = progress.recv_timeout(Duration::from_millis(300)) {
        last =
            Some(serde_json::from_slice::<CaptureProgress>(&sample.payload().to_bytes()).unwrap());
    }
    let last = last.expect("no progress");
    assert_eq!(last.state, CaptureState::Done);
    assert_eq!(last.detail, "lease_expired");
    assert_eq!(last.frames_total, 0);
}
