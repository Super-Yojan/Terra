use std::time::Duration;
use terra_mobile::{MobileController, OccupancyGrid, default_control_settings};
use zenoh::Wait;

#[test]
fn oversized_map_never_heartbeats_until_a_baseline_is_accepted() {
    let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/127.0.0.1:{}", socket.local_addr().unwrap().port());
    drop(socket);
    let controller = MobileController::new(default_control_settings()).unwrap();
    controller
        .host_dashboard(endpoint.clone(), "map-rejection".into(), 7)
        .unwrap();
    let mut config = zenoh::Config::default();
    config
        .insert_json5(
            "connect/endpoints",
            &serde_json::json!([endpoint]).to_string(),
        )
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    let client = zenoh::open(config).wait().unwrap();
    let sub = client
        .declare_subscriber("map-rejection/7/map/occupancy/**")
        .wait()
        .unwrap();
    let publish = |width, sequence, timestamp| {
        controller
            .autonomy_map(
                OccupancyGrid {
                    width,
                    height: width,
                    resolution: 0.1,
                    origin_x: 0.,
                    origin_y: 0.,
                    occupancy: vec![100; width as usize * width as usize],
                },
                timestamp,
                sequence,
            )
            .unwrap();
        controller.step(timestamp).unwrap();
    };
    // Valid onboard dimensions, but incompatible with dashboard visualization.
    publish(1000, 1, 1.);
    std::thread::sleep(Duration::from_millis(220));
    publish(1000, 2, 1.2);
    assert!(
        sub.recv_timeout(Duration::from_millis(250))
            .unwrap()
            .is_none()
    );
    publish(2, 3, 1.4);
    let full = sub.recv_timeout(Duration::from_secs(2)).unwrap().unwrap();
    assert!(full.key_expr().as_str().ends_with("/map/occupancy"));
    let value: serde_json::Value = serde_json::from_slice(&full.payload().to_bytes()).unwrap();
    assert_eq!(value["sequence"], 3);
    std::thread::sleep(Duration::from_millis(220));
    publish(2, 4, 1.6);
    let heartbeat = sub.recv_timeout(Duration::from_secs(2)).unwrap().unwrap();
    assert!(
        heartbeat
            .key_expr()
            .as_str()
            .ends_with("/map/occupancy/heartbeat")
    );
    let value: serde_json::Value = serde_json::from_slice(&heartbeat.payload().to_bytes()).unwrap();
    assert_eq!(value["sequence"], 3);

    for (index, (width, height, resolution, cell)) in [
        (600, 500, 0.1, 0), // Below byte cap, above dashboard dimension/cell limits.
        (512, 500, 0.1, 0), // Dimensions fit, total cell count does not.
        (513, 1, 0.1, 0),
        (2, 2, 2.1, 0),
        (2, 2, 0.1, 101),
        (2, 2, 0.1, -2),
    ]
    .into_iter()
    .enumerate()
    {
        let sequence = index as u64 + 5;
        let timestamp = 2. + index as f64 * 0.2;
        let grid = OccupancyGrid {
            width,
            height,
            resolution,
            origin_x: 0.,
            origin_y: 0.,
            occupancy: vec![cell; width as usize * height as usize],
        };
        assert!(serde_json::to_vec(&grid.occupancy).unwrap().len() < 2_000_000);
        // Still admitted for onboard autonomy; only visualization is filtered.
        controller.autonomy_map(grid, timestamp, sequence).unwrap();
        controller.step(timestamp).unwrap();
        assert!(
            sub.recv_timeout(Duration::from_millis(150))
                .unwrap()
                .is_none()
        );
    }
    // Same content as the old baseline must re-baseline after invalid input.
    for (width, sequence, timestamp) in [(2, 20, 4.), (400, 21, 4.2), (500, 22, 4.4)] {
        publish(width, sequence, timestamp);
        let full = sub.recv_timeout(Duration::from_secs(2)).unwrap().unwrap();
        assert!(full.key_expr().as_str().ends_with("/map/occupancy"));
        let value: serde_json::Value = serde_json::from_slice(&full.payload().to_bytes()).unwrap();
        assert_eq!(value["sequence"], sequence);
        assert_eq!(value["width"], width);
    }
}
