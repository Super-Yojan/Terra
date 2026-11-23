//! terra-av's pipeline talking to a terra-comms base station through a local
//! Zenoh router.
#![cfg(feature = "zenoh")]

use std::net::TcpListener;
use std::thread::sleep;
use std::time::{Duration, Instant};

use terra_av::config::PipelineConfig;
use terra_av::geometry::{Rotation3, Vec3};
use terra_av::pipeline::Pipeline;
use terra_av::planning::Goal;
use terra_av::sense::{SensorFrame, VioPose};
use terra_comms::{
    BaseLink, Capabilities, CommsConfig, Depth, Estop, Params, Pose, Status, Task, TaskState, TaskStatus,
};
use zenoh::Wait;

const ROVER_ID: u64 = 4;

fn free_port() -> u16 {
    TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port()
}

fn router(port: u16) -> zenoh::Session {
    let mut c = zenoh::Config::default();
    c.insert_json5("mode", r#""router""#).unwrap();
    c.insert_json5("listen/endpoints", &format!(r#"["tcp/127.0.0.1:{port}"]"#)).unwrap();
    c.insert_json5("scouting/multicast/enabled", "false").unwrap();
    zenoh::open(c).wait().unwrap()
}

fn rover_pipeline(port: u16) -> Pipeline {
    let toml = format!(
        r#"
        [vehicle]
        type = "diff_drive"
        track = 0.5
        max_wheel_speed = 0.8
        [mount]
        roll = 0
        pitch = 0
        yaw = 0
        offset = [0.0, 0.0, 0.23]
        [estimator]
        type = "vio"
        [mapper]
        type = "none"
        [path_planner]
        type = "straight_line"
        [local_planner]
        type = "teleop"
        [comms]
        endpoint = "tcp/127.0.0.1:{port}"
        prefix = "test/{port}"
        rover_id = {ROVER_ID}
        "#
    );
    PipelineConfig::from_toml(&toml).unwrap().build(None).unwrap()
}

fn base(port: u16) -> BaseLink {
    let toml = format!("[comms]\nendpoint = \"tcp/127.0.0.1:{port}\"\nprefix = \"test/{port}\"\n");
    terra_comms::connect(&CommsConfig::from_toml(&toml).unwrap()).unwrap().base()
}

fn frame(t: f64) -> SensorFrame {
    let v = |x, y, z| Vec3 { x, y, z };
    SensorFrame {
        t,
        estop: false,
        vio: Some(VioPose {
            position: v(0.0, 0.0, 0.0),
            rotation: Rotation3::from_matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
            tracked: true,
        }),
        gyro: None,
        gps: None,
        depth: vec![v(0.1, -0.2, -1.5), v(0.3, 0.0, -2.0)],
    }
}

/// Tick the pipeline at 20 Hz until `f` returns something.
fn tick_until<T>(pipeline: &mut Pipeline, what: &str, mut f: impl FnMut() -> Option<T>) -> T {
    let start = Instant::now();
    loop {
        let out = pipeline.tick(frame(start.elapsed().as_secs_f64()), &Goal::None);
        assert_eq!((out.target.v, out.target.w), (0.0, 0.0), "with no goal and no task the rover holds still");
        if let Some(v) = f() {
            return v;
        }
        assert!(start.elapsed() < Duration::from_secs(10), "timed out waiting for {what}");
        sleep(Duration::from_millis(50));
    }
}

#[test]
fn pipeline_talks_to_the_base_station() {
    let port = free_port();
    let _router = router(port);
    let base = base(port);
    let mut rover = rover_pipeline(port);

    // Capabilities, pose and status reach the base station.
    let (caps, _) = tick_until(&mut rover, "capabilities", || base.latest::<Capabilities>(ROVER_ID));
    assert_eq!(caps.default_controller, "teleop");
    tick_until(&mut rover, "pose", || base.latest::<Pose>(ROVER_ID));
    tick_until(&mut rover, "rover listed", || base.rovers().contains(&ROVER_ID).then_some(()));
    let (status, _) = tick_until(&mut rover, "status", || base.latest::<Status>(ROVER_ID));
    assert!(!status.estopped);

    // Depth flows while the base subscribes, camera frame as f32.
    let (tx, rx) = std::sync::mpsc::channel();
    let _sub = base.subscribe::<Depth>(move |id, d| {
        let _ = tx.send((id, d));
    });
    let (id, depth) = tick_until(&mut rover, "depth", || rx.try_recv().ok());
    assert_eq!(id, ROVER_ID);
    assert_eq!(depth.points[0], [0.1, -0.2, -1.5]);

    // A remote e-stop reaches the pipeline (reported back in Status).
    base.send(ROVER_ID, &Estop { t: 0.0, engaged: true });
    tick_until(&mut rover, "estopped", || base.latest::<Status>(ROVER_ID).filter(|(s, _)| s.estopped));
    base.send(ROVER_ID, &Estop { t: 0.0, engaged: false });
    tick_until(&mut rover, "released", || base.latest::<Status>(ROVER_ID).filter(|(s, _)| !s.estopped));

    // A task its controller can't run is refused with a reason: `teleop`
    // is driven by a stick and takes no goal.
    base.send(
        ROVER_ID,
        &Task {
            id: 9,
            goal: terra_comms::Goal::Pose { x: 1.0, y: 0.0, yaw: None, tolerance: 0.2 },
            controller: "teleop".into(),
            params: Params::new(),
            planner: None,
            planner_params: Params::new(),
        },
    );
    let (status, _) = tick_until(&mut rover, "rejection", || {
        base.latest::<TaskStatus>(ROVER_ID).filter(|(s, _)| s.last_rejected.is_some())
    });
    assert_eq!(status.state, TaskState::Idle);
    let rejected = status.last_rejected.unwrap();
    assert_eq!(rejected.task_id, 9);
    assert!(rejected.reason.contains("takes no goal"), "{}", rejected.reason);
}

#[test]
fn build_returns_at_once_without_a_router() {
    let start = Instant::now();
    let mut rover = rover_pipeline(free_port());
    rover.tick(frame(0.0), &Goal::None);
    assert!(start.elapsed() < Duration::from_secs(1), "building and ticking must not wait for the network");
}
