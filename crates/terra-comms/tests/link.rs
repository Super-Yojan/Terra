//! End to end through an in-process Zenoh router on localhost.
//!
//! Instead of sleeping for subscriptions to propagate, tests handshake: they
//! resend a probe until it arrives. The only fixed sleeps are where a test
//! checks that something does *not* happen, or needs time to pass.
#![cfg(feature = "zenoh")]

use std::net::TcpListener;
use std::sync::mpsc;
use std::thread::sleep;
use std::time::{Duration, Instant};

use terra_comms::*;
use zenoh::Wait;

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

fn config(port: u16, rover_id: Option<u64>) -> CommsConfig {
    let mut text = format!("[comms]\nendpoint = \"tcp/127.0.0.1:{port}\"\nprefix = \"test/{port}\"\n");
    if let Some(id) = rover_id {
        text += &format!("rover_id = {id}\n");
    }
    CommsConfig::from_toml(&text).unwrap()
}

fn rover(port: u16, id: u64) -> RoverLink {
    connect(&config(port, Some(id))).unwrap().rover().unwrap()
}

fn base(port: u16) -> BaseLink {
    connect(&config(port, None)).unwrap().base()
}

fn wait_for<T>(what: &str, mut f: impl FnMut() -> Option<T>) -> T {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        if let Some(v) = f() {
            return v;
        }
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        sleep(Duration::from_millis(20));
    }
}

fn pose(t: f64) -> Pose {
    Pose { t, x: 0.0, y: 0.0, yaw: 0.0, tracked: true }
}

/// Both directions flow between this rover and base.
fn handshake(rover: &RoverLink, base: &BaseLink) {
    let id = rover.rover_id();
    let mut t = 0.0;
    wait_for("handshake", || {
        t += 1.0;
        rover.publish(&pose(t));
        base.send(id, &Teleop { t, v: 0.0, w: 0.0 });
        (base.latest::<Pose>(id).is_some() && rover.teleop().is_some()).then_some(())
    });
}

fn status(t: f64) -> Status {
    Status { t, estopped: false, link_ok: true, detail: String::new(), gps_anchor: None }
}

#[test]
fn rover_and_base_talk() {
    let port = free_port();
    let _router = router(port);
    let rover = rover(port, 7);
    let base = base(port);
    handshake(&rover, &base);

    // rover → base
    let p = Pose { t: 100.0, x: 2.0, y: 3.0, yaw: 0.5, tracked: true };
    rover.publish(&p);
    let (got, age) = wait_for("pose", || base.latest::<Pose>(7).filter(|(p, _)| p.t == 100.0));
    assert_eq!(got, p);
    assert!(age < 1.0);
    assert_eq!(base.rovers(), vec![7]);

    // base → rover
    let task = Task {
        id: 1,
        goal: Goal::Pose { x: 1.0, y: 0.0, yaw: None, tolerance: 0.2 },
        controller: "dwa".into(),
        params: Params::new(),
        planner: None,
        planner_params: Params::new(),
    };
    base.send(7, &task);
    assert_eq!(wait_for("task", || rover.take_task()), task);
    assert_eq!(rover.take_task(), None, "each task is handed out once");

    base.send(7, &TaskCommand { task_id: 1, action: TaskAction::Pause });
    base.send(7, &TaskCommand { task_id: 1, action: TaskAction::Resume });
    assert_eq!(wait_for("cmd 1", || rover.take_task_command()).action, TaskAction::Pause);
    assert_eq!(wait_for("cmd 2", || rover.take_task_command()).action, TaskAction::Resume);

    // Teleop expires after COMMAND_LEASE.
    base.send(7, &Teleop { t: 0.0, v: 0.4, w: 0.0 });
    wait_for("teleop", || rover.teleop().filter(|t| t.v == 0.4));
    sleep(COMMAND_LEASE + Duration::from_millis(100));
    assert_eq!(rover.teleop(), None);

    base.send(7, &RemoteCmd(Teleop { t: 0.0, v: 0.1, w: 0.2 }));
    assert_eq!(wait_for("remote", || rover.remote_cmd()).w, 0.2);
    assert_eq!(rover.teleop(), None, "remote/cmd is not teleop");

    // E-stop is latched until explicitly released.
    assert!(!rover.estop());
    base.send(7, &Estop { t: 0.0, engaged: true });
    wait_for("estop", || rover.estop().then_some(()));
    base.send(7, &Estop { t: 0.0, engaged: false });
    wait_for("estop release", || (!rover.estop()).then_some(()));

    // Commands for another rover are not ours.
    base.send(8, &Estop { t: 0.0, engaged: true });
    sleep(Duration::from_millis(300));
    assert!(!rover.estop());
}

#[test]
fn depth_is_pulled_once_while_subscribed() {
    let port = free_port();
    let _router = router(port);
    let rover = rover(port, 4);
    let base = base(port);
    handshake(&rover, &base);

    let seen = |tx: mpsc::Sender<u64>| move |_: u64, d: Depth| tx.send(d.sequence).unwrap();
    let (tx_a, rx_a) = mpsc::channel();
    let (tx_b, rx_b) = mpsc::channel();
    let a = base.subscribe::<Depth>(seen(tx_a));
    let b = base.subscribe::<Depth>(seen(tx_b));
    let mut sequence = 0;
    wait_for("depth to both", || {
        sequence += 1;
        rover.publish(&Depth { t: 0.0, sequence, points: vec![[1.0, 2.0, 3.0]] });
        (base.latest::<Depth>(4).is_some() && rx_b.try_recv().is_ok()).then_some(())
    });
    sleep(Duration::from_millis(300)); // let in-flight frames land
    let frames_a = rx_a.try_iter().count();
    let frames_b = 1 + rx_b.try_iter().count();
    let received = base.stats()[&Topic::Depth].received as usize;
    assert_eq!(frames_a, frames_b);
    assert_eq!(received, frames_a, "each frame is counted once");

    drop(a);
    assert!(base.latest::<Depth>(4).is_some(), "still pulled by b");
    drop(b);
    assert!(base.latest::<Depth>(4).is_none(), "dropped with the last subscription");
}

#[test]
fn bad_payloads_are_counted_not_delivered() {
    let port = free_port();
    let router = router(port);
    let rover = rover(port, 1);
    let teleop = format!("test/{port}/1/teleop");
    wait_for("rover subscribed", || {
        router.put(&teleop, Teleop { t: 0.0, v: 0.0, w: 0.0 }.encode()).wait().unwrap();
        rover.teleop()
    });
    router.put(format!("test/{port}/1/task"), b"garbage".to_vec()).wait().unwrap();
    router.put(format!("test/{port}/1/task"), br#"{"version":99}"#.to_vec()).wait().unwrap();
    wait_for("bad payloads", || (rover.stats()[&Topic::Task].bad_payload == 2).then_some(()));
    assert_eq!(rover.take_task(), None);
}

#[test]
fn nothing_counts_as_sent_without_a_router() {
    let start = Instant::now();
    let link = connect(&config(free_port(), Some(1))).unwrap();
    assert!(start.elapsed() < Duration::from_secs(1));
    assert!(!link.connected());
    let rover = link.rover().unwrap();
    rover.publish(&pose(0.0));
    rover.publish(&pose(1.0));
    assert!(start.elapsed() < Duration::from_secs(1), "publish must not block");

    let base = base(free_port());
    base.send(1, &Estop { t: 0.0, engaged: true });
    base.send(1, &Teleop { t: 0.0, v: 0.0, w: 0.0 });
    sleep(Duration::from_millis(300));
    for (topic, link) in [(Topic::Pose, rover.stats()), (Topic::Estop, base.stats()), (Topic::Teleop, base.stats())] {
        assert_eq!(link[&topic].sent, 0, "{topic}");
    }
    assert_eq!(rover.stats()[&Topic::Pose].dropped, 1, "the first pose was replaced");
    assert!(base.stats()[&Topic::Estop].dropped >= 1);
}

#[test]
fn rover_needs_an_id() {
    assert!(connect(&config(free_port(), None)).unwrap().rover().is_err());
}

#[test]
fn estop_reaches_a_restarted_rover() {
    let port = free_port();
    let _router = router(port);
    let base = base(port);
    let first = rover(port, 2);
    handshake(&first, &base);
    base.send(2, &Estop { t: 0.0, engaged: true });
    wait_for("estop", || first.estop().then_some(()));

    // Restart: the new rover starts unlatched, then hears the repeat.
    drop(first);
    let second = rover(port, 2);
    assert!(!second.estop());
    wait_for("repeated estop", || second.estop().then_some(()));

    // A release goes out once and is not repeated.
    base.send(2, &Estop { t: 1.0, engaged: false });
    wait_for("release", || (!second.estop()).then_some(()));
    let sent = base.stats()[&Topic::Estop].sent;
    sleep(ESTOP_REPEAT * 2 + Duration::from_millis(200));
    assert!(!second.estop());
    assert_eq!(base.stats()[&Topic::Estop].sent, sent, "release must not repeat");
}

#[test]
fn two_bases_disagreeing_leave_the_rover_stopped() {
    let port = free_port();
    let _router = router(port);
    let rover = rover(port, 5);
    let a = base(port);
    let b = base(port);
    handshake(&rover, &a);
    handshake(&rover, &b);

    a.send(5, &Estop { t: 0.0, engaged: true });
    wait_for("engaged", || rover.estop().then_some(()));
    b.send(5, &Estop { t: 0.0, engaged: false });
    // B's single release may get through briefly; A's next repeat
    // re-latches within ESTOP_REPEAT, and after that nothing flips it back.
    sleep(ESTOP_REPEAT + Duration::from_millis(300));
    assert!(rover.estop(), "A's repeat didn't re-latch");

    let deadline = Instant::now() + ESTOP_REPEAT * 3;
    while Instant::now() < deadline {
        assert!(rover.estop(), "rover was released while a base held e-stop");
        sleep(Duration::from_micros(500));
    }

    // Only A can release its own hold.
    a.send(5, &Estop { t: 1.0, engaged: false });
    wait_for("released by a", || (!rover.estop()).then_some(()));
}

#[test]
fn repeats_do_not_look_like_a_live_rover() {
    let port = free_port();
    let _router = router(port);
    let base = base(port);
    let rover = rover(port, 3);
    handshake(&rover, &base);

    // The tick publishes once, then "hangs"; the link keeps repeating.
    rover.publish(&status(5.0));
    wait_for("status", || base.latest::<Status>(3));
    sleep(ROVER_SEEN_WINDOW + Duration::from_millis(500));

    assert!(base.stats()[&Topic::Status].received >= 3, "repeats arrived");
    let (_, age) = base.latest::<Status>(3).unwrap();
    assert!(age > ROVER_SEEN_WINDOW.as_secs_f64(), "age {age} was reset by a repeat");
    assert!(base.rovers().is_empty(), "a hung rover is still listed");

    // A base that starts now sees the same thing from the first repeat.
    let late = self::base(port);
    let (_, age) = wait_for("late status", || late.latest::<Status>(3));
    assert!(age > ROVER_SEEN_WINDOW.as_secs_f64(), "late joiner thinks age is {age}");
    assert!(late.rovers().is_empty(), "late joiner lists a hung rover");

    // A new message from the tick counts again.
    rover.publish(&status(6.0));
    let (_, age) = wait_for("fresh status", || base.latest::<Status>(3).filter(|(s, _)| s.t == 6.0));
    assert!(age < 1.0);
    assert_eq!(base.rovers(), vec![3]);
}

#[test]
fn a_task_replaced_before_the_tick_is_counted() {
    let port = free_port();
    let _router = router(port);
    let rover = rover(port, 6);
    let base = base(port);
    handshake(&rover, &base);
    let task = |id| Task {
        id,
        goal: Goal::Stop,
        controller: "dwa".into(),
        params: Params::new(),
        planner: None,
        planner_params: Params::new(),
    };
    base.send(6, &task(1));
    base.send(6, &task(2));
    wait_for("both tasks", || (rover.stats()[&Topic::Task].received == 2).then_some(()));
    assert_eq!(rover.take_task().map(|t| t.id), Some(2));
    assert_eq!(rover.stats()[&Topic::Task].dropped, 1);
}
