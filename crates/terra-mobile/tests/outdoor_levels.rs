//! A GPS destination at L2, L4 explore and L5 task. L3 is covered in `outdoor.rs`.
//!
//! The simulated rover starts at latitude 0, longitude 0 with local +x pointing
//! true north, so local +y (left) is west.
use terra_mobile::*;

const M_PER_DEG: f64 = 111_195.;

fn status(c: &MobileController) -> serde_json::Value {
    serde_json::from_str(&c.autonomy_status()).unwrap()
}
fn hardware(c: &MobileController, output_active: bool) {
    c.publish_hardware_status(
        serde_json::json!({"schema_version":1,"control_mode":"streamed_relay","connected":true,"ready":true,"output_active":output_active,"fault":null})
            .to_string(),
    )
    .unwrap();
}
/// Tracked IMU and VIO at local `(x, y, yaw)`.
fn pose(c: &MobileController, t: f64, x: f64, y: f64, yaw: f64) {
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
    // The runtime takes VIO already in the rover frame: x forward, y left, z up.
    let (s, c2) = ((yaw / 2.).sin(), (yaw / 2.).cos());
    c.push_vio(VioReading {
        timestamp: t,
        position_x: x,
        position_y: y,
        position_z: 0.,
        quaternion_x: 0.,
        quaternion_y: 0.,
        quaternion_z: s,
        quaternion_w: c2,
        velocity_x: 0.,
        velocity_y: 0.,
        velocity_z: 0.,
        tracked: true,
    })
    .unwrap();
}
/// Compass heading for local yaw, in [0, 360). `rem_euclid` can return 360.0
/// for a tiny negative angle, which Terra rejects.
fn true_heading(yaw: f64) -> f64 {
    let h = (-yaw.to_degrees()).rem_euclid(360.);
    if h >= 360. { 0. } else { h }
}
/// GPS at the true position (offset by `noise_north` metres) and the true heading.
fn geo(c: &MobileController, t: f64, x: f64, y: f64, yaw: f64, noise_north: f64) {
    c.push_gps(
        GpsFix {
            timestamp: t,
            latitude: (x + noise_north) / M_PER_DEG,
            longitude: -y / M_PER_DEG,
            accuracy: noise_north.abs().max(2.),
        },
        t,
    )
    .unwrap();
    c.push_heading(
        HeadingFix {
            timestamp: t,
            true_heading: true_heading(yaw),
            accuracy: 3.,
        },
        t,
    )
    .unwrap();
}
/// A 20 x 20 m grid: seen free within `free` metres of the origin, unknown outside.
fn map(c: &MobileController, t: f64, free: f64) {
    let mut cells = vec![-1; 6400];
    for row in 0..80 {
        for col in 0..80 {
            let (x, y) = (
                -10. + (col as f64 + 0.5) * 0.25,
                -10. + (row as f64 + 0.5) * 0.25,
            );
            if x.abs() < free && y.abs() < free {
                cells[row * 80 + col] = 0;
            }
        }
    }
    c.autonomy_map(
        OccupancyGrid {
            width: 80,
            height: 80,
            resolution: 0.25,
            origin_x: -10.,
            origin_y: -10.,
            occupancy: cells,
        },
        t,
        1,
    )
    .unwrap();
}

struct Sim {
    c: std::sync::Arc<MobileController>,
    run: serde_json::Value,
    t: f64,
    pose: (f64, f64, f64),
    /// Free half-width of the map, or `None` to stop sending maps.
    free: Option<f64>,
    /// GPS error in metres north, flipping sign on every fix.
    noise: f64,
}
impl Sim {
    fn new(level: &str, free: Option<f64>) -> Self {
        let c = MobileController::new(default_control_settings()).unwrap();
        c.enable_outdoor_gps().unwrap();
        c.set_task_model(Some("test-model".into())).unwrap();
        let mut sim = Sim {
            c,
            run: serde_json::Value::Null,
            t: 1.,
            pose: (0., 0., 0.),
            free,
            noise: 0.,
        };
        sim.sense(true);
        hardware(&sim.c, false);
        sim.c.step(1.).unwrap();
        sim.run = status(&sim.c)["status"]["run_id"].clone();
        sim.c.autonomy_request("navigation/source".into(),serde_json::json!({"schema_version":1,"source":"outdoor_gps","token":"select","run_id":sim.run,"navigation_revision":0}).to_string(),1.).unwrap();
        hardware(&sim.c, true);
        let request = match level {
            "explore" => {
                serde_json::json!({"level":"explore","budget_seconds":300,"token":"level"})
            }
            "task" => {
                serde_json::json!({"level":"task","token":"level","task":{"task_id":"t1","instruction":"Drive to the GPS destination."}})
            }
            _ => serde_json::json!({"level":level,"token":"level"}),
        };
        sim.c
            .autonomy_request("autonomy".into(), request.to_string(), 1.)
            .unwrap();
        sim.tick(0.05);
        sim
    }
    fn sense(&self, gps: bool) {
        let (x, y, yaw) = self.pose;
        pose(&self.c, self.t, x, y, yaw);
        if gps {
            let sign = if (self.t.round() as i64) % 2 == 0 {
                1.
            } else {
                -1.
            };
            geo(&self.c, self.t, x, y, yaw, sign * self.noise);
        }
        if let Some(free) = self.free {
            map(&self.c, self.t, free);
        }
        if self.t > 1. {
            hardware(&self.c, true);
        }
    }
    /// Send a GPS goal `north` metres north and `west` metres west of the start.
    fn goal(&self, north: f64, west: f64, radius: f64) {
        self.c.autonomy_request("goal".into(),serde_json::json!({"frame":"wgs84","latitude":north/M_PER_DEG,"longitude":-west/M_PER_DEG,"token":"gps-goal","run_id":self.run,"navigation_revision":1,"arrival_radius_m":radius}).to_string(),self.t).unwrap();
    }
    fn tick(&mut self, dt: f64) -> ControlOutput {
        self.t += dt;
        // GPS once a second; pose and heading every tick.
        let gps = (self.t * 20.).round() as i64 % 20 == 0;
        self.sense(gps);
        if !gps {
            let yaw = self.pose.2;
            self.c
                .push_heading(
                    HeadingFix {
                        timestamp: self.t,
                        true_heading: true_heading(yaw),
                        accuracy: 3.,
                    },
                    self.t,
                )
                .unwrap();
        }
        let out = self.c.step(self.t).unwrap();
        self.pose.2 += out.target_yaw_rate * dt;
        self.pose.0 += out.target_forward * self.pose.2.cos() * dt;
        self.pose.1 += out.target_forward * self.pose.2.sin() * dt;
        out
    }
    fn run_until(&mut self, seconds: f64, done: impl Fn(&serde_json::Value) -> bool) -> bool {
        let end = self.t + seconds;
        while self.t < end {
            self.tick(0.05);
            if done(&status(&self.c)) {
                return true;
            }
        }
        false
    }
}

#[test]
fn l2_direct_drives_to_a_gps_goal_without_a_map() {
    // The map stops after start-up; L2 never needs it.
    let mut sim = Sim::new("waypoint_direct", Some(10.));
    sim.free = None;
    sim.goal(6., 2., 2.);
    let arrived = sim.run_until(60., |s| s["outdoor"]["phase"] == "arrived");
    let s = status(&sim.c);
    assert!(arrived, "{}", s["outdoor"]);
    let (x, y, _) = sim.pose;
    assert!((x - 6.).hypot(y - 2.) <= 2.5, "stopped at ({x:.2}, {y:.2})");
    assert_eq!(s["outdoor"]["goal_levels"][0], "waypoint_direct");
    assert!(s["outdoor"]["anchor"]["sigma_m"].as_f64().unwrap() <= 2.);
}

#[test]
fn l2_arrives_inside_three_metres_with_noisy_five_metre_gps() {
    // The field test stopped 4 m short under the old 10 m default radius. With
    // the 3 m default and the smoothed position, ±5 m fixes still land close.
    let mut sim = Sim::new("waypoint_direct", Some(10.));
    sim.free = None;
    sim.noise = 5.;
    sim.goal(10., 0., 3.);
    let arrived = sim.run_until(90., |s| s["outdoor"]["phase"] == "arrived");
    let s = status(&sim.c);
    assert!(arrived, "{}", s["outdoor"]);
    let (x, y, _) = sim.pose;
    assert!((x - 10.).hypot(y) <= 3.5, "stopped at ({x:.2}, {y:.2})");
    assert!(s["outdoor"]["remaining_m"].as_f64().unwrap() <= 3.);
}

#[test]
fn l4_explore_pulls_frontiers_toward_the_gps_destination() {
    // Seen free space 12 m across; the destination is 30 m north, 10 m west, off the map.
    let mut sim = Sim::new("explore", Some(6.));
    sim.goal(30., 10., 5.);
    sim.run_until(1., |_| false);
    let s = status(&sim.c);
    let local = &s["outdoor"]["destination_local"];
    assert!((local["x"].as_f64().unwrap() - 30.).abs() < 1., "{local}");
    assert!((local["y"].as_f64().unwrap() - 10.).abs() < 1., "{local}");
    // The first frontier is the north-west edge of seen space, facing the destination.
    let target = &s["exploration"]["target"];
    assert!(target["x"].as_f64().unwrap() > 4., "{target}");
    assert!(target["y"].as_f64().unwrap() > 0., "{target}");
    sim.run_until(25., |_| false);
    let s = status(&sim.c);
    assert_eq!(s["exploration"]["phase"], "running", "{}", s["exploration"]);
    assert!(sim.pose.0 > 3., "rover at {:?}", sim.pose);

    // Mirrored: a destination to the south-east picks the opposite corner.
    let mut sim = Sim::new("explore", Some(6.));
    sim.goal(-30., -10., 5.);
    sim.run_until(1., |_| false);
    let target = &status(&sim.c)["exploration"]["target"];
    assert!(target["x"].as_f64().unwrap() < -4., "{target}");
    assert!(target["y"].as_f64().unwrap() < 0., "{target}");
}

#[test]
fn l4_explore_ends_when_gps_confirms_arrival() {
    let mut sim = Sim::new("explore", Some(4.));
    sim.goal(3., 0., 2.);
    let done = sim.run_until(60., |s| s["exploration"]["phase"] == "ended");
    let s = status(&sim.c);
    assert!(done, "{} {}", s["exploration"], s["outdoor"]);
    assert_eq!(s["outdoor"]["phase"], "arrived");
    assert_eq!(s["exploration"]["end_reason"], "objective_complete");
}

#[test]
fn l5_task_replans_and_lists_the_gps_destination() {
    let mut sim = Sim::new("task", Some(4.));
    let first = sim.c.take_task_query().expect("first query");
    assert!(!first.prompt.contains("GPS DESTINATION"));
    sim.goal(20., 0., 3.);
    sim.tick(0.05);
    // The unanswered first query is dropped for one that names G1.
    assert!(
        !sim.c
            .submit_task_response(first.query_id, r#"{"decision":"done"}"#.into(), 0.5)
    );
    let query = sim
        .c
        .take_task_query()
        .expect("a query with the destination");
    assert!(query.prompt.contains("G1"), "{}", query.prompt);
    assert!(
        query.prompt.contains("not reachable yet"),
        "{}",
        query.prompt
    );
    assert!(sim.c.submit_task_response(
        query.query_id,
        r#"{"decision":"go","waypoints":[{"target":"G1"}]}"#.into(),
        0.5,
    ));
    let reached = sim.run_until(30., |s| {
        s["task"]["history"].as_array().is_some_and(|h| {
            h.iter().any(|e| {
                e["text"]
                    .as_str()
                    .is_some_and(|t| t.starts_with("reached G1"))
            })
        })
    });
    assert!(reached, "{}", status(&sim.c)["task"]);
    // G1 resolved to the seen point closest to the destination, at the map's north edge.
    assert!(sim.pose.0 > 2., "rover at {:?}", sim.pose);
}

#[test]
fn a_paused_mission_withdraws_the_destination_but_explore_keeps_going() {
    let mut sim = Sim::new("explore", Some(3.));
    sim.goal(25., 0., 5.);
    sim.tick(0.05);
    assert!(status(&sim.c)["outdoor"]["destination_local"].is_object());
    // GPS stops: the mission pauses and explore carries on in the local frame.
    let end = sim.t + 5.;
    while sim.t < end {
        sim.t += 0.05;
        let (x, y, yaw) = sim.pose;
        pose(&sim.c, sim.t, x, y, yaw);
        map(&sim.c, sim.t, 3.);
        sim.c.step(sim.t).unwrap();
    }
    let s = status(&sim.c);
    assert_eq!(s["outdoor"]["phase"], "paused");
    assert!(s["outdoor"]["destination_local"].is_null());
    assert_eq!(s["exploration"]["phase"], "running", "{}", s["exploration"]);
}
