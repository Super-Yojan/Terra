use terra_mobile::outdoor::{
    CourseFix, Evidence, GpsFix, HeadingFix, OutdoorController, distance_bearing,
};
use terra_navigation::Pose;
fn evidence() -> Evidence {
    Evidence {
        pose: Pose {
            x: 12.,
            y: -9.,
            yaw: 0.7,
        },
        pose_ready: true,
        map_ready: true,
        hardware_ready: true,
        authority: true,
    }
}
fn sensors(c: &mut OutdoorController, t: f64, lat: f64) {
    sensors_at(c, t, lat, 0., evidence().pose);
}
fn sensors_at(c: &mut OutdoorController, t: f64, lat: f64, heading: f64, pose: Pose) {
    c.observe_pose(pose, t);
    c.gps(
        GpsFix {
            timestamp: t,
            latitude: lat,
            longitude: 0.,
            accuracy: 2.,
        },
        t,
    )
    .unwrap();
    c.heading(
        HeadingFix {
            timestamp: t,
            true_heading: heading,
            accuracy: 3.,
        },
        t,
    )
    .unwrap();
}
#[test]
fn cardinal_guidance_and_no_geographic_transform() {
    for (lat, lon, bearing) in [
        (0.001, 0., 0.),
        (0., 0.001, 90.),
        (-0.001, 0., 180.),
        (0., -0.001, 270.),
    ] {
        let (d, b) = distance_bearing(0., 0., lat, lon);
        assert!((d - 111.195).abs() < 0.1);
        assert!((b - bearing).abs() < 0.001);
        let mut mission = OutdoorController::default();
        sensors_at(&mut mission, 1.1, 0., 270., evidence().pose);
        mission
            .start(lat, lon, "cardinal".into(), 5., 1.1, evidence())
            .unwrap();
        let target = mission.tick(1.1, evidence()).unwrap();
        let yaw = 0.7 + (270. - bearing).to_radians();
        assert!((target.0 - 12. - 3. * yaw.cos()).abs() < 1e-8);
        assert!((target.1 + 9. - 3. * yaw.sin()).abs() < 1e-8);
    }
    let mut c = OutdoorController::default();
    sensors(&mut c, 1., 0.);
    c.start(0.001, 0., "one".into(), 5., 1., evidence())
        .unwrap();
    let p = c.tick(1., evidence()).unwrap();
    assert!((p.0 - 12. - 3. * 0.7_f64.cos()).abs() < 1e-8);
    assert!((p.1 + 9. - 3. * 0.7_f64.sin()).abs() < 1e-8);
    assert!(c.tick(1.1, evidence()).is_none());
}
#[test]
fn stale_pause_requires_explicit_resume_and_distinct_arrival() {
    let mut c = OutdoorController::default();
    sensors(&mut c, 1., 0.);
    c.start(0.001, 0., "one".into(), 5., 1., evidence())
        .unwrap();
    c.tick(5., evidence());
    assert_eq!(c.phase(), "paused");
    sensors(&mut c, 5., 0.);
    assert!(c.tick(5., evidence()).is_none());
    c.resume(5., evidence()).unwrap();
    assert!(c.tick(5., evidence()).is_some());
    sensors(&mut c, 6., 0.001);
    assert!(c.tick(6., evidence()).is_none());
    assert_eq!(c.phase(), "confirming_arrival");
    c.tick(6.1, evidence());
    assert_ne!(c.phase(), "arrived");
    sensors(&mut c, 7., 0.001);
    c.tick(7., evidence());
    sensors(&mut c, 8., 0.001);
    c.tick(8., evidence());
    assert_eq!(c.phase(), "arrived");
}
#[test]
fn invalid_samples_and_hardware_fail_closed() {
    let mut c = OutdoorController::default();
    sensors(&mut c, 1., 0.);
    assert!(
        c.gps(
            GpsFix {
                timestamp: 2.,
                latitude: 0.,
                longitude: 0.,
                accuracy: 2.
            },
            1.
        )
        .is_err()
    );
    c.gps(
        GpsFix {
            timestamp: 1.1,
            latitude: 0.,
            longitude: 0.,
            accuracy: 2.,
        },
        1.1,
    )
    .unwrap();
    let mut e = evidence();
    e.hardware_ready = false;
    assert!(c.start(0.001, 0., "x".into(), 5., 1.1, e).is_err());
    assert!(c.start(1., 0., "x".into(), 5., 1.1, evidence()).is_err());
    c.start(0.001, 0., "x".into(), 5., 1.1, evidence()).unwrap();
    e = evidence();
    e.map_ready = false;
    assert!(c.tick(1.1, e).is_none());
}
#[test]
fn unsupported_platform_rejects_selection() {
    let c = terra_mobile::MobileController::new(terra_mobile::default_control_settings()).unwrap();
    assert!(c.autonomy_request("navigation/source".into(),r#"{"schema_version":1,"source":"outdoor_gps","token":"pick","run_id":"x","navigation_revision":0}"#.into(),0.).is_err());
    c.enable_outdoor_gps().unwrap();
    local_sensors(&c, 1.);
    c.push_gps(
        GpsFix {
            timestamp: 1.,
            latitude: 0.,
            longitude: 0.,
            accuracy: 2.,
        },
        1.,
    )
    .unwrap();
    c.push_heading(
        HeadingFix {
            timestamp: 1.,
            true_heading: 0.,
            accuracy: 2.,
        },
        1.,
    )
    .unwrap();
}
fn local_sensors(c: &terra_mobile::MobileController, t: f64) {
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
            width: 100,
            height: 100,
            resolution: 0.1,
            origin_x: -5.,
            origin_y: -5.,
            occupancy: vec![0; 10000],
        },
        t,
        (t * 1000.) as u64,
    )
    .unwrap();
}
fn state(c: &terra_mobile::MobileController) -> serde_json::Value {
    serde_json::from_str(&c.autonomy_status()).unwrap()
}
fn hardware(c: &terra_mobile::MobileController, output_active: bool) {
    c.publish_hardware_status(
        serde_json::json!({"schema_version":1,"control_mode":"streamed_relay","connected":true,"ready":true,"output_active":output_active,"fault":null})
            .to_string(),
    )
    .unwrap();
}
#[test]
fn mobile_outdoor_runs_checked_l3_and_preserves_mission_on_pause() {
    let mut settings = terra_mobile::default_control_settings();
    settings.max_forward = 0.1;
    let c = terra_mobile::MobileController::new(settings).unwrap();
    c.enable_outdoor_gps().unwrap();
    local_sensors(&c, 1.);
    hardware(&c, false);
    c.step(1.).unwrap();
    let run = state(&c)["status"]["run_id"].clone();
    c.autonomy_request("navigation/source".into(),serde_json::json!({"schema_version":1,"source":"outdoor_gps","token":"select","run_id":run,"navigation_revision":0}).to_string(),1.).unwrap();
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"waypoint","token":"level"}"#.into(),
        1.,
    )
    .unwrap();
    local_sensors(&c, 1.01);
    c.step(1.01).unwrap();
    assert_eq!(
        state(&c)["status"]["safety"],
        "clear",
        "Selecting outdoor mode must permit idle ready hardware"
    );
    hardware(&c, true);
    c.push_gps(
        GpsFix {
            timestamp: 1.,
            latitude: 0.,
            longitude: 0.,
            accuracy: 2.,
        },
        1.,
    )
    .unwrap();
    c.push_heading(
        HeadingFix {
            timestamp: 1.,
            true_heading: 0.,
            accuracy: 2.,
        },
        1.,
    )
    .unwrap();
    c.autonomy_request("goal".into(),serde_json::json!({"frame":"wgs84","latitude":0.001,"longitude":0,"token":"mission","run_id":run,"navigation_revision":1,"arrival_radius_m":2.}).to_string(),1.01).unwrap();
    local_sensors(&c, 1.05);
    let output = c.step(1.05).unwrap();
    let s = state(&c);
    assert!(output.target_forward > 0. && output.target_forward <= 0.1);
    assert_eq!(s["outdoor"]["arrival_radius_m"], 2.);
    let trace = &s["status"]["planner_trace"];
    let selected = trace["selected"].as_u64().unwrap() as usize;
    assert_eq!(trace["candidates"][selected]["velocity"], s["twist"]);
    assert_eq!(s["goal"]["token"], "mission");
    assert_eq!(s["goal"]["latitude"], 0.001);
    assert_eq!(s["goal"]["state"], "active");
    assert_eq!(
        s["twist"]["linear"].as_f64().unwrap(),
        output.target_forward
    );
    local_sensors(&c, 5.);
    c.step(5.).unwrap();
    assert_eq!(state(&c)["outdoor"]["phase"], "paused");
    c.push_gps(
        GpsFix {
            timestamp: 5.,
            latitude: 0.,
            longitude: 0.,
            accuracy: 2.,
        },
        5.,
    )
    .unwrap();
    c.push_heading(
        HeadingFix {
            timestamp: 5.,
            true_heading: 0.,
            accuracy: 2.,
        },
        5.,
    )
    .unwrap();
    hardware(&c, false); // Neutral output does not resume the paused mission.
    local_sensors(&c, 5.05);
    let held = c.step(5.05).unwrap();
    assert_eq!(held.target_forward, 0.);
    assert_eq!(held.target_yaw_rate, 0.);
    assert_eq!(
        state(&c)["status"]["safety"],
        "clear",
        "Recovered sensors must not resume motion automatically"
    );
    assert_eq!(state(&c)["outdoor"]["phase"], "paused");
    c.autonomy_request("navigation/resume".into(),serde_json::json!({"schema_version":1,"token":"resume","run_id":run,"navigation_revision":1}).to_string(),5.05).unwrap();
    local_sensors(&c, 5.1);
    assert!(c.step(5.1).unwrap().target_forward > 0.);
    assert_eq!(state(&c)["goal"]["token"], "mission");
    assert!(c.autonomy_request("goal".into(),serde_json::json!({"frame":"wgs84","latitude":0.001,"longitude":0,"token":"stale-goal","run_id":"old-run","navigation_revision":1,"arrival_radius_m":2.}).to_string(),5.1).is_err());
    assert_eq!(state(&c)["goal"]["token"], "mission");
    for (i, cell) in [100, -1].into_iter().enumerate() {
        let t = 5.15 + i as f64 * 0.05;
        local_sensors(&c, t);
        c.autonomy_map(
            terra_mobile::OccupancyGrid {
                width: 100,
                height: 100,
                resolution: 0.1,
                origin_x: -5.,
                origin_y: -5.,
                occupancy: vec![cell; 10000],
            },
            t,
            6000 + i as u64,
        )
        .unwrap();
        assert_eq!(c.step(t).unwrap().target_forward, 0.);
        assert_eq!(state(&c)["goal"]["state"], "active");
    }
    c.reset_outdoor_frame().unwrap();
    local_sensors(&c, 5.3);
    c.step(5.3).unwrap();
    assert_eq!(state(&c)["outdoor"]["phase"], "idle");
    assert!(state(&c)["outdoor"]["destination"].is_null());
}
#[test]
fn rejected_sensor_latches_pause_immediately() {
    let mut c = OutdoorController::default();
    sensors(&mut c, 1., 0.);
    c.start(0.001, 0., "m".into(), 5., 1., evidence()).unwrap();
    assert!(
        c.gps(
            GpsFix {
                timestamp: 2.,
                latitude: f64::NAN,
                longitude: 0.,
                accuracy: 2.
            },
            1.
        )
        .is_err()
    );
    assert_eq!(c.phase(), "paused");
    assert!(c.tick(1.1, evidence()).is_none());
}
#[test]
fn invalidation_does_not_erase_replay_watermark() {
    let mut c = OutdoorController::default();
    sensors(&mut c, 1., 0.);
    c.invalidate_sensors();
    assert!(
        c.gps(
            GpsFix {
                timestamp: 1.,
                latitude: 0.,
                longitude: 0.,
                accuracy: 2.
            },
            1.1
        )
        .is_err()
    );
    assert!(
        c.heading(
            HeadingFix {
                timestamp: 1.,
                true_heading: 0.,
                accuracy: 2.
            },
            1.1
        )
        .is_err()
    );
}

#[test]
fn simulated_geographic_mission_confirms_only_after_stopping() {
    let mut c = OutdoorController::default();
    let mut e = evidence();
    e.pose = Pose::default();
    sensors_at(&mut c, 0., 0., 0., e.pose);
    c.start(0.000063, 0., "simulation".into(), 2., 0., e)
        .unwrap();
    let mut north = 0.;
    let mut saw_confirmation = false;
    for tick in 0..2000 {
        let t = tick as f64 * 0.05;
        if tick > 0 && tick % 20 == 0 {
            sensors_at(&mut c, t, north / 111_195., 0., e.pose);
        }
        c.tick(t, e);
        if c.phase() == "planning" {
            north += 0.3 * 0.05;
            e.pose.x = north;
        }
        if c.phase() == "confirming_arrival" {
            saw_confirmation = true;
        }
        if c.phase() == "arrived" {
            break;
        }
    }
    assert!(saw_confirmation);
    assert_eq!(c.phase(), "arrived");
    assert!((5.0..6.1).contains(&north));
}

#[test]
fn vio_loss_pauses_without_encoders_and_recovery_does_not_resume() {
    let mut c = OutdoorController::default();
    sensors(&mut c, 1., 0.);
    c.start(0.001, 0., "vio-mission".into(), 5., 1., evidence())
        .unwrap();
    let mut e = evidence();
    e.pose_ready = false;
    assert!(c.tick(1.1, e).is_none());
    assert_eq!(c.phase(), "paused");
    assert!(c.tick(1.2, evidence()).is_none());
    c.resume(1.2, evidence()).unwrap();
    assert!(c.tick(1.2, evidence()).is_some());
}

#[test]
fn default_radius_accepts_ten_metre_gps_but_not_worse() {
    let mut c = OutdoorController::default();
    sensors(&mut c, 1., 0.);
    c.gps(
        GpsFix {
            timestamp: 1.1,
            latitude: 0.,
            longitude: 0.,
            accuracy: 10.,
        },
        1.1,
    )
    .unwrap();
    assert_eq!(c.radius, 10.);
    assert!(c.readiness(1.1, evidence(), c.radius).is_none());
    c.gps(
        GpsFix {
            timestamp: 1.2,
            latitude: 0.,
            longitude: 0.,
            accuracy: 10.1,
        },
        1.2,
    )
    .unwrap();
    assert_eq!(
        c.readiness(1.2, evidence(), c.radius),
        Some("gps_uncertain")
    );
}

fn gps_only(c: &mut OutdoorController, t: f64) {
    c.gps(
        GpsFix {
            timestamp: t,
            latitude: 0.,
            longitude: 0.,
            accuracy: 2.,
        },
        t,
    )
    .unwrap();
}
fn compass(c: &mut OutdoorController, t: f64, true_heading: f64, accuracy: f64) {
    c.heading(
        HeadingFix {
            timestamp: t,
            true_heading,
            accuracy,
        },
        t,
    )
    .unwrap();
}
fn heading_status(c: &OutdoorController, t: f64) -> serde_json::Value {
    c.status(t, evidence(), "run")
}
#[test]
fn temporary_heading_limit_keeps_accuracy_and_freshness_gates() {
    let mut c = OutdoorController::default();
    gps_only(&mut c, 1.1);
    c.observe_pose(evidence().pose, 1.1);
    compass(&mut c, 1.1, 0., 35.);
    assert_eq!(c.readiness(1.1, evidence(), c.radius), None);
    // Without fresh VIO the fused heading cannot be carried forward.
    assert_eq!(
        c.readiness(3.2, evidence(), c.radius),
        Some("heading_uncertain")
    );
    let mut c = OutdoorController::default();
    gps_only(&mut c, 3.3);
    c.observe_pose(evidence().pose, 3.3);
    compass(&mut c, 3.3, 0., 35.1);
    assert_eq!(
        c.readiness(3.3, evidence(), c.radius),
        Some("heading_uncertain")
    );
}
#[test]
fn compass_alone_never_claims_better_than_its_accuracy() {
    let mut c = OutdoorController::default();
    for i in 0..100 {
        let t = 1. + i as f64 * 0.1;
        gps_only(&mut c, t);
        c.observe_pose(evidence().pose, t);
        compass(&mut c, t, 10., 48.);
    }
    let s = heading_status(&c, 10.9);
    assert!(s["heading_accuracy_deg"].as_f64().unwrap() >= 48.);
    assert_eq!(s["heading_source"], "compass");
    assert_eq!(
        c.readiness(10.9, evidence(), c.radius),
        Some("heading_uncertain")
    );
}
/// Drive along local direction `travel` (radians) at 1 m/s while facing `yaw`.
fn drive(c: &mut OutdoorController, from: f64, to: f64, travel: f64, yaw: f64) {
    let mut t = from;
    while t <= to + 1e-9 {
        let d = t - from;
        c.observe_pose(
            Pose {
                x: d * travel.cos(),
                y: d * travel.sin(),
                yaw,
            },
            t,
        );
        t += 0.05;
    }
}
fn course(c: &mut OutdoorController, t: f64, course: f64) {
    c.course(
        CourseFix {
            timestamp: t,
            course,
            accuracy: 4.,
            speed: 1.,
        },
        t,
    )
    .unwrap();
}
#[test]
fn gps_course_locks_heading_and_vio_carries_it() {
    let mut c = OutdoorController::default();
    // Travel 0.3 rad left of local +X while facing that way, heading true east.
    drive(&mut c, 0., 3., 0.3, 0.3);
    gps_only(&mut c, 3.);
    compass(&mut c, 3., 140., 48.);
    assert_eq!(
        c.readiness(3., evidence(), c.radius),
        Some("heading_uncertain")
    );
    for _ in 0..12 {
        course(&mut c, 3., 90.);
    }
    let s = heading_status(&c, 3.);
    assert_eq!(s["heading_source"], "gps_course");
    assert!((s["heading_deg"].as_f64().unwrap() - 90.).abs() < 1.);
    assert!(s["heading_accuracy_deg"].as_f64().unwrap() <= 4.);
    // A biased compass much worse than the estimate does not drag it.
    compass(&mut c, 3.01, 140., 48.);
    assert!((heading_status(&c, 3.01)["heading_deg"].as_f64().unwrap() - 90.).abs() < 1.);
    // VIO yaw carries heading through a turn with no further corrections.
    for i in 1..=100 {
        let t = 3. + i as f64 * 0.05;
        c.observe_pose(
            Pose {
                x: 0.,
                y: 0.,
                yaw: 0.3 + 0.01 * i as f64,
            },
            t,
        );
    }
    gps_only(&mut c, 8.);
    let s = heading_status(&c, 8.);
    let expected = 90. - 1_f64.to_degrees();
    assert!((s["heading_deg"].as_f64().unwrap() - expected).abs() < 1.);
    assert!(s["heading_accuracy_deg"].as_f64().unwrap() < 5.);
    assert_eq!(c.readiness(8., evidence(), c.radius), None);
}
#[test]
fn gps_course_handles_reversing() {
    let mut c = OutdoorController::default();
    // Facing local -X while backing along +X, travelling true north.
    drive(&mut c, 0., 2., 0., std::f64::consts::PI);
    course(&mut c, 2., 0.);
    let (h, _, _) = c.heading_estimate(2.).unwrap();
    assert!((h - 180.).abs() < 1.);
}
#[test]
fn course_is_ignored_while_turning_or_stopped() {
    let mut c = OutdoorController::default();
    for i in 0..=40 {
        let t = i as f64 * 0.05;
        c.observe_pose(
            Pose {
                x: t,
                y: 0.,
                yaw: 0.2 * t,
            },
            t,
        );
    }
    course(&mut c, 2., 0.);
    assert!(c.heading_estimate(2.).is_none());
    let mut c = OutdoorController::default();
    drive(&mut c, 0., 2., 0., 0.);
    c.course(
        CourseFix {
            timestamp: 2.,
            course: 0.,
            accuracy: 4.,
            speed: 0.2,
        },
        2.,
    )
    .unwrap();
    assert!(c.heading_estimate(2.).is_none());
}
#[test]
fn local_frame_jump_drops_fused_heading() {
    let mut c = OutdoorController::default();
    drive(&mut c, 0., 2., 0., 0.);
    course(&mut c, 2., 0.);
    assert!(c.heading_estimate(2.).is_some());
    c.observe_pose(
        Pose {
            x: 2.,
            y: 0.,
            yaw: 1.5,
        },
        2.05,
    );
    assert!(c.heading_estimate(2.05).is_none());
}
#[test]
fn persistent_disagreement_restarts_from_new_corrections() {
    let mut c = OutdoorController::default();
    drive(&mut c, 0., 2., 0., 0.);
    course(&mut c, 2., 0.);
    drive(&mut c, 2.05, 4., 0., 0.);
    for _ in 0..9 {
        course(&mut c, 4., 120.);
    }
    assert!(c.heading_estimate(4.).unwrap().0 < 5.);
    course(&mut c, 4., 120.);
    assert!((c.heading_estimate(4.).unwrap().0 - 120.).abs() < 1.);
}

#[test]
fn geographic_mission_survives_two_minute_cloud_outage_but_hardware_loss_latches() {
    let mut settings = terra_mobile::default_control_settings();
    settings.max_forward = 0.1;
    let c = terra_mobile::MobileController::new(settings).unwrap();
    c.enable_outdoor_gps().unwrap();
    local_sensors(&c, 1.);
    hardware(&c, false);
    c.step(1.).unwrap();
    let run = state(&c)["status"]["run_id"].clone();
    c.autonomy_request("navigation/source".into(),serde_json::json!({"schema_version":1,"source":"outdoor_gps","token":"select","run_id":run,"navigation_revision":0}).to_string(),1.).unwrap();
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"waypoint","token":"level"}"#.into(),
        1.,
    )
    .unwrap();
    local_sensors(&c, 1.01);
    c.step(1.01).unwrap();
    assert_eq!(
        state(&c)["status"]["safety"],
        "clear",
        "Selecting outdoor mode must permit idle ready hardware"
    );
    hardware(&c, true);
    c.push_gps(
        GpsFix {
            timestamp: 1.,
            latitude: 0.,
            longitude: 0.,
            accuracy: 2.,
        },
        1.,
    )
    .unwrap();
    c.push_heading(
        HeadingFix {
            timestamp: 1.,
            true_heading: 0.,
            accuracy: 2.,
        },
        1.,
    )
    .unwrap();
    c.autonomy_request("goal".into(),serde_json::json!({"frame":"wgs84","latitude":0.001,"longitude":0,"token":"mission","run_id":run,"navigation_revision":1,"arrival_radius_m":2.}).to_string(),1.01).unwrap();
    local_sensors(&c, 1.05);
    let output = c.step(1.05).unwrap();
    let s = state(&c);
    assert!(output.target_forward > 0. && output.target_forward <= 0.1);
    assert_eq!(s["outdoor"]["arrival_radius_m"], 2.);
    let trace = &s["status"]["planner_trace"];
    let selected = trace["selected"].as_u64().unwrap() as usize;
    assert_eq!(trace["candidates"][selected]["velocity"], s["twist"]);
    assert_eq!(s["goal"]["token"], "mission");
    assert_eq!(s["goal"]["latitude"], 0.001);
    assert_eq!(s["goal"]["state"], "active");
    assert_eq!(
        s["twist"]["linear"].as_f64().unwrap(),
        output.target_forward
    );

    assert!(c.accepted_onboard_mission());
    c.handle_dashboard_link_loss().unwrap();
    for tick in 1..=1200 {
        let t = 1.05 + tick as f64 * 0.1;
        local_sensors(&c, t);
        hardware(&c, true);
        c.push_gps(
            GpsFix {
                timestamp: t,
                latitude: 0.,
                longitude: 0.,
                accuracy: 2.,
            },
            t,
        )
        .unwrap();
        c.push_heading(
            HeadingFix {
                timestamp: t,
                true_heading: 0.,
                accuracy: 2.,
            },
            t,
        )
        .unwrap();
        c.step(t).unwrap();
        assert_eq!(state(&c)["outdoor"]["phase"], "planning");
        assert_eq!(state(&c)["goal"]["token"], "mission");
    }
    c.publish_hardware_status(serde_json::json!({"schema_version":1,"control_mode":"streamed_relay","connected":false,"ready":false,"output_active":false,"fault":"disconnected"}).to_string()).unwrap();
    local_sensors(&c, 121.2);
    c.step(121.2).unwrap();
    assert_eq!(state(&c)["outdoor"]["phase"], "paused");
    hardware(&c, true);
    local_sensors(&c, 121.3);
    c.step(121.3).unwrap();
    assert_eq!(state(&c)["outdoor"]["phase"], "paused");
    assert!(!c.accepted_onboard_mission());
}
#[test]
fn manual_takeover_drives_while_an_outdoor_mission_is_paused() {
    let c = terra_mobile::MobileController::new(terra_mobile::default_control_settings()).unwrap();
    c.enable_outdoor_gps().unwrap();
    local_sensors(&c, 1.);
    hardware(&c, false);
    c.step(1.).unwrap();
    let run = state(&c)["status"]["run_id"].clone();
    c.autonomy_request("navigation/source".into(),serde_json::json!({"schema_version":1,"source":"outdoor_gps","token":"select","run_id":run,"navigation_revision":0}).to_string(),1.).unwrap();
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"waypoint","token":"level"}"#.into(),
        1.,
    )
    .unwrap();
    c.push_gps(
        GpsFix {
            timestamp: 1.,
            latitude: 0.,
            longitude: 0.,
            accuracy: 2.,
        },
        1.,
    )
    .unwrap();
    c.push_heading(
        HeadingFix {
            timestamp: 1.,
            true_heading: 0.,
            accuracy: 2.,
        },
        1.,
    )
    .unwrap();
    c.autonomy_request("goal".into(),serde_json::json!({"frame":"wgs84","latitude":0.001,"longitude":0,"token":"mission","run_id":run,"navigation_revision":1,"arrival_radius_m":2.}).to_string(),1.01).unwrap();
    local_sensors(&c, 1.05);
    c.step(1.05).unwrap();
    // GPS and heading go stale, as with a heading hold in the field.
    local_sensors(&c, 5.);
    c.step(5.).unwrap();
    assert_eq!(state(&c)["outdoor"]["phase"], "paused");
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"teleop","token":"takeover"}"#.into(),
        5.,
    )
    .unwrap();
    local_sensors(&c, 5.05);
    c.set_manual_input(0.5, 0., 5.05).unwrap();
    let output = c.step(5.05).unwrap();
    let s = state(&c);
    assert_eq!(s["status"]["safety"], "clear", "{s}");
    assert!(output.left_effort > 0. && output.right_effort > 0., "{s}");
    assert_eq!(
        s["outdoor"]["phase"], "paused",
        "the mission still needs an explicit resume"
    );
}
fn local_sensors_at(c: &terra_mobile::MobileController, t: f64, x: f64, y: f64, yaw: f64) {
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
    // ARKit world frame: x right, y up, z back; rover x forward is -z, y left is -x.
    let (s, c2) = ((yaw / 2.).sin(), (yaw / 2.).cos());
    c.push_vio(terra_mobile::VioReading {
        timestamp: t,
        position_x: -y,
        position_y: 0.,
        position_z: -x,
        quaternion_x: 0.,
        quaternion_y: s,
        quaternion_z: 0.,
        quaternion_w: c2,
        velocity_x: 0.,
        velocity_y: 0.,
        velocity_z: 0.,
        tracked: true,
    })
    .unwrap();
}
/// Drive a point rover through `seconds` of explore and return its final status.
fn explore_run(gps: bool, seconds: f64) -> serde_json::Value {
    let c = terra_mobile::MobileController::new(terra_mobile::default_control_settings()).unwrap();
    c.enable_outdoor_gps().unwrap();
    let mut pose = (0., 0., 0.);
    let observe = |t: f64, pose: (f64, f64, f64)| {
        local_sensors_at(&c, t, pose.0, pose.1, pose.2);
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
            1,
        )
        .unwrap();
    };
    observe(1., pose);
    hardware(&c, false);
    c.step(1.).unwrap();
    if gps {
        let run = state(&c)["status"]["run_id"].clone();
        c.autonomy_request("navigation/source".into(),serde_json::json!({"schema_version":1,"source":"outdoor_gps","token":"select","run_id":run,"navigation_revision":0}).to_string(),1.).unwrap();
    }
    hardware(&c, true);
    c.autonomy_request(
        "autonomy".into(),
        r#"{"level":"explore","budget_seconds":120,"token":"explore"}"#.into(),
        1.,
    )
    .unwrap();
    let dt = 0.05;
    let mut t = 1.;
    while t < 1. + seconds {
        t += dt;
        observe(t, pose);
        let out = c.step(t).unwrap();
        pose.2 += out.target_yaw_rate * dt;
        pose.0 += out.target_forward * pose.2.cos() * dt;
        pose.1 += out.target_forward * pose.2.sin() * dt;
    }
    state(&c)
}
#[test]
fn explore_runs_in_the_local_frame_while_outdoor_gps_is_selected() {
    // Before the fix the outdoor idle phase cancelled explore's goal every tick,
    // so arrival never registered and the rover drove past its frontier.
    let local = explore_run(false, 30.);
    let gps = explore_run(true, 30.);
    assert_eq!(gps["outdoor"]["source"], "outdoor_gps");
    assert_eq!(gps["exploration"]["phase"], "running", "{gps}");
    assert!(local["exploration"]["frontiers_attempted"].as_u64() > Some(1));
    assert!(
        gps["exploration"]["frontiers_attempted"].as_u64() > Some(1),
        "{}",
        gps["exploration"]
    );
}
