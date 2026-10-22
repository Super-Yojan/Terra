use terra_mobile::outdoor::{Evidence, GpsFix, HeadingFix, OutdoorController, distance_bearing};
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
        armed: true,
        authority: true,
    }
}
fn sensors(c: &mut OutdoorController, t: f64, lat: f64) {
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
            true_heading: 0.,
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
        sensors(&mut mission, 1., 0.);
        mission
            .heading(
                HeadingFix {
                    timestamp: 1.1,
                    true_heading: 270.,
                    accuracy: 2.,
                },
                1.1,
            )
            .unwrap();
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
fn hardware(c: &terra_mobile::MobileController, armed: bool) {
    c.publish_hardware_status(
        serde_json::json!({"ready":true,"armed":armed,"arming":false,"wheel_feedback":false})
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
        "Selecting outdoor mode must leave idle arming possible"
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
    hardware(&c, false); // The phone disarms on the sensor hold.
    local_sensors(&c, 5.05);
    let held = c.step(5.05).unwrap();
    assert_eq!(held.target_forward, 0.);
    assert_eq!(held.target_yaw_rate, 0.);
    assert_eq!(
        state(&c)["status"]["safety"],
        "clear",
        "Recovered sensors must allow rearming without resuming motion"
    );
    assert_eq!(state(&c)["outdoor"]["phase"], "paused");
    assert!(c.autonomy_request("navigation/resume".into(),serde_json::json!({"schema_version":1,"token":"unarmed-resume","run_id":run,"navigation_revision":1}).to_string(),5.05).is_err());
    hardware(&c, true);
    assert_eq!(
        c.step(5.05).unwrap().target_forward,
        0.,
        "Arming alone must not resume the mission"
    );
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
    sensors(&mut c, 0., 0.);
    c.start(0.000063, 0., "simulation".into(), 2., 0., e)
        .unwrap();
    let mut north = 0.;
    let mut saw_confirmation = false;
    for tick in 0..2000 {
        let t = tick as f64 * 0.05;
        if tick > 0 && tick % 20 == 0 {
            sensors(&mut c, t, north / 111_195.);
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

#[test]
fn temporary_heading_limit_keeps_accuracy_and_freshness_gates() {
    let mut c = OutdoorController::default();
    sensors(&mut c, 1., 0.);
    c.heading(
        HeadingFix {
            timestamp: 1.1,
            true_heading: 0.,
            accuracy: 35.,
        },
        1.1,
    )
    .unwrap();
    assert_eq!(c.readiness(1.1, evidence(), c.radius), None);
    assert_eq!(
        c.readiness(3.2, evidence(), c.radius),
        Some("heading_uncertain")
    );
    c.heading(
        HeadingFix {
            timestamp: 3.3,
            true_heading: 0.,
            accuracy: 35.1,
        },
        3.3,
    )
    .unwrap();
    assert_eq!(
        c.readiness(3.3, evidence(), c.radius),
        Some("heading_uncertain")
    );
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
        "Selecting outdoor mode must leave idle arming possible"
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
    hardware(&c, false);
    local_sensors(&c, 121.2);
    c.step(121.2).unwrap();
    assert_eq!(state(&c)["outdoor"]["phase"], "paused");
    hardware(&c, true);
    local_sensors(&c, 121.3);
    c.step(121.3).unwrap();
    assert_eq!(state(&c)["outdoor"]["phase"], "paused");
    assert!(!c.accepted_onboard_mission());
}
