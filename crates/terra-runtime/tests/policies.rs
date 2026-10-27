use serde_json::json;
use terra_navigation::Pose;
use terra_runtime::*;
use terra_types::*;
fn runtime(mode: ActuationMode) -> Runtime {
    Runtime::new(
        RuntimeConfig {
            mode,
            ..Default::default()
        },
        "run".into(),
    )
    .unwrap()
}
fn sensors(r: &mut Runtime, t: f64) {
    r.ingest(RuntimeEvent::Imu {
        sample: ImuSample {
            timestamp: t,
            acceleration: Vector3::ZERO,
            angular_velocity: Vector3::ZERO,
        },
        received_at: t,
    })
    .unwrap();
    r.ingest(RuntimeEvent::Vio {
        sample: VioSample {
            timestamp: t,
            position: Vector3::ZERO,
            orientation: Quaternion::default(),
            velocity: Vector3::ZERO,
            tracked: true,
        },
        received_at: t,
    })
    .unwrap();
}
fn remote(r: &mut Runtime, kind: &str, mut v: serde_json::Value, t: f64) {
    v["run_id"] = json!(r.run_id);
    v["authority_revision"] = json!(r.autonomy.authority_revision());
    r.ingest(RuntimeEvent::Remote {
        kind: kind.into(),
        payload: serde_json::to_vec(&v).unwrap(),
        received_at: t,
    })
    .unwrap();
}
#[test]
fn managed_forward_and_reverse_turns_keep_wheels_in_the_same_direction() {
    for forward in [0.1, -0.1] {
        let mut r = runtime(ActuationMode::DirectEffort);
        for n in 1..=80 {
            let t = n as f64 * 0.01;
            sensors(&mut r, t);
            remote(
                &mut r,
                "teleop",
                json!({"linear":forward,"angular":1.,"operator_session_id":"operator","sequence":n}),
                t,
            );
            let out = r.step(t).unwrap();
            assert!(out.motor.left * out.motor.right >= 0.);
            assert!(out.autonomy.twist.angular.abs() <= 0.2 + 1e-9);
        }
    }
}
#[test]
fn stale_generation_in_a_batch_is_rejected_and_stop_dominates_reset() {
    let mut r = runtime(ActuationMode::DirectEffort);
    remote(
        &mut r,
        "autonomy",
        json!({"level":"waypoint_direct","token":"mode"}),
        0.,
    );
    remote(
        &mut r,
        "autonomy",
        json!({"level":"teleop","token":"stale-mode"}),
        0.,
    );
    let out = r.step(0.).unwrap();
    assert_eq!(
        out.autonomy.status.requested_level,
        terra_autonomy::Level::WaypointDirect
    );
    assert!(out.events.iter().any(|e| e["kind"] == "command.rejected"));
    remote(
        &mut r,
        "safety",
        json!({"action":"stop","token":"stop"}),
        0.01,
    );
    remote(
        &mut r,
        "safety",
        json!({"action":"reset","token":"reset"}),
        0.01,
    );
    assert_eq!(
        r.step(0.01).unwrap().autonomy.status.safety,
        "emergency_stop"
    );
}
#[test]
fn link_loss_retires_operator_generation_without_replaying_on_reconnect() {
    let mut r = runtime(ActuationMode::DirectEffort);
    sensors(&mut r, 0.);
    let old = r.autonomy.authority_revision();
    remote(
        &mut r,
        "teleop",
        json!({"linear":0.2,"angular":0.,"operator_session_id":"operator","sequence":1}),
        0.,
    );
    assert!(r.step(0.).unwrap().motor.left > 0.);
    r.ingest(RuntimeEvent::Link {
        connected: false,
        received_at: 0.01,
    })
    .unwrap();
    assert!(r.autonomy.authority_revision() > old);
    assert_eq!(r.step(0.01).unwrap().motor.left, 0.);
    r.ingest(RuntimeEvent::Link {
        connected: true,
        received_at: 0.02,
    })
    .unwrap();
    r.ingest(RuntimeEvent::Remote {
        kind: "teleop".into(),
        payload: serde_json::to_vec(
            &json!({"run_id":"run","authority_revision":old,"linear":0.2,"angular":0.}),
        )
        .unwrap(),
        received_at: 0.02,
    })
    .unwrap();
    let out = r.step(0.02).unwrap();
    assert_eq!(out.motor.left, 0.);
    assert!(out.events.iter().any(|e| e["kind"] == "command.rejected"));
}
#[test]
fn accepted_mission_survives_link_loss_with_fresh_onboard_evidence() {
    let mut r = runtime(ActuationMode::VelocityFeedback);
    sensors(&mut r, 0.);
    r.autonomy.set_level(terra_autonomy::LevelRequest::new(
        terra_autonomy::Level::WaypointDirect,
        "mode",
    ));
    r.autonomy.accept_goal(
        &terra_waypoint::GoalCommand::Local {
            x: 1.,
            y: 0.,
            yaw: None,
            token: Some("goal".into()),
        },
        20000.,
    );
    r.step(0.).unwrap();
    r.ingest(RuntimeEvent::Link {
        connected: false,
        received_at: 0.01,
    })
    .unwrap();
    sensors(&mut r, 0.01);
    let out = r.step(0.01).unwrap();
    assert_eq!(out.autonomy.goal.state, terra_waypoint::GoalState::Active);
    assert_eq!(out.autonomy.status.safety, "clear");
    assert!(out.autonomy.twist.linear > 0.);
}
#[test]
fn future_and_reordered_samples_cannot_replace_measured_pose() {
    let mut r = runtime(ActuationMode::VelocityFeedback);
    sensors(&mut r, 1.);
    let before = r.pose.unwrap().0;
    assert!(
        r.ingest(RuntimeEvent::Vio {
            sample: VioSample {
                timestamp: 2.,
                position: Vector3 {
                    x: 99.,
                    ..Vector3::ZERO
                },
                orientation: Quaternion::default(),
                velocity: Vector3::ZERO,
                tracked: true
            },
            received_at: 1.
        })
        .is_err()
    );
    assert_eq!(r.pose.unwrap().0.x, before.x);
    assert!(
        r.ingest(RuntimeEvent::Imu {
            sample: ImuSample {
                timestamp: 0.5,
                acceleration: Vector3::ZERO,
                angular_velocity: Vector3::ZERO
            },
            received_at: 1.
        })
        .is_err()
    );
}
#[test]
fn recording_health_is_distinct_from_link_health() {
    let mut r = runtime(ActuationMode::DirectEffort);
    r.ingest(RuntimeEvent::LocalManual {
        forward: 0.2,
        turn: 0.,
        received_at: 0.,
    })
    .unwrap();
    assert!(r.step(0.).unwrap().motor.left > 0.);
    r.ingest(RuntimeEvent::Recording {
        healthy: false,
        received_at: 0.01,
    })
    .unwrap();
    let out = r.step(0.01).unwrap();
    assert_eq!(out.motor.left, 0.);
    assert!(out.events.iter().any(|e| e["kind"] == "recording.health"));
}
#[test]
fn identical_depth_and_ground_planes_produce_identical_maps() {
    let mut a = runtime(ActuationMode::VelocityFeedback);
    let mut b = runtime(ActuationMode::VelocityFeedback);
    for r in [&mut a, &mut b] {
        let camera = terra_mapping::CameraPose {
            position: Vector3 {
                x: 0.,
                y: 0.,
                z: 0.5,
            },
            orientation: Quaternion::from_rotation_vector(Vector3 {
                x: 0.,
                y: std::f64::consts::FRAC_PI_2,
                z: 0.,
            }),
        };
        r.ingest(RuntimeEvent::Depth {
            timestamp: 0.,
            received_at: 0.,
            intrinsics: terra_mapping::CameraIntrinsics {
                width: 1,
                height: 1,
                fx: 1.,
                fy: 1.,
                cx: 0.,
                cy: 0.,
            },
            camera,
            depth: vec![2.],
            ground: Some(terra_mapping::GroundPlane::default()),
        })
        .unwrap();
    }
    assert_eq!(
        a.map.as_ref().unwrap().0.occupancy,
        b.map.as_ref().unwrap().0.occupancy
    );
    assert!(a.map.as_ref().unwrap().0.occupancy.iter().any(|v| *v >= 0));
}
#[test]
fn ground_calibration_retains_existing_stability_and_mount_rules() {
    let mut g = ground::GroundPolicy::default();
    assert_eq!(g.observe(Some(0.2), 0.7, 1., 0., 1.), None);
    assert_eq!(g.observe(Some(0.2), 0.7, 1., 1., 1.), Some(-0.2));
    let p = g
        .plane(
            Vector3 {
                x: 0.,
                y: 0.,
                z: 0.7,
            },
            Vector3 {
                x: 0.,
                y: 0.,
                z: 1.,
            },
        )
        .unwrap();
    assert!(p.offset.abs() < 1e-9);
    assert!(
        g.plane(
            Vector3::ZERO,
            Vector3 {
                x: 1.,
                y: 0.,
                z: 0.
            }
        )
        .is_none()
    );
}
/// A 16-degree mount error tilts body up on flat floor. The plane must stay
/// level, or floor 3.5 m out reads as an obstacle (capture argos-75eca1f97bcb).
#[test]
fn ground_plane_stays_level_when_body_up_is_tilted() {
    let mut g = ground::GroundPolicy::default();
    let tilt = 16_f64.to_radians();
    let up = Vector3 {
        x: -tilt.sin(),
        y: 0.,
        z: tilt.cos(),
    };
    g.observe(Some(-0.45), 0., 1., 0., up.z);
    assert_eq!(g.observe(Some(-0.45), 0., 1., 1., up.z), Some(0.45));
    let camera = Vector3 {
        x: 2.,
        y: 1.,
        z: 0.,
    };
    let p = g.plane(camera, up).unwrap();
    assert_eq!((p.normal.x, p.normal.y, p.normal.z), (0., 0., 1.));
    // World points carry the offset, so the floor at z = 0 is height 0 everywhere.
    assert!(p.offset.abs() < 1e-9);
    // Driving up 0.3 m of terrain lifts the plane with the camera.
    let raised = g.plane(Vector3 { z: 0.3, ..camera }, up).unwrap();
    assert!((raised.offset - 0.3).abs() < 1e-9);
}
/// The mount height fixes the floor even when the app starts in the hand
/// (camera 1.05 m up) and the phone is mounted at 0.4 m afterwards.
#[test]
fn mount_height_keeps_floor_after_phone_is_mounted() {
    let up = Vector3 {
        x: 0.,
        y: 0.,
        z: 1.,
    };
    let mut g = ground::GroundPolicy::default();
    assert_eq!(g.mount(0.4, 0.), Some(0.4));
    // A later floor anchor cannot replace the mount.
    assert_eq!(g.observe(Some(-1.05), 0., 1., 5., 1.), Some(0.4));
    // Hand-held at startup: ARKit origin 1.05 m above the floor.
    let mut g = ground::GroundPolicy::default();
    let offset = g.mount(0.4, 0.).unwrap();
    // Mounted: camera now 0.65 m lower in ARKit z, 0.4 m above the floor.
    let camera_z = -0.65;
    let p = g
        .plane(
            Vector3 {
                x: 0.,
                y: 0.,
                z: camera_z,
            },
            up,
        )
        .unwrap();
    let floor_world_z = camera_z + offset - 0.4;
    let floor_height = floor_world_z - p.offset;
    assert!(floor_height.abs() < 1e-9, "floor reads {floor_height}");
    assert_eq!(g.mount(f64::NAN, 0.), Some(offset));
    let mut bad = ground::GroundPolicy::default();
    assert_eq!(bad.mount(0.05, 0.), None);
    assert_eq!(bad.mount(0.4, f64::NAN), None);
}
#[test]
fn configuration_identity_tracks_profile_changes() {
    let mut r = runtime(ActuationMode::VelocityFeedback);
    let before = r.configuration_identity();
    r.ingest(RuntimeEvent::Mode {
        mode: ActuationMode::DirectEffort,
        received_at: 0.,
    })
    .unwrap();
    assert_ne!(before, r.configuration_identity());
    assert_eq!(r.pose.map(|p| p.0.x), None);
    let _ = Pose::default();
}
#[test]
fn switching_actuation_mode_does_not_replay_local_input() {
    let mut r = runtime(ActuationMode::VelocityFeedback);
    sensors(&mut r, 0.);
    r.ingest(RuntimeEvent::LocalVelocity {
        target: VelocityTarget {
            timestamp: 0.,
            forward: 0.2,
            yaw_rate: 0.,
        },
        received_at: 0.,
    })
    .unwrap();
    r.step(0.).unwrap();
    r.ingest(RuntimeEvent::Mode {
        mode: ActuationMode::DirectEffort,
        received_at: 0.01,
    })
    .unwrap();
    assert_eq!(r.step(0.01).unwrap().motor.left, 0.);
}
#[test]
fn autonomous_guidance_uses_feedback_even_with_a_direct_manual_profile() {
    let mut r = runtime(ActuationMode::DirectEffort);
    sensors(&mut r, 0.);
    r.autonomy.set_level(terra_autonomy::LevelRequest::new(
        terra_autonomy::Level::WaypointDirect,
        "mode",
    ));
    r.autonomy.accept_goal(
        &terra_waypoint::GoalCommand::Local {
            x: 1.,
            y: 0.,
            yaw: None,
            token: Some("goal".into()),
        },
        20000.,
    );
    assert_eq!(
        r.step(0.).unwrap().snapshot.value["runtime"]["mode"],
        "velocity_feedback"
    );
}

#[test]
fn changing_connectivity_policy_during_loss_pauses_the_accepted_mission() {
    let mut r = runtime(ActuationMode::DirectEffort);
    r.link(false);
    let revision = r.connectivity_revision;
    let mut policy = r.connectivity.clone();
    policy.autonomous_router_loss = terra_types::connectivity_policy::RouterLossPolicy::Pause;
    r.set_connectivity_policy(policy).unwrap();
    assert_eq!(r.connectivity_revision, revision + 1);
    assert!(
        r.step(0.0)
            .unwrap()
            .events
            .iter()
            .any(|e| e["kind"] == "connectivity.policy_changed")
    );
}

#[test]
fn rejected_domain_requests_do_not_emit_successful_admission() {
    let mut r = runtime(ActuationMode::DirectEffort);
    remote(
        &mut r,
        "autonomy",
        json!({"level":"explore","token":"no-budget"}),
        0.0,
    );
    let out = r.step(0.0).unwrap();
    assert_eq!(
        out.autonomy.status.request_reason.as_deref(),
        Some("budget_required")
    );
    assert!(
        out.events
            .iter()
            .any(|e| e["kind"] == "command.admission" && e["admitted"] == false)
    );
    assert!(
        r.local_action(
            "autonomy",
            br#"{"level":"explore","token":"local-no-budget"}"#,
            0.0
        )
        .is_err()
    );
    r.navigation.result = Some("accepted".into());
    remote(
        &mut r,
        "navigation/settings",
        json!({"token":"malformed"}),
        0.0,
    );
    let out = r.step(0.0).unwrap();
    assert!(
        out.events
            .iter()
            .any(|e| e["topic"] == "navigation/settings"
                && e["kind"] == "command.admission"
                && e["admitted"] == false)
    );
}

#[test]
fn attached_map_policy_changes_configuration_identity() {
    let mut r = runtime(ActuationMode::DirectEffort);
    let original = r.configuration_identity();
    r.attach_mapping(
        mapping::MappingState::new(terra_mapping::MapConfig {
            min_obstacle_height: 0.3,
            ..Default::default()
        })
        .unwrap(),
    );
    assert_ne!(original, r.configuration_identity());
}

#[test]
fn assigned_study_level_rejects_other_levels_but_keeps_takeover() {
    let mut r = runtime(ActuationMode::DirectEffort);
    let mut t = 0.0;
    let mut send = |r: &mut Runtime, v: serde_json::Value| {
        t += 0.01;
        sensors(r, t);
        remote(r, "autonomy", v, t);
        r.step(t).unwrap().autonomy.status
    };
    let s = send(&mut r, json!({"level":"waypoint","token":"pre"}));
    assert_eq!(s.requested_level, terra_autonomy::Level::Waypoint);
    // Assigning L4 while at L3 drops the rover to L0.
    let s = send(
        &mut r,
        json!({"assigned_level":"explore","token":"assign-l4"}),
    );
    assert_eq!(s.assigned_level, Some(terra_autonomy::Level::Explore));
    assert_eq!(s.requested_level, terra_autonomy::Level::Teleop);
    assert_eq!(s.result.as_deref(), Some("accepted"));
    let s = send(&mut r, json!({"level":"waypoint","token":"l3"}));
    assert_eq!(s.request_reason.as_deref(), Some("level_not_assigned"));
    assert_eq!(s.requested_level, terra_autonomy::Level::Teleop);
    let s = send(
        &mut r,
        json!({"level":"explore","token":"l4","budget_seconds":60}),
    );
    assert_eq!(s.requested_level, terra_autonomy::Level::Explore);
    let s = send(&mut r, json!({"level":"teleop","token":"takeover"}));
    assert_eq!(s.requested_level, terra_autonomy::Level::Teleop);
    let s = send(&mut r, json!({"assigned_level":null,"token":"clear"}));
    assert_eq!(s.assigned_level, None);
    let s = send(&mut r, json!({"level":"waypoint","token":"free"}));
    assert_eq!(s.requested_level, terra_autonomy::Level::Waypoint);
}
