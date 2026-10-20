use terra_autonomy::*;
use terra_navigation::{Pose, Twist};
/// Build healthy origin-pose evidence at `now` seconds with no map and zero measured motion.
fn input(now: f64) -> ArbiterInput<'static> {
    ArbiterInput {
        now,
        pose: Pose::default(),
        pose_time: now,
        map: None,
        map_time: None,
        map_revision: 0,
        measured: Twist::default(),
        healthy: true,
    }
}
/// Build an unbound 1 m/s forward teleop request with no yaw rate or session ordering.
fn tele() -> TeleopRequest {
    TeleopRequest {
        linear: 1.,
        angular: 0.,
        run_id: None,
        authority_revision: None,
        operator_session_id: None,
        sequence: None,
    }
}
/// Reselect teleop to clear an old motion lease, then verify stop survives an unhealthy reset and healthy reset does not replay motion.
#[test]
fn takeover_clears_old_lease_and_stop_is_latched() {
    let mut a = AutonomyArbiter::default();
    a.accept_operator(tele(), 0.);
    assert_eq!(a.step(input(0.1)).twist.linear, 1.);
    a.set_level(LevelRequest {
        level: Level::Teleop,
        token: "take-1".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    assert_eq!(a.step(input(0.2)).twist, Twist::default());
    a.accept_operator(tele(), 0.3);
    a.set_safety(
        SafetyRequest {
            action: SafetyAction::Stop,
            token: "s-1".into(),
        },
        true,
    );
    assert_eq!(a.step(input(0.4)).status.safety, "emergency_stop");
    a.set_safety(
        SafetyRequest {
            action: SafetyAction::Reset,
            token: "r-1".into(),
        },
        false,
    );
    assert_eq!(a.step(input(0.5)).status.safety, "emergency_stop");
    a.set_safety(
        SafetyRequest {
            action: SafetyAction::Reset,
            token: "r-2".into(),
        },
        true,
    );
    assert_eq!(a.step(input(0.6)).twist, Twist::default());
}
/// Reject goals outside waypoint authority, expire operator input, and hold on missing maps or backward control time.
#[test]
fn stale_inputs_and_no_implicit_goal_mode() {
    let mut a = AutonomyArbiter::default();
    assert!(!a.accept_goal(
        &terra_waypoint::GoalCommand::Local {
            x: 3.,
            y: 0.,
            yaw: None,
            token: Some("g".into())
        },
        50.
    ));
    a.accept_operator(tele(), 0.);
    assert_eq!(a.step(input(0.6)).twist, Twist::default());
    a.set_level(LevelRequest {
        level: Level::Waypoint,
        token: "mode".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    assert_eq!(a.step(input(0.7)).status.reason, "map_stale");
    assert_eq!(a.step(input(0.65)).status.reason, "invalid_time");
}
/// Replay an identical takeover token after accepting fresh operator input; idempotency must preserve the new lease.
#[test]
fn duplicate_takeover_does_not_clear_fresh_input_twice() {
    let mut a = AutonomyArbiter::default();
    let r = LevelRequest {
        level: Level::Teleop,
        token: "take-1".into(),
        budget_seconds: None,
        budget_minutes: None,
    };
    a.set_level(r.clone());
    a.accept_operator(tele(), 0.);
    a.set_level(r);
    assert_eq!(a.step(input(0.1)).twist.linear, 1.);
}
/// Offer a frontier on a known free core, hold motion until approval, and accept approval against fresh map and pose evidence.
#[test]
fn supervised_proposes_but_waits_for_approval() {
    let mut a = AutonomyArbiter::default();
    a.set_level(LevelRequest {
        level: Level::Supervised,
        token: "s".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    let mut m = terra_mapping::MapSnapshot {
        width: 40,
        height: 40,
        resolution: 0.25,
        origin_x: -5.,
        origin_y: -5.,
        occupancy: vec![-1; 1600],
    };
    for y in 12..28 {
        for x in 12..28 {
            m.occupancy[y * 40 + x] = 0;
        }
    }
    let i = ArbiterInput {
        map: Some(&m),
        map_time: Some(0.1),
        ..input(0.1)
    };
    let o = a.step(i);
    assert!(o.proposal.is_some());
    assert_eq!(o.twist, Twist::default());
    let p = o.proposal.unwrap();
    a.decide_proposal(
        ProposalDecision {
            run_id: p.run_id.clone(),
            proposal_id: Some(p.proposal_id),
            decision: Decision::Approve,
            token: "yes".into(),
        },
        &ArbiterInput {
            map: Some(&m),
            map_time: Some(0.2),
            ..input(0.2)
        },
    );
    assert_eq!(
        a.step(ArbiterInput {
            map: Some(&m),
            map_time: Some(0.3),
            ..input(0.3)
        })
        .status
        .result
        .as_deref(),
        Some("accepted")
    );
}
/// Accept a goal, switch to teleop, and replay the old token; the receipt stays accepted without activating a goal.
#[test]
fn duplicate_goal_token_does_not_relaunch() {
    let mut a = AutonomyArbiter::default();
    a.set_level(LevelRequest {
        level: Level::Waypoint,
        token: "mode".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    let g = terra_waypoint::GoalCommand::Local {
        x: 3.,
        y: 0.,
        yaw: None,
        token: Some("goal-1".into()),
    };
    assert!(a.accept_goal(&g, 50.));
    a.set_level(LevelRequest {
        level: Level::Teleop,
        token: "take".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    assert!(a.accept_goal(&g, 50.));
    assert_eq!(
        a.step(input(0.1)).goal.state,
        terra_waypoint::GoalState::Idle
    );
}
/// Submit a goal with nonfinite yaw under waypoint authority and require rejection.
#[test]
fn invalid_direct_goal_is_rejected() {
    let mut a = AutonomyArbiter::default();
    a.set_level(LevelRequest {
        level: Level::Waypoint,
        token: "m".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    assert!(!a.accept_goal(
        &terra_waypoint::GoalCommand::Local {
            x: 1.,
            y: 0.,
            yaw: Some(f64::NAN),
            token: Some("g".into())
        },
        50.
    ));
}
/// Accept a zero command then a backward-time motion packet before ticking; the rover must stay stopped.
#[test]
fn older_input_cannot_overwrite_newer_zero_between_steps() {
    let mut a = AutonomyArbiter::default();
    let mut zero = tele();
    zero.linear = 0.;
    a.accept_operator(zero, 0.2);
    a.accept_operator(tele(), 0.1);
    assert_eq!(a.step(input(0.3)).twist, Twist::default());
}
/// Accept session sequence 2 with zero velocity, then reject sequence 1 even when its receipt time is later.
#[test]
fn reordered_session_packet_cannot_renew_motion() {
    let mut a = AutonomyArbiter::default();
    let mut zero = tele();
    zero.linear = 0.;
    zero.operator_session_id = Some("session".into());
    zero.sequence = Some(2);
    a.accept_operator(zero, 0.1);
    let mut old = tele();
    old.operator_session_id = Some("session".into());
    old.sequence = Some(1);
    a.accept_operator(old, 0.2);
    assert_eq!(a.step(input(0.3)).twist, Twist::default());
}
/// Alternate held keyboard stop and healthy resets with unique tokens; each held stop must latch again.
#[test]
fn keyboard_stop_relatches_after_each_reset() {
    let mut a = AutonomyArbiter::default();
    for n in 0..3 {
        a.hold_emergency_stop();
        assert_eq!(a.step(input(n as f64)).status.safety, "emergency_stop");
        a.set_safety(
            SafetyRequest {
                action: SafetyAction::Reset,
                token: format!("reset-{n}"),
            },
            true,
        );
        assert_eq!(a.step(input(n as f64 + 0.1)).status.safety, "clear");
    }
}
/// Generate a proposal in a new run and reject an approval carrying the old run ID despite a matching proposal number.
#[test]
fn previous_run_approval_cannot_launch_reused_proposal_id() {
    let mut a = AutonomyArbiter::default();
    a.run_id = "new".into();
    a.set_level(LevelRequest {
        level: Level::Supervised,
        token: "level".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    let mut m = terra_mapping::MapSnapshot {
        width: 40,
        height: 40,
        resolution: 0.25,
        origin_x: -5.,
        origin_y: -5.,
        occupancy: vec![-1; 1600],
    };
    for y in 12..28 {
        for x in 12..28 {
            m.occupancy[y * 40 + x] = 0;
        }
    }
    let p = a
        .step(ArbiterInput {
            map: Some(&m),
            map_time: Some(0.1),
            ..input(0.1)
        })
        .proposal
        .unwrap();
    a.decide_proposal(
        ProposalDecision {
            run_id: "old".into(),
            proposal_id: Some(p.proposal_id),
            decision: Decision::Approve,
            token: "old-approval".into(),
        },
        &ArbiterInput {
            map: Some(&m),
            map_time: Some(0.2),
            ..input(0.2)
        },
    );
    let out = a.step(ArbiterInput {
        map: Some(&m),
        map_time: Some(0.3),
        ..input(0.3)
    });
    assert_eq!(out.status.result.as_deref(), Some("rejected"));
    assert_eq!(out.twist, Twist::default());
}
/// Reject teleop packets bound to an old run or a superseded authority revision.
#[test]
fn canonical_motion_is_bound_to_authority_and_run() {
    let mut a = AutonomyArbiter::default();
    a.run_id = "new-run".into();
    let mut r = tele();
    r.run_id = Some("old-run".into());
    r.authority_revision = Some(0);
    a.accept_operator(r, 0.);
    assert_eq!(a.step(input(0.1)).twist, Twist::default());
    a.set_level(LevelRequest {
        level: Level::Teleop,
        token: "takeover".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    let mut r = tele();
    r.run_id = Some("new-run".into());
    r.authority_revision = Some(0);
    a.accept_operator(r, 0.2);
    assert_eq!(a.step(input(0.3)).twist, Twist::default());
}

/// Follow a direct waypoint without a map, then require stale-pose and cancellation holds.
#[test]
fn direct_waypoint_needs_no_map_but_retains_sensor_and_cancel_safety() {
    let mut a = AutonomyArbiter::default();
    a.set_level(LevelRequest {
        level: Level::WaypointDirect,
        token: "direct".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    assert!(a.accept_goal(
        &terra_waypoint::GoalCommand::Local {
            x: 2.,
            y: 0.,
            yaw: None,
            token: Some("direct-goal".into())
        },
        50.
    ));
    let out = a.step(input(0.1));
    assert!(out.twist.linear > 0.);
    assert!(out.status.supported_levels.contains(&Level::WaypointDirect));
    let mut stale = input(1.);
    stale.pose_time = 0.;
    assert_eq!(a.step(stale).twist, Twist::default());
    assert!(a.accept_goal(&terra_waypoint::GoalCommand::Cancel, 50.));
    assert_eq!(a.step(input(1.1)).twist, Twist::default());
}

/// Compare direct and planned waypoint authority on a fully occupied map; stop must still latch and reject subsequent goals.
#[test]
fn l2_bypasses_occupied_map_while_l3_stops_and_stop_still_latches() {
    let map = terra_mapping::MapSnapshot {
        width: 40,
        height: 40,
        resolution: 0.25,
        origin_x: -5.,
        origin_y: -5.,
        occupancy: vec![100; 1600],
    };
    for (level, moves) in [(Level::WaypointDirect, true), (Level::Waypoint, false)] {
        let mut a = AutonomyArbiter::default();
        a.set_level(LevelRequest {
            level,
            token: "mode".into(),
            budget_seconds: None,
            budget_minutes: None,
        });
        assert!(a.accept_goal(
            &terra_waypoint::GoalCommand::Local {
                x: 2.,
                y: 0.,
                yaw: None,
                token: Some("goal".into())
            },
            50.
        ));
        let out = a.step(ArbiterInput {
            map: Some(&map),
            map_time: Some(0.1),
            ..input(0.1)
        });
        assert_eq!(out.twist.linear > 0., moves);
        a.set_safety(
            SafetyRequest {
                action: SafetyAction::Stop,
                token: "stop".into(),
            },
            true,
        );
        assert_eq!(a.step(input(0.2)).twist, Twist::default());
        assert!(!a.accept_goal(
            &terra_waypoint::GoalCommand::Local {
                x: 3.,
                y: 0.,
                yaw: None,
                token: Some("new-goal".into())
            },
            50.
        ));
    }
}

/// Cancel a direct waypoint and replay its accepted token; return the cached receipt without restarting motion.
#[test]
fn cancelled_l2_token_never_replays_motion() {
    let mut a = AutonomyArbiter::default();
    a.set_level(LevelRequest {
        level: Level::WaypointDirect,
        token: "direct".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    let goal = terra_waypoint::GoalCommand::Local {
        x: 2.,
        y: 0.,
        yaw: None,
        token: Some("direct-goal".into()),
    };
    assert!(a.accept_goal(&goal, 50.));
    assert!(a.step(input(0.1)).twist.linear > 0.);
    assert!(a.accept_goal(&terra_waypoint::GoalCommand::Cancel, 50.));
    assert!(a.accept_goal(&goal, 50.)); // Correlated receipt, not a new execution.
    assert_eq!(a.step(input(0.2)).twist, Twist::default());
}

/// Block assisted teleop with an occupied map, verify the bounded 121-candidate trace, and clear it on emergency stop.
#[test]
fn blocked_planner_trace_is_published_and_cleared_on_stop() {
    let mut arbiter = AutonomyArbiter::default();
    arbiter.set_level(LevelRequest {
        level: Level::AssistedTeleop,
        token: "trace".into(),
        budget_seconds: None,
        budget_minutes: None,
    });
    arbiter.accept_operator(tele(), 0.0);
    let map = terra_mapping::MapSnapshot {
        width: 40,
        height: 40,
        resolution: 0.25,
        origin_x: -5.,
        origin_y: -5.,
        occupancy: vec![100; 1600],
    };
    let evidence = |now| ArbiterInput {
        now,
        pose: Pose::default(),
        pose_time: now,
        map: Some(&map),
        map_time: Some(now),
        map_revision: 1,
        measured: Twist::default(),
        healthy: true,
    };
    let output = arbiter.step(evidence(0.1));
    assert_eq!(output.status.reason, "obstacle_blocked");
    let trace = output.status.planner_trace.as_ref().unwrap();
    assert_eq!(trace.candidates.len(), 121);
    assert_eq!(trace.age, 0.);
    assert!(trace.selected.is_none());
    assert!(serde_json::to_vec(&output.status).unwrap().len() < 65_536);
    arbiter.set_safety(
        SafetyRequest {
            action: SafetyAction::Stop,
            token: "stop-trace".into(),
        },
        true,
    );
    assert!(arbiter.step(evidence(0.2)).status.planner_trace.is_none());
}

#[test]
fn manual_does_not_need_pose_but_still_stops_for_health_and_expired_input() {
    let mut a = AutonomyArbiter::default();
    a.accept_operator(tele(), 1.);
    let mut evidence = input(1.1);
    evidence.pose_time = 0.;
    evidence.pose.x = f64::NAN;
    assert_eq!(a.step(evidence).twist.linear, 1.);
    let mut failed_link = input(1.2);
    failed_link.healthy = false;
    assert_eq!(a.step(failed_link).status.reason, "sensor_unhealthy");
    assert_eq!(a.step(input(2.)).twist, Twist::default());
}
