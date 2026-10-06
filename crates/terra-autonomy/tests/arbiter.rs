use terra_autonomy::*;
use terra_navigation::{Pose, Twist};
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
#[test]
fn takeover_clears_old_lease_and_stop_is_latched() {
    let mut a = AutonomyArbiter::default();
    a.accept_operator(tele(), 0.);
    assert_eq!(a.step(input(0.1)).twist.linear, 1.);
    a.set_level(LevelRequest {
        level: Level::Teleop,
        token: "take-1".into(),
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
    });
    assert_eq!(a.step(input(0.7)).status.reason, "map_stale");
    assert_eq!(a.step(input(0.65)).status.reason, "invalid_time");
}
#[test]
fn duplicate_takeover_does_not_clear_fresh_input_twice() {
    let mut a = AutonomyArbiter::default();
    let r = LevelRequest {
        level: Level::Teleop,
        token: "take-1".into(),
    };
    a.set_level(r.clone());
    a.accept_operator(tele(), 0.);
    a.set_level(r);
    assert_eq!(a.step(input(0.1)).twist.linear, 1.);
}
#[test]
fn supervised_proposes_but_waits_for_approval() {
    let mut a = AutonomyArbiter::default();
    a.set_level(LevelRequest {
        level: Level::Supervised,
        token: "s".into(),
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
#[test]
fn duplicate_goal_token_does_not_relaunch() {
    let mut a = AutonomyArbiter::default();
    a.set_level(LevelRequest {
        level: Level::Waypoint,
        token: "mode".into(),
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
    });
    assert!(a.accept_goal(&g, 50.));
    assert_eq!(
        a.step(input(0.1)).goal.state,
        terra_waypoint::GoalState::Idle
    );
}
#[test]
fn invalid_direct_goal_is_rejected() {
    let mut a = AutonomyArbiter::default();
    a.set_level(LevelRequest {
        level: Level::Waypoint,
        token: "m".into(),
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
#[test]
fn older_input_cannot_overwrite_newer_zero_between_steps() {
    let mut a = AutonomyArbiter::default();
    let mut zero = tele();
    zero.linear = 0.;
    a.accept_operator(zero, 0.2);
    a.accept_operator(tele(), 0.1);
    assert_eq!(a.step(input(0.3)).twist, Twist::default());
}
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
#[test]
fn previous_run_approval_cannot_launch_reused_proposal_id() {
    let mut a = AutonomyArbiter::default();
    a.run_id = "new".into();
    a.set_level(LevelRequest {
        level: Level::Supervised,
        token: "level".into(),
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
    });
    let mut r = tele();
    r.run_id = Some("new-run".into());
    r.authority_revision = Some(0);
    a.accept_operator(r, 0.2);
    assert_eq!(a.step(input(0.3)).twist, Twist::default());
}
