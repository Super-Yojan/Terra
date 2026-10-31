//! Central timing defaults for onboard autonomy and its planning dependencies.
//!
//! All values are seconds. Edit defaults here and rebuild. Existing per-instance
//! configuration (map timeout, planner horizon, exploration/confirmation settings)
//! can still override its corresponding default. Mission budgets are supplied per
//! run, not chosen here. These values preserve the previous behavior; they are not
//! a claim that the defaults have been validated for every physical rover.
//!
//! Freshness checks use age < limit. Interval/expiry checks generally use >=.
//! Sensor timestamps must share the control clock and cannot be in the future.

// DATA AND COMMAND FRESHNESS: separate limits for separate evidence streams.
/// Maximum occupancy observation age. Runtime override: arbiter map timeout setter.
pub const MAP_FRESHNESS_SECONDS: f64 = 2.5;
/// Maximum pose age for modes that require fresh localization.
pub const POSE_FRESHNESS_SECONDS: f64 = 0.5;
/// Maximum age of a teleop command before its intent expires.
pub const OPERATOR_COMMAND_FRESHNESS_SECONDS: f64 = 0.5;
/// Maximum age of advertised detector capability.
pub const DETECTOR_FRESHNESS_SECONDS: f64 = 0.5;
/// Maximum displayed planner-trace age; this is a diagnostic lifetime.
pub const PLANNER_TRACE_FRESHNESS_SECONDS: f64 = 0.5;

/// Maximum age of relay status used for source/configuration admission.
pub const HARDWARE_FRESHNESS_SECONDS: f64 = 0.5;
/// Execution-time admission age for settings and other control requests.
pub const CONTROL_REQUEST_FRESHNESS_SECONDS: f64 = 0.5;
/// IMU and VIO estimator defaults. The IMU limit was 0.1 s; the 2026-10-27 sensor timing
/// runs saw a Debug build push it past that while iOS delivered every sample within 3 ms.
pub const IMU_FRESHNESS_SECONDS: f64 = 0.5;
pub const VIO_FRESHNESS_SECONDS: f64 = 0.35;

// PLANNING: frequency and prediction, rather than mission expiry.
/// Maximum reuse age for an admissible cached local plan.
pub const LOCAL_PLAN_CACHE_SECONDS: f64 = 0.1;
/// Step duration supplied to the local planner for velocity sampling.
pub const PLANNER_STEP_SECONDS: f64 = 0.1;
/// Interval between periodic route-replanning opportunities.
pub const ROUTE_RECHECK_SECONDS: f64 = 0.2;
/// Minimum interval between supervised frontier proposals.
pub const FRONTIER_PROPOSAL_INTERVAL_SECONDS: f64 = 0.1;
/// Default local-planner prediction horizon (also configurable through PlannerConfig).
pub const PLANNER_HORIZON_SECONDS: f64 = 2.0;

// OPERATOR APPROVAL: how long a proposed destination remains eligible.
/// Lifetime of a supervised proposal awaiting approval.
pub const PROPOSAL_LIFETIME_SECONDS: f64 = 30.0;
/// How long to exclude a location rejected by the operator.
pub const REJECTED_PROPOSAL_SECONDS: f64 = 30.0;

// TURNING: elapsed time controls allowed changes in yaw rate.
/// Assumed first timestep after the turning limiter is reset.
pub const TURNING_INITIAL_STEP_SECONDS: f64 = 0.01;
/// Largest accepted gap between turning updates.
pub const TURNING_MAX_STEP_SECONDS: f64 = 0.1;
/// Numerical tolerance on the maximum gap; not an operational grace period.
pub const TURNING_STEP_TOLERANCE_SECONDS: f64 = 0.000000001;

// EXPLORATION: a zero recovery grace means end on the first qualifying hold.
/// Time blocked or without sufficient progress before blacklisting an exploration goal.
pub const EXPLORATION_STUCK_SECONDS: f64 = 5.0;
/// Exclusion period for failed exploration frontiers.
pub const EXPLORATION_BLACKLIST_SECONDS: f64 = 30.0;
/// Prediction horizon cap when approaching an exploration frontier.
pub const EXPLORATION_APPROACH_HORIZON_SECONDS: f64 = 1.0;
/// Minimum interval between exploration progress log events.
pub const EXPLORATION_PROGRESS_PERIOD_SECONDS: f64 = 1.0;
/// Continuous map-stale hold before ending exploration. Zero means immediate.
///
/// Motion is held from the first stale tick; this only decides when the run ends.
/// A rendered or phone depth map can arrive later than the 0.5 s freshness limit
/// for a frame or two, which used to end a run within its first second.
pub const EXPLORATION_MAP_STALE_GRACE_SECONDS: f64 = 3.0;
/// Longest in-place look-around before exploration gives up on it (a full turn
/// at the default 0.35 rad/s takes about 18 s).
pub const EXPLORATION_SCAN_TIMEOUT_SECONDS: f64 = 40.0;
/// Continuous unhealthy-sensor hold before ending exploration. Zero means immediate.
pub const EXPLORATION_SAFETY_HOLD_GRACE_SECONDS: f64 = 0.0;

// TARGET SEARCH: confirmation and navigation progress.
/// Minimum time span across supporting detection frames.
pub const SEARCH_CONFIRMATION_SPAN_SECONDS: f64 = 0.5;
/// Maximum target-observation age and gap between candidate frames.
pub const SEARCH_OBSERVATION_FRESHNESS_SECONDS: f64 = 0.5;
/// Time without target-search navigation progress before invalidating the goal.
pub const SEARCH_STUCK_SECONDS: f64 = 10.0;
/// Exclusion period for failed target-search locations.
pub const SEARCH_BLACKLIST_SECONDS: f64 = 30.0;

/// Maximum cached authority age when issuing a new remote mission request.
/// This does not extend manual intent leases or any onboard sensor limit.
pub const REMOTE_AUTHORITY_CACHE_SECONDS: f64 = 90.0;
