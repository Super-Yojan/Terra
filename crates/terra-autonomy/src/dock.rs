//! Bucket docking: line the dump outlet up over a detected bucket, then run the
//! dump gate.
//!
//! A dock starts on a confirmed bucket from the object map, at most
//! [`DockConfig::max_start_distance`] away. From then on it steers on fresh
//! per-frame sightings of that bucket, not on the map's averaged position, and
//! carries the estimate between camera frames with the rover pose. Phases:
//!
//! `lock` (wait for agreeing fixes) → `creep` (point at the bucket, slow down
//! near the dock range) → `settle` (stop, re-check) → `aligned` (flag raised,
//! drive locked) → `dumping` (gate open, then closed) → `done`.
//! `backoff` reverses a short way (without checking the map) and locks again; `failed` hands the rover
//! back to the operator.
//!
//! The gate opens only from `aligned`, with the rover still, after operator
//! approval or the arbiter's auto-dump policy. Any hold closes it at once.
//! The controller never touches actuators: [`DockTick::gate_open`] is the
//! request, and the host maps it to the gate servo (or the simulator's DUMP).

use serde::{Deserialize, Serialize};
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, Twist};

/// Occupancy at or above this blocks the docking corridor (the planner's own threshold).

/// How far past the dock range the camera may already have lost the bucket
/// (out of view under the phone, or nearer than the depth minimum), metres.
pub const DOCK_BLIND_MARGIN: f64 = 0.5;

fn default_target_clear_margin() -> f64 {
    0.3
}

fn default_open_range_timeout() -> f64 {
    15.
}

/// How far past `blind_range` a dock started too close reverses before it
/// waits for a lock, metres.
const OPEN_RANGE_MARGIN: f64 = 0.15;

/// Geometry, tolerances and speeds for docking. Defaults fit Terra (CrabTrapBot).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DockConfig {
    /// Rover centre to dump-outlet centre along the body x axis when the outlet
    /// is over the bucket centre, metres.
    pub outlet_ahead: f64,
    /// Bucket radius: sightings hit the near wall, so the centre is this much further.
    pub target_radius: f64,
    /// Aligned when the bucket centre is within this of the rover axis, metres.
    pub lateral_tolerance: f64,
    /// Aligned when the bucket centre is within this of `outlet_ahead`, metres.
    pub range_tolerance: f64,
    /// A fix counts as "in line" (for a verified blind finish) when the bucket
    /// bearing is within this, radians. Not part of the line-up test: for a
    /// round bucket only where the outlet sits matters, and close in a small
    /// lateral offset is a large bearing.
    pub heading_tolerance: f64,
    /// Once aligned, the line-up may drift to this multiple of the lateral and
    /// range tolerances before the dock backs off, so fix noise while parked
    /// does not undo an alignment.
    pub aligned_slack: f64,
    /// Farthest bucket a dock may start on, metres.
    pub max_start_distance: f64,
    /// Forward speed cap within 1 m of the dock range, m/s.
    pub creep_speed: f64,
    /// Forward speed cap farther out, m/s.
    pub approach_speed: f64,
    /// Smallest forward command while creeping, so the motors do not stall, m/s.
    pub min_speed: f64,
    /// Yaw-rate cap, rad/s.
    pub max_turn: f64,
    /// Whether the rover may turn in place.
    pub can_spin: bool,
    /// Tightest turn while moving when it cannot spin, metres.
    pub min_turn_radius: f64,
    /// Spinners turn in place above this bearing, radians.
    pub spin_heading: f64,
    /// Back off above this bearing while creeping, radians.
    pub backoff_heading: f64,
    /// Reverse this far before locking again, metres.
    pub backoff_distance: f64,
    pub backoff_speed: f64,
    /// Backoffs allowed before the dock fails.
    pub max_retries: u32,
    /// Agreeing fixes needed to lock.
    pub lock_fixes: u32,
    /// Fail when no lock within this many seconds.
    pub lock_timeout: f64,
    /// Stop and wait for the bucket when the newest fix is older than this
    /// while still far, seconds.
    pub fix_timeout: f64,
    /// Seconds stopped waiting for a fix before backing off with `lost_fix`.
    pub reacquire_seconds: f64,
    /// Inside this body-x range of the bucket centre the camera may lose it;
    /// the rover finishes on odometry and settle reports the result unverified.
    pub blind_range: f64,
    /// Occupied cells within this distance of the bucket's estimated rim are
    /// the bucket itself, not an obstacle in the corridor, metres. Covers the
    /// error of a centre estimate frozen since the camera lost the bucket.
    #[serde(default = "default_target_clear_margin")]
    pub target_clear_margin: f64,
    /// Seconds a dock started inside `blind_range` may spend reversing out to
    /// where the camera can see the bucket before it fails `too_close`.
    #[serde(default = "default_open_range_timeout")]
    pub open_range_timeout: f64,
    /// A blind finish still counts as verified when the last in-line camera fix
    /// came within this much travel of it, metres: an outlet over the camera
    /// always finishes blind, with the bucket under the phone.
    pub blind_verify_travel: f64,
    /// Seconds still before `aligned`.
    pub settle_seconds: f64,
    /// Seconds settle waits for a fresh fix before accepting an unverified alignment.
    pub settle_timeout: f64,
    /// Seconds the gate stays open, then closed before `done`.
    pub gate_open_seconds: f64,
    pub gate_close_seconds: f64,
    /// Rover half width and half length for the corridor check, metres.
    pub half_width: f64,
    pub half_length: f64,
    /// Fail after the corridor stays blocked this long, seconds.
    pub blocked_timeout: f64,
    /// Fixes farther than this from the estimate are another object, metres.
    pub match_radius: f64,
    /// Before the lock the map estimate may be off by this much, metres.
    pub lock_match_radius: f64,
    /// Weight of a new fix in the estimate, 0..=1.
    pub blend: f64,
}

impl Default for DockConfig {
    fn default() -> Self {
        Self::for_body(0.85, 0.85, false, 0.5)
    }
}

impl DockConfig {
    /// Defaults for a body of `length` × `width` metres with the outlet just
    /// past the front bumper (as in the Zorvane dumper).
    pub fn for_body(length: f64, width: f64, can_spin: bool, min_turn_radius: f64) -> Self {
        let target_radius = 0.1524;
        Self {
            outlet_ahead: length / 2. + target_radius + 0.03,
            target_radius,
            // A tennis ball (0.067 m) through the outlet lands inside the
            // 0.30 m rim while the outlet centre is within ~0.12 m of the
            // bucket centre. 0.06 m lateral and 0.07 m range, at the aligned
            // slack, is 0.101 m worst case, leaving ~17 mm for the ball sitting
            // off-centre in a 0.10 m outlet (the rest of the margin is thin:
            // fix noise and centre-estimate bias are not in it).
            lateral_tolerance: 0.06,
            range_tolerance: 0.07,
            heading_tolerance: 10f64.to_radians(),
            aligned_slack: 1.1,
            max_start_distance: 3.,
            creep_speed: 0.1,
            approach_speed: 0.25,
            min_speed: 0.03,
            max_turn: 0.4,
            can_spin,
            min_turn_radius,
            spin_heading: 12f64.to_radians(),
            backoff_heading: 25f64.to_radians(),
            backoff_distance: 0.3,
            backoff_speed: 0.08,
            max_retries: 4,
            lock_fixes: 3,
            lock_timeout: 5.,
            fix_timeout: 1.5,
            reacquire_seconds: 2.,
            blind_range: length / 2. + target_radius + DOCK_BLIND_MARGIN,
            target_clear_margin: default_target_clear_margin(),
            open_range_timeout: default_open_range_timeout(),
            blind_verify_travel: 0.7,
            settle_seconds: 0.5,
            settle_timeout: 2.,
            gate_open_seconds: 1.5,
            gate_close_seconds: 0.5,
            half_width: width / 2.,
            half_length: length / 2.,
            blocked_timeout: 3.,
            match_radius: 0.25,
            lock_match_radius: 0.5,
            blend: 0.6,
        }
    }

    pub fn valid(&self) -> bool {
        let positive = [
            self.outlet_ahead,
            self.lateral_tolerance,
            self.range_tolerance,
            self.heading_tolerance,
            self.max_start_distance,
            self.creep_speed,
            self.approach_speed,
            self.max_turn,
            self.min_turn_radius,
            self.spin_heading,
            self.backoff_heading,
            self.backoff_distance,
            self.backoff_speed,
            self.lock_timeout,
            self.fix_timeout,
            self.reacquire_seconds,
            self.settle_timeout,
            self.gate_open_seconds,
            self.half_width,
            self.half_length,
            self.blocked_timeout,
            self.match_radius,
            self.lock_match_radius,
            self.blend,
            self.target_clear_margin,
            self.open_range_timeout,
        ];
        positive.iter().all(|v| v.is_finite() && *v > 0.)
            && [
                self.target_radius,
                self.min_speed,
                self.blind_range,
                self.blind_verify_travel,
                self.settle_seconds,
                self.gate_close_seconds,
            ]
            .iter()
            .all(|v| v.is_finite() && *v >= 0.)
            && self.blend <= 1.
            && self.min_speed <= self.creep_speed
            && self.creep_speed <= self.approach_speed
            && self.lock_fixes > 0
            && self.aligned_slack.is_finite()
            && self.aligned_slack >= 1.
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DockPhase {
    Lock,
    Creep,
    Settle,
    Aligned,
    Dumping,
    Closing,
    Backoff,
    Done,
    Failed,
    Cancelled,
}

impl DockPhase {
    pub fn terminal(self) -> bool {
        matches!(self, Self::Done | Self::Failed | Self::Cancelled)
    }
}

/// Who opens the gate once the dock is aligned.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DumpApproval {
    /// The operator approves each dump (L0–L3).
    Operator,
    /// The dock dumps on its own once aligned and verified (L4–L5).
    Auto,
}

/// The bucket a dock is aimed at, from the object map.
#[derive(Clone, Debug, PartialEq)]
pub struct DockTarget {
    pub object_id: Option<u32>,
    pub color: Option<String>,
    /// Local world position of the bucket centre, metres.
    pub x: f64,
    pub y: f64,
}

/// Errors of the bucket centre against the dock pose, in the rover frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DockErrors {
    /// Bucket centre left of the rover axis, metres.
    pub lateral: f64,
    /// Bucket centre beyond the dock range, metres (negative is too close).
    pub range: f64,
    /// Bearing to the bucket centre, degrees, left positive.
    pub heading_deg: f64,
}

/// Dashboard snapshot of a dock, kept after it ends.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DockStatus {
    pub schema_version: u32,
    /// Dock number in this run; a dump request must name it.
    pub dock_id: u64,
    pub token: String,
    pub phase: DockPhase,
    /// Machine-readable reason for the phase, such as `lost_fix` or `operator_cancel`.
    pub reason: String,
    pub object_id: Option<u32>,
    pub color: Option<String>,
    /// Current estimate of the bucket centre, local world metres.
    pub target: [f64; 2],
    pub errors: DockErrors,
    /// True while every alignment tolerance holds.
    pub in_tolerance: bool,
    /// Seconds since the last accepted fix, when there was one.
    pub fix_age: Option<f64>,
    pub fixes: u32,
    pub retries: u32,
    /// Aligned on a fresh camera fix. False when it finished on odometry alone.
    pub verified: bool,
    pub approval: DumpApproval,
    /// Aligned and waiting for the operator to approve the dump.
    pub awaiting_approval: bool,
    pub gate_open: bool,
    pub elapsed: f64,
}

/// One tick of the dock's request to the arbiter.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DockTick {
    pub twist: Twist,
    /// The twist is a turn in place, which the vehicle may run when it can spin.
    pub in_place: bool,
    pub gate_open: bool,
    /// Machine-readable reason, used as the arbiter's tick reason.
    pub reason: &'static str,
}

/// One dock from start to its terminal phase.
#[derive(Clone, Debug)]
pub struct Dock {
    pub config: DockConfig,
    pub id: u64,
    pub token: String,
    pub phase: DockPhase,
    pub reason: String,
    target: DockTarget,
    started: f64,
    phase_since: f64,
    last_fix: Option<f64>,
    fixes: u32,
    lock_count: u32,
    retries: u32,
    verified: bool,
    /// Bucket body range at the last fix that had the rover in line.
    inline_fix_at: Option<f64>,
    approved: bool,
    approval: DumpApproval,
    still_since: Option<f64>,
    blocked_since: Option<f64>,
    backoff_from: Option<(f64, f64)>,
    /// When a dock started inside the blind range began reversing out.
    opening_since: Option<f64>,
    errors: DockErrors,
    in_tolerance: bool,
    now: f64,
    events: Vec<(&'static str, String)>,
}

impl Dock {
    /// Start on `target`; fails right away when it is too far from `pose`.
    pub fn start(
        config: DockConfig,
        id: u64,
        token: String,
        target: DockTarget,
        pose: Pose,
        now: f64,
    ) -> Result<Self, &'static str> {
        if !config.valid() {
            return Err("invalid_dock_config");
        }
        if !target.x.is_finite() || !target.y.is_finite() {
            return Err("invalid_target");
        }
        if (target.x - pose.x).hypot(target.y - pose.y) > config.max_start_distance {
            return Err("target_too_far");
        }
        let mut dock = Self {
            config,
            id,
            token,
            phase: DockPhase::Lock,
            reason: "started".into(),
            target,
            started: now,
            phase_since: now,
            last_fix: None,
            fixes: 0,
            lock_count: 0,
            retries: 0,
            verified: false,
            inline_fix_at: None,
            approved: false,
            approval: DumpApproval::Operator,
            still_since: None,
            blocked_since: None,
            backoff_from: None,
            opening_since: None,
            errors: DockErrors::default(),
            in_tolerance: false,
            now,
            events: Vec::new(),
        };
        dock.event("dock_started", "started");
        Ok(dock)
    }

    pub fn active(&self) -> bool {
        !self.phase.terminal()
    }

    /// Backing off, or reversing out of a start too close to see the bucket.
    pub fn reversing(&self) -> bool {
        self.phase == DockPhase::Backoff
            || (self.phase == DockPhase::Lock && self.opening_since.is_some())
    }

    pub fn target(&self) -> &DockTarget {
        &self.target
    }

    /// Events since the last call: `(kind, reason)`.
    pub fn take_events(&mut self) -> Vec<(&'static str, String)> {
        std::mem::take(&mut self.events)
    }

    fn event(&mut self, kind: &'static str, reason: &str) {
        self.events.push((kind, reason.into()));
    }

    fn enter(&mut self, phase: DockPhase, reason: &str) {
        if self.phase == phase {
            return;
        }
        self.phase = phase;
        self.reason = reason.into();
        self.phase_since = self.now;
        self.still_since = None;
        self.blocked_since = None;
        self.opening_since = None;
        match phase {
            DockPhase::Aligned => self.event("dock_aligned", reason),
            DockPhase::Dumping => self.event("dock_gate_opened", reason),
            DockPhase::Closing => self.event("dock_gate_closed", reason),
            DockPhase::Backoff => self.event("dock_backoff", reason),
            DockPhase::Done => self.event("dock_done", reason),
            DockPhase::Failed => self.event("dock_failed", reason),
            DockPhase::Cancelled => self.event("dock_cancelled", reason),
            DockPhase::Lock | DockPhase::Creep | DockPhase::Settle => {}
        }
    }

    /// Stop the dock. A dump in progress is cut short and the gate closes.
    pub fn cancel(&mut self, reason: &str) {
        if self.active() {
            self.enter(DockPhase::Cancelled, reason);
        }
    }

    /// Operator approval for the dump. Only valid while aligned.
    pub fn approve(&mut self) -> Result<(), &'static str> {
        if self.phase != DockPhase::Aligned {
            return Err("dock_not_aligned");
        }
        self.approved = true;
        Ok(())
    }

    /// One detector sighting in the local world frame (box centre at its
    /// measured depth). Sightings of another class, colour or position are
    /// ignored, except another-colour bucket in the path, which fails the dock.
    pub fn observe(
        &mut self,
        class: &str,
        color: Option<&str>,
        x: f64,
        y: f64,
        pose: Pose,
        now: f64,
    ) {
        if !self.active() || class != "bucket" || !x.is_finite() || !y.is_finite() {
            return;
        }
        let (dx, dy) = (x - pose.x, y - pose.y);
        let distance = dx.hypot(dy);
        if distance < 1e-3 {
            return;
        }
        // The near wall was hit; move the point to the bucket's centre.
        let scale = (distance + self.config.target_radius) / distance;
        let (cx, cy) = (pose.x + dx * scale, pose.y + dy * scale);
        if color != self.target.color.as_deref() {
            if self.target.color.is_some() && color.is_some() && self.in_path(cx, cy, pose) {
                self.enter(DockPhase::Failed, "other_bucket_in_path");
            }
            return;
        }
        let radius = if self.phase == DockPhase::Lock && self.lock_count < self.config.lock_fixes {
            self.config.lock_match_radius
        } else {
            self.config.match_radius
        };
        if (cx - self.target.x).hypot(cy - self.target.y) > radius {
            return;
        }
        let w = self.config.blend;
        self.target.x += w * (cx - self.target.x);
        self.target.y += w * (cy - self.target.y);
        self.last_fix = Some(now);
        self.fixes += 1;
        let (bx, by) = body(pose, self.target.x, self.target.y);
        self.inline_fix_at = (by.abs() <= self.config.lateral_tolerance
            && by.atan2(bx).abs() <= self.config.heading_tolerance)
            .then_some(bx);
        if self.phase == DockPhase::Lock {
            self.lock_count += 1;
        }
    }

    /// Another bucket sits between the rover and the target, or beside the target
    /// where the body will be.
    fn in_path(&self, x: f64, y: f64, pose: Pose) -> bool {
        let (bx, by) = body(pose, x, y);
        let (tx, _) = body(pose, self.target.x, self.target.y);
        bx > 0.
            && bx < tx + self.config.target_radius
            && by.abs() < self.config.half_width + 2. * self.config.target_radius
    }

    /// One control tick. `held` is the arbiter's safety hold for this tick, if
    /// any: it zeroes motion and closes the gate (a dump in progress fails).
    /// `approval` is the policy for the level the arbiter is running.
    pub fn step(
        &mut self,
        now: f64,
        pose: Pose,
        measured: Twist,
        map: Option<&MapSnapshot>,
        approval: DumpApproval,
        held: Option<&str>,
    ) -> DockTick {
        self.now = now;
        self.approval = approval;
        let (bx, by) = body(pose, self.target.x, self.target.y);
        let heading = by.atan2(bx);
        self.errors = DockErrors {
            lateral: by,
            range: bx - self.config.outlet_ahead,
            heading_deg: heading.to_degrees(),
        };
        let slack = if self.phase == DockPhase::Aligned {
            self.config.aligned_slack
        } else {
            1.
        };
        self.in_tolerance = by.abs() <= self.config.lateral_tolerance * slack
            && self.errors.range.abs() <= self.config.range_tolerance * slack;
        let still = measured.linear.abs() < 0.03 && measured.angular.abs() < 0.08;
        if still {
            self.still_since.get_or_insert(now);
        } else {
            self.still_since = None;
        }
        let still_for = self.still_since.map_or(0., |t| now - t);
        let fix_age = self.last_fix.map(|t| now - t);
        let stop = DockTick {
            reason: "dock_hold",
            ..DockTick::default()
        };
        if let Some(why) = held {
            if matches!(self.phase, DockPhase::Dumping | DockPhase::Closing) {
                self.enter(DockPhase::Failed, &format!("gate_interrupted:{why}"));
            }
            return stop;
        }
        match self.phase {
            DockPhase::Lock => {
                if self.lock_count >= self.config.lock_fixes {
                    self.enter(DockPhase::Creep, "locked");
                    return self.step(now, pose, measured, map, approval, None);
                }
                // Started (or backed off) inside the blind range: the camera
                // cannot see the bucket from here, so waiting would only time
                // out. Reverse straight out first; the lock clock starts once
                // the bucket can be in view.
                if self.lock_count == 0 && bx <= self.config.blind_range + OPEN_RANGE_MARGIN {
                    let since = *self.opening_since.get_or_insert(now);
                    if now - since > self.config.open_range_timeout {
                        self.enter(DockPhase::Failed, "too_close");
                        return stop;
                    }
                    self.phase_since = now;
                    return self.reverse(heading, "dock_open_range");
                }
                self.opening_since = None;
                if now - self.phase_since > self.config.lock_timeout {
                    self.enter(DockPhase::Failed, "no_fix");
                    return stop;
                }
                // A spinner may face the map estimate; anything else waits.
                if self.config.can_spin && heading.abs() > self.config.spin_heading {
                    return DockTick {
                        twist: Twist {
                            linear: 0.,
                            angular: (1.5 * heading)
                                .clamp(-self.config.max_turn, self.config.max_turn),
                        },
                        in_place: true,
                        gate_open: false,
                        reason: "dock_lock",
                    };
                }
                DockTick {
                    reason: "dock_lock",
                    ..stop
                }
            }
            DockPhase::Creep => {
                let blind = bx <= self.config.blind_range;
                if fix_age.is_none_or(|age| age > self.config.fix_timeout) && !blind {
                    // Stop and give the detector a still frame before paying
                    // for a reverse and a fresh lock.
                    if fix_age.is_none_or(|age| {
                        age > self.config.fix_timeout + self.config.reacquire_seconds
                    }) {
                        return self.back_off(pose, "lost_fix");
                    }
                    return DockTick {
                        reason: "dock_reacquire",
                        ..stop
                    };
                }
                if heading.abs() > self.config.backoff_heading {
                    return self.back_off(pose, "heading");
                }
                if self.errors.range < -self.config.range_tolerance {
                    return self.back_off(pose, "overshoot");
                }
                if self.errors.range <= self.config.range_tolerance * 0.5 {
                    self.enter(DockPhase::Settle, "at_range");
                    return DockTick {
                        reason: "dock_settle",
                        ..stop
                    };
                }
                // Stop short of the cells the bucket itself occupies: one and a
                // half cells before its near wall.
                let cell = map.map_or(0.1, |m| m.resolution);
                let ahead = bx - self.config.target_radius - 1.5 * cell;
                if self.corridor_blocked(pose, map, 0., ahead.max(0.)) {
                    let since = *self.blocked_since.get_or_insert(now);
                    if now - since > self.config.blocked_timeout {
                        self.enter(DockPhase::Failed, "path_blocked");
                    }
                    return DockTick {
                        reason: "dock_path_blocked",
                        ..stop
                    };
                }
                self.blocked_since = None;
                self.creep(heading)
            }
            DockPhase::Settle => {
                if !still {
                    return DockTick {
                        reason: "dock_settle",
                        ..stop
                    };
                }
                if still_for < self.config.settle_seconds {
                    return DockTick {
                        reason: "dock_settle",
                        ..stop
                    };
                }
                if !self.in_tolerance {
                    return self.back_off(pose, "not_aligned");
                }
                let fresh_fix = self.last_fix.is_some_and(|t| t >= self.phase_since);
                if fresh_fix {
                    self.verified = true;
                    self.enter(DockPhase::Aligned, "verified");
                } else if now - self.phase_since >= self.config.settle_timeout {
                    if bx > self.config.blind_range {
                        return self.back_off(pose, "lost_fix");
                    }
                    self.verified = self
                        .inline_fix_at
                        .is_some_and(|at| at - bx <= self.config.blind_verify_travel);
                    self.enter(
                        DockPhase::Aligned,
                        if self.verified {
                            "verified_blind"
                        } else {
                            "odometry_only"
                        },
                    );
                }
                DockTick {
                    reason: "dock_settle",
                    ..stop
                }
            }
            DockPhase::Aligned => {
                if !self.in_tolerance {
                    self.approved = false;
                    return self.back_off(pose, "drifted");
                }
                let go = self.approved || (approval == DumpApproval::Auto && self.verified);
                if go && still {
                    self.enter(
                        DockPhase::Dumping,
                        if self.approved {
                            "operator_approved"
                        } else {
                            "auto_dump"
                        },
                    );
                    return DockTick {
                        gate_open: true,
                        reason: "dock_dumping",
                        ..stop
                    };
                }
                DockTick {
                    reason: "dock_aligned",
                    ..stop
                }
            }
            DockPhase::Dumping => {
                if !still {
                    self.enter(DockPhase::Failed, "gate_interrupted:moving");
                    return stop;
                }
                if now - self.phase_since >= self.config.gate_open_seconds {
                    self.enter(DockPhase::Closing, "gate_timer");
                    return DockTick {
                        reason: "dock_closing",
                        ..stop
                    };
                }
                DockTick {
                    gate_open: true,
                    reason: "dock_dumping",
                    ..stop
                }
            }
            DockPhase::Closing => {
                if now - self.phase_since >= self.config.gate_close_seconds {
                    self.enter(DockPhase::Done, "dumped");
                }
                DockTick {
                    reason: "dock_closing",
                    ..stop
                }
            }
            DockPhase::Backoff => {
                let from = self.backoff_from.unwrap_or((pose.x, pose.y));
                if (pose.x - from.0).hypot(pose.y - from.1) >= self.config.backoff_distance {
                    self.backoff_from = None;
                    self.lock_count = 0;
                    self.enter(DockPhase::Lock, "backed_off");
                    return DockTick {
                        reason: "dock_lock",
                        ..stop
                    };
                }
                self.reverse(heading, "dock_backoff")
            }
            DockPhase::Done | DockPhase::Failed | DockPhase::Cancelled => stop,
        }
    }

    /// Reverse at backoff speed, steering the nose toward the bucket.
    /// Backing off never waits on the map (see `reverse_requested`).
    fn reverse(&self, heading: f64, reason: &'static str) -> DockTick {
        let linear = -self.config.backoff_speed;
        let mut angular = (1.5 * heading).clamp(-self.config.max_turn, self.config.max_turn);
        if !self.config.can_spin {
            let cap = linear.abs() / self.config.min_turn_radius;
            angular = angular.clamp(-cap, cap);
        }
        DockTick {
            twist: Twist { linear, angular },
            in_place: false,
            gate_open: false,
            reason,
        }
    }

    fn back_off(&mut self, pose: Pose, reason: &str) -> DockTick {
        if self.retries >= self.config.max_retries {
            self.enter(DockPhase::Failed, &format!("retries_exhausted:{reason}"));
        } else {
            self.retries += 1;
            self.inline_fix_at = None;
            self.backoff_from = Some((pose.x, pose.y));
            self.enter(DockPhase::Backoff, reason);
        }
        DockTick {
            reason: "dock_backoff",
            ..DockTick::default()
        }
    }

    fn creep(&self, heading: f64) -> DockTick {
        let c = &self.config;
        let cap = if self.errors.range > 1. {
            c.approach_speed
        } else {
            c.creep_speed
        };
        let angular = (1.5 * heading).clamp(-c.max_turn, c.max_turn);
        if c.can_spin && heading.abs() > c.spin_heading {
            return DockTick {
                twist: Twist {
                    linear: 0.,
                    angular,
                },
                in_place: true,
                gate_open: false,
                reason: "dock_creep",
            };
        }
        let mut linear = (0.6 * self.errors.range).clamp(c.min_speed, cap);
        if c.can_spin {
            linear *= (3. * heading).cos().max(0.);
            linear = linear.max(c.min_speed);
        }
        let angular = if c.can_spin {
            angular
        } else {
            let limit = linear / c.min_turn_radius;
            angular.clamp(-limit, limit)
        };
        DockTick {
            twist: Twist { linear, angular },
            in_place: false,
            gate_open: false,
            reason: "dock_creep",
        }
    }

    /// Any occupied cell in the body-frame box `x ∈ [from, to]`, `|y| ≤ half width`,
    /// other than cells within `target_clear_margin` of the bucket rim (the bucket itself).
    fn corridor_blocked(&self, pose: Pose, map: Option<&MapSnapshot>, from: f64, to: f64) -> bool {
        let Some(map) = map else { return false };
        if to <= from || map.resolution <= 0. {
            return false;
        }
        let step = map.resolution * 0.5;
        let clear = self.config.target_radius + self.config.target_clear_margin;
        let (sin, cos) = pose.yaw.sin_cos();
        let mut x = from;
        while x <= to {
            let mut y = -self.config.half_width;
            while y <= self.config.half_width {
                let (wx, wy) = (pose.x + x * cos - y * sin, pose.y + x * sin + y * cos);
                let near_target = (wx - self.target.x).hypot(wy - self.target.y) <= clear;
                if !near_target
                    && let Some((c, r)) = map.cell(wx, wy)
                    && map.occupancy[r * map.width as usize + c] >= terra_mapping::OCCUPIED
                {
                    return true;
                }
                y += step;
            }
            x += step;
        }
        false
    }

    pub fn status(&self, now: f64) -> DockStatus {
        DockStatus {
            schema_version: 1,
            dock_id: self.id,
            token: self.token.clone(),
            phase: self.phase,
            reason: self.reason.clone(),
            object_id: self.target.object_id,
            color: self.target.color.clone(),
            target: [self.target.x, self.target.y],
            errors: self.errors,
            in_tolerance: self.in_tolerance,
            fix_age: self.last_fix.map(|t| (now - t).max(0.)),
            fixes: self.fixes,
            retries: self.retries,
            verified: self.verified,
            approval: self.approval,
            awaiting_approval: self.phase == DockPhase::Aligned
                && !self.approved
                && !(self.approval == DumpApproval::Auto && self.verified),
            gate_open: self.phase == DockPhase::Dumping,
            elapsed: (now - self.started).max(0.),
        }
    }
}

/// What a `dock` request asks for.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DockActionKind {
    /// Dock on the confirmed bucket `object_id` from the `objects` topic.
    Start,
    /// Approve the dump for `dock_id` (operator approval at L0–L3).
    Dump,
    /// Stop docking; an open gate closes.
    Cancel,
}

/// `dock` topic: `{"action":"start","object_id":4,"token":"…"}`,
/// `{"action":"dump","dock_id":1,"token":"…"}` or `{"action":"cancel","token":"…"}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DockRequest {
    pub action: DockActionKind,
    pub token: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub object_id: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dock_id: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub run_id: Option<String>,
}

/// Decode a `dock` request of at most 2048 bytes. `start` needs `object_id`
/// and `dump` needs `dock_id`.
pub fn decode_dock(b: &[u8]) -> Option<DockRequest> {
    if b.is_empty() || b.len() > 2048 {
        return None;
    }
    serde_json::from_slice::<DockRequest>(b).ok().filter(|r| {
        crate::valid_token(&r.token)
            && r.run_id.as_ref().is_none_or(|s| crate::valid_token(s))
            && match r.action {
                DockActionKind::Start => r.object_id.is_some(),
                DockActionKind::Dump => r.dock_id.is_some(),
                DockActionKind::Cancel => true,
            }
    })
}

/// `(x, y)` in the rover frame: x forward, y left.
fn body(pose: Pose, x: f64, y: f64) -> (f64, f64) {
    let (dx, dy) = (x - pose.x, y - pose.y);
    let (sin, cos) = pose.yaw.sin_cos();
    (dx * cos + dy * sin, -dx * sin + dy * cos)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.01;

    fn target(x: f64, y: f64) -> DockTarget {
        DockTarget {
            object_id: Some(4),
            color: Some("orange".into()),
            x,
            y,
        }
    }

    /// A unicycle that follows the twist exactly, with a camera that sees the
    /// bucket centre at `bucket` (reporting its near wall) every 0.2 s while
    /// the bucket is farther than `blind` ahead.
    struct Sim {
        pose: Pose,
        twist: Twist,
        now: f64,
        bucket: (f64, f64),
        blind: f64,
        noise: f64,
        /// No fixes arrive between these sim times, seconds.
        dropout: (f64, f64),
        gate_opened: bool,
        map: Option<MapSnapshot>,
    }

    impl Sim {
        fn new(pose: Pose, bucket: (f64, f64)) -> Self {
            Self {
                pose,
                twist: Twist::default(),
                now: 0.,
                bucket,
                blind: 0.,
                noise: 0.,
                dropout: (0., 0.),
                gate_opened: false,
                map: None,
            }
        }

        fn run(&mut self, dock: &mut Dock, seconds: f64, approval: DumpApproval) {
            let end = self.now + seconds;
            let mut frame = 0u64;
            while self.now < end && dock.active() {
                self.now += DT;
                frame += 1;
                let dropped = (self.dropout.0..self.dropout.1).contains(&self.now);
                if frame.is_multiple_of(20) && !dropped {
                    let (bx, _) = body(self.pose, self.bucket.0, self.bucket.1);
                    if bx > self.blind {
                        let (dx, dy) = (self.bucket.0 - self.pose.x, self.bucket.1 - self.pose.y);
                        let d = dx.hypot(dy);
                        let wall = (d - dock.config.target_radius) / d;
                        let n = self.noise * ((frame as f64) * 1.7).sin();
                        dock.observe(
                            "bucket",
                            Some("orange"),
                            self.pose.x + dx * wall + n,
                            self.pose.y + dy * wall - n,
                            self.pose,
                            self.now,
                        );
                    }
                }
                let tick = dock.step(
                    self.now,
                    self.pose,
                    self.twist,
                    self.map.as_ref(),
                    approval,
                    None,
                );
                self.gate_opened |= tick.gate_open;
                self.twist = tick.twist;
                self.pose.yaw += tick.twist.angular * DT;
                self.pose.x += tick.twist.linear * self.pose.yaw.cos() * DT;
                self.pose.y += tick.twist.linear * self.pose.yaw.sin() * DT;
            }
        }
    }

    fn outlet_error(sim: &Sim, config: &DockConfig) -> f64 {
        let outlet = (
            sim.pose.x + config.outlet_ahead * sim.pose.yaw.cos(),
            sim.pose.y + config.outlet_ahead * sim.pose.yaw.sin(),
        );
        (outlet.0 - sim.bucket.0).hypot(outlet.1 - sim.bucket.1)
    }

    #[test]
    fn docks_and_auto_dumps_from_an_offset_start() {
        let config = DockConfig::default();
        let mut sim = Sim::new(
            Pose {
                x: 0.,
                y: 0.,
                yaw: 0.1,
            },
            (2.2, 0.25),
        );
        // The map estimate is 0.2 m off; live fixes pull it in.
        let mut dock = Dock::start(
            config.clone(),
            1,
            "t".into(),
            target(2.35, 0.15),
            sim.pose,
            0.,
        )
        .unwrap();
        sim.run(&mut dock, 60., DumpApproval::Auto);
        assert_eq!(dock.phase, DockPhase::Done, "{:?}", dock.status(sim.now));
        assert!(sim.gate_opened);
        assert!(dock.verified);
        assert!(
            outlet_error(&sim, &config) <= BALL_IN_BUCKET,
            "{}",
            outlet_error(&sim, &config)
        );
        let kinds: Vec<_> = dock.take_events().into_iter().map(|e| e.0).collect();
        assert!(kinds.contains(&"dock_aligned"));
        assert!(kinds.contains(&"dock_gate_opened"));
    }

    #[test]
    fn operator_approval_is_required_below_l4() {
        let config = DockConfig::default();
        let mut sim = Sim::new(Pose::default(), (1.8, 0.));
        let mut dock = Dock::start(config, 1, "t".into(), target(1.8, 0.), sim.pose, 0.).unwrap();
        sim.run(&mut dock, 40., DumpApproval::Operator);
        assert_eq!(dock.phase, DockPhase::Aligned);
        assert!(dock.status(sim.now).awaiting_approval);
        assert!(!sim.gate_opened);
        dock.approve().unwrap();
        sim.run(&mut dock, 5., DumpApproval::Operator);
        assert_eq!(dock.phase, DockPhase::Done);
        assert!(sim.gate_opened);
    }

    #[test]
    fn short_blind_finish_in_line_is_verified_and_auto_dumps() {
        // The outlet over the camera: the bucket leaves view under the phone.
        let config = DockConfig::default();
        let mut sim = Sim::new(Pose::default(), (2.0, 0.));
        sim.blind = config.blind_range;
        let mut dock = Dock::start(config, 1, "t".into(), target(2.0, 0.), sim.pose, 0.).unwrap();
        sim.run(&mut dock, 40., DumpApproval::Auto);
        assert_eq!(dock.phase, DockPhase::Done, "{:?}", dock.status(sim.now));
        assert!(dock.verified);
        assert!(sim.gate_opened);
    }

    #[test]
    fn long_blind_finish_is_unverified_and_never_auto_dumps() {
        let mut config = DockConfig::default();
        config.blind_range = config.outlet_ahead + 1.0;
        let mut sim = Sim::new(Pose::default(), (2.4, 0.));
        sim.blind = config.blind_range;
        let mut dock = Dock::start(config, 1, "t".into(), target(2.4, 0.), sim.pose, 0.).unwrap();
        sim.run(&mut dock, 40., DumpApproval::Auto);
        assert_eq!(dock.phase, DockPhase::Aligned, "{:?}", dock.status(sim.now));
        assert!(!dock.verified);
        assert!(!sim.gate_opened);
        assert!(dock.status(sim.now).awaiting_approval);
    }

    #[test]
    fn spinner_turns_in_place_toward_a_side_bucket() {
        let config = DockConfig::for_body(0.178, 0.178, true, 0.15);
        let mut sim = Sim::new(Pose::default(), (0.9, 0.35));
        let mut dock = Dock::start(
            config.clone(),
            1,
            "t".into(),
            target(0.9, 0.35),
            sim.pose,
            0.,
        )
        .unwrap();
        sim.run(&mut dock, 40., DumpApproval::Auto);
        assert_eq!(dock.phase, DockPhase::Done, "{:?}", dock.status(sim.now));
        assert!(outlet_error(&sim, &config) <= BALL_IN_BUCKET);
    }

    #[test]
    fn lost_bucket_fails_without_opening_the_gate() {
        let mut sim = Sim::new(Pose::default(), (2.0, 0.));
        sim.blind = 10.;
        let mut dock = Dock::start(
            DockConfig::default(),
            1,
            "t".into(),
            target(2.0, 0.),
            sim.pose,
            0.,
        )
        .unwrap();
        sim.run(&mut dock, 10., DumpApproval::Auto);
        assert_eq!(dock.phase, DockPhase::Failed);
        assert_eq!(dock.reason, "no_fix");
        assert!(!sim.gate_opened);
    }

    /// Outlet centre to bucket centre with the whole ball inside the rim.
    const BALL_IN_BUCKET: f64 = 0.1524 - 0.0335;
    /// How far the ball can sit off the outlet centre in a 0.10 m outlet.
    const OUTLET_PLAY: f64 = (0.10 - 0.067) / 2.;

    #[test]
    fn default_tolerances_keep_the_ball_in_the_bucket() {
        let c = DockConfig::default();
        assert!(c.valid());
        let worst =
            (c.lateral_tolerance * c.aligned_slack).hypot(c.range_tolerance * c.aligned_slack);
        assert!(worst + OUTLET_PLAY <= BALL_IN_BUCKET, "{worst}");
    }

    /// One fix of the bucket centre at body `(x, y)` from `pose` at the origin.
    fn fix_at(dock: &mut Dock, x: f64, y: f64, now: f64) {
        let d = x.hypot(y);
        let wall = (d - dock.config.target_radius) / d;
        dock.observe(
            "bucket",
            Some("orange"),
            x * wall,
            y * wall,
            Pose::default(),
            now,
        );
    }

    /// Settle on a blind finish (no fix after the last one) and report the result.
    fn blind_finish(by: f64) -> Dock {
        let mut config = DockConfig::default();
        config.outlet_ahead = 0.22;
        config.blind_range = config.outlet_ahead + DOCK_BLIND_MARGIN;
        let pose = Pose::default();
        let mut dock = Dock::start(config, 1, "t".into(), target(0.22, by), pose, 0.).unwrap();
        fix_at(&mut dock, 0.22, by, 0.5);
        dock.now = 1.;
        dock.enter(DockPhase::Settle, "at_range");
        let mut now = 1.;
        while now < 5. && dock.phase == DockPhase::Settle {
            now += DT;
            dock.step(now, pose, Twist::default(), None, DumpApproval::Auto, None);
        }
        dock
    }

    #[test]
    fn a_fix_15_degrees_off_does_not_verify_a_blind_finish() {
        // 0.059 m at 0.22 m: lateral and range in tolerance, bearing 15°.
        let dock = blind_finish(0.059);
        let status = dock.status(dock.now);
        assert!(status.in_tolerance, "{status:?}");
        assert!(status.errors.heading_deg > 14.9, "{status:?}");
        assert_eq!(dock.phase, DockPhase::Aligned);
        assert_eq!(dock.reason, "odometry_only");
        assert!(!dock.verified);
        assert!(dock.status(dock.now).awaiting_approval);
        // Control: the same finish in line (8°) is verified.
        let dock = blind_finish(0.03);
        assert_eq!(dock.reason, "verified_blind");
        assert!(dock.verified);
    }

    #[test]
    fn close_in_lateral_offset_is_aligned_despite_its_bearing() {
        // Outlet over the camera, close behind the bucket: 0.055 m off the axis
        // is a 12° bearing, which the old 6° heading test refused.
        let mut config = DockConfig::default();
        config.outlet_ahead = 0.25;
        config.blind_range = config.outlet_ahead + DOCK_BLIND_MARGIN;
        let pose = Pose::default();
        let mut dock = Dock::start(config, 1, "t".into(), target(0.25, 0.055), pose, 0.).unwrap();
        dock.enter(DockPhase::Settle, "at_range");
        dock.step(
            0.1,
            pose,
            Twist::default(),
            None,
            DumpApproval::Operator,
            None,
        );
        let status = dock.status(0.1);
        assert!(status.errors.heading_deg > 10., "{:?}", status.errors);
        assert!(status.in_tolerance);
    }

    #[test]
    fn a_short_fix_gap_while_creeping_waits_instead_of_backing_off() {
        let config = DockConfig::default();
        let mut sim = Sim::new(Pose::default(), (2.2, 0.));
        // Locks in 0.6 s, then the detector drops out for 2.5 s far from the bucket.
        sim.dropout = (1.5, 4.0);
        let mut dock = Dock::start(config, 1, "t".into(), target(2.2, 0.), sim.pose, 0.).unwrap();
        sim.run(&mut dock, 3.5, DumpApproval::Auto);
        assert_eq!(dock.phase, DockPhase::Creep, "{:?}", dock.status(sim.now));
        assert_eq!(dock.status(sim.now).retries, 0);
        sim.run(&mut dock, 60., DumpApproval::Auto);
        assert_eq!(dock.phase, DockPhase::Done, "{:?}", dock.status(sim.now));
        assert_eq!(dock.status(sim.now).retries, 0);
    }

    #[test]
    fn a_long_fix_gap_while_creeping_backs_off_with_lost_fix() {
        let config = DockConfig::default();
        let mut sim = Sim::new(Pose::default(), (2.2, 0.));
        sim.dropout = (1.5, 20.);
        let mut dock = Dock::start(config, 1, "t".into(), target(2.2, 0.), sim.pose, 0.).unwrap();
        sim.run(&mut dock, 6., DumpApproval::Auto);
        let kinds: Vec<_> = dock.take_events();
        assert!(
            kinds.contains(&("dock_backoff", "lost_fix".into())),
            "{kinds:?}"
        );
    }

    #[test]
    fn requests_decode() {
        assert_eq!(
            decode_dock(br#"{"action":"start","object_id":4,"token":"a"}"#)
                .unwrap()
                .object_id,
            Some(4)
        );
        assert!(decode_dock(br#"{"action":"start","token":"a"}"#).is_none());
        assert!(decode_dock(br#"{"action":"dump","token":"a"}"#).is_none());
        assert!(decode_dock(br#"{"action":"cancel","token":"a","extra":1}"#).is_none());
        assert!(decode_dock(br#"{"action":"cancel","token":"a"}"#).is_some());
    }

    #[test]
    fn far_targets_are_refused() {
        let r = Dock::start(
            DockConfig::default(),
            1,
            "t".into(),
            target(5., 0.),
            Pose::default(),
            0.,
        );
        assert_eq!(r.err(), Some("target_too_far"));
    }

    #[test]
    fn hold_during_dump_closes_the_gate_and_fails() {
        let mut sim = Sim::new(Pose::default(), (1.8, 0.));
        let mut dock = Dock::start(
            DockConfig::default(),
            1,
            "t".into(),
            target(1.8, 0.),
            sim.pose,
            0.,
        )
        .unwrap();
        sim.run(&mut dock, 40., DumpApproval::Operator);
        dock.approve().unwrap();
        let open = dock.step(
            sim.now + DT,
            sim.pose,
            Twist::default(),
            None,
            DumpApproval::Operator,
            None,
        );
        assert!(open.gate_open);
        let held = dock.step(
            sim.now + 2. * DT,
            sim.pose,
            Twist::default(),
            None,
            DumpApproval::Operator,
            Some("operator_stop"),
        );
        assert!(!held.gate_open);
        assert_eq!(dock.phase, DockPhase::Failed);
        assert_eq!(dock.reason, "gate_interrupted:operator_stop");
    }

    #[test]
    fn other_colour_bucket_in_path_fails() {
        let pose = Pose::default();
        let mut dock = Dock::start(
            DockConfig::default(),
            1,
            "t".into(),
            target(2.0, 0.),
            pose,
            0.,
        )
        .unwrap();
        dock.observe("bucket", Some("blue"), 1.0, 0.1, pose, 0.1);
        assert_eq!(dock.phase, DockPhase::Failed);
        assert_eq!(dock.reason, "other_bucket_in_path");
    }

    #[test]
    fn noisy_fixes_still_align_within_tolerance() {
        let config = DockConfig::default();
        let mut sim = Sim::new(
            Pose {
                x: 0.,
                y: 0.,
                yaw: -0.05,
            },
            (2.0, -0.1),
        );
        sim.noise = 0.03;
        let mut dock = Dock::start(
            config.clone(),
            1,
            "t".into(),
            target(2.0, -0.1),
            sim.pose,
            0.,
        )
        .unwrap();
        sim.run(&mut dock, 60., DumpApproval::Auto);
        assert_eq!(dock.phase, DockPhase::Done, "{:?}", dock.status(sim.now));
        assert!(outlet_error(&sim, &config) <= BALL_IN_BUCKET);
    }

    fn ring_map(centre: (f64, f64), radius: f64) -> MapSnapshot {
        let mut map = MapSnapshot {
            width: 60,
            height: 40,
            resolution: 0.1,
            origin_x: -1.,
            origin_y: -2.,
            occupancy: vec![0; 2400],
        };
        for r in 0..40 {
            for c in 0..60 {
                let x = map.origin_x + (c as f64 + 0.5) * 0.1;
                let y = map.origin_y + (r as f64 + 0.5) * 0.1;
                if (x - centre.0).hypot(y - centre.1) <= radius {
                    map.occupancy[r * 60 + c] = 100;
                }
            }
        }
        map
    }

    /// Field run 3866429a: the camera lost the bucket at ~1.06 m and the
    /// mapped bucket sat closer than the frozen centre estimate, so the
    /// corridor check stopped the rover 0.33 m short with `path_blocked`.
    #[test]
    fn mapped_bucket_closer_than_the_estimate_does_not_block() {
        // Camera reports the bucket at 2.0 m; depth mapped its wall ~0.2 m nearer.
        let mut sim = Sim::new(Pose::default(), (2.0, 0.));
        sim.map = Some(ring_map((1.8, 0.), 0.2));
        let config = DockConfig::default();
        sim.blind = config.blind_range + 0.1;
        let mut dock =
            Dock::start(config.clone(), 1, "t".into(), target(2.0, 0.), sim.pose, 0.).unwrap();
        sim.run(&mut dock, 40., DumpApproval::Operator);
        assert_ne!(dock.reason, "path_blocked", "{:?}", dock.status(sim.now));
        assert_eq!(dock.phase, DockPhase::Aligned, "{:?}", dock.status(sim.now));
        assert!(outlet_error(&sim, &config) <= BALL_IN_BUCKET);
    }

    /// Field run 3866429a: three docks started 0.33 m short, inside the blind
    /// range, waited 5 s for a sighting that could never come and failed
    /// `no_fix`. Now the dock reverses out until the bucket is in view first.
    #[test]
    fn start_inside_blind_range_reverses_out_then_docks() {
        let config = DockConfig::default();
        let bucket = config.outlet_ahead + 0.33;
        let mut sim = Sim::new(Pose::default(), (bucket, 0.));
        sim.blind = config.blind_range;
        let mut dock = Dock::start(
            config.clone(),
            1,
            "t".into(),
            target(bucket, 0.),
            sim.pose,
            0.,
        )
        .unwrap();
        let tick = dock.step(
            0.01,
            sim.pose,
            Twist::default(),
            None,
            DumpApproval::Auto,
            None,
        );
        assert_eq!(tick.reason, "dock_open_range");
        assert!(tick.twist.linear < 0.);
        assert!(dock.reversing());
        sim.run(&mut dock, 60., DumpApproval::Auto);
        assert_eq!(dock.phase, DockPhase::Done, "{:?}", dock.status(sim.now));
        assert!(outlet_error(&sim, &config) <= BALL_IN_BUCKET);
    }

    #[test]
    fn reversing_out_gives_up_when_the_rover_cannot_back_away() {
        let config = DockConfig::default();
        let pose = Pose::default();
        let mut dock = Dock::start(
            config.clone(),
            1,
            "t".into(),
            target(config.outlet_ahead + 0.3, 0.),
            pose,
            0.,
        )
        .unwrap();
        let mut t = 0.;
        while dock.active() && t < 30. {
            t += 0.1;
            dock.step(t, pose, Twist::default(), None, DumpApproval::Auto, None);
        }
        assert_eq!(dock.reason, "too_close");
        assert!(t > config.open_range_timeout);
    }

    /// The bucket's own occupied cells are not an obstacle in the corridor.
    #[test]
    fn the_mapped_bucket_does_not_block_its_own_dock() {
        let mut map = MapSnapshot {
            width: 60,
            height: 40,
            resolution: 0.1,
            origin_x: -1.,
            origin_y: -2.,
            occupancy: vec![0; 2400],
        };
        for r in 0..40 {
            for c in 0..60 {
                let x = map.origin_x + (c as f64 + 0.5) * 0.1;
                let y = map.origin_y + (r as f64 + 0.5) * 0.1;
                if (x - 2.0).hypot(y) <= 0.2 {
                    map.occupancy[r * 60 + c] = 100;
                }
            }
        }
        let mut sim = Sim::new(Pose::default(), (2.0, 0.));
        sim.map = Some(map);
        let mut dock = Dock::start(
            DockConfig::default(),
            1,
            "t".into(),
            target(2.0, 0.),
            sim.pose,
            0.,
        )
        .unwrap();
        sim.run(&mut dock, 40., DumpApproval::Auto);
        assert_eq!(dock.phase, DockPhase::Done, "{:?}", dock.status(sim.now));
    }

    #[test]
    fn blocked_corridor_holds_then_fails() {
        let pose = Pose::default();
        let mut map = MapSnapshot {
            width: 40,
            height: 40,
            resolution: 0.1,
            origin_x: -2.,
            origin_y: -2.,
            occupancy: vec![0; 1600],
        };
        // A box at x = 0.9 m on the rover axis, short of the bucket at 2 m.
        let (c, r) = map.cell(0.9, 0.).unwrap();
        map.occupancy[r * 40 + c] = 100;
        let mut dock = Dock::start(
            DockConfig::default(),
            1,
            "t".into(),
            target(2.0, 0.),
            pose,
            0.,
        )
        .unwrap();
        for i in 1..=3 {
            dock.observe("bucket", Some("orange"), 1.85, 0., pose, 0.1 * i as f64);
        }
        let mut t = 0.3;
        let tick = dock.step(
            t,
            pose,
            Twist::default(),
            Some(&map),
            DumpApproval::Auto,
            None,
        );
        assert_eq!(tick.reason, "dock_path_blocked");
        assert_eq!(tick.twist, Twist::default());
        while dock.active() && t < 10. {
            t += 0.1;
            dock.observe("bucket", Some("orange"), 1.85, 0., pose, t);
            dock.step(
                t,
                pose,
                Twist::default(),
                Some(&map),
                DumpApproval::Auto,
                None,
            );
        }
        assert_eq!(dock.reason, "path_blocked");
    }
}
