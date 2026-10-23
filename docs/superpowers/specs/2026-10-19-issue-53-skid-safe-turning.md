# Issue 53: skid-safe fleet and autonomous turning

Status: design approved for implementation on 2026-10-19; implementation in progress.

Issue: https://github.com/Super-Yojan/Terra/issues/53

GPS integration reference:
https://github.com/Super-Yojan/ARGOS/blob/main/docs/superpowers/specs/2026-10-18-outdoor-gps-waypoints-design.md

The reference explicitly describes an unimplemented outdoor GPS controller.
Existing WGS84-to-local conversion is not that controller. Issue 53 applies its
turn policy to existing local/WGS84 goal outputs and to future outdoor local
guidance through the shared managed-command boundary. It does not enable the
outdoor capability or lift ARGOS's geographic-command gate. A complete outdoor
mission controller, Core Location ingestion, and ARGOS outdoor UI/admission
remain the separate work described in that reference. In particular, do not
introduce a GPS-to-AR transform, change GPS arrival semantics, or call reaching
an intermediate guidance target GPS mission completion.

## Agreed scope

Apply turn constraints to ARGOS teleoperation and all autonomous commands,
including planner goals received through Zenoh and phone-initiated autonomous
waypoints. Leave direct TerraPhone Bluetooth manual driving unchanged, including
its joystick mapping, wheel-effort routing, and tracking-loss policy. The
exemption follows command origin, not transport or the selected screen: ARGOS
commands also reach the Pi over the phone's Bluetooth connection.

Do not modify existing uncommitted pairing, motor shutdown, connection, or UI
work except where a narrow integration change is necessary. No live driving,
Pi deployment, or phone installation is part of this issue implementation.

## Current behavior and evidence

- `terra-control` already clamps target yaw rate and uses measured yaw-rate
  feedback. It does not limit angular acceleration or enforce a turn radius.
- `terra-mobile::MobileController::step` admits dashboard commands, advances
  the autonomy arbiter, publishes status, and feeds the motor controller.
- `PhoneController.update` and `TerraFleetManualDrive.command` can convert
  fleet teleop output directly to wheel effort. Limits only in the feedback
  controller would not protect that route.
- `terra-waypoint::pursuit` already finishes final heading alignment using
  measured yaw error. It requests zero forward speed for alignment and for
  large heading errors, which conflicts with a no-spin default.
- The inspected control/navigation paths contain no identified elapsed-time
  angle turn. Continue the audit during implementation; do not remove safety
  timeouts, planner rollout durations, search budgets, or ESC arming delays.
- Estimator health and pose freshness exist. Pose publication is suppressed
  when unhealthy, but explicit degraded-pose telemetry is needed.

## Command ownership and enforcement

Represent local manual, fleet teleop, and autonomous command origin explicitly
in the shared control state. Derive this value at accepted ingress and arbitration;
never accept an exemption flag supplied by a remote velocity packet. Clear origin
and limiter history on authority/run changes, disconnect, cancellation, and reset.
Local manual input must not relabel an active fleet or autonomous command.

Use one reusable Rust turning policy for managed commands. Apply it before
the selected twist is exposed to the Swift fleet path or fed to motor control.
Ensure every managed output route consumes the same constrained command; avoid
applying a slew limiter twice. Preserve the direct local manual route as-is.

Feedback can itself request opposing wheel effort even for a bounded velocity
target. Managed drive output therefore also needs a final no-counterrotation
constraint when spins are disabled, with anti-windup using the actual constrained
effort. A wheel-effort ratio is not proof of achieved physical turn radius on a
skidding rover: use measured yaw-rate feedback and document that limitation.

## Configuration and policy

Proposed initial values, configurable and requiring subsequent bench tuning:

| Parameter | Initial value |
| --- | --- |
| Maximum yaw rate | 0.5 rad/s |
| Angular acceleration | 0.5 rad/s² |
| Angular deceleration | 0.8 rad/s² |
| Minimum turn radius | 0.5 m |
| Allow in-place spins | false |

Validate finite positive numeric values with explicit supported bounds. Extend
the existing revision/authority-checked navigation configuration mechanism,
supplying defaults for new fields so existing clients remain compatible. Reject
invalid changes atomically, and expose applied values in status. A configuration
update must not arm or resume motion.

For managed velocity commands, clamp yaw magnitude and angular slew using
monotonic elapsed time. Reversal decelerates through zero before accelerating in
the opposite direction. Enforce `abs(yaw_rate) <= abs(forward)/minimum_radius`
when spins are disabled, including after slew limiting. Never add forward motion
to a stationary teleop request merely to manufacture an arc.

Emergency stop, disarm, stale commands/sensors, invalid time, and authority loss
produce immediate zero output and clear limiter history. Radius/no-spin constraints
take priority if obeying a normal deceleration ramp would violate them. State
resets must not permit a large first-step acceleration or replay old intent.

## Autonomous heading and trajectory planning

Evaluate turn constraints during candidate generation and collision rollout,
not solely after a collision-checked trajectory has been selected. The executed
arc and braking path must match the checked motion; do not transform a safe spin
into unchecked translation.

Waypoint completion remains based on measured position/yaw tolerance, including
angle wraparound. Generate forward arc approaches where a checked trajectory
can reach the requested heading. If no such maneuver is available, hold with an
explicit `heading_requires_spin_or_replan` reason instead of silently hanging,
claiming arrival, enabling spins, or introducing a timed maneuver. Position-only
goals remain position-only. An explicit spin configuration permits bounded
closed-loop heading alignment; timeouts can abort but cannot establish success.

Audit search/exploration and direct-waypoint paths for zero-radius assumptions.
Paths without sufficient obstacle evidence must hold rather than invent a safe
arc. Planner goals cannot bypass the policy.

## Pose health and telemetry

Publish a versioned `pose/quality` Zenoh message with rover/run identity,
monotonic sequence, a degraded flag, a reason, and pose/IMU freshness information.
Publish unhealthy transitions even when no pose is being emitted, and continue
periodic bounded updates. Include the same state in autonomy status so clients
can correlate intent, health, and holds. Do not publish NaN/Infinity ages.

Reject/defer new autonomous goals and hold active autonomous motion when pose
quality is degraded. Managed feedback commands must not rely on stale sensor
evidence. Preserve the direct local manual exemption. Fresh data alone must not
clear an emergency-stop latch or restore expired command authority.

Document the wire contract for ARGOS. Terra enforces the hold independently;
an ARGOS warning UI/subscriber update is separate integration work, and must
not be claimed complete solely because Terra publishes the flag. Existing ARGOS
clients must continue to decode existing status and pose messages.

## Verification and acceptance

Tests must cover yaw clamp, acceleration/deceleration, reversal, invalid timing,
radius restrictions at low/zero/reverse speed, explicit spin enablement,
post-feedback wheel constraints, immediate safety stops, and history reset.
Heading tests must prove elapsed time alone cannot complete a turn and that
measured yaw does complete it, including wraparound and degraded tracking.

Integration tests exercise ARGOS `teleop`/`cmd_vel`, autonomous goals, and the
Swift fleet effort route. A direct Bluetooth local-manual regression test must
retain its current wheel outputs, including stationary steering. Verify that
remote packets cannot claim the exemption and that source switches do not leak
limiter state or revive stale commands.

Test backward-compatible settings, invalid configuration rejection, explicit
degraded telemetry without a pose publication, goal admission/hold behavior,
and recovery while preserving safety/authority rules. Run the relevant Rust
workspace and Swift tests and report any unavailable platform checks. Simulated
tests establish software behavior, not skid resistance or VIO accuracy on the
physical rover.

## Approved scope addition: reverse planning

The user added reverse arcs, conservative configurable reverse speed and cost,
measured multi-point maneuvers before holding, strict unknown-rear collision checks
including braking tails, and trace direction labels. Direct phone Bluetooth manual
remains exempt and does not require VIO. No new rear-camera ingestion is included.
