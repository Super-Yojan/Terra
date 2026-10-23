# Managed turning and reversing (issue 53)

ARGOS teleop (`teleop` and `cmd_vel`) and autonomous local/WGS84 waypoint,
exploration and search commands use a shared turning policy. Command provenance
is assigned by trusted ingress. Direct TerraPhone Bluetooth manual driving keeps
its existing joystick mapping and does not require VIO. A connected fleet owns
velocity intent; local updates cannot replace that intent through the Rust API.

## Defaults and configuration

| Setting | Default | Units |
| --- | --- | --- |
| `turning.max_yaw_rate` | 0.5 | rad/s |
| `turning.angular_acceleration` | 0.5 | rad/s² |
| `turning.angular_deceleration` | 0.8 | rad/s² |
| `turning.min_turn_radius` | 0.5 | m |
| `turning.allow_in_place` | false | boolean |
| `max_reverse` | 0.15 | m/s |
| `reverse_penalty` | 0.05 | cost per reverse m/s |

These fields are part of the existing `navigation/settings.parameters` contract.
Old clients receive defaults for omitted fields. Configuration still requires
fresh disarmed hardware, no active mission or pending arm, valid run/authority
and settings revisions, and a new token. Invalid settings reject atomically.
Yaw/radius must be finite in `(0, 5]`; acceleration/deceleration in `(0, 10]`;
reverse cap in `(0, 2]`, with the effective cap also bounded by `max_linear`;
reverse penalty in `[0, 10]`. Applied settings are echoed in navigation status.
These defaults are software starting points, not physically calibrated limits.

Ordinary turns slew from the previously commanded yaw rate. Reversals decelerate
through zero. The no-spin radius bound applies to forward and reverse motion;
reducing forward speed can tighten this bound immediately. Invalid time, safety
holds, stale evidence and authority changes reset history. Safety stops take
precedence over smooth deceleration. Final Rust feedback and Swift fleet wheel
mixing also prevent counterrotation when spins are disabled.

## Planning and multi-point maneuvers

Planner candidates are shaped before collision rollout. The selected velocity
in `planner_trace.candidates[selected]` is the executed managed twist, not an
unconstrained precursor. Rollouts check the full conservative circular footprint
and a braking tail. Reverse motion does not get the current-footprint unknown-cell
exemption: unknown or occupied space under/behind the rover rejects the rollout.
The forward-facing camera may have mapped rear space earlier, but this change
does not add a rear/selfie-camera feed or certify historical cells against newly
arriving obstacles.

Reverse arcs use the lower speed cap and incur an explicit cost penalty. If a
heading request would require a disabled spin, the planner tries observed arcs,
including alternating forward/reverse three-point maneuvers. Segments advance
on measured heading progress, not elapsed time. Each tick rechecks the actual
slew-limited candidate against the latest map. Without a safe maneuver, the
controller holds with `heading_requires_spin_or_replan`. Unknown space does not
become free simply because a maneuver would otherwise be impossible.

`planner_trace.candidates[].direction` is `forward`, `reverse` or `stationary`.
`planner_trace.maneuver_segments[]` contains `start`, `end`, `velocity`, and
`direction` for nominal maneuver segments. Poses are in the existing AR/local
frame. These additive fields allow ARGOS to draw reverse segments; an ARGOS
renderer update is not included here. Existing `global_route` points remain the
geometric route rather than falsely claiming a timed vehicle trajectory.

## Pose quality

The phone publishes `<prefix>/<rover_id>/pose/quality`, even while pose publication
is suppressed, and includes the object in autonomy status. Version 1 contains:
`schema_version`, `rover_id`, `run_id`, `sequence`, `degraded`, `reason`,
`pose_age_s`, and `imu_age_s`. Missing/future ages are null, never NaN or infinity.
Reasons include `ready`, `missing_imu`, `missing_vio`, `tracking_lost`, `stale_imu`,
`stale_vio`, `pose_stale`, and `invalid_time`. Ages use original accepted sample
times. Degraded pose rejects new autonomous goals (cancellation remains allowed)
and unhealthy autonomous motion holds. This is an evidence-health flag, not a
quantitative VIO-accuracy or slip detector. ARGOS warning UI integration is separate.

## GPS and verification limits

Existing WGS84 goals are projected through a configured origin into the same
local follower; both goal forms receive turning constraints. This is not the
proposed outdoor GPS mission controller. No outdoor capability is advertised,
no GPS-to-AR transform is introduced, and ARGOS's outdoor capability gate is not
changed. Future outdoor guidance must pass through the same managed boundary and
keep intermediate local targets distinct from GPS mission arrival.

The audited waypoint heading completion already uses measured yaw; no elapsed-time
angle-completion implementation was found in the control/navigation paths. Safety
timeouts, search budgets, rollout horizons and ESC arming durations are retained.
Automated tests cover bounds, source separation, reverse visibility, trace equality,
measured multi-point progress and local/WGS84 equivalence. Physical slip, camera
coverage, VIO quality, braking and outdoor GPS arrival still require supervised
hardware verification. No hardware is driven or deployed by these tests.
