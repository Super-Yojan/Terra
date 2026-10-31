# Reading the autonomy arbiter

Start with `AutonomyArbiter::step()` in `../arbiter.rs`.
It answers one question each control tick: **what motion may the rover execute now?**

| Stage | Function | What it does |
| --- | --- | --- |
| 1 | `begin_tick` | Reset cached turning state when authority changes; check the clock |
| 2 | `check_initial_safety` | Check stop, sensors, map freshness and supervision pause |
| 3 | `select_motion` | Choose operator or goal intent; check the path and obstacles |
| 4 | `limit_turning` | Apply the vehicle's turning constraints |
| 5 | `finish_holds` | Update search holds and force held motion to zero |
| 6 | `status_for_tick` | Build the status report that ARGOS displays |

## The four values to follow

- `decision.intent_command`: what the operator or goal follower wanted.
- `decision.twist`: what the rover is actually allowed to execute.
- `decision.safety`: `clear`, `hold`, or `emergency_stop`.
- `decision.reason`: the specific explanation, such as `map_stale` or `operator_stop`.

`decision.held` means that motion must be stopped. `finish_holds` sets the final
command to zero and changes `clear` to `hold`. It preserves `emergency_stop`.
A clear safety state does not itself arm hardware or guarantee nonzero motion.

For example, in waypoint mode with a stale map:

1. `check_initial_safety` sets `held = true` and `reason = "map_stale"`.
2. `step` skips motion selection.
3. `finish_holds` sets zero motion and `safety = "hold"`.
4. `status_for_tick` copies those values into the report consumed by ARGOS.

Checks preserve the original priority: invalid time, latched stop, unhealthy
sensors, required map freshness, then supervision pause. Reverse requests
(assisted-teleop or L5 agent reverse, dock backoff) skip the map freshness
check, and operator/agent reverse also skips the local planner. Local manual mode and
fleet/autonomous commands retain their existing differences; this refactor does
not change thresholds, command admission, turning policy or telemetry strings.

## Where the details live

- `requests.rs`: mode changes, operator commands, goals, safety stop/reset and proposal decisions.
- `search_requests.rs`: detector capability, target-search actions and report acknowledgements.
- `motion.rs`: intent selection, waypoint routing, heading maneuvers and collision checks.
- `exploration.rs`: choosing frontier goals and tracking exploration hold timers.
- `status.rs`: telemetry formatting, visible routes and planner traces.

Within `motion.rs`, read `select_motion` first. It calls `update_search_goal`,
`choose_intent`, `heading_maneuver`, `check_motion` and `update_frontier_proposal`.
`waypoint_route_intent` contains the detailed A* route maintenance.

Run the behavioral regression suite from the repository root:

```sh
cargo test -p terra-autonomy --locked
```

## Change timing defaults in one place

Edit [terra-types/src/timing.rs](../../../terra-types/src/timing.rs), then rebuild.
Every value is in seconds. Separate named settings cover data/command freshness,
planning intervals and horizons, proposal expiry, turning timestep limits,
exploration recovery and target search. The previous defaults are preserved.
Existing per-instance configuration can override its corresponding default.
Mission budgets remain explicit per-run inputs; application-level publishing and
sensor checks outside these crates are separate.

For the map timeout, change `MAP_FRESHNESS_SECONDS` (default: `0.5`).
`DEFAULT_MAP_FRESHNESS_TIMEOUT_SECONDS` remains a compatibility alias.
All arbiter map-age checks use that default:
safety holds, maneuver execution, proposal approval, search admission/resume and
route visibility. Existing arbiters can also be configured individually:

```rust
let mut arbiter = terra_autonomy::AutonomyArbiter::default();
assert!(arbiter.set_map_freshness_timeout(1.0)); // Example override, in seconds.
```

The setter accepts only finite positive values. An observation exactly as old as
the configured limit is stale; missing, future or invalid timestamps still fail.
This setting does not change pose, command or detector freshness, the exploration
stale-hold termination timer, or independent sensor/publishing checks in applications
such as Zorvane and TerraPhone.

Keep freshness limits distinct from recovery grace periods. For example,
`MAP_FRESHNESS_SECONDS` determines when the map becomes stale;
`EXPLORATION_MAP_STALE_GRACE_SECONDS` determines how long exploration can remain
held before ending. A grace period of zero means immediate termination on the
first qualifying hold. Increasing grace does not permit motion with a stale map.
