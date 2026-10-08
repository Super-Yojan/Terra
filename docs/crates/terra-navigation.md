# terra-navigation

Deterministic local planning over an observed occupancy grid. The phone and the simulator share this crate. It does not build a global map.

[API](https://super-yojan.dev/Terra/api/terra_navigation/index.html) · [source](https://github.com/Super-Yojan/Terra/blob/main/crates/terra-navigation/src/lib.rs)

## Planner defaults

`PlannerConfig` uses a circumscribed radius of **0.65 m** (the autonomy contract pairs that with a simulated 0.9 × 0.8 m chassis), max linear and angular speeds of 2, linear acceleration and braking of 1, yaw acceleration of 2 rad/s², and a 2 s horizon.

A cell counts as occupied at probability **65** or above. The same threshold is what the autonomy contract tells ARGOS to render. Unknown cells are not treated as free.

`LocalPlanner` rolls a short trajectory forward and picks a twist inside the acceleration limits. The reason string is `active` when the result stays close to the requested twist, and `assisted` when obstacle avoidance changes it. If nothing is admissible it stops.

## Frontier and clearance

`frontier` is a breadth-first search through inflated free cells. Stable ordering breaks ties. The radius must be finite and in `(0, 5]` metres. Excluded points are skipped. Supervised mode uses this to propose a reachable target. Approval still has to come from the operator through `terra-autonomy`.

`reachable` reports whether a point is connected through observed free space. `observed_clearance` is the distance to the nearest occupied cell, which `terra-experiment` turns into near-miss events. These values come from the observed grid, which is not simulator ground truth.
