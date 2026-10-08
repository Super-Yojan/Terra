# terra-navigation

!!! tip "TL;DR"
    Plans only on cells the rover has seen.
    Occupied means probability ≥ 65.
    A frontier is a proposal. It does not move the rover by itself.
    Autonomous runs live in terra-exploration.

![Observed grid the planner reads](../assets/occupancy.svg)

*Unknown cells are not free. Default footprint radius is 0.65 m.*

[API](https://super-yojan.dev/Terra/api/terra_navigation/index.html) · [source](https://github.com/Super-Yojan/Terra/blob/main/crates/terra-navigation/src/lib.rs)

```mermaid
flowchart TD
  Intent[requested twist] --> Roll[2 s rollout]
  Grid[observed cells] --> Roll
  Roll --> Ok{Fits in free space?}
  Ok -->|yes, close to intent| Active[reason active]
  Ok -->|yes, changed| Assisted[reason assisted]
  Ok -->|no| Stop[stop]
```

*Caps: 2 m/s, 2 rad/s, 1 m/s², 2 rad/s² yaw.*

`frontier` searches inflated free cells for supervised proposals. `terra-exploration` uses the same clearance helpers and adds the time budget, blacklist, and stop reasons. `observed_clearance` feeds near-miss logs in `terra-experiment`.
