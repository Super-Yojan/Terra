# terra-autonomy

The arbiter answers which operator level is allowed to move the rover right now. It does not pick an optimal level by itself.

[API](https://super-yojan.dev/Terra/api/terra_autonomy/index.html) · [contract](../autonomy/PROTOCOL.md) · [mission guide](../autonomy/README.md)

## Levels

`Level` serializes as snake_case:

| Level | What moves the rover |
| --- | --- |
| `teleop` | Held operator input. Commands expire. |
| `assisted_teleop` | Operator intent, passed through the shared obstacle and braking check. |
| `waypoint` | An operator-selected goal, followed by `terra-waypoint` and checked by `terra-navigation`. |
| `supervised` | A frontier proposal. Motion waits for approval. |

`AutonomyStatus` keeps the assigned level, the requested level, and the effective level on separate fields. Effective level is empty during a safety hold. A takeover clears the old goal and teleop intent. Emergency stop stays latched until a healthy explicit reset. Reset does not restore the previous goal.

Tokens are 1–64 ASCII characters from `A-Z`, `a-z`, `0-9`, and `. _ : -`. Codecs reject unknown JSON fields and payloads over 2048 bytes. A 128-entry token cache drops replays. A new run id starts a new cache.

## Requests the arbiter accepts

`decode_level`, `decode_safety`, `decode_teleop`, and `decode_decision` are the codecs. Safety actions are `stop` and `reset`. Proposal decisions are `approve`, `reject`, and `resume`. Approval rechecks the current free space. It does not require the map revision to be unchanged.

`occupancy_telemetry` builds the JSON body for `map/occupancy`: schema version, rover id, run id, map sequence, size, resolution, origin, and the observed cell array. Hidden mission targets are not included.

The reference disaster-search scenario runs in Zorvane (`TERRA_MISSION=1`). Mission rules, trial variables, and log fields are in the [mission guide](../autonomy/README.md). Physical navigation, braking calibration, and real survivor detection are not inferred from a Zorvane run.
