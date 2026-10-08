# terra-experiment

Versioned JSONL recordings and the search-mission observations those recordings summarize.

[API](https://super-yojan.dev/Terra/api/terra_experiment/index.html) · [source](https://github.com/Super-Yojan/Terra/tree/main/crates/terra-experiment/src)

## Recording

`RunRecorder::create` writes a manifest and then one JSON object per line. `record` appends an event. `failed` reports a broken recording. `close` finishes the file. `read_records` parses a buffer back into JSON values.

A complete file and a successful mission are different results. A truncated recording is not a complete-data run.

Zorvane writes run logs under `simulator/runs` in that process, or under `TERRA_RUN_DIR` when set. Summarize one log from this checkout:

```sh
cargo run -p terra-experiment --bin terra-run-summary -- /path/to/<run-id>.jsonl
```

`terra-run-summary` prints `summarize` as pretty JSON. `RunSummary` includes duration, mode changes, takeovers, interventions per minute, time to first discovery, mission completion time, approvals, rejections, redirects, near misses, contacts, operator control seconds, and whether the recording and the mission completed.

## Near misses and the mission

`NearMiss` starts an episode when observed clearance drops below **0.3 m** and ends it above **0.4 m**. Missing clearance is unavailable. It is not recorded as zero collisions.

`MissionConfig::rescue(seed)` builds the reference search: three simulated survivors, a time budget, and a return to the origin. `Mission::observe` confirms a sighting only when the rover is close enough and the occupancy ray to the target is free. Those rules are simulated detection, which is not a validated person detector. `report` confirms an observation for a rover id. `status` omits hidden targets.

The phone records sandbox JSONL through the same types and can share or export a run. ARGOS exports its own operator JSONL. Join the two on action tokens and on teleop session and sequence. Counting both ends as two interventions double-counts the same action.
