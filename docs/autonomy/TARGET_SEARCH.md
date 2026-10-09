# L4 target search

L4 accepts a target class, bounded local search rectangle and finite time budget. The rover selects frontier routes without per-frontier approval, confirms a target from sensor observations, holds motion and reports the result. The existing `supervised` level still waits for frontier approval.

The simulator detector supports `survivor`. Physical TerraPhone has no recognition model registered by default and therefore does not advertise L4. The shared runtime, typed UniFFI ingress and operator controls are implemented; registering an adapter is not evidence of physical detection performance.

## Run

Use the adjacent Zorvane checkout with the paired local Terra dependencies:

```sh
cd ../Zorvane
TERRA_MISSION=1 TERRA_MISSION_SEED=42 cargo run -p zorvane
```

In ARGOS, connect to the rover bus, choose L4 and wait for acknowledgement. Enter `survivor`, local minimum/maximum x/y coordinates containing the rover, and a time budget in seconds; start the search. TerraPhone's missions screen offers the same flow when its connected rover advertises `target_search`. Bounds use the rover's local frame even when the dashboard map is geographically aligned.

Stop/takeover cancels the search and clears its navigation intent. Reset or a new level selection never resumes old intent. Pause clears motion; resume explicitly revalidates the detector and navigation inputs. The time budget continues during pause and attention holds.

## Wire contract

Topics live under `terra/rover/<id>`:

| Suffix | Purpose |
| --- | --- |
| `/autonomy` | Select `target_search`; selection alone does not start motion |
| `/search` | Version 1 request below |
| `/search/action` | Version/run/search/token and `pause`, `resume`, or `cancel` |
| `/search/status` | Correlated phase, receipt, elapsed time, goal, detector health and confirmed report |
| `/search/report` | Individual confirmed reports replayed fairly until acknowledged |
| `/search/report/ack` | Version/run/search/report/token acknowledgement |

```json
{"version":1,"run_id":"COPY_FROM_AUTONOMY_STATUS","search_id":"search-42","token":"start-42","target_class":"survivor","bounds":{"min_x":-20,"min_y":-20,"max_x":20,"max_y":20},"time_budget_s":600}
```

Requests are at most 2048 bytes. Run/search/token IDs follow existing 1–64 character ASCII alphanumeric/`._:-` rules. Target classes use 1–64 ASCII alphanumeric/underscore/hyphen characters. Each rectangle side is at most 100 metres; budget is >0 and <=3600 seconds. The entire inflated rover footprint must start and remain within the bounds. Report IDs permit up to 128 characters. Reusing a search ID in the same arbiter session is rejected; identical request tokens cannot relaunch motion.

A search finishes as `completed`, `cancelled`, `timed_out` or `exhausted`. `exhausted` means no presently reachable exploration vantage; it is not proof that the target is absent. `needs_attention` requires explicit resume after recovery. Confirmation stops while unavailable/stale observations cannot count toward success.

## Observation and safety rules

Detector registration supplies class names, a version and local monotonic heartbeat time. An adapter must refresh its heartbeat even when a frame contains no detection. Missing required pose, map, controller health or detector freshness holds search through the shared arbiter. Sensor evidence is accepted only through onboard adapter ingress, not an operator Zenoh topic.

Default confirmation requires confidence >=0.8, at least three distinct increasing frame IDs spanning >=0.5 seconds, <=1 metre pairwise position disagreement, and each observation received within 0.5 seconds. These are proposed configurable engineering defaults; they are not calibrated physical recognition thresholds. The Rust controller supports a confirmation configuration; experiment manifests record it. Evidence includes the contributing frame/evidence IDs and an estimated target position.

Navigation keeps mission-scoped observed cells across rolling-map shifts, with a 250,000-cell capacity. It routes through footprint-inflated observed free cells and restricts local-planner trajectories to the search boundary. A resolution change or incompatible/invalid input holds rather than silently resetting the map. Route failure recovery is bounded to three failures; normal pause/resume does not consume recovery attempts.

Pending reports are bounded to 128 per rover; reaching capacity rejects new searches rather than losing completed reports. Clients retain reports before acknowledging them. Report retries and acknowledgements are separate from the local confirmed outcome. Native clients use string levels, so the new name does not renumber existing levels.

## Validation

See [L4 evidence](evidence/l4/README.md). The deterministic replay validates the portable control loop and the simulator observation adapter with simulated range/field-of-view/occlusion. It does not measure camera-model accuracy, real vehicle braking, or rendered physics performance.
