# Shared vehicle runtime

`terra-runtime` owns portable estimator/controller, occupancy-map, autonomy,
command admission, navigation/search and connectivity policy. TerraPhone uses
`terra-mobile` as its UniFFI/transport/recording adapter. Each Zorvane rover owns
one runtime; simulated IMU/VIO feed it and returned motor efforts drive physics.
There is no second simulator autonomy implementation in the production schedule.

Create a `RuntimeConfig`, ingest timestamped `RuntimeEvent`s, then call `step(now)`
at 100 Hz. Time is monotonic seconds supplied by the adapter. Equal-time events
retain arrival order; backwards steps fail. Stop requests are evaluated last in
a queued batch. Sensor source times cannot be after their receipt times. Remote
requests are checked against the current run/revision and original receipt age
at execution; local input enters through separate typed events. Transport loss
retires remote drive authority without replaying input. The configured policy
may retain an accepted onboard mission while local evidence remains valid.

`DirectEffort` preserves the phone's normalized manual mixing. Autonomous levels
use `VelocityFeedback`; mode changes clear previous operator intent/controller
accumulation. Managed wheel constraints remain distinct from local manual input.
Calibration may differ between physical and simulated plants; behavioral policy
comes from the same navigation/configuration types. Existing freshness defaults
are in `terra-types::timing`; no new timeout durations were introduced.

`MappingState` is a shared handle so platform exposure callbacks can integrate
frames without copying a second mapper. `GroundPolicy` is the portable phone
floor/mount policy. Simulated clearance uses the camera mount and body dimensions,
not absolute terrain elevation. Runtime identities include the attached map's
settings. Runtime source revision, configuration identity and actuation mode are
published in status/control records. The adapters persist/upload events; this
crate performs no transport or recording I/O.

The Pi wire protocol remains actuator ID plus integer effort from -100 to 100,
streamed every 50 ms. Its image and relay code do not need changes for this runtime.

## Verification

- `cargo test -p terra-runtime -p terra-mobile`: portable policy and phone facade replay.
- Zorvane `cargo test -p zorvane simulator_sensor_adapter`: actual sensor adapter replay.
- `node scripts/check-runtime-parity.mjs`: run the same trace natively and in
  WebAssembly. Discrete status hashes must match exactly; effort/twist tolerance
  is 1e-5. Requires Node and the wasm32-unknown-unknown Rust target.
- Full workspace suites include outdoor navigation, exploration/search, transport,
  recording and dashboard fixtures. Their physical calibration is not a live-run
  qualification.

Zorvane's `terra-source.json` pins the entire sibling Terra checkout for CI and
releases. Update it only after the corresponding Terra revision is pushed. Keep
all Terra dependencies on that checkout; do not mix registry/Git/source revisions.

Rollback checkpoints and rollout verification are recorded in
`docs/superpowers/plans/2026-10-23-shared-runtime-ledger.md`. The remaining live
rollout requires a connected rover and controlled manual/waypoint/fault runs.
Hosted Rerun remains stopped.
