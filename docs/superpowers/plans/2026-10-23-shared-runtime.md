# Shared Terra Runtime Implementation Plan

Goal: one pure Rust runtime for phone and simulator control, mapping, autonomy,
command admission, navigation settings, connectivity, and structured diagnostics.
The approved design is the Shared Terra Runtime Implementation Plan supplied in
this chat on 2026-10-23. Execute sequentially with the executing-plans workflow.

## Constraints and interfaces

- Preserve both DirectEffort and VelocityFeedback modes. Preserve managed no-spin
  constraints and distinct trusted local manual input.
- Preserve Pi actuator ID/int8 -100..100 payloads and 50 ms output streaming.
- Do not introduce arm states, gate logic, watchdogs, or new timeout values.
- Runtime owns estimator, map, arbiter, controller, settings and policy; adapters
  own transport, platform conversion, publication, recording and actuation.
- No sockets, filesystem, Bluetooth, Bevy, UniFFI or wall clocks in the runtime.
- Runtime::new(RuntimeConfig, String) -> Result<Runtime, RuntimeError>;
  ingest(RuntimeEvent) -> Result<(), RuntimeError>;
  step(f64) -> Result<RuntimeOutput, RuntimeError>;
  snapshot(f64) -> RuntimeSnapshot.
- Events retain sensor source and receipt timestamps; expiry is checked when
  applying actions, with current run/revision and packet ordering. Provenance is
  trusted adapter input and cannot be forged in a remote payload.
- Both adapters step at 100 Hz; physical relay output remains 20 Hz. Simulation
  true state generates measurements and evaluates error, not navigation inputs.

## Checkpoint

Verified pushed working-state checkpoints, not green releases:
Terra 6166b8c587fef645df9269a809dfe655fd012d4f;
Zorvane a3d69c21ca627225fe0ba732dd0dd162d60552c2;
ARGOS fb1fb3171ce7b94cf5ef5e3f0f588e01ff8fac6c.
Existing tests include obsolete hardware/arm and target-search fixtures.
No force-pushes or merges. Push existing branches Terra, then Zorvane, ARGOS.

## Phases

- [x] Checkpoint tracked and intended untracked source in all three repositories,
  push existing unpushed commits, verify each remote branch SHA.
- [ ] A: deterministic baseline fixtures for manual/assisted/waypoint/turn/stop,
  settings/link/sensor failure; update obsolete fixtures; pin the complete Terra
  checkout in Zorvane terra-source.json and CI, eliminating mixed Git sources.
- [ ] B: extract orchestration from terra-mobile into terra-runtime, retaining
  underlying crate algorithms; leave UniFFI/publication/recording in adapters.
- [ ] C: unify remote admission, configuration revisions/receipts, execution-time
  expiry, stop/neutral behavior, sequence/replay handling; remove legacy hardware
  arm/disarm ingress and unbound cmd_vel; move Swift effort mixing into Rust.
- [ ] D: one runtime per simulated rover; synthetic IMU/VIO samples feed shared
  estimation and autonomy; returned efforts drive existing physics; mode switches
  clear controller accumulation; no parallel fallback autonomy implementation.
- [ ] E: share depth/ground validation, recentering, map integration, GPS/heading,
  source selection/resume, geographic missions, search/detection/reports and
  exploration. Coordinate conversion stays in platform adapters.
- [ ] F: shared reconnect policy, invalidate remote authority/no replay, continue
  accepted missions with fresh onboard evidence; recording health separate from
  router health; common event/reason codes and runtime/config/mode provenance.
  Do not restart hosted Rerun.
- [ ] G: ARGOS consumes readiness/capabilities/settings and receipts; display
  rejections, preserve controls; delete superseded orchestration; update focused
  docs. Publish Terra first, update simulator pin, publish simulator/dashboard.
  Build/install phone and desktop apps. Pi reflash only if protocol incompatible.

## Verification and rollout

Exact equality: admission, transitions, reasons, revisions, origin. Float absolute
error <=1e-6 native; <=1e-5 native/wasm. Replay identical configuration and ordered,
timestamped events through mobile/simulator harnesses. Test both actuation modes,
spin/forward/reverse/reversal/reset, missing/stale/reordered/future sensor data,
stale run/revision/sequence/expiry/stop dominance, router loss/reconnect, settings,
geometry/wheel constraints, depth/mount/slope/ground/stale maps, local/GPS goals,
GPS/heading loss/resume, search confirmation/loss/ack, recorder absence/failure.
Run focused tests per phase, workspace checks, native/browser builds, mobile
binding generation and iPhone build. Roll out isolated simulation, phone/Pi
neutral/config check, controlled manual/waypoint runs, controlled fault injection,
then compare recordings/provenance. Never deploy unintentional baseline changes.
Keep checkpoint builds available. Report every remaining verification failure.
