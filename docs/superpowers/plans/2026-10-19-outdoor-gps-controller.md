# Outdoor GPS controller implementation

Spec: ARGOS/docs/superpowers/specs/2026-10-18-outdoor-gps-waypoints-design.md.

1. Add a tested mission state machine: timestamped GPS and mount-corrected true heading, spherical distance/bearing, 2 Hz local guidance, immutable goal/radius, distinct-fix arrival, latched sensor pauses and explicit resume.
2. Integrate source selection and mission admission with mobile control, hardware evidence, run/revision and L3 authority. Preserve local planning collision checks and bound speed before candidate evaluation. Publish version-1 outdoor reports and geographic mission status.
3. Feed measured iOS samples with original timestamps and paired compass/AR axes; publish hardware feedback availability; regenerate bindings and build.
4. Reconcile ARGOS protocol, test end-to-end and run independent review.

Ruling: continue in the existing issue branch because outdoor guidance depends on its uncommitted turning work; preserve unrelated changes. No deployment or physical motion is implied by implementation.
Ruling: ARGOS's implemented drag-release confirmation supersedes the design's earlier separate confirm button. Do not weaken capability or hardware gates.

Progress: design read; implementation started. The previous turning review identified pose freshness, downstream clipping, and forward-only maneuver omissions; resolve before outdoor verification.

Implementation record:
- Mission engine, mobile/transport bridge, iOS measured ingestion and GPUI resume implemented.
- Geographic wire envelope now includes reviewed run/revision/radius. Ruling: this extends the earlier coordinates-only draft because the rover must verify the review rather than infer its radius from mutable defaults.
- Independent reviewer checked safety/state boundaries. Fixed rejected-sensor invalidation, replay watermark preservation, dynamic-window endpoint caps, stale goal age, source acknowledgement revision, sensor readiness vs L3 admission, and guidance restoration after authority reset.
- Additional checks cover idle arming after source selection and AR-frame invalidation.
- Full Terra workspace and ARGOS core/server/Zenoh/GPUI tests passed before final frame-reset adjustment; final rerun and app build recorded below when complete.

Final verification:
- `cargo test --workspace` in Terra: passed, including 8 outdoor cases and turning regressions.
- ARGOS core/server/Zenoh/GPUI library and integration suites: passed, including geographic envelope publication and source acknowledgement revision.
- Swift mount-heading, joystick, fleet manual/turning, stop diagnostics and Swift-to-Rust binding smoke: passed.
- Swift/Kotlin bindings regenerated; iOS framework and full simulator app build succeeded. Kotlin ingestion/Android deployment not enabled.
- Final frame reset also discards previous pose/map/estimator evidence before admitting a new mission.
- No phone installation or physical motion test performed. User-approved follow-up replaces the wheel-feedback gate with fresh phone VIO and compatible differential actuator routing. Hardware readiness, arming and sensor checks remain mandatory.

VIO feedback follow-up (2026-10-19):
- Removed encoder admission from outdoor readiness; phone VIO remains the measured feedback source with existing freshness checks.
- Allowed compatible differential manual-mixer layouts to consume closed-loop L3 efforts. Direct phone Bluetooth manual routing remains unchanged.
- Fixed pause/rearm recovery by clearing local guidance while paused; sensor recovery and arming do not resume geographic motion without an explicit Resume.
- Verified Terra workspace, 9 outdoor regressions, Swift joystick/fleet/turning/profile/stop checks, ARGOS outdoor hold-label tests, and full iOS framework/simulator app build. Independent review found no remaining blocker.
- Updated app has not been installed on the phone; no physical motion test performed.
