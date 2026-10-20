# Issue 53 Skid-Safe Turning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Bound ARGOS teleop and autonomous turning for local and existing WGS84 waypoints while preserving direct TerraPhone Bluetooth manual control.

**Architecture:** A reusable Rust turning policy constrains managed commands before both fleet-effort and feedback routes. Planner collision checks evaluate the constrained trajectory, and a final feedback constraint prevents counterrotation when spins are disabled. Explicit command origin preserves the local manual exemption.

**Tech Stack:** Rust workspace, serde/Zenoh telemetry, UniFFI, Swift iOS integration and regression tests.

**Spec:** `docs/superpowers/specs/2026-10-19-issue-53-skid-safe-turning.md`

## Global Constraints

- Direct TerraPhone Bluetooth manual driving remains unchanged.
- Initial managed limits: yaw 0.5 rad/s, angular acceleration 0.5 rad/s², angular deceleration 0.8 rad/s², minimum radius 0.5 m, spins disabled.
- Stops, disarm, stale evidence and authority loss override smoothing immediately.
- Do not alter or commit unrelated existing local changes; preserve source timestamps and command authority.
- Apply turn policy to local and WGS84 goal outputs. Outdoor GPS mission support is a separate dependency, not implemented or advertised by this issue. Preserve the linked ARGOS design's capability gate, independent AR frame, and distinct mission/intermediate-goal semantics.
- No remote motion testing, Pi deployment or phone installation.

## Review Focus

1. A local joystick update or forged network field cannot exempt a fleet command (Task 2).
2. Rapid forward reduction cannot leave an invalid radius while yaw decelerates (Task 1).
3. Feedback integrals cannot generate counterrotating wheels or a restart surge (Task 2).
4. A constrained trajectory must not differ from its collision-checked trajectory (Task 3).
5. Reaching a local guidance target cannot imply a future outdoor GPS mission completed (Tasks 3 and 5).

### Task 1: Stateful turning policy and configuration

**Files:** Create `crates/terra-control/src/turning.rs`, `crates/terra-control/tests/turning.rs`; modify `crates/terra-control/src/lib.rs`, `crates/terra-control/Cargo.toml`.

**Interfaces:** Export `TurningConfig` with `max_yaw_rate`, `angular_acceleration`, `angular_deceleration`, `min_turn_radius`, `allow_in_place`; `validate(&self) -> Result<(), InputError>`; `TurningLimiter::new(config: TurningConfig) -> Result<Self, InputError>`; `reset(&mut self)`; `step(&mut self, target: VelocityTarget) -> Result<VelocityTarget, InputError>`; `TurningConfig::constrain_wheels(&self, left: f64, right: f64) -> (f64, f64)`.

- [ ] Add failing tests: steady yaw cannot exceed 0.5; a 0.1 s acceleration step changes yaw by at most 0.05; deceleration changes it by at most 0.08 except immediate radius enforcement; a reversal crosses zero; zero forward yields zero yaw by default; 0.1 m/s forward limits yaw to 0.2 rad/s; reverse motion obeys the same radius; explicitly enabled spins permit bounded nonzero yaw at zero forward. Repeated/backward timestamps, NaN, infinity and overlong gaps fail closed. Reset begins from zero with no inherited time credit.
- [ ] Run `cargo test -p terra-control --test turning`; confirm failure from missing behavior.
- [ ] Implement the interfaces. Use at most 0.01 s on the first step and reject gaps exceeding 0.1 s. Validate numeric settings in `(0, 5]` for yaw/radius and `(0, 10]` for acceleration/deceleration. Saturate desired yaw before slew and enforce radius afterward; never invent translation. Invalid input resets state. Constrain managed wheel mixing without changing forward direction or increasing either wheel magnitude; pure counterrotation becomes zero when spins are disabled.
- [ ] Run the test again and all `terra-control` tests. Verify configuration serialization defaults and rejection of invalid settings.

### Task 2: Command origin, fleet route and feedback enforcement

**Files:** Modify `crates/terra-mobile/src/lib.rs`, `crates/terra-mobile/src/autonomy.rs`, `crates/terra-mobile/src/transport.rs`, `crates/terra-control/src/lib.rs`, `crates/terra-autonomy/src/arbiter.rs`, `mobile/ios/TerraPhone/PhoneController.swift`; extend `crates/terra-mobile/tests/dashboard_manual.rs`; create `crates/terra-mobile/tests/managed_turning.rs`.

**Interfaces:** Add `terra_autonomy::CommandOrigin::{LocalManual,FleetTeleop,Autonomous}` stored with accepted intent, not inferred from the screen. Add `AutonomyArbiter::accept_operator_from(request: TeleopRequest, now: f64, origin: CommandOrigin) -> bool` with existing acceptance validation; ordinary `accept_operator` delegates to local origin. Expose the accepted origin in `ArbiterOutput`. Add `VelocityController::set_turning_constraint(Option<TurningConfig>)`; consume Task 1 policy once per managed output, not twice.

- [ ] Add failing tests for accepted fleet `teleop` and `cmd_vel`, rejected/replayed requests, a forged exemption field, local input during remote ownership, remote-to-local transition, authority revision changes, disconnect, stop and cancellation. Assert direct local manual wheel outputs remain exactly unchanged, including stationary steering. Assert fleet status twist and the Swift fleet route use constrained output.
- [ ] Run `cargo test -p terra-mobile --test managed_turning` and the existing dashboard manual test; record baseline failures.
- [ ] Carry origin through accepted arbiter intent; reset limiter history on origin/run/authority changes and safety holds. Ensure existing local-manual behavior bypasses only the new policy, not existing authority/safety checks. Apply final no-counterrotation inside managed feedback output before anti-windup calculation. Do not assume clamping desired yaw constrains actual PI wheel effort.
- [ ] Update only the Swift fleet branch if necessary to consume the constrained twist; keep its direct disconnected/local joystick path byte-for-byte behavior compatible. Reuse existing stopped-output routing.
- [ ] Run `cargo test -p terra-control -p terra-autonomy -p terra-mobile`. Compile and execute `tests/ios/DriveJoystickTests.swift` with its existing production dependencies plus a regression fixture for fleet output and local exemption.

### Task 3: Planner-consistent arcs and measured heading completion

**Files:** Modify `crates/terra-navigation/src/lib.rs`, `crates/terra-navigation/Cargo.toml`, `crates/terra-autonomy/src/arbiter.rs`, `crates/terra-waypoint/src/lib.rs`; extend `crates/terra-navigation/tests/planner.rs`; create `crates/terra-autonomy/tests/turning.rs` and `crates/terra-waypoint/tests/turning.rs`.

**Interfaces:** Extend planner input with the current managed turning window so candidate generation, trace, rollout and selected output share the exact bounded velocity. Carry the resulting allowed command unchanged into Task 2's managed-output boundary. Add explicit `heading_requires_spin_or_replan` status when a heading-only maneuver cannot execute under the current policy.

- [ ] Add failing tests for a free-space arc, occupied/unknown arc rejection, low-speed curvature, final-heading/no-spin hold, enabled-spin heading alignment, and all autonomous command levels. Verify the selected trace velocity equals the executed command. Test local and WGS84 goals with equivalent projected destinations.
- [ ] Add heading tests: advancing time with unchanged yaw never reports arrival; a fresh measured yaw inside tolerance does; crossing ±π takes the short error; stale/degraded pose cannot complete a turn. Keep geographic destination/token metadata unchanged.
- [ ] Run `cargo test -p terra-navigation -p terra-waypoint -p terra-autonomy`; confirm failures for new behavior.
- [ ] Constrain candidates before collision rollout, including braking tails. For heading alignment prefer an observed collision-free forward arc; if none is supported, hold with the explicit reason. Do not fabricate translation for direct/unchecked paths. Preserve position-only arrival and explicit-spin measured-yaw alignment. Retain all safety/search/arming timeouts.
- [ ] Run the same tests plus existing search/exploration suites. Audit source for elapsed-time angle completion and document exact findings; do not claim removal if none exists. Never reinterpret WGS84 conversion as live outdoor GPS guidance.

### Task 4: Settings, pose quality and autonomous admission

**Files:** Modify `crates/terra-mobile/src/navigation_settings.rs`, `crates/terra-mobile/src/navigation_settings_tests.rs`, `crates/terra-mobile/src/lib.rs`, `crates/terra-mobile/src/autonomy.rs`; create `crates/terra-mobile/src/pose_quality.rs`; extend `crates/terra-mobile/tests/dashboard_manual.rs` and `crates/terra-mobile/tests/dashboard_navigation.rs`.

**Interfaces:** Add serde-defaulted `turning: TurningConfig` to navigation parameters. Publish version-1 `pose/quality` JSON containing `rover_id`, `run_id`, `sequence`, `degraded`, `reason`, `pose_age_s`, and `imu_age_s`; unavailable ages are null. Include this object in autonomy status. Centralize quality calculation for both network and local autonomous goal admission.

- [ ] Add failing tests for old settings payloads receiving safe defaults, invalid settings rejecting atomically, stale authority/revision rejection, and explicit spin enablement only through accepted settings. Test degraded messages without pose publication, stale/future samples, duplicate sample freshness, tracking recovery and preserved stop latches.
- [ ] Run navigation-settings and dashboard integration tests; confirm expected failures.
- [ ] Apply validated settings to planner, limiter and final managed feedback constraints in one accepted revision. Reset controller/limiter history without arming. Publish quality transitions and periodic updates with finite ages from original sensor times.
- [ ] Gate new autonomous goals and hold active autonomy on degraded pose through all ingress paths; cancellation remains accepted. Leave direct local manual tracking-loss policy unchanged. Avoid granting outdoor capability or bypassing geographic command gating in ARGOS.
- [ ] Run `cargo test -p terra-mobile` and inspect captured Zenoh messages for healthy, degraded and absent-pose cases.

### Task 5: End-to-end regression, documentation and review

**Files:** Update `docs/crates/terra-control.md`, `docs/MOBILE_CONTROL.md`, and relevant navigation/telemetry documentation; add `docs/autonomy/turning-policy.md`.

- [ ] Add end-to-end regression cases connecting network command acceptance, origin, constrained twist, simulated wheel output and health holds. Cover local and WGS84 goals plus unchanged direct Bluetooth manual mapping. Test source switches and recovery without renewed authority. Assert GPS arrival/capability semantics are unchanged.
- [ ] Run `cargo fmt --all -- --check`, `cargo test --workspace`, relevant Swift regression executables, and the project iOS build. Regenerate UniFFI bindings only if exported interfaces change; run `scripts/check-swift.sh` for binding drift. Report unavailable checks and every observed failure.
- [ ] Document units, defaults, override bounds, no-spin hold behavior, command-source exemption, pose-quality contract and GPS dependency. Clearly state that software checks do not establish physical skid resistance, GPS accuracy or VIO quality.
- [ ] Review only issue-53 changes against the approved spec, preserving unrelated dirty work. Prepare the implementation for user review; do not deploy or close the issue based on software tests alone.

## Execution and dependency note

Recommended execution: native in this chat, because command-source ownership,
planner trajectories and final output constraints share state and must be reviewed
together. The required execution review occurs after this written plan is approved.
Outdoor GPS mission implementation is not silently bundled into issue 53: use
the linked ARGOS design as its separate implementation contract.

## Execution record

User approved native execution with no further approval stops. Existing local
changes are preserved on `codex/issue-53-skid-safe-turning`. Implementation covers
Tasks 1–4 and the subsequently requested reverse/multi-point planning extension.
Full Rust workspace tests, direct-manual and fleet Swift tests, and Swift/UniFFI
smoke checks passed. iOS framework build and independent review are in progress.

Rulings: keep work in the existing checkout on a dedicated branch to preserve
uncommitted runtime integrations; retain existing local-origin WGS84 semantics
and leave outdoor GPS missions separate; make cached managed planner commands
re-evaluate each tick so changed slew limits cannot bypass collision checks.
The latter trades additional planning work for exact checked/executed agreement.
