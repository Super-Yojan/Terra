# Autonomous Operation Through Network Loss Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Continue accepted onboard autonomy through cloud interruptions while preserving remote-command expiry and all local stop causes.

**Architecture:** Give automatic link loss its own controller operation, distinct from operator cancellation and local fault handling. Mission eligibility comes from accepted mission state and local evidence; router availability governs remote supervision/intent, not whether a locally executable mission exists.

**Tech Stack:** Rust autonomy/mobile/outdoor controllers, Zenoh, Swift connection policy, simulation/loopback tests.

**Spec:** [Rural connectivity and reliable run recording](../specs/2026-10-22-rural-connectivity-reliability.md)

## Global Constraints

- Use `ConnectivityPolicySettings::autonomous_router_loss` from the visibility plan; default `ContinueAcceptedMission`, alternative `Pause`.
- Continue only already accepted locally executable autonomy with healthy local requirements; no automatic arming or fault-pause resumption.
- Keep current manual command leases, emergency-stop latches, explicit cancel semantics, local hardware/sensor checks and mission budgets.
- Logs/evidence retry after reconnection; historical motor/arm/servo commands never replay.
- The 120-second outage in tests is a scenario, not a new sensor cutoff.

## Review Focus

- Network loss during teleop must expire intent rather than continue driving.
- Healthy local autonomy must survive router loss without silently accepting new work.
- Local sensor/hardware faults and emergency stop must still stop/pause during an outage.
- Reconnection after cancel/completion/fault must not arm or restart motion.
- Session changes, duplicate goals and delayed commands must preserve authority and replay protection.

### Task 1: Model router loss separately from mission cancellation

**Files:** Modify `crates/terra-mobile/src/{autonomy,outdoor_bridge}.rs`, `crates/terra-autonomy/src/arbiter/{motion,exploration,search_requests}.rs` where mission/remote authority are coupled, and `terra-types` policy access; extend `crates/terra-mobile/tests/{dashboard_reconnect,outdoor,managed_turning,search}.rs` and the associated `terra-autonomy/tests` scenarios.

**Interfaces:** Add `MobileController::handle_dashboard_link_loss() -> Result<(), ControllerError>` for automatic transport failure. Preserve `disconnect_dashboard()` as explicit cancellation/disconnect. Add `set_connectivity_policy_settings(settings_json: String) -> Result<(), ControllerError>` with validation and an observable policy revision; changing policy never arms/resumes a paused mission. Reconnect keeps the accepted mission identity.

- [ ] Write failing simulated-clock tests for 120 seconds without a router: accepted local waypoint, geographic waypoint, exploration and supported search continue with fresh required local inputs and continuing mission budgets. Add `Pause` policy tests and supervised-mode tests that require new approval before a new proposal executes.
- [ ] Add tests for local map/pose/IMU failures, hardware-link loss, emergency stop, explicit cancel, expired mission budget, duplicate goal tokens and new run IDs during that outage. Assert each retains its distinct reason and reconnect never resumes a latched pause or completed/cancelled goal.
- [ ] Run `cargo test -p terra-mobile --test dashboard_reconnect --test outdoor --test managed_turning` plus the corresponding autonomy/search suites and verify the intended network-coupling failures.
- [ ] Implement the new operation: clear expired remote intent and pending hardware requests; close transport asynchronously; preserve eligible mission state. Remove unconditional `outdoor.pause("authority_lost")` on automatic link loss while retaining it for explicit cancellation or the configured pause policy. Revalidate local conditions every execution tick, including the ticks before Swift handles the failed transport.
- [ ] Run the tests to passing and verify existing short remote command leases and replay protections. Log link-loss policy, mission identity and the actual continuation/pause decision.
- [ ] Commit this controller change independently.

### Task 2: Route phone lifecycle/reconnect paths consistently

**Files:** Modify `mobile/ios/TerraPhone/{PhoneController,TerraAutoConnectionPolicy,DriveJoystickCommand}.swift`; extend `tests/ios/{FleetManualDriveTests,PhoneLocalizationTests,TerraAutoConnectionTests,TerraConnectionLifecycleTests}.swift`. Update runtime bindings and `docs/observability/operations.md`.

**Interfaces:** Swift calls `handleDashboardLinkLoss` for automatic router failures and explicit disconnect/cancel APIs for user actions. Replace the waypoint-only continuation predicate with an accepted-onboard-mission eligibility report from the native controller. All reconnect paths preserve mission/run identity and request fresh operator authority for new remote intent.

- [ ] Write failing Swift policy tests for each execution mode, reconnect/backoff, phone lifecycle stops, manual driving expiry and local Bluetooth loss. Assert only the network-only case permits eligible autonomy to continue; explicit stop/cancel is never reclassified as a network interruption.
- [ ] Add a loopback integration scenario: accepted mission, router down/up, logging backlog, current-state recovery and duplicated/delayed old commands. Assert mission identity is retained, old motion/arm commands are rejected, and GCP evidence has the full outage/recovery timeline after catch-up.
- [ ] Run the Swift policy runners and affected native suites; verify the intended failures.
- [ ] Wire automatic failure/retry branches to the new native operation and policy; remove Swift's waypoint-only transport exception. Preserve local runtime/sensor operation and per-tick admission. Do not send historical teleop or hardware intent after reconnect.
- [ ] Run `cargo test -p terra-mobile -p terra-autonomy -p terra-transport -p terra-observability`; run the Swift policy tests and signed device build. Apply storage/visibility changes first, then verify disarmed reconnection, simulation outages and a supervised low-speed physical run.
- [ ] Commit and record policy values, versions, accepted mission ID, local stop tests and post-reconnection GCP/Rerun catch-up evidence.

## Release sequence

1. Deploy log storage changes to phone and rebuild GPUI; confirm measured backlog drain.
2. Deploy the GPS report/presence UI; confirm the rover remains visible without a fresh fix.
3. Enable the explicit autonomous router-loss policy after simulation and local-stop checks.
4. Preserve rollback builds and all recorded evidence. Diagnose future actual Zenoh session failures using the independent sidecars; do not declare them solved by these changes alone.
