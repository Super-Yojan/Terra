# Rover Visibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep connected and last-known rovers visible when GPS or network updates age.

**Architecture:** Separate fleet membership, local pose, last geographic observation and map-marker eligibility. Keep raw GPS observations with their timestamps; display freshness labels must not erase measurements or alter navigation admission.

**Tech Stack:** Swift localization reporting, Rust/serde telemetry, GPUI fleet model and UI.

**Spec:** [Rural connectivity and reliable run recording](../specs/2026-10-22-rural-connectivity-reliability.md)

## Global Constraints

- Initial GPS display freshness is 3 seconds and fleet stale presentation is 90 seconds; neither hides a rover or authorizes motion.
- Keep permission/quality reasons and observation ages; never relabel an old measurement as fresh.
- Do not infer a geographic alignment from an unaligned local AR pose.
- No changes to existing navigation GPS eligibility or local sensor cutoffs in this deliverable.

## Review Focus

- A fleet member with no pose or GPS must still have a visible row.
- An old but finite GPS observation must remain explicitly last-known.
- A new run/frame must invalidate incompatible cached map placement.
- Permission revocation and malformed coordinates must preserve truthful status.
- Router interruption and recovery must update ages without duplicating membership or resetting timestamps.

### Task 1: Preserve GPS observations and configure display freshness

**Files:** Modify `mobile/ios/TerraPhone/{PhoneLocalization,PhoneController}.swift`; extend `tests/ios/PhoneLocalizationTests.swift`; create `crates/terra-types/src/connectivity_policy.rs` and export it from `lib.rs`; add serde derive support to `crates/terra-types/Cargo.toml` using the repository's existing serde version; modify `crates/terra-mobile/src/lib.rs` for binding the default display policy.

**Interfaces:** Add serialized `ConnectivityPolicySettings` with `gps_display_freshness_seconds: f64 = 3.0`, `fleet_display_stale_seconds: f64 = 90.0`, and `autonomous_router_loss: RouterLossPolicy` (default `ContinueAcceptedMission`, alternative `Pause`). Validation requires finite positive display ages. Export `default_connectivity_policy_settings() -> String` for Swift; ARGOS consumes the same type through an explicit path dependency and re-export in `argos-core`. Keep navigation eligibility separate from this type. These defaults live in one source file; deployment-specific overrides must be recorded with each run.

- [ ] Write failing Swift tests for a valid GPS measurement at ages 0, 3, 8.2 and 120 seconds: retain coordinates/source timestamp at every age; only the display freshness changes. Test missing, invalid and permission-revoked measurements and a new frame ID.
- [ ] Add Rust default/serde/validation tests; demonstrate that display-policy updates do not change map/pose/GPS motion admission.
- [ ] Compile/run `PhoneLocalization.swift` with `tests/ios/PhoneLocalizationTests.swift` using `swiftc`, then run `cargo test -p terra-types -p terra-mobile`; verify expected failures before implementation. If importing the new native policy requires a test fixture, pass the policy through the localization initializer so pure policy tests remain independent of device/FFI setup.
- [ ] Preserve the last valid observation separately from eligibility; add explicit freshness and source-time fields to the localization report. Avoid clearing the original measured fix merely because it ages. Include permission and source-frame status. Bind the shared display policy and regenerate the iOS bindings.
- [ ] Run the tests to passing and document where the shared policy is edited. Keep existing consumers backward compatible during deployment.
- [ ] Commit the report/policy change independently.

### Task 2: Show membership independently from map placement

**Files in `/Users/yojan/git/ARGOS`:** Modify `crates/argos-gpui/src/{fleet,main}.rs`, `crates/argos-core/src/{lib,state,telemetry}.rs` and `crates/argos-core/Cargo.toml`; extend `tests/argos-gpui/unit/fleet_tests.rs` and `tests/argos-gpui/fleet_feed.rs`. Add `terra-types` with path `../../../Terra/crates/terra-types` to `argos-core` and re-export the policy type for GPUI.

**Interfaces:** Add `RoverPresence` as a separate row model containing ID, link/membership, observation age, pose availability and position reason. `FleetPicture` exposes presence rows separately from geographic `RoverMark` values. `describe` counts presence rows, not map markers. Keep existing command admission checks on current telemetry.

- [ ] Write failing model/feed tests: one online member with fresh local pose, local localization and no GPS gives one visible rover and zero map markers. The text is `Position unavailable`, not `0 rovers`.
- [ ] Test old GPS, never-observed GPS, reconnect, duplicate membership, explicit retirement, invalid coordinates and new localization frame/run. Assert last-known positions retain their original age and cannot silently cross frames.
- [ ] Run `cargo test -p argos-gpui -p argos-core` from ARGOS and confirm failures.
- [ ] Implement presence rows, age/quality labels and retained last-known geographic markers. Move the 90-second presentation value into the shared policy. Keep navigation eligibility checks separate. Read applicable GPUI design skills before changing the screen.
- [ ] Run the suites to passing, rebuild GPUI and reproduce the supplied recording: Rover 0 remains visible while GPS exceeds 3 seconds and while the router is temporarily unavailable. Capture UI proof with no credentials visible.
- [ ] Commit the dashboard change and record the phone/dashboard compatibility check.
