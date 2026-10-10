# Actuator Pagination Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Read and edit one actuator per configuration request, preserve atomic whole-layout commit and safe driving, and build rover and phone artifacts.

**Architecture:** The rover owns one revision-bound edit draft. The phone lazily fetches selected editable actuators, while separately synchronizing compact runtime profiles one by one. Existing owner admission, persistence rollback and motor timing are retained.

**Tech Stack:** Python/dbus-next/Bless, Swift/SwiftUI/CoreBluetooth, existing Rust mobile bindings, Docker ARM64, Xcode.

**Spec:** docs/superpowers/specs/2026-10-10-actuator-pagination-design.md

## Global Constraints

- One complete actuator per configuration request/reply; pagination capability `actuator_pagination_v1`.
- Drive/priority assembly 100 ms, configuration JSON assembly 2 seconds, configuration request deadline 5 seconds.
- One draft, 16 actuators, 4096-byte paginated envelopes. Revision/token/version checks bind operations to one connection.
- Atomic validate/commit; gates and motor watchdog unchanged. Legacy operations retained for old phones.
- No deployment, commit or push unless separately requested; build requested artifacts.

## Review Focus

- A revision changes mid-profile synchronization: clear profiles and block arm until a coherent new set arrives.
- Lost commit acknowledgment: retry same request returns cached result without a second revision increment.
- Partial preset upload: never apply an incomplete replacement draft automatically.
- Page load replies arrive after selection/session changes: ignore stale replies, retain unsent edits.
- A single actuator exceeds the transport deadline: show retryable error, preserve active layout and local draft.

### Task 1: Rover transactions and protocol

Files: hardware/raspberry-pi/terra_rover/{protocol,configuration,ble}.py; hardware/raspberry-pi/tests/test_actuator_pagination.py and existing tests.

Interfaces: `layout_index` returns `{revision,layout_available,actuator_count,actuators:[{id,name,kind}]}`; `read_actuator` returns `{revision,actuator}`; `read_drive_profile` returns `{revision,actuator}` with keys id/kind/limits/route/safe. Edit replies return `{edit_token,base_revision,edit_version}` with optional actuator_id; validate marks exact version; commit returns `{revision}`. Tokens are strings. All reads use expected_revision except layout_index.

- [x] Add failing tests for paginated reads, tokens/versions, incremental add/remove, validation conflicts, atomic/replayed commit, disconnect and legacy isolation.
- [x] Extend strict control shape validation and implement one draft using existing store safety/persistence logic.
- [x] Bound paginated replies and advertise capability; run complete rover tests.

### Task 2: Rover transport deadlines

Files: protocol.py, ble.py, transport tests.

- [x] Test multi-fragment JSON arriving over 100 ms succeeds while binary drive/priority expire and >2-second JSON fails.
- [x] Parameterize FragmentAssembler(timeout=0.1), set JSON timeout 2.0 and route expiry by selected assembler; preserve size and concurrency bounds.
- [x] Verify suite and diagnostic timing logs.

### Task 3: Compact phone synchronization

Files: BluetoothLink.swift, BluetoothSession.swift; new ActuatorDriveProfile.swift; profile tests; Xcode/source generator references.

Interfaces: BluetoothLink publishes `layoutIndexJSON` and `driveProfileJSON` (revision + compact actuators), never full editable layouts. All external paginated replies continue via replyJSON. `refreshConfigurationIndex()` triggers new index/profile synchronization. Phone runtime routing/safe/feedback helpers consume compact JSON directly. Editing readiness requires capability, session and coherent index; arm readiness requires all profiles.

- [x] Add profile routing/validation and configurable JSON assembly tests.
- [x] Replace automatic read_layout with capabilities/index then serial profile reads; add revision/generation invalidation and stale-reply rejection.
- [x] Apply 5-second configuration request deadline with retryable error replies; keep drive/priority deadlines.
- [x] Verify Foundation tests and compile app.

### Task 4: Lazy editor and controller

Files: new ActuatorPagination.swift state/models; PhoneController.swift; Views/ActuatorLayoutView.swift; new selected-actuator editor view; Swift tests.

Interfaces: consume layoutIndexJSON/driveProfileJSON, read_actuator reply and token/version protocol from Task 1. UI uses index and selected actuator only. Maintain edit draft index, unsent edits, serial preset acknowledgments, validate/apply and commit refresh. Phone runtime must call compact profile routing/safe/feedback methods instead of old full-layout Rust helper; no fake fields.

- [x] Add failing state tests for stale page responses, replacement presets, draft invalidation and commit acknowledgment.
- [x] Implement one-at-a-time editing and root controller integration; preserve safety controls and servo controls using compact profiles.
- [x] Verify tests, build and review flows.

### Task 5: Integration and artifacts

- [x] Review interfaces and run all Python rover tests plus relevant Swift tests.
- [x] Build iPhone Debug app and ARM64 Docker rover bundle; verify bundle smoke checks.
- [x] Update BLE protocol docs and report artifacts and physical verification remaining.
