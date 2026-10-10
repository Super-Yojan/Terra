# Automatic Rover Connection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace manual rover discovery and connection with pairing consent, remembered-rover reconnection, and automatic ARGOS Zenoh connection.

**Architecture:** A foreground connection coordinator owns consent and retry decisions; BluetoothLink remains the serialized radio adapter. PhoneController applies coordinator effects and starts the existing Rust dashboard transport after authentication/configuration availability. A pure Swift policy supplies deterministic lifecycle tests without radio hardware.

**Tech Stack:** Swift/SwiftUI, CoreBluetooth, Rust/UniFFI/Zenoh, Python/Bless/BlueZ.

**Spec:** `docs/superpowers/specs/2026-10-08-automatic-rover-connection-design.md`, approved by the user's “Create” response to the specification review request.

## Global Constraints

- Preserve existing uncommitted work; stage only changes made for this plan, using selective staging for shared files.
- Pairing name: `terra-<existing rover suffix>-pair`; operational name: `terra-<existing rover suffix>`; retain the existing service UUID.
- Retry delays: 1, 2, 4, 8, then 15 seconds, capped at 15 seconds.
- Save ownership preference after encrypted enrollment confirmation or authenticated normal status, never merely discovery.
- Backgrounding, explicit stop/disconnect, and disabling automation cancel pending work; inactivity disarms without canceling pairing.
- One BLE attempt and one router attempt at a time; obsolete generations cannot publish success or reconnect.
- Use `dashboardEndpoint`, `dashboardPrefix`, `dashboardRoverID`; no endpoint discovery or automatic Bevy connection.
- Automatic connections leave hardware disarmed and targets zero; do not restore motion or missions.
- No new AR/IMU requirement for Bluetooth manual mode; preserve feedback and simulator behavior.
- Hardware verification is required to establish actual pairing, identifier continuity, and radio behavior.

## Review Focus

- Pairing advertisement changes to operational mode with a cached name: fresh advertisement data must win (Tasks 1–3).
- Pairing system dialog briefly makes the app inactive: consent must survive while motion is revoked (Tasks 2, 5).
- A rover has a saved owner but no valid actuator layout: remember it and retain repair access (Task 3).
- Settings change while a blocking router open is completing: discard obsolete completion and close its session (Task 4).
- Switching from manual to phone feedback during router connection: keep the active runtime and reject stale callbacks (Task 4).

## File Responsibilities

- `hardware/raspberry-pi/terra_rover/onboarding.py`: shared pairing-name helper.
- `hardware/raspberry-pi/terra_rover/pairing.py`, `__main__.py`: apply helper only for enrollment.
- `mobile/ios/TerraPhone/TerraAutoConnectionPolicy.swift`: existing selection policy plus pure coordinator/event/effect types.
- `mobile/ios/TerraPhone/BluetoothLink.swift`: advertisement parsing, typed lifecycle events, bounded radio attempts.
- `mobile/ios/TerraPhone/PhoneController.swift`: lifecycle integration and router/manual-runtime bridge.
- `mobile/ios/TerraPhone/TerraDashboard.swift`, `ActuatorLayoutView.swift`, `ContentView.swift`: consent, status, settings and deliberate actions.
- Existing Python and Swift test locations: extend coverage; add `tests/ios/TerraConnectionLifecycleTests.swift` for coordinator tests.
- `crates/terra-mobile/src/autonomy.rs`, `lib.rs`: change only if targeted runtime tests reveal a required transport-service seam.

### Task 1: Advertise Enrollment Explicitly

**Files:** Modify onboarding.py, pairing.py, __main__.py; extend `hardware/raspberry-pi/tests/test_advertisement.py` and `test_pairing_integration.py`.

**Interfaces:** Produce `pairing_name(name: str) -> str` in onboarding.py. Input is an operational Terra name; output appends `-pair` exactly once. Both enrollment entry points use it for BlessTransport construction; normal-operation transport does not.

- [ ] Write failing `test_pairing_name` assertions: `pairing_name('terra-ABC123') == 'terra-ABC123-pair'` and repeated application remains unchanged. Extend integration fake transport to assert its enrollment name and normal-operation name differ exactly by the suffix.
- [ ] Run `python3 -m unittest discover -s hardware/raspberry-pi/tests -p 'test_advertisement.py'`; establish failure against the absent helper.
- [ ] Implement the helper and apply it to both enrollment transport constructors. Preserve enrollment deadlines, encrypted reads, and cleanup logic.
- [ ] Run advertisement and pairing integration suites; assert successful enrollment still persists the owner and cleans up the setup service.
- [ ] Commit only task changes: `feat: advertise rover enrollment mode`.

### Task 2: Deterministic Connection Policy

**Files:** Modify TerraAutoConnectionPolicy.swift and TerraAutoConnectionTests.swift; create TerraConnectionLifecycleTests.swift.

**Interfaces:** Define `TerraDiscoveredRover: Equatable` with `identifier: UUID`, `name: String`, `pairing: Bool`; `TerraConnectionPolicy` with `mutating func handle(_ event: Event, now: TimeInterval) -> [Effect]`. Nested events cover foreground(preferred: UUID?), background, inactive, discovered(TerraDiscoveredRover), consent(UUID, Bool), paired(UUID), authenticated(UUID), configurationAvailable(UUID), bluetoothFailed(retryable: Bool), routerResult(generation: UInt64, connected: Bool, retryable: Bool), settingsChanged(valid: Bool), automationEnabled(Bool), forget, feedbackChanged(Bool), stop, routerDisconnect, retry, and timer(generation: UInt64, channel: RetryChannel). Effects cover scan, stopScan, offer(TerraDiscoveredRover), connect(UUID), disconnect, remember(UUID), connectRouter(generation: UInt64), disconnectRouter, disarm, and schedule(generation: UInt64, channel: RetryChannel, delay: TimeInterval); RetryChannel distinguishes bluetooth, router, connectionDeadline, and enrollmentDeadline.

- [ ] Write failed assertions for unknown normal rover → no connect; preferred normal rover → exactly one connect; unknown pairing rover → exactly one offer. Replace the lone-unknown automatic-selection expectation with nil.
- [ ] Compile/run existing selection tests and new lifecycle tests separately using `swiftc mobile/ios/TerraPhone/TerraAutoConnectionPolicy.swift tests/ios/<test file> -o /tmp/terra-connection-tests`; confirm failures first.
- [ ] Implement policy: ordered consent queue, session refusal set, connection identity, generation invalidation, independent BLE/router retry counters and suppression, foreground enable state. Give pending BLE connection/authentication a 15-second deadline and enrollment-to-operational rediscovery a 30-second deadline; expiry reports failure and follows the specified retry/refusal behavior. These are concrete defaults for the spec's bounded attempts.
- [ ] Add assertions for delays `[1,2,4,8,15,15]`, stale timer no effects, background disconnect, inactive disarm without cancel, explicit stop suppression, new foreground resets refusal, two candidates offer serially, pairing remembers then scans for that identifier, layout-unready authentication remembers, and router failures do not disconnect BLE.
- [ ] Run both binaries; all preconditions pass. Commit: `feat: model automatic connection lifecycle`.

### Task 3: Connect the Policy to Bluetooth

**Files:** Modify BluetoothLink.swift and PhoneController.swift; extend lifecycle tests.

**Interfaces:** BluetoothLink produces typed discovery and lifecycle callbacks on the main queue, tagged with attempt generation. Add `startDiscovery()` and `connect(identifier: UUID, generation: UInt64)`; retain existing manual connect compatibility until Task 5. PhoneController owns `TerraConnectionPolicy`, dispatches events, and executes effects; published `pairingCandidate: TerraDiscoveredRover?` feeds the UI.

- [ ] Add failed lifecycle tests proving discovery cannot save preference, setup confirmation does save it, auth rejection suppresses retry, and transient disconnect schedules one retry. Test repeated advertisements with pairing then normal names for the same UUID.
- [ ] Run the lifecycle binary and confirm the new adapter-facing assertions expose missing event behavior.
- [ ] Parse pairing mode from the fresh advertised local name, never a stale peripheral.name; require the Terra service scan. Enable duplicate discovery while idle, publish only meaningful changes, and retain one active scan. Remove silent lone-unknown selection.
- [ ] Emit pairing confirmation before tearing down setup. Reconnect only after retiring peripheral teardown and a fresh operational advertisement. Persist preference on pairing/authentication events even with invalid layout. Attach generations to callbacks and clear timers on cancellation. Classify protocol/authentication failures as terminal and radio disconnects as retryable.
- [ ] Apply policy deadlines with cancellable queue work items; route outcomes through coordinator events. Keep BluetoothSession generation, synchronization, and motion checks intact.
- [ ] Run policy tests and an unsigned simulator build; inspect device compile path in Task 6. Commit only owned hunks: `feat: automate rover pairing and reconnection`.

### Task 4: Start and Maintain the Router in Manual Mode

**Files:** Modify PhoneController.swift; extend lifecycle tests; extend `crates/terra-mobile/tests/` dashboard tests using the existing router-test setup. Rust transport changes, if needed, belong in autonomy.rs/lib.rs.

**Interfaces:** Add `applyDashboardSettings(endpoint: String, prefix: String, roverID: String)` to PhoneController. Consume configurationAvailable and connectRouter effects from Task 2. Add `serviceManualDashboard(now: TimeInterval)` on controlQueue: service the existing MobileController dashboard, observe failures, discard its motion output, and never route it to actuators. It must not call setTarget or change local target; report missing tracking truthfully in telemetry.

- [ ] Write failed policy tests for blank endpoint → no attempt, invalid ID/endpoint → wait for settings change, settings generation replacement → old result ignored, and explicit router disconnect → no retries this session. Add a Rust router test connecting a fresh controller without IMU/VIO, stepping it, observing dashboard telemetry, then disconnecting and confirming cleared remote intent.
- [ ] Run targeted Swift tests and `cargo test -p terra-mobile --tests`; establish the missing behaviors before implementation.
- [ ] Ensure one MobileController exists when authenticated/configuration-ready BLE needs the router. Permit Bluetooth manual mode in the existing connectDashboard guard. Validate nonempty TCP endpoint and UInt64 ID before attempting; keep existing transport validation authoritative. Blank settings publish “Set router endpoint in Settings”.
- [ ] Service manual dashboard at 10 Hz with existing controller.step(timestamp:), discard ControlOutput, and check dashboardStatus. Manual actuator routing remains based solely on local target. A service failure disconnects only the dashboard and feeds the coordinator's retry path. Keep existing phone-mode telemetry/mission servicing.
- [ ] Serialize runtime transitions and transport teardown on controlQueue. Bind each attempt to foreground/settings/runtime generation; after a blocking open returns, close obsolete sessions before another attempt starts. Feedback mode changes invalidate pending opens without replacing a healthy active runtime unnecessarily. BLE loss clears dashboard remote intent.
- [ ] Run targeted Rust tests, Swift policy tests, and `scripts/check-dashboard-swift.sh` if its ARGOS dependency is available. Confirm telemetry never implicitly arms hardware. Commit owned hunks: `feat: connect saved router after rover authentication`.

### Task 5: Replace Manual Setup UI

**Files:** Modify TerraDashboard.swift, ActuatorLayoutView.swift, ContentView.swift, PhoneController.swift; update mobile/ios/README.md and docs/MOBILE_CONTROL.md.

**Interfaces:** PhoneController adds `respondToPairing(connect: Bool)`, `retryAutomaticConnection()`, `disconnectHardware()`, `forgetHardwareRover()`, and `setHardwareFeedback(enabled: Bool)`. Forget stops the session, clears preference, then explicitly starts discovery; ordinary disconnect suppresses automatic reconnect until foreground return or Retry.

- [ ] Add failing policy tests for forget/switch, automation disabled, and feedback toggling without reselection. Keep existing compatible-layout/tracking checks as the authority for enabling feedback.
- [ ] Run lifecycle tests to confirm missing actions.
- [ ] Present a single root alert “Connect to this rover?” with the candidate name and Connect / Not now; wait until splash dismissal. Map actions to respondToPairing. Foreground/background feed the coordinator; inactivity only zeros/disarms. Preserve simulator guards.
- [ ] Replace Find rovers/picker/Connect selected rover with status, Retry, Disconnect, and Forget rover actions. Keep the automation and phone-feedback toggles; feedback applies to the connected rover. Display Bluetooth and router status separately.
- [ ] Feed saved dashboard settings changes into applyDashboardSettings, keep manual router controls as deliberate retry/disconnect, and update obsolete setup/background copy in the UI and docs. Explain the one-time router endpoint configuration.
- [ ] Run lifecycle tests and simulator build. Inspect first-launch, denied Bluetooth, pairing consent, connected, missing endpoint, and retry states in the app where available. Commit owned hunks: `refactor: simplify rover connection interface`.

### Task 6: Verify the Complete Flow

**Files:** Update the above tests if integration reveals a missing case; document verification in this plan.

**Interfaces:** No new public interfaces; verify Tasks 1–5 against the approved specification.

- [ ] Run both Swift policy binaries and `python3 -m unittest discover -s hardware/raspberry-pi/tests`; require all targeted cases to pass, report unavailable dependencies accurately.
- [ ] Run `cargo test -p terra-mobile -p terra-transport`; run `scripts/check-swift.sh` and `scripts/check-dashboard-swift.sh` with built libraries and available ARGOS dependency. Do not regenerate unrelated artifacts unless changed interfaces require it.
- [ ] Run `xcodebuild -project mobile/ios/TerraPhone.xcodeproj -scheme TerraPhone -sdk iphonesimulator -destination 'generic/platform=iOS Simulator' -derivedDataPath /tmp/terra-auto-simulator CODE_SIGNING_ALLOWED=NO build`; require BUILD SUCCEEDED.
- [ ] Run the equivalent unsigned device build using `-sdk iphoneos -destination 'generic/platform=iOS'` with `/tmp/terra-auto-device` to compile physical-Bluetooth branches. If bundled libraries/toolchains are missing, record the limitation rather than claim verification.
- [ ] Review the diff for preserved existing edits, zero/disarmed reconnection, teardown ordering, stale callbacks, and independent router retry; run `git diff --check`.
- [ ] When hardware is available, verify first pairing, same-UUID transition, cold reconnect, radio/router loss, declined prompt, two rovers, invalid layout repair, background return, and inactive pairing prompt. Record observed outputs remain disarmed. Otherwise explicitly mark physical acceptance unverified.
- [ ] Commit any integration corrections separately. Report changed behavior, passing checks, and remaining hardware limitations; do not deploy or flash firmware as part of this plan.
