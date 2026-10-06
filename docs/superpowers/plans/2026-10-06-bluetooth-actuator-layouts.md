# Terra Bluetooth Actuator Layouts Implementation Plan

## Execution status and overriding instruction

Tasks 1–8 are implemented and independently source-reviewed. Task 9 delivered
documentation only. The user override is **implement only; do not write or run
tests, builds, smoke/check scripts, binding generation, screenshots or hardware
checks**. It supersedes every conflicting step and the test-oriented commit title
below. No test/bench checkbox is marked complete; unchecked historical steps are
retained for a future separately authorized phase, not pending work authorized
in this execution. No compilation/runtime/timing/physical compatibility or issue
acceptance proof is claimed. See [evidence status](../../hardware/evidence/README.md).

Execution deviations: no new test files/fixtures were written as verification;
product presets remain editable examples. Task 9 omits `test_roundtrip.py` and all
acceptance execution, adding BENCH/evidence documentation instead. Numeric u32
request IDs plus staged request/revision bind commits; confirmed-open physical
gate is required for real configuration; vendor support targets version 1.14.0.
Provisioning uses encrypted read-only status and explicit restart into normal
service. Source reviews and fix commits are recorded in the SDD ledger/reports.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Configure user-defined Fusion HAT motor, ESC, and servo layouts from TerraPhone and drive them over BLE with independent rover safety.

**Architecture:** Portable Rust models route phone control output into actuator values and encode the protocol. Swift owns CoreBluetooth and the mobile UI; Python owns BlueZ, persistent configuration, hardware outputs, and watchdog enforcement. Keep Zenoh and existing Rust feedback control intact.

**Tech Stack:** Rust/serde/UniFFI, SwiftUI/CoreBluetooth, Python 3.11+/stdlib unittest/dbus-next, SunFounder fusion_hat, Linux BlueZ.

**Spec:** [Approved design](../specs/2026-10-06-bluetooth-actuator-layouts-design.md).

## Global Constraints

- Support up to 16 user-defined actuator IDs in 0–255; never assume a fixed motor count or left/right chassis.
- Motor effort [-1, 1]; bidirectional ESC effort [-1, 1]; unidirectional ESC effort [0, 1]; positional servo position [-1, 1].
- BLE commands at 20 Hz, status at 10 Hz, Pi motor tick every 10 ms, command expiry at age >=200 ms; request safe output within 210 ms while service and I/O are running.
- Drop phone samples older than 100 ms; do not refresh an old effort to maintain enable.
- Commands have session, layout revision, and increasing sequence; watchdog and disconnect disarm; reconnect requires explicit arm.
- Physical arming requires an independent hardware gate; mock mode uses a simulated gate.
- Configuration stage/commit requires disarmed state; save atomically; invalid configuration never becomes active.
- Servo and ESC channels use 50 Hz; ESC safe output is a continuous neutral/stop pulse.
- Require encrypted bonded owner access for command/configuration; pairing setup is explicit, time limited, and disarmed.
- TurboPi, legacy RV v2 support, camera streaming, and new arbitrary-geometry autonomy are excluded.
- iOS deployment target remains 17.0. Generated Swift/FFI artifacts stay ignored and are regenerated through existing build tooling.

## Review Focus

1. Partially failed multi-actuator hardware writes must fault and attempt safe output on every actuator, including channels already written (Task 4).
2. Power loss during configuration persistence must leave either the old or new complete valid layout, never a partially applied file (Task 5).
3. Old connection callbacks and delayed command fragments must not arm or move a new session (Tasks 2, 6, 7).
4. Library versions can change motor method names, pulse units, channel aliases, and timer ownership; unsupported capability combinations must block arming (Task 4).
5. Phone tracking loss or switching from autonomy to custom manual control must revoke previous output rather than leave a second producer active (Task 8).

## File boundaries

- `crates/terra-actuators/src/{lib,layout,routing,protocol}.rs`: portable layout, routing, wire types.
- `crates/terra-mobile/src/actuators.rs`: UniFFI JSON and byte interfaces, keeping platform types out of portable Rust.
- `tests/fixtures/actuators/{layouts,protocol,safety}.json`: shared contract examples.
- `hardware/raspberry-pi/terra_rover/{layout,protocol,safety,backend,configuration,ble,__main__}.py`: corresponding single-purpose Pi modules.
- `hardware/raspberry-pi/tests/test_*.py`: stdlib unit tests using fake time and fake hardware.
- `mobile/ios/TerraPhone/{BluetoothLink,BluetoothSession,ActuatorLayoutView}.swift`: BLE I/O, platform-independent connection policy, and configuration UI.
- Modify existing `PhoneController.swift` and `ContentView.swift` only at mode/command/lifecycle boundaries.
- `docs/hardware/{BLUETOOTH,FUSION_HAT,BENCH}.md`: shipping protocol, setup, and acceptance evidence instructions.

Tasks form one vertical feature with shared contracts, so retain one plan. Each task has an independently testable deliverable and a local commit. Begin execution in an isolated worktree created through the using-git-worktrees workflow; do not implement on the unrelated mission-autonomy branch.

### Task 1: Layout validation and phone routing

**Files:** Create `crates/terra-actuators/Cargo.toml`, `src/lib.rs`, `src/layout.rs`, `src/routing.rs`, `tests/layout_routing.rs`; modify root `Cargo.toml`; create `tests/fixtures/actuators/layouts.json`.

**Interfaces:** `Layout { schema_version: u8, revision: u32, actuators: Vec<Actuator> }`; `Actuator { id: u8, name: String, port: String, kind: OutputKind, inverted: bool, limits: CommandLimits, calibration: Calibration, route: Route, safe: SafeOutput }`. `validate_layout(&Layout, &Capabilities) -> Result<(), Vec<LayoutError>>`; `route_commands(&Layout, &RoutingInput) -> Result<Vec<ActuatorValue>, ActuatorError>`. `RoutingInput` carries left/right effort, normalized manual forward/turn, and servo positions keyed by ID. `Capabilities` carries usable ports, occupied resource sets, supported output kinds, and library/board identity. `supports_feedback(&Layout) -> bool` requires at least one left and one right propulsion route and no manual propulsion routes.

- [ ] Write `rejects_alias_collision` asserting M0 plus P10 fails; `routes_duplicate_left_outputs` asserting one left effort can drive two IDs; `positive_yaw_turns_left` asserting left effort < right effort for positive manual yaw. Test 17 entries, repeated IDs, NaN/infinity, unsupported pins, asymmetric ESC ranges, unidirectional negative effort, servo limits, and incompatible feedback layouts.
- [ ] Run `cargo test -p terra-actuators --test layout_routing`; confirm failure before implementing the interfaces.
- [ ] Implement serde layout types and validation. Manual routing uses `forward_coefficient * forward + turn_coefficient * turn`, constrained to finite coefficients in [-1, 1], then clamps to the entry's command limits. Apply inversion at the hardware mapping boundary once; Rust must not double-invert commands. Servo slider defaults to the configured safe normalized position. Store presets as editable JSON with schema version 1.
- [ ] Run the same command; all named cases pass. Fixtures include valid terra-mini, ESC, mixed servo layouts, and alias conflicts with expected errors.
- [ ] Commit the crate, workspace membership, and fixtures as `feat: define configurable actuator layouts`.

### Task 2: Shared wire protocol and fragments

**Files:** Create `crates/terra-actuators/src/protocol.rs`, `tests/protocol.rs`, `hardware/raspberry-pi/terra_rover/{__init__,protocol}.py`, `hardware/raspberry-pi/tests/test_protocol.py`, `tests/fixtures/actuators/protocol.json`.

**Interfaces:** Rust `encode_frame(&CommandFrame) -> Result<Vec<u8>, ActuatorError>` and `FragmentAssembler::push(&mut self, bytes: &[u8], now_ms: u64) -> Result<Option<Vec<u8>>, ActuatorError>`; Python `decode_frame(data: bytes) -> CommandFrame`, `FragmentAssembler.push(data: bytes, now: float) -> bytes | None`. Frame fields and little-endian 16-byte header follow the spec. Define JSON control envelopes with schema version, request ID, operation, and payload. Use one explicitly documented JSON representation in both languages, with unit-labelled calibration fields.

- [ ] Write `shared_frame_vectors` in both languages asserting identical bytes for each fixture; assert a 16-actuator drive is 96 bytes and wrong length, NaN, duplicate IDs, and unsupported version reject. Write `assembly_expires_at_100ms`, `old_session_rejected`, `duplicate_fragment_rejected`, and `stop_clears_drive_assembly` with exact boundary assertions. Include minimum ATT payload fragmentation and message ID wrap without reusing an active assembly ID.
- [ ] Run `cargo test -p terra-actuators --test protocol` and `python3 -m unittest discover -s hardware/raspberry-pi/tests -p test_protocol.py`; confirm failures before implementation.
- [ ] Implement encoders, decoders, and fragmentation. Fragment header is u16 message ID/u8 index/u8 count. Logical limit is 96 bytes for commands and 16 KiB for configuration/replies. Count is at most 255; fail clearly if negotiated chunk size cannot encode a requested JSON document in 255 fragments. Never silently truncate. Stop uses a separately assembled priority control path so it cannot wait behind a drive assembly. Session/revision/sequence validation occurs after full assembly and before acceptance.
- [ ] Run both commands; compare every accepted/rejected fixture. Incomplete or rejected data never produces an accepted-command event.
- [ ] Commit as `feat: share BLE actuator protocol fixtures`.

### Task 3: Python safety controller

**Files:** Create `hardware/raspberry-pi/terra_rover/{layout,safety}.py`, `tests/test_safety.py`, `tests/test_layout.py`, `tests/fixtures/actuators/safety.json`.

**Interfaces:** Python `validate_layout(layout: dict, capabilities: dict) -> list[dict]` mirrors Task 1 fixtures. `SafetyController.configure(layout: dict, now: float)`, `.connect(session: int, now: float)`, `.accept(frame: CommandFrame, received_at: float, now: float) -> bool`, `.tick(now: float, hardware_gate: bool) -> OutputBatch`, `.disconnect(now: float)`, `.reset_fault(now: float)`. `OutputBatch` contains commands for every actuator plus `armed`, `arming`, and latched fault/stop reason. `.status(now: float) -> dict` supplies protocol status fields. Constructor starts disconnected, unconfigured, and disarmed.

- [ ] Write `expires_exactly_at_200ms`: command accepted at 1.0; active at 1.199; disarmed with safe outputs at 1.200. Assert traffic from layout writes does not renew it. Test gate opening, boot, invalid active frame, delayed mailbox stamp, backward clock, ESC neutral arming interval, stop latch/reset, sequence replay, sequence exhaustion, reconnect and stale previous session.
- [ ] Run `python3 -m unittest discover -s hardware/raspberry-pi/tests -p 'test_safety.py'`; confirm failure.
- [ ] Implement the state machine using injected monotonic time. New sessions reset sequence and command mailbox. Arm requires subscribed status, gate closed, valid revision, no fault/stop, and a full safe frame received while disarmed; that frame cannot actuate. Require subsequent frames after arming completion. Fault and stop reset must not arm. Persist only layout, never armed/session/sequence state.
- [ ] Run Python discovery for layout and safety; run `cargo test -p terra-motors` to compare existing safety semantics with deliberate stronger rearming requirements. Add shared boundary fixtures consumed by both languages where semantics match.
- [ ] Commit as `feat: enforce Pi actuator watchdog and arming`.

### Task 4: Fusion HAT and mock adapters

**Files:** Create `hardware/raspberry-pi/terra_rover/backend.py`, `tests/test_backend.py`, `docs/hardware/FUSION_HAT.md`, `hardware/raspberry-pi/pyproject.toml`.

**Interfaces:** `Backend.capabilities() -> dict`, `.configure(layout: dict) -> None`, `.apply(batch: OutputBatch) -> None`, `.read_gate() -> bool`, `.close() -> None`; implementations `MockBackend` and `FusionHatBackend`. Dependency injection takes motor/PWM factories and gate reader; importing backend on macOS must not import fusion_hat. `BackendError` reports port and operation. Mock records actual applied output and permits controllable gate and injected failures.

- [ ] Write `motor_effort_maps_to_percent` (0.5 -> 50); `esc_zero_keeps_neutral_pulse` (0 -> calibrated 1500 us, continuously enabled); `servo_maps_center_and_safe`; `mixed_port_aliases_reject`; `partial_write_failure_safes_every_channel`; `missing_gate_blocks_arm`; `unsupported_library_blocks_arm`. Assert inversion happens once and unidirectional zero maps to stop, never pulse disable.
- [ ] Run `python3 -m unittest discover -s hardware/raspberry-pi/tests -p test_backend.py`; confirm failure.
- [ ] Implement supported library adaptation with explicit version/capability checks and resource mapping. Validate real pulse units against installed source, not ambiguous docs. Validate frequency sharing before opening outputs. A motor tick error latches controller fault and attempts safe outputs on each actuator independently even if one safe write fails. Continue safe attempts each tick. Missing or unreadable physical gate is closed-to-motion (treated open), never simulated outside mock mode.
- [ ] Run backend tests and full Python discovery without Fusion HAT installed. Document supported versions, physical gate wiring interface, exposed ports, shared timers, and pulse verification procedure; leave hardware version/results explicitly unverified until tested.
- [ ] Commit as `feat: add configurable Fusion HAT output backend`.

### Task 5: Configuration transactions and status

**Files:** Create `hardware/raspberry-pi/terra_rover/configuration.py`, `tests/test_configuration.py`; extend Python layout and safety modules and shared layouts fixtures.

**Interfaces:** `ConfigurationStore.read() -> dict | None`, `.stage(request_id: str, layout: dict, expected_revision: int) -> dict`, `.commit(request_id: str, now: float) -> dict`, `.handle(envelope: dict, now: float) -> dict`. Store receives safety controller/backend; typed replies include request ID, active revision, errors. Stage token is request ID; commit references that token. Single staged layout per owner connection, cleared on disconnect. Duplicate committed request IDs return cached result for that connection; expected revision prevents replay across restarts.

- [ ] Write `armed_commit_rejected`, `stale_revision_rejected`, `duplicate_commit_does_not_increment_revision`, `persist_failure_preserves_old_layout`, `backend_failure_rollback_faults`, `restart_loads_complete_layout`, and `configuration_does_not_kick_watchdog`. Fault injection covers write, fsync, atomic replace, and directory fsync failures. Old/new complete layout are the only permitted files after recovery.
- [ ] Run `python3 -m unittest discover -s hardware/raspberry-pi/tests -p test_configuration.py`; confirm failure.
- [ ] Implement strict JSON parsing rejecting nonfinite numbers, unknown required schema versions, missing fields, and excessive sizes. Revalidate at commit; apply safe backend initialization and atomically save via same-directory temp file/fsync/replace. On uncertain persistence results reread and reconcile before reporting an active revision. Hardware rollback failure stays disarmed and faulted. Return nullable battery with `unsupported` reason, actual service state, command age, and accepted sequence.
- [ ] Run tests and full Python discovery; compare persisted layout round-trip with Rust fixtures. No command routes or revisions disappear on reload.
- [ ] Commit as `feat: transact rover configuration and publish status`.

### Task 6: BlueZ peripheral and standalone BLE mock

**Files:** Create `hardware/raspberry-pi/terra_rover/{ble,__main__}.py`, `tests/test_ble.py`, `packaging/terra-rover.service`, `docs/hardware/BLUETOOTH.md`, `scripts/check-rover.sh`.

**Interfaces:** `BlePeripheral.run(backend: Backend, store_path: Path, owner_store: Path)`, `.handle_write(peer: str, characteristic: str, value: bytes, now: float)`, `.disconnect(peer: str, now: float)`. CLI `python3 -m terra_rover --mock --name 'Terra Rover' --config PATH`; `--setup-owner --setup-seconds 60` is a separate disarmed provisioning mode. Runtime stores authenticated owner identity locally with restricted permissions. Normal mode disables new pairing.

- [ ] Write adapter-level tests using fake D-Bus: UUID/property registration matches spec; unencrypted/unbonded/non-owner writes reject; a second peer cannot take control; setup expires at 60 s; disconnect disarms; old-generation fragments reject; full command receives an acceptance status; writes/status callbacks cannot starve the motor tick.
- [ ] Run `python3 -m unittest discover -s hardware/raspberry-pi/tests -p test_ble.py`; confirm failure.
- [ ] Implement encrypted GATT characteristics and owner checks using BlueZ device identity and properties. Run safety/backend ticks in a dedicated worker with monotonic receive timestamps and a bounded latest-frame mailbox, separate from D-Bus transactions. Marshal status back to the event loop. A disarm/stop event preempts queued motion. Register service before advertisement, unregister on shutdown, safe outputs on exceptions/termination. Document install, provisioning, persistent storage, service ownership, and mock operation without Bevy.
- [ ] Run `scripts/check-rover.sh` (all Python tests and syntax checks). On Linux BlueZ, run mock peripheral and a test central to verify secure registration and read/write/notify. Report Linux radio checks blocked if no host is available; do not substitute fake D-Bus tests as physical radio evidence.
- [ ] Commit as `feat: serve Terra actuator BLE peripheral`.

### Task 7: UniFFI and Swift BLE connection policy

**Files:** Create `crates/terra-mobile/src/actuators.rs`, `mobile/ios/TerraPhone/{BluetoothSession,BluetoothLink}.swift`, `scripts/BluetoothSmoke.swift`, `scripts/check-bluetooth-swift.sh`; modify mobile Rust Cargo/lib files, `scripts/SwiftSmoke.swift`, `scripts/create-ios-project.rb`, Xcode project, and `Info.plist`.

**Interfaces:** Export Rust `actuator_validate_layout(layout_json: String, capabilities_json: String) -> Result<String, MobileActuatorError>`, `actuator_route(layout_json: String, input_json: String) -> Result<String, MobileActuatorError>`, `actuator_encode_frame(frame_json: String) -> Result<Vec<u8>, MobileActuatorError>`, `actuator_fragment(message_id: u16, payload: Vec<u8>, maximum_write_length: u32) -> Result<Vec<Vec<u8>>, MobileActuatorError>`. `BluetoothSession` is Foundation-only, owns generation/session/revision/sequence, and returns explicit send/disarm/state effects. `BluetoothLink` wraps CoreBluetooth on a serial queue, publishes main-thread status, and provides `scan()`, `connect(identifier: UUID)`, `send(frame: Data, producedAt: TimeInterval)`, `request(envelope: Data)`, `disconnect()`.

- [ ] Write Rust binding round-trips; Swift `old_callback_cannot_rearm`, `100ms_sample_is_dropped`, `one_outstanding_write_replaces_pending_motion`, `missing_status_blocks_arm`, `reconnect_resets_targets`, and `fragmented_configuration_reply_reassembles`. Assert drive write response does not set applied-command status. Start with failing smoke assertions.
- [ ] Run `cargo test -p terra-mobile` and `scripts/check-bluetooth-swift.sh`; confirm relevant new tests fail before implementation.
- [ ] Implement wrapper and state policy. Scan by service, show names plus stable peripheral identifier, select explicitly. Discover characteristics, subscribe replies/status, synchronize capabilities/layout, then allow arm. Negotiate write size, fragment with response, no motion retry after error. Prioritize disarm over pending motion; stalled write/status disarms and requires recovery. Add `NSBluetoothAlwaysUsageDescription`, CoreBluetooth framework, and all new source references to both project generator and checked-in project. Avoid requiring ARKit/CoreBluetooth in the Foundation-only smoke test.
- [ ] Regenerate bindings via existing build flow, run Cargo and both Swift smoke scripts. Verify direct project and generator describe the same sources/frameworks. A real iPhone pairing check remains in Task 9.
- [ ] Commit as `feat: connect TerraPhone to actuator BLE service`.

### Task 8: Mobile layout editor and actuator commands

**Files:** Create `mobile/ios/TerraPhone/ActuatorLayoutView.swift`; modify `PhoneController.swift`, `ContentView.swift`, Bluetooth session tests, generator/project references, `docs/MOBILE_CONTROL.md`.

**Interfaces:** PhoneController exposes discovered rover list, BLE status, committed layout, capabilities, arm/arming/fault state, and validation errors. Add `startBluetooth(identifier: UUID, feedback: Bool)`, `stageActuatorLayout(json: String)`, `commitActuatorLayout()`, `armHardware()`, `disarmHardware()`, `setServoTarget(id: UInt8, position: Double)`. Hardware mode has one command producer on the existing control queue; BluetoothLink owns only transmission and never computes replacement effort.

- [ ] Write smoke/policy tests `tracking_loss_revokes_hardware_motion`, `mode_switch_has_one_producer`, `custom_layout_blocks_feedback_autonomy`, `arm_does_not_reuse_slider_target`, `layout_edit_requires_disarmed`, and `commit_waits_for_rover_reply`. Verify positional servo changes do not change propulsion and emergency-stop reset does not arm.
- [ ] Run Swift policy tests and confirm failures.
- [ ] Implement discovery, capability-driven form, editable presets, per-actuator routing/calibration/inversion/limits/safe policy, stage/commit results, and explicit arm/disarm. Preserve draft until validated commit acknowledgement; display active revision separately. Manual effort mode labels units as normalized effort; positional servo controls label position. Feedback mode uses fresh Rust left/right output only when layout is compatible and phone sensing healthy. Dispatch 20 Hz command snapshots with production time; safe/disarm on background, tracking loss, stop, errors, and mode switch. Disable unsupported autonomy choices in custom manual mode. Keep existing Zenoh screens and lifecycle intact.
- [ ] Run all Swift scripts, `cargo test --workspace`, regenerate bindings, and build with `xcodebuild -project mobile/ios/TerraPhone.xcodeproj -scheme TerraPhone -sdk iphonesimulator -configuration Debug CODE_SIGNING_ALLOWED=NO build`. Record failures caused by unavailable tooling separately from code failures.
- [ ] Commit as `feat: configure and drive actuator layouts from TerraPhone`.

### Task 9: End-to-end acceptance and evidence

**Files:** Create `docs/hardware/BENCH.md`, `docs/hardware/evidence/README.md`, `hardware/raspberry-pi/tests/test_roundtrip.py`; update root README and hardware/mobile docs.

**Interfaces:** A fake central drives the same dispatch/framing/configuration paths as real BLE; MockBackend records each safe and commanded output. Evidence distinguishes portable contract tests, mocked transport, Linux radio round-trip, and physical Fusion HAT tests.

- [ ] Write `mixed_layout_roundtrip_without_bevy`: read capabilities; stage/commit motor+ESC+servo layout; safe frame; arm; wait configured ESC interval; send fresh complete commands; observe accepted sequence/output; drop commands; at 200 ms observe disarmed safe motor/ESC/servo outputs; reconnect cannot replay old frame. Assert request IDs and active revision throughout.
- [ ] Run full portable verification: `cargo test --workspace`, `scripts/check-rover.sh`, `scripts/check-swift.sh`, `scripts/check-bluetooth-swift.sh`, `git diff --check`, and iOS build. All must pass or have a clearly recorded environment blocker before claiming completion.
- [ ] Run TerraPhone against Linux BLE mock without Bevy: provisioning, named discovery, configuration edit, motor command/status, timeout, phone background, stop, and reconnect. Record actual device/OS/tool versions and status sequences; leave unchecked if unavailable.
- [ ] Run raised-wheel Fusion HAT bench with terra-mini and ESC/servo template: inspect library pulse units/resources; measure neutral/full configured pulses; verify inversion, cutoff, timeout, partial-backend failure handling, and no startup/reconnect motion. Record actual results; do not assume the mock proves hardware behavior.
- [ ] Update docs with achieved evidence and remaining physical gaps; perform whole-branch review focused on safety, owner security, resource conflicts, timestamp freshness, and Zenoh regression. Fix confirmed defects and rerun only affected checks.
- [ ] Commit as `test: verify Bluetooth actuator round trips`.

## Execution handoff

Recommend subagent-driven execution because the cross-language protocol, Pi safety, and mobile lifecycle each benefit from independent review before integration. Native execution is available if the user prefers one implementer. Plan approval and execution-method selection precede implementation. Hardware absence must not prevent completing portable code/tests and the documented mock, but actual BLE/hardware acceptance remains explicitly unverified until exercised.
