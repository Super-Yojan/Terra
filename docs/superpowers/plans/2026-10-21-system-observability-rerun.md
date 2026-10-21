# System-wide observability and Rerun implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task if native execution is selected, or superpowers:subagent-driven-development if that method is selected. Steps use checkbox syntax for tracking.

**Goal:** Preserve normal operation and failure evidence across the robotics system and replay it with synchronized camera, depth, ground reference and motion in Rerun.

**Architecture:** A new independent observability library supplies bounded durable journals. Background HTTPS delivery sends events to an independent collector on the GCP host. A Python exporter produces Rerun recordings from retained source evidence; control code never waits for network, disk or conversion.

**Tech stack:** Rust/Serde/SQLite for journals and collector; Swift and UniFFI for phone integration; Python/SQLite for the Pi forwarder; HTTPS behind existing Caddy; Python Rerun SDK for replay. Select compatible dependency versions from the available toolchain, pin them in lockfiles, and record the tested exporter/viewer version.

**Spec:** `docs/superpowers/specs/2026-10-21-system-observability-rerun-design.md`.

**Status:** Implemented locally and independently reviewed; all eight review findings fixed with regression coverage. Final full suites: Terra262, ARGOS150, Zorvane113 passed. Phone simulator build, scoped collector/reconnect/Rerun export integration and Caddy validation passed. Deployment and physical acceptance remain separate operator-controlled gates. See `docs/observability/operations.md` and the implementation record for deviations and limits.

## Global constraints

- Observe all defined lifecycle, command, connection, sensor, mapping, planning and recording event kinds; maintain a coverage inventory. Do not claim all raw samples or arbitrary OS logs.
- Preserve existing timeout, control, arming, routing and experiment-recorder policies. Diagnostic replay must never execute commands.
- Phone/desktop/simulator journal: 256 MiB; Pi: 512 MiB; local retention: seven days. Collector event quota: 10 GiB, retention: 30 days. All configurable.
- Local attachment quota: 512 MiB; collector attachment quota: 5 GiB. Report every retention gap.
- Sample control summaries at 10 Hz, maps at 2 Hz, synchronized camera/depth/cloud evidence at 1 Hz. Keep ten seconds of sampled evidence before and after a hold/fault. Sampling is not a control rate.
- Original exposure/source times, producer sequence and clock uncertainty survive upload. Missing evidence is explicit.
- No cloud credentials in source or recordings; separate ingestion and reader access. Preserve existing Zenoh TLS deployment.
- Keep tests in `tests/` directories, outside implementation source. Do not stage unrelated dirty files from ongoing timing/refactoring work.

## Review focus

1. Disk stalls/full disks must not stall control or masquerade as sensor faults — journal fault injection in task 1 and control replay in task 3.
2. A lost HTTP response after commit must not duplicate events — collector/retry tests in tasks 2 and 6.
3. Phone rover-selection changes and process restart must not move queued events to another rover — tasks 3 and 6.
4. Clock skew, frame resets and late evidence must not create false causal order or fabricated image alignment — tasks 5 and 7.
5. Quotas, malicious oversized attachments and scoped credentials must not starve live control or leak other rovers' evidence — tasks 2, 5 and 8.

## File structure and delivery boundaries

New Rust crates in Terra: `terra-observability` (schema/journal/uploader) and `terra-observability-collector` (ingestion/history/export source). New `tools/observability/` owns the Rerun exporter and its tests. Phone adapters live alongside the existing iOS controller. The Pi forwarder lives in `hardware/raspberry-pi/terra_rover/observability.py`. ARGOS and Zorvane depend on the shared library without duplicating schemas. GCP service packaging stays with ARGOS's existing GCP deployment scripts.

Each numbered task yields an independently testable component. Review each component before connecting it to the next; deployment is last.

### Task 1: Durable event schema and nonblocking journal

**Files:** Create `crates/terra-observability/{Cargo.toml,src/lib.rs,src/event.rs,src/journal.rs,src/config.rs,tests/journal.rs}` and `docs/observability/event-schema.md`; modify root `Cargo.toml` and lockfile.

**Interfaces:** `EventInput { rover_id, session_id, component, kind, severity, wall_time_ns, monotonic_ns, cause_id, attributes }`; `EventId { producer_id, boot_id, sequence }`; `EventEnvelope { schema_version, id, input }`. New journal instances create unique boot IDs; sequence allocation is per journal. `Journal::open(config: JournalConfig) -> Result<Journal, JournalError>`, `Journal::try_record(event: EventInput) -> RecordOutcome`, `Journal::health() -> JournalHealth`, `JournalReader::batch(limit: usize) -> Result<Vec<EventEnvelope>, JournalError>`, `JournalReader::ack(ids: &[EventId]) -> Result<(), JournalError>`.

- [x] Write `records_survive_reopen`, `disk_stall_does_not_block_try_record`, `overflow_reports_gap`, `ack_eviction_precedes_pending_eviction`, and `new_boot_keeps_old_identity` in `tests/journal.rs`. Assert original payload/time survives reopen, overflow increments a visible counter, and a blocked worker does not block the caller.
- [x] Run `cargo test -p terra-observability --test journal`; verify failure from missing interfaces.
- [x] Implement Serde envelopes and a bounded worker queue, SQLite WAL background persistence, indexed acknowledgments and retention. Reserve gap records independently of ordinary queue capacity; attach overflow counts to health snapshots when disk is unavailable. Fsync/transaction durability belongs to the worker; admission to memory is not described as durable storage.
- [x] Run the journal suite and `cargo clippy -p terra-observability --lib --no-deps -- -D warnings`; require passing new-code checks.
- [x] Commit only this task's files with `feat: add durable observability journal`.

### Task 2: Independent collector and authenticated event ingestion

**Files:** Create `crates/terra-observability-collector/{Cargo.toml,src/main.rs,src/api.rs,src/store.rs,src/auth.rs,tests/ingestion.rs}` and modify root workspace membership.

**Interfaces:** `POST /v1/events` accepts `{events:[EventEnvelope]}` and returns `{acknowledged:[EventId]}` only after transaction commit. Duplicate identical IDs acknowledge existing rows; conflicts return a rejection. `GET /v1/sessions` lists scoped sessions; `GET /v1/events?session=...&cursor=...` provides stable paginated history. Source scope comes from credential configuration, never the body alone.

- [x] Write tests `lost_response_retry_is_idempotent`, `conflicting_id_rejected`, `scope_cannot_be_overridden`, `uncommitted_batch_not_acknowledged` and `expired_records_leave_gap`.
- [x] Run `cargo test -p terra-observability-collector --test ingestion`; verify expected initial failure.
- [x] Implement SQLite transactions, scoped ingestion/read tokens, redaction checks and explicit configurable limits. Initial caps: 256 events/batch, 1 MiB event-body request, 200 events/s per source, 1,000/s globally. Store hash of canonical validated payload for conflict detection. Never log auth headers or rejected payload bodies.
- [x] Run ingestion tests, including simulated failure before/after transaction commit, with deterministic clocks.
- [x] Commit task files with `feat: add authenticated observability collector`.

### Task 3: Phone and shared-runtime event coverage

**Files:** Create `crates/terra-mobile/src/observability.rs`, `mobile/ios/TerraPhone/TerraObservation.swift`, `tests/ios/ObservationTests.swift`, `crates/terra-mobile/tests/observability.rs`, and `docs/observability/coverage.md`. Modify mobile manifest/bindings, `PhoneController.swift`, `BluetoothLink.swift`, `TerraLog.swift`, Xcode project and targeted autonomy/transport event sources.

**Interfaces:** UniFFI methods `start_observability(config_json: String) -> Result<(), ControllerError>`, `record_observation(event_json: String) -> bool`, `observability_health() -> String`; typed runtime event sink remains optional and defaults to no-op. Swift wrapper `TerraObservation.record(component: String, kind: String, attributes: [String: Any])` snapshots rover/session identity at creation, not upload.

- [x] Test `stop_retains_first_cause`, `rover_switch_keeps_buffer_identity`, `logging_failure_does_not_change_control`, and normal lifecycle coverage. Replay identical inputs with logging disabled, healthy and failed; assert identical control output and safety state.
- [x] Run relevant Swift tests and `cargo test -p terra-mobile observability`; verify expected missing-interface failures first.
- [x] Add independent journal setup and adapter, retaining OSLog. Instrument exact disarm/pause callers, Bluetooth readiness, app lifecycle, router/session events, command admission/expiry, authority changes, actual freshness ages/limits and planner rejection transitions. Add 10 Hz summaries and an event-to-call-site inventory. Do not replace the experiment recorder or its failure policy.
- [x] Run focused tests, existing mobile/autonomy regressions, `./scripts/build-ios.sh`, and Xcode TerraPhone build. Report physical behavior as untested until field acceptance.
- [x] Commit only event instrumentation and generated binding changes with `feat: record phone and autonomy event history`.

### Task 4: Pi, ARGOS, simulator and router producers

**Files:** Create `hardware/raspberry-pi/terra_rover/observability.py`, `tests/rover/test_observability.py`; modify `safety.py`, `lifecycle.py`, `ble.py`. Create ARGOS `crates/argos-zenoh/src/observability.rs` and `tests/argos-zenoh/observability.rs`; instrument Zenoh/GPUI command and connection paths. Create Zorvane `simulator/src/observability.rs`; instrument simulator lifecycle/control. Create collector `src/journal_source.rs` and tests for allowlisted system journal conversion.

**Interfaces:** Python `ObservationJournal.record(event: dict) -> bool` produces the same envelope, backed by a bounded worker/SQLite queue. Router journal source maps journal cursor to stable event identity. All producers consume task 1 schema and task 2 ingestion contract. Source-specific boot/sequence order is retained.

- [x] Write tests for one actual arm/disarm transition producing one causal event, independent Pi backlog survival while phone is absent, router restart journal replay deduplication, and sent-versus-received command correlation.
- [x] Run the new Python/Rust test targets and observe expected failures.
- [x] Implement producers and coverage entries. Forward only rover/Bluetooth service units on Pi and router/TLS service units on GCP. Collect no unrelated journal units. Observer process gets no actuator privileges. Keep command traffic and telemetry paths operational without the observer.
- [x] Run Pi tests, ARGOS affected tests and Zorvane build. Confirm integration fixtures cover absent producers explicitly.
- [x] Commit each repository's task files separately with matching descriptive messages.

### Task 5: Camera/depth and ground-reference evidence

**Files:** Create shared `src/evidence.rs` and `tests/evidence.rs`; phone `PhoneObservationCapture.swift` and `tests/ios/ObservationCaptureTests.swift`; modify phone map integration at its existing exposure boundary. Create Zorvane `simulator/src/observation_capture.rs`; modify existing sensor/occupancy integration.

**Interfaces:** `EvidenceManifest { id, event_id, exposure_ns, frame_id, attachments, rgb_intrinsics, depth_intrinsics, camera_to_world, camera_to_body, body_pose, ground_plane, clearance, world_offset, thresholds, map_revision }`; `Attachment { sha256, encoding, byte_length, dimensions, units }`. `EvidenceWriter::try_capture(bundle: EvidenceBundle) -> CaptureOutcome` copies bounded owned buffers for background encoding; never retains an unlocked borrowed image buffer.

- [x] Write fixtures asserting actual camera transform, principal point and ground plane are preserved; mismatched RGB exposure is marked missing; localization reset breaks the frame association; quota loss produces a gap event.
- [x] Run new evidence tests and Swift capture tests; verify expected initial failures.
- [x] Implement 1 Hz capture, ten-second pre/post-trigger sampled evidence window, content-addressed attachments, explicit confidence/validity masks, and separate quotas. Capture mapper inputs and actual configured thresholds. Use existing rendered simulator sensor output; mark RGB unavailable if no corresponding image exists. Do not add a fictitious matching image.
- [x] Run flat-ground, ramp, pitch/roll and changed-height fixtures; assert classifications against mapper results and compare coordinates numerically.
- [x] Commit with `feat: capture synchronized mapping evidence`.

### Task 6: Reconnect uploads and attachment delivery

**Files:** Create shared `src/uploader.rs`, `tests/uploader.rs`; collector `src/attachments.rs`, `tests/attachments.rs`; Pi upload worker in its observability module.

**Interfaces:** `Uploader::start(journal: JournalReader, config: UploadConfig) -> UploadHandle`; `UploadHandle::health() -> UploadHealth`. `PUT /v1/attachments/{sha256}` verifies scoped ownership, hash and bounded streamed content; immutable identical retries succeed. `GET` requires reader scope. Event attachment references can be pending, available or evicted, never silently assumed complete.

- [x] Test no-network/reconnect, auth denial, interrupted upload, lost acknowledgment, hash mismatch, cross-rover access, process restart and source-selection changes. Assert an old event retains its original timestamp and command admission is never invoked.
- [x] Run uploader/attachment tests and observe expected failures.
- [x] Implement background HTTPS retries with bounded exponential backoff and jitter, cancellation, persistent acknowledgment, and source health. Cap attachments at 32 MiB each and batches per task 2; reject oversized input without unbounded allocation. HTTP credentials are injected from secure configuration, not committed.
- [x] Run deterministic fault-server tests; verify every accepted event appears once and any evicted evidence has a visible gap.
- [x] Commit with `feat: synchronize observability records after reconnect`.

### Task 7: Rerun exporter and timeline layout

**Files:** Create `tools/observability/{pyproject.toml,README.md,terra_replay/__init__.py,terra_replay/export.py,terra_replay/layout.py,tests/test_export.py,tests/fixtures/}` and collector export-job support.

**Interfaces:** `export_session(source_dir: Path, output_rrd: Path, config: ExportConfig) -> ExportResult`; local CLI `terra-replay export INPUT --output OUTPUT`. Collector `POST /v1/exports` accepts scoped rover/session/time range; `GET /v1/exports/{job}` returns status and authenticated artifact reference. Run conversion outside request handlers with bounded concurrency (one job initially).

- [x] Write tests preserving source/receipt time, uncertainty, sequence and reset boundaries; verify late-event revision and missing-image handling. Use actual pinned Rerun SDK to save and read a small recording, not only mocked log calls.
- [x] Run `python -m pytest tools/observability/tests`; verify initial failures.
- [x] Implement event text and control curves, connection/sensor states, camera/depth views, pose/frustum, ground plane/height band, occupancy, route and classified points. Entity paths distinguish source, frame and rover. Journal remains authoritative. Include JSONL, manifest, gap summary and event index alongside the RRD. Pin tested SDK/viewer versions.
- [x] Run exporter tests and open a generated fixture in Rerun. Inspect image orientation, alignment, unknown map cells and an event before/after a disconnect. Record visual verification separately from numerical checks.
- [x] Commit with `feat: export system timelines to Rerun`.

### Task 8: Local end-to-end acceptance and GCP packaging

**Files:** Create `tests/observability/test_system.py`, `docs/observability/operations.md`; ARGOS `packaging/gcp/terra-observability.service`, `tests/deployment/test_observability.py`; modify GCP install/Caddy configuration only to add the collector route and service.

**Interfaces:** Authenticated session list/export from tasks 2/7; explicit credential-file/service configuration. Collector binds loopback behind existing HTTPS. Read-only export/open instructions do not require a drive-control claim.

- [x] Write end-to-end tests simulating phone/Pi/dashboard producers, internet loss, reconnect, collector crash, quota pressure, clock skew and missing attachments. Assert diagnostic failures never change reference control output.
- [x] Run tests and confirm expected failure before packaging/integration exists.
- [x] Add isolated service user, data directories, scoped credential provisioning procedure, resource limits, backups, rollback and documented retention. Preserve router auth/listeners and TLS renewal behavior. Build local packages and validate Caddy/service configuration with deployment mocks.
- [x] Run all affected test suites once, rebuild TerraPhone and Zorvane, and generate/open one complete Rerun fixture. Inventory coverage and explicitly list uninstrumented sources before declaring completion.
- [ ] Deployment/physical gate: present the prepared release for deployment; after operator-authorized installation verify authenticated upload/read and a recording export. Physical acceptance is operator-led; do not cause motion, pairing reset or disarm remotely for a test.
- [x] Commit packaging/docs separately and report code, deployment and physical acceptance status distinctly.

## Review and execution

Native implementation in this chat is recommended because schema, timestamps and evidence identity must stay consistent across tasks. No subagents are required for implementation unless the operator selects that approach. A final independent review follows the selected execution workflow. Written-plan review precedes product-code implementation under the active planning skill.
