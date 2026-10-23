# Remote Rover Logs Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Display retained, live operational logs and the original disarm cause in the ARGOS web dashboard, including replay after network outages.

**Architecture:** ARGOS validates and stores events independently of command processing, then serves history and a dedicated log SSE stream. A compiled journal forwarder uploads Pi/server/router logs over HTTPS; TerraPhone buffers operational events and sends them through a separate acknowledged Zenoh diagnostic channel. The browser subscribes for the selected rover and fleet-global context.

**Tech Stack:** Rust/Axum/sqlx/SQLite/Zenoh 1.10.1; Python standard library and Nuitka ARM64 packaging; Swift/Foundation/OSLog; React/TypeScript/Vitest/Playwright.

**Spec:** [Approved design](../specs/2026-10-19-web-rover-logs-design.md).

## Global Constraints

- Observation must never arm, disarm, renew command freshness, replay a command, or change the current safety policy.
- Event schema version 1; 4 KiB per event; 32 events and 128 KiB per ingestion request; 2 KiB message; 64-character component/kind; 16 flat scalar context keys.
- Each collector keeps at most 5,000 events and 20 MiB on disk, with 20 percent of capacity reserved for warnings/errors.
- Send batches at most every 500 ms; retry after 1, 2, 4, 8, 15, then 30 seconds, capped at 30 seconds with jitter.
- Store at most seven days and 10,000 events per rover/global group, with an overall cap of 100,000 events; pages up to 200, export up to 10,000, rendered rows up to 1,000.
- Ingress limits: 200 events/second per credential and 1,000 accepted events/second globally.
- Viewer/operator sessions may read logs without taking control; write-only scoped tokens may ingest but cannot command/read fleet data.
- Persist events before acknowledgement; retain event IDs across retries; use receipt IDs for browser cursors and source timestamps for labeled source time.
- No new logging for unrelated applications, full sensor payloads, Bluetooth owner addresses, or credentials.
- Preserve existing uncommitted Terra ownership/rover-selection changes and do not include them in logging commits.
- Work in isolated checkouts per the worktree skill; acquire required filesystem permission before changing the sibling ARGOS checkout. Do not reset/stash the shared workspace or regenerate the whole Xcode project.
- This plan prepares deployable artifacts. Identify the hosted dashboard destination and configuration before production deployment; never trigger motion/ownership reset to test logging remotely.

## Review Focus

1. A pre-existing SQLite database and active sessions survive additive schema migration; Task 1 tests this.
2. JS-unsafe 64-bit IDs and source clock regression do not corrupt ordering/replay; Tasks 1 and 7 test decimal-string receipt IDs.
3. The journal rotates or a phone changes rovers while offline; Tasks 3 and 6 test explicit gaps and immutable event attribution.
4. A revoked credential or slow consumer floods retries; Tasks 2, 3, and 5 test bounded work, paused authentication failure, and acknowledgements only after commit.
5. Simultaneous log failure and a real disarm preserve the original cause without delaying safe outputs; Tasks 4 and 8 test this boundary.

## Baseline and integration procedure

At execution time, create isolated checkouts using the required worktree skill. In
Terra, seed the checkout with a snapshot of the existing tracked/untracked source
changes, excluding build caches and secrets, and record that snapshot as a local
baseline commit. Do not change the original checkout, stash it, or publish the
baseline commit. Logging commits are measured from that baseline. ARGOS starts
from its clean current revision. Record both baseline hashes for review. At final
integration, apply only baseline-to-feature changes; check the shared working tree
for concurrent edits and resolve conflicts without discarding them.

## File and interface map

Paths below start with `ARGOS/` or `Terra/` to identify their repositories.

- `ARGOS/crates/argos-server/src/logs/{mod.rs,contract.rs,store.rs,auth.rs,http.rs,bus.rs}`: event validation, persistent history, collector credentials, HTTP/SSE, and independent fleet-bus ingest.
- `ARGOS/crates/argos-server/src/schema.sql`, `server.rs`, `lib.rs`, `config.rs`: additive schema and lifecycle wiring only; log handlers live outside the main HTTP file.
- `ARGOS/crates/argos-server/src/bin/argos-log-token.rs`: administrative token provisioning/revocation.
- `Terra/hardware/raspberry-pi/terra_logs/{__init__.py,contract.py,queue.py,journal.py,uploader.py,__main__.py}`: portable event model and independent stdlib-based journal forwarder.
- `Terra/hardware/raspberry-pi/terra_rover/operational_log.py`, `safety.py`, `ble.py`, `lifecycle.py`, `pairing.py`, `configuration.py`: nonblocking transition/event production.
- `Terra/crates/terra-transport/src/diagnostics.rs`, `control_plane.rs`, `Terra/crates/terra-mobile/src/diagnostics.rs`: dedicated bounded diagnostic transport and UniFFI interface.
- `Terra/mobile/ios/TerraPhone/{OperationalEvent.swift,OperationalEventStore.swift,OperationalLogger.swift,TerraLog.swift,PhoneController.swift}`: phone capture, durable queue, and asynchronous delivery.
- `ARGOS/apps/web/src/{LogsPanel.tsx,useLogs.ts,logs.ts,LogsPanel.module.css}`: selected-rover/fleet history and live viewer.
- Packaging/docs in both repositories: disabled-until-configured collectors, credentials, built artifacts, rollback and verification instructions.

## Task 1: Add the event contract and persistent history

**Files:** Create `ARGOS/crates/argos-server/src/logs/{mod.rs,contract.rs,store.rs}` and `tests/log_store.rs`; modify `src/{lib.rs,schema.sql}` and `tests/types.rs`.

**Interfaces:** Produce `LogEvent`, `LogLevel`, `LogBatch { events: Vec<LogEvent> }`, `LogAck { event_ids: Vec<String> }`, `StoredLogEvent`, `LogQuery`, `LogPage`, and `LogStore::new(SqlitePool)`. IDs, sequences, and receipt IDs serialize as decimal strings where integer precision could exceed JS limits. `validate_batch(&LogBatch) -> Result<(), LogError>` validates the entire batch. `LogStore::{insert(batch, now), query(query), export(query), prune(now)}` are async; insertion returns ACK IDs only after transactional commit. `LogQuery` defines rover/global inclusion, source, minimum level, text, exclusive after/before receipt cursors, and limit; after/before cannot both be set. `LogPage` returns rows, earliest retained ID, and a gap flag.

- [ ] **Step 1: Write contract/storage tests.** Build literal valid/invalid fixtures, then assert:

```rust
assert!(validate_batch(&batch_with_33_events()).is_err());
assert!(validate_batch(&event_with_nested_context()).is_err());
assert_eq!(store.insert(batch.clone(), now).await?.event_ids.len(), 1);
assert_eq!(store.insert(batch, now).await?.event_ids.len(), 1);
assert_eq!(store.query(query).await?.rows.len(), 1); // duplicate not reinserted
```

Also test 4 KiB/128 KiB boundaries, all length/source/level/context constraints, rollback without ACK on failed commit, literal seven-day cutoff, group/global caps, gap reporting, case-insensitive substring search, and migration of a database already holding sessions/operation audit. Verify receipt `9007199254740993` survives JSON as a string and old source timestamps do not change receipt ordering.
- [ ] **Step 2: Run `cargo test -p argos-server --test log_store` in ARGOS.** Expect the new API to be missing; verify failure names the intended missing behavior.
- [ ] **Step 3: Implement contract, additive tables/indexes, transactional deduplication, query/export, and bounded pruning.** Use server receipt time for retention, text-safe bound SQL parameters, and separate log tables. No modifications to command/audit recovery.
- [ ] **Step 4: Rerun the new tests and `cargo test -p argos-server`.** Expect all tests passing; regenerate/check TypeScript declarations through `tests/types.rs` rather than hand-editing generated types.
- [ ] **Step 5: Commit only Task 1 files** as `feat: store bounded operational log history`.

## Task 2: Expose authenticated ingestion, history, SSE, and export

**Files:** Create `ARGOS/crates/argos-server/src/logs/{auth.rs,http.rs}` and `src/bin/argos-log-token.rs`; modify `src/{server.rs,config.rs}` and `logs/mod.rs`; create `tests/log_http.rs`, `tests/log_tokens.rs`.

**Interfaces:** Produce `LogService` holding `LogStore`, rate-budget state, and a wakeup channel; `LogService::ingest(scope: &LogScope, batch: LogBatch, now: i64) -> Result<LogAck, LogError>`. Scope binds credential to allowed rover, sources, source instance, and journal-unit allowlist. Token CLI supports `create --database ... --rover-id ... --source ... --output ...`, `list`, and `revoke --id ...`; global tokens use an explicit global scope. Raw tokens are written only to a new 0600 output file, never stdout/logs. HTTP routes and filters exactly match the specification. Request bodies are limited to 128 KiB only on ingestion; existing control-route 4 KiB limits stay unchanged.

- [ ] **Step 1: Write HTTP/token tests.** Assert unauthorized read is 401, wrong rover/source token is 403, revoked tokens fail immediately, invalid envelope is 400, oversized body is 413, and budget exhaustion is 429 without ACK. Assert ingestion token cannot call `/api/v1/fleet` or any mutation. A viewer session can query/export without a claim. Verify token output mode is 0600 and neither `list` nor error messages include raw secrets.
- [ ] **Step 2: Add streaming tests.** Persist events, resume SSE from the last receipt ID, and assert exactly the missing rows arrive. Cover cookie expiry/revocation, lagged receivers reloading SQLite, pruning gap markers, and a slow stream not blocking another client or fleet SSE. Check the 200-row history and 10,000-row export limits, and stable filtering across pages.
- [ ] **Step 3: Run `cargo test -p argos-server --test log_http --test log_tokens`.** Verify expected failures before handlers exist.
- [ ] **Step 4: Implement `LogService`, token CLI, scoped HTTP handlers and independent SSE.** Wakeups carry hints, not the sole copy of event payloads; SQLite/cursors recover missed wakeups. Apply global and per-token event budgets before commit and acknowledge already-committed IDs safely. HTTP logging excludes authorization and event bodies. Stream export as JSONL.
- [ ] **Step 5: Run new tests and the full ARGOS server suite; run token CLI against a temporary database.** Expect no changes to fleet claim behavior when a log stream closes.
- [ ] **Step 6: Commit Task 2 files** as `feat: expose authenticated operational log APIs`.

## Task 3: Build the independent journal collector and durable queue

**Files:** Create `Terra/hardware/raspberry-pi/terra_logs/*` from the file map and `log_forwarder.py`; modify package inclusion in `pyproject.toml`; create `tests/test_log_{queue,journal,uploader}.py`.

**Interfaces:** `JournalReader(unit_names, cursor, initial_since_seconds=86400, initial_limit=5000)` yields normalized events with deterministic UUIDs derived from source instance and journal cursor. `DiskQueue(path, max_events=5000, max_bytes=20*1024*1024)` exposes `append(event, cursor=None)`, `batch(limit=32, max_bytes=128*1024)`, `ack(event_ids)`, and `state()` including pending loss counts/cursor. `Uploader(config, queue)` owns HTTPS validation and `run_once(now)`; retry timing/HTTP are injected for deterministic tests. CLI validates configuration before following journals and never imports actuator modules.

- [ ] **Step 1: Write queue/journal tests.** Append, close/reopen, and assert stable IDs/order/cursor; crash replay must not invent new IDs. Fill warning/info fixtures and assert count/byte caps, 20-percent high-priority reserve, oldest lower-priority eviction, and a persisted counted loss event. Test journal rotation/cursor invalidation produces an explicit gap, bootstrap reads at most 24 hours/5,000 entries, non-allowlisted units are rejected, and messages truncate with a visible marker. Account for SQLite/journal overhead in the physical 20 MiB ceiling, not just event payload bytes.
- [ ] **Step 2: Write uploader tests.** With an unreachable fake HTTPS transport, assert nothing is acknowledged and queue survives restart; reconnect ACK removes only matching committed IDs. Test exact retry progression `[1,2,4,8,15,30,30]` before bounded jitter, 500 ms batch interval, timeout/429 retry, and 401/403 pause until configuration changes. Reject non-HTTPS except an explicit loopback-only test transport. Ensure tokens are never embedded in events or errors.
- [ ] **Step 3: Run `python -m unittest discover -s tests -p 'test_log_*.py'` in the Pi package venv.** Expect missing-module/interface failures.
- [ ] **Step 4: Implement stdlib SQLite queue, bounded journal subprocess reader, scalar normalization, redaction, and HTTPS upload.** Persist cursor and queue insertion atomically; use a non-recursive collector journal for internal errors. Do not fetch arbitrary URLs from event payloads.
- [ ] **Step 5: Run the new tests and complete Pi unittest suite.** Verify a network outage needs no Wi-Fi/BlueZ mock to keep queued records intact.
- [ ] **Step 6: Commit Task 3 files** as `feat: add independent buffered journal forwarder`.

## Task 4: Capture actual Pi safety causes without blocking outputs

**Files:** Create `Terra/hardware/raspberry-pi/terra_rover/operational_log.py`, `tests/test_safety_event_log.py`; modify `safety.py`, `ble.py`, `lifecycle.py`, `pairing.py`, `configuration.py` only at event boundaries.

**Interfaces:** `OperationalLog.emit(kind, level, message, context)` accepts bounded scalar data through a nonblocking bounded queue. Its background listener writes one JSON line to stdout/journald. Add an optional event sink to `SafetyController` for tests and production wiring. `safety.disarmed` includes previous armed/arming, original reason, session, last sequence, and command age computed before clearing state. The event sink cannot raise into safety logic. Record process/Bluetooth/ownership/configuration events with the same envelope and installed source identity; omit owner addresses.

- [ ] **Step 1: Add real-controller tests.** Arm using the existing safety fixture, expire a drive at 200 ms, and assert one `safety.disarmed` event with `reason == "watchdog"` and original command age. Repeat disarmed ticks and a subsequent disconnect; assert the original event remains unchanged and no repeated armed-to-disarmed event appears. Test explicit DISARM, rejected/malformed drive, lost subscription, gate loss, backend fault, and arming interruption.
- [ ] **Step 2: Add failure tests.** An event sink that raises or is full must not stop the safe-output transition. Assert startup/USR events contain no Bluetooth address and event creation does not call the uploader or disk on the output thread.
- [ ] **Step 3: Run `python -m unittest discover -s tests -p test_safety_event_log.py`.** Verify it fails on absent transition capture.
- [ ] **Step 4: Implement nonblocking event emission at the initiating transitions.** Preserve existing stop reasons and thresholds; use separate context for later state changes rather than overwriting prior events. Wire listener shutdown without preventing existing output-worker cleanup.
- [ ] **Step 5: Run the new tests and complete Pi suite.** Recheck existing `test_output_shutdown.py` and lifecycle/reset tests from the prior feature.
- [ ] **Step 6: Commit only logging changes** as `feat: record original rover disarm causes`; preserve pre-existing ownership diffs as required by the baseline procedure.

## Task 5: Add acknowledged phone-event transport and server bus ingest

**Files:** Create `Terra/crates/terra-transport/src/diagnostics.rs`, `tests/diagnostic_transport.rs`; modify `src/{lib.rs,control_plane.rs}`. Create `Terra/crates/terra-mobile/src/diagnostics.rs` and `tests/diagnostics.rs`; modify module declarations. Create `ARGOS/crates/argos-server/src/logs/bus.rs`, `tests/log_bus.rs`; modify server lifecycle wiring.

**Interfaces:** Produce `DiagnosticBatch { rover_id: u64, source_instance: String, payload: String }`, `ControlPlane::publish_diagnostics(batch) -> bool` (bounded enqueue only), and `ControlPlane::take_diagnostic_acks() -> Vec<String>` (max 32 per drain). Diagnostic queues/subscriptions are separate from state maps, control inboxes, and freshness. UniFFI methods `MobileController::publish_diagnostic_batch(payload: String) -> Result<bool, ControllerError>` and `take_diagnostic_acks() -> Vec<String>` expose only validated diagnostics. Backend `DiagnosticIngestor::start(endpoint, prefix, credentials, LogService)` uses its own Zenoh session/async bounded persistence worker and produces ACKs after `LogService::ingest` commits.

- [ ] **Step 1: Write loopback transport tests.** Publish two different events before a worker tick and assert both arrive; ensure latest-state replacement is never used. Fail a backend commit and assert no ACK; retry the same IDs after recovery and assert one stored row per ID plus ACK. Test wrong-instance ACK, unknown IDs, malformed payload, invalid rover topic, bus backpressure, and bounded drains.
- [ ] **Step 2: Add separation tests.** Send diagnostic-shaped messages to the diagnostic namespace and assert no ARM/DISARM/drive/action inbox receives them. Flood the bounded queue and assert rejection/loss reporting while a normal fleet telemetry/command exchange completes. Old-rover batches keep the original topic or remain queued if disallowed by credentials. Apply ingress budgets per authenticated bus source instance plus the global budget; label that identity as bus-reported.
- [ ] **Step 3: Run `cargo test -p terra-transport --test diagnostic_transport`, `cargo test -p terra-mobile --test diagnostics`, and ARGOS `cargo test -p argos-server --test log_bus`.** Confirm expected missing-interface failures.
- [ ] **Step 4: Implement isolated diagnostic transport, FFI validation, server subscriber, durable ACKs, and shutdown ownership.** Bounded callback enqueue only; never perform SQLite writes in Zenoh callbacks or control ticks. Disable the live subscriber in demo mode. Generate Swift bindings with the existing bindgen command.
- [ ] **Step 5: Run all new tests and both relevant Rust package suites.** Verify diagnostic traffic does not refresh ordinary telemetry TTLs or command freshness.
- [ ] **Step 6: Commit in both repositories** as `feat: deliver durable phone diagnostic events` and `feat: ingest fleet diagnostic batches` respectively.

## Task 6: Record and replay phone events

**Files:** Create `Terra/mobile/ios/TerraPhone/{OperationalEvent.swift,OperationalEventStore.swift,OperationalLogger.swift}` and `tests/ios/OperationalEventTests.swift`; modify `TerraLog.swift`, `PhoneController.swift`, `BluetoothLink.swift`, `Views/ContentView.swift`, explicit disarm callers, and targeted Xcode source entries.

**Interfaces:** `OperationalEvent` implements the approved wire contract. `OperationalEventStore(directory:)` exposes `record(event)`, `pendingBatch()`, `acknowledge(ids)`, and `flush()` on an independent utility queue with the Task 3 limits/reservation/loss policy. `OperationalLogger` provides `record(level:kind:message:context:)` and bounded string `notice/info/debug/warning/error` methods while also writing to OSLog. `PhoneController` snapshots association `{roverID, peripheralID, sourceInstance}` and delivers through Task 5 without placing filesystem/network work on `controlQueue`.

- [ ] **Step 1: Write Foundation-only tests.** Persist phone events for rover A, switch association to B, restart the store, and assert A events retain A while B events retain B. Ignore export of unassociated events. Test stable IDs/sequence after app restart, unknown/duplicate ACKs, outage replay, queue caps/high-priority reservation, truncation/redaction, and disk failures.
- [ ] **Step 2: Add event-caller tests.** Assert inactive/background/leaving-drive/explicit button/dashboard command/tracking-loss events have distinct initiating `kind`/context values. Preserve existing disarm policy. Validate the encoded wire fixtures with Task 1's Rust contract tests.
- [ ] **Step 3: Compile the event/store tests with `swiftc -module-cache-path /private/tmp/terra-log-swift-cache mobile/ios/TerraPhone/OperationalEvent.swift mobile/ios/TerraPhone/OperationalEventStore.swift tests/ios/OperationalEventTests.swift -o /private/tmp/terra-operational-event-tests`, then run `/private/tmp/terra-operational-event-tests`. Keep the event and store sources Foundation-only so this check runs on macOS without ARKit.** Expect failure before the new store/recording methods exist.
- [ ] **Step 4: Implement asynchronous capture and delivery.** Convert OSLog privacy interpolations carefully when migrating to string-based wrappers; do not expand messages with credentials. Ensure log-store failure is locally visible once and does not recursively record itself. Keep app events in OSLog as before. App suspension pauses delivery without discarding pending rows.
- [ ] **Step 5: Run Swift tests, existing connection lifecycle/selection checks, and Xcode simulator plus unsigned device builds.** Add source references to the existing project rather than recreating it. Verify diagnostic flushing requires no active motion loop.
- [ ] **Step 6: Commit Task 6 files** as `feat: capture and replay phone operational logs`.

## Task 7: Build the web Logs viewer

**Files:** Create `ARGOS/apps/web/src/{logs.ts,useLogs.ts,LogsPanel.tsx,LogsPanel.module.css,useLogs.test.tsx,LogsPanel.test.tsx}`; modify `App.tsx`, `api.ts` only as needed, and generated type registration in Rust. Create `apps/web/e2e/logs.spec.ts`.

**Interfaces:** `useLogs(user, selection, filters)` returns `{rows, connected, paused, newCount, gap, error, pause, resume, loadOlder}`. Selection is a rover ID or fleet-global view; rover selection includes clearly labeled global context by default. `LogsPanel` consumes the hook, renders messages as text, and exports the current filters through the authenticated JSONL route. SSE uses `/api/v1/logs/events`; it must not invoke the `useFleet` disconnect/control callback.

- [ ] **Step 1: Write hook/component tests.** Assert selected-rover plus global filtering, minimum severity/text filtering, disconnect keeps loaded rows, pause freezes rows while counting new events, resume loads current tail, and older-page merge deduplicates by string receipt ID. Include `9007199254740993`, a late response for a previous rover, and explicit retention/queue-loss indicators.
- [ ] **Step 2: Add display/permission tests.** A viewer without a claim can read/export logs. A historical-only rover is selectable outside the map. HTML-shaped messages render as text. Large streams cap rendered rows at 1,000. Export sends current filters and exposes the 10,000-event cap; log SSE failure leaves command-stream state unchanged.
- [ ] **Step 3: Run `npm --prefix apps/web test -- useLogs.test.tsx LogsPanel.test.tsx` in ARGOS.** Verify expected failures for the absent viewer.
- [ ] **Step 4: Implement the panel/hook and fleet-log entry point.** Reuse current typography, black/white tokens, 44px touch targets, responsive layout, and accessible labels. Mark disarm rows by text/kind, show source time in details, and avoid color-only severity. Cancel stale requests and EventSources on selection/filter changes.
- [ ] **Step 5: Run web tests/build and Playwright against the local demo plus log fixture ingress.** Browser assertions cover ingest → live row → disconnect/reconnect replay → filtering/export, plus keyboard/mobile-width access. Visually inspect the resulting view before declaring UI complete.
- [ ] **Step 6: Commit Task 7 files** as `feat: show live rover logs in web dashboard`.

## Task 8: Package, verify boundaries, and prepare rollout

**Files:** Modify Terra `scripts/build-rover.sh`, `packaging/rover/{Dockerfile,install.sh}`, package tests, and hardware docs; create `packaging/rover/{terra-log-forwarder.service,terra-log-forwarder.env}`. Modify ARGOS deployment docs/packaging to install/configure the collector and token CLI; add end-to-end recovery tests in both repositories.

**Interfaces:** Release includes the compiled `terra-log-forwarder` alongside `terra-rover`. Collector units have journal-read access, their own state directory, and no Fusion HAT actuator/sysfs access. They remain disabled when origin/rover/token configuration is absent. Configuration files are root/collector-readable only; use HTTPS with normal certificate validation. Existing owner/name/layout/environment survive upgrades. The ARGOS-host collector uses server/router unit allowlists and a global-source token.

- [ ] **Step 1: Add package/recovery tests.** Assert the release contains both ARM64 binaries, installer preserves owner/config and existing collector credentials, unconfigured collector cannot start, token output files are 0600, and hardening denies actuator writes. In a local integrated test, disconnect delivery, generate a real controller watchdog event, restart collector/backend, reconnect, and assert original cause plus one retained event per ID.
- [ ] **Step 2: Run these tests before packaging changes.** Confirm they fail on missing collector artifacts/configuration.
- [ ] **Step 3: Extend Nuitka build/package/installer for the second entry point and systemd unit.** Include SQLite/runtime dependencies and token provisioning instructions without real credentials. Add clean-runtime help/dependency/queue-recovery checks. Document offline limits, source clocks, loss markers, and viewing logs without a claim.
- [ ] **Step 4: Run full relevant suites.** Terra: Pi unittest suite, `cargo test -p terra-transport -p terra-mobile`, Swift connection/event checks, simulator/device builds. ARGOS: `cargo test --workspace`, `npm --prefix apps/web test`, `npm --prefix apps/web run build`, and log Playwright tests. Report every failure by name; do not silently exclude older failing tests.
- [ ] **Step 5: Build ARM64 release via the existing Docker builder.** Both executables must run in the Python-free verification stage; source archives and checksums must include the exact log-feature code.
- [ ] **Step 6: Perform the required independent branch review and fix important findings with regression tests.** Check especially that log failures cannot block safe outputs or renew motion freshness. Re-run checks only as needed for new changes/findings.
- [ ] **Step 7: Commit packaging/docs and present deployment artifacts.** Identify actual ARGOS host/origin, configure the Pi rover-ID mapping and restricted credentials, and make a concrete backup/rollback deployment plan. Do not claim live deployment or physical acceptance from builds/tests. Deploy backend/schema/web before enabling collectors/phone producers once the destination and deployment scope are established.

## Plan self-review and handoff

All approved spec sections are assigned: contract/retention to Task 1; access, rate limits, history/SSE/export to Task 2; offline queue/journal/upload to Task 3; original Pi causes to Task 4; independent bus acknowledgements to Task 5; phone caller/source capture to Task 6; web UX to Task 7; distribution, recovery, regression checks, and rollout to Task 8. Review Focus cases are pinned to the owning task tests. No motion-policy changes are included.

The tasks are one end-to-end feature with shared contracts, not independently selected products. Execute in dependency order; Tasks 3 and 4 can be developed after the contract is fixed, and Task 7 depends on completed read APIs. Keep intermediate collectors disabled until the receiving path is available.

Recommended execution: native implementation in this session, task by task, followed by one fresh independent reviewer. Most tasks share the event/ACK contract, so carrying that context reduces coordination overhead. The user must review this plan and select/confirm the execution method before implementation.
