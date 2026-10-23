# Log Storage Reliability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate local acknowledgment starvation and drain existing phone/dashboard backlogs without losing evidence.

**Architecture:** Keep the public journal API and event format. Route event inserts and acknowledgment mutations through one background storage writer per database path, with fair scheduling, short transactions, incremental accounting and indexed retention. Keep HTTP requests outside storage transactions.

**Tech Stack:** Rust, rusqlite/WAL, existing HTTPS uploader, Swift/UniFFI, GPUI.

**Spec:** [Rural connectivity and reliable run recording](../specs/2026-10-22-rural-connectivity-reliability.md)

## Global Constraints

- Preserve original event identity/time, pending evidence, quotas and existing GCP authentication/Rerun contracts.
- No database/network wait on control or sensor callbacks; admission remains bounded.
- Migrate existing journals in place; do not delete the backlog.
- Existing independent diagnostic sidecars remain available.

## Review Focus

- Multiple journal instances opening the same database must share the writer and agree on persisted storage policy.
- Process death during insert/ack/migration must not corrupt byte accounting or event identity.
- Continuous event production must not starve acknowledgments or retention.
- GCP acceptance followed by local persistence failure must not trigger a network retry storm.
- Disk pressure and cross-process lock contention must leave honest health/loss evidence.

### Task 1: Replace per-event scans and competing writers

**Files:** Modify `crates/terra-observability/src/{journal,lib}.rs`; create `crates/terra-observability/src/storage.rs`; extend `crates/terra-observability/tests/journal.rs` and add `tests/storage_load.rs` in that crate.

**Interfaces:** Preserve `Journal::open`, `try_record`, `flush`, `health` and `JournalReader::{open,batch,ack,pending_count}`. Internal `StorageHandle` owns append/ack scheduling and storage policy; no second mutation connection per journal instance. Reader-only batch/export queries must not rerun schema/WAL initialization.

- [ ] Write failing tests using two journal instances, a prepopulated 100 MiB database, sustained 20-events/second admission, and repeated 64-ID acknowledgments. Assert acknowledgments progress, admission never waits for disk, and final stored IDs match admitted IDs minus explicit loss markers.
- [ ] Add accounting tests: append, acknowledge, prune, rollback, reopen and migration from the current schema all yield `logical_bytes == SUM(events.bytes)`. Include a subprocess crash/reopen and two different spellings of the same canonical path.
- [ ] Run `cargo test -p terra-observability --test journal --test storage_load` and confirm the intended contention/accounting failures before implementation.
- [ ] Implement a shared writer registry by canonical database path, bounded append batches of at most 64 records, and acknowledgment service between batches. Keep accounting transactional with the event mutations. Add indexes for retention and acknowledged-first eviction; initialize totals once during migration, not for each insert. Retain quota enforcement/loss markers and reject conflicting storage policies explicitly.
- [ ] Run the tests to passing, then the full `terra-observability` suite. Check writer-loss/queue-age diagnostics during the load scenario; compare with the captured `save_ack/sqlite_busy` failure.
- [ ] Commit this independently reviewable storage change with its tests and migration notes.

### Task 2: Separate delivery from local acknowledgment retries

**Files:** Modify `crates/terra-observability/src/uploader.rs`; extend `crates/terra-observability/tests/uploader.rs`; update `docs/observability-diagnostics.md`.

**Interfaces:** Keep `UploadTransport::send` and collector event IDs unchanged. Add uploader state `awaiting_local_ack: Option<Vec<EventId>>`; retry those IDs locally before selecting or sending another batch. Existing `upload_once` callers must retain correct deduplication semantics.

- [ ] Write failing transport/storage integration tests: HTTP accepts 64 IDs, local ack is temporarily blocked, then recovers. Assert one HTTP send during the block, eventual pending-count reduction, original timestamps unchanged, and a process restart can safely resend the identical IDs.
- [ ] Add transient network failure, 401/403, 429, disk-full and permanent local-write failure cases. Assert honest stage/error reporting and bounded retries, with no credential/response-body leakage.
- [ ] Run `cargo test -p terra-observability --test uploader` and verify the intended failures.
- [ ] Implement local acknowledgment retry state, separate local/network backoff accounting and diagnostics. Never hold a database transaction while making HTTP or attachment requests.
- [ ] Run `cargo test -p terra-observability -p terra-mobile`; run the affected ARGOS tests against the shared crate. Build/install phone and rebuild GPUI. Verify existing pending counts decrease over a measured interval without clearing the journals; interrupt and restore uplink and confirm deduplicated catch-up in GCP/Rerun.
- [ ] Commit the delivery change and record queue-before/after, acknowledgment throughput, dropped counts and deployed build IDs.
