# Diagnosing phone logging and router interruptions

The main event journal and its GCP uploader can themselves become unhealthy.
The phone now keeps independent JSONL diagnostic files next to `events.db` in
`Library/Application Support/Observability/`. These do not require a successful
SQLite insert or a working router/cloud connection.

| File | What it records |
| --- | --- |
| `events.journal-diagnostics.jsonl` | Worker checkpoints, in-memory queue depth, cumulative drops/failures, event queue wait, persistence duration, SQLite busy/locked codes, and sampled in-flight persistence markers. |
| `events.upload-diagnostics.jsonl` | Queue reads, attachment/event sends, acknowledgment writes, pending-count reads, batch age, acknowledged counts, attempt duration, error category/status, and retry delay. |
| `events.router-diagnostics.jsonl` | Router state and failure reason, freshest producer telemetry age and its existing TTL, plus selected phone lifecycle, Bluetooth and router transition metadata. |

Each file rotates at 1 MiB and keeps one previous segment (`.jsonl.1`), for a
maximum of 6 MiB across the three streams. Files are created with mode `0600`.
Records include wall-clock time and process identity; async router records also
include the time they were observed and a diagnostic-queue drop counter. These
sidecars are local troubleshooting files, not an additional GCP upload queue.
They can be retrieved through the connected Mac even while main ingestion fails.

Journal progress/checkpoints and router status are sampled every five seconds;
router state/freshness transitions are recorded immediately when polled. Upload
stages are recorded per attempt. Router recording uses a separate bounded queue
and does not wait for disk or network on the control tick. Journal/upload file
writes run on their existing background workers. Diagnostic file failures never
change driving authority, timeouts, reconnect policy, or safety decisions.

## Reading the evidence

- A growing `queued` value or `queue_wait_ms`, with `sqlite_busy`/`sqlite_locked`,
  points to local journal contention. `persist.started` followed by no later
  checkpoint helps locate a storage stall. The sampled marker identifies the
  event kind/sequence, not its payload.
- `upload.stage` identifies the last entered operation. A failure at `save_ack`
  means the network send returned but the local acknowledgment write failed;
  retries retain the existing event identity and remain deduplicated by GCP.
- `http_timeout`, `http_connect`, and numeric `http_status` distinguish transport
  failures, authentication rejection (401/403), throttling (429), and server
  rejection. Response bodies, credentials, and endpoint URLs are not copied.
- `router_unavailable` means Zenoh reported no connected router. Other reasons
  distinguish fleet, telemetry, and batch publication failure. These are the
  failure sites, not a claim about the underlying Wi-Fi/TLS cause.
- `state=connected` with `telemetry_fresh=false` means producer telemetry has
  stopped refreshing; fleet advertisements can then expire without a router
  disconnect. Compare the timing with lifecycle and Bluetooth records.
  A busy telemetry lock is reported as unavailable/unknown, never as stale.
- Router session-open failures are classified into safe categories. Categories
  inferred from native error text are hints; the original text is not retained
  because it could contain connection details or credentials.

The update requires rebuilding and installing Terra on the phone. It adds
diagnostics and does not repair the underlying database contention or establish
that contention caused a Zenoh disconnect. Existing journals are preserved.

## Storage reliability update (2026-10-22)

The subsequent storage fix migrates existing databases in place. A single writer
per canonical database path handles both event batches and acknowledgment updates;
readers no longer initialize schema on every query. Transactional metadata/triggers
account for logical bytes, and indexed retention replaces full-history per-event
scans. Event batches contain at most 64 records. Quota cleanup commits at most 64
quota evictions per pass and services acknowledgments between passes. A batch and
its loss marker may temporarily exceed the logical quota; further event batches
pause until cleanup restores it. An already oversized database is reclaimed in
bounded passes with explicit loss markers under the configured retention/quota.
SQLite file size also includes page/index/WAL overhead, as before.

When GCP accepts a batch, its IDs are retained in the shared delivery session
until local acknowledgment persistence succeeds. Local acknowledgment failures
retry locally at the existing one-second base interval; they do not resend the
accepted batch or escalate network backoff. A process crash may still resend the
same original IDs, which the collector deduplicates. Health now includes
`local_ack_failures`; upload diagnostics include `retry_scope=local_ack|network`.
`quota.pass` records bounded cleanup progress. No motion or safety policy changes
are part of this storage update.
