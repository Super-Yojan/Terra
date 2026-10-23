# Operational event schema v1

An envelope contains `schema_version: 1`, `id` (producer_id, boot_id, sequence)
and `input` (rover_id, session_id, component, kind, severity, wall_time_ns,
monotonic_ns, cause_id, attributes). IDs remain stable across reconnect uploads.
The event's rover/session/time belongs to creation, not upload. Source sequence
orders one producer boot. Wall time alone cannot establish cross-device causality.

`Journal::try_record` admits a record to bounded memory without disk or network I/O.
`Queued` does not claim disk durability. The worker persists transactions with
SQLite WAL and synchronous FULL. Administrative `flush` waits for earlier work;
never call it on the control path. Worker failures and queue overflow increment
health counters and produce a separate `observability.gap` when storage recovers.
Closing the journal drains its channel in the background; a process crash can lose
in-memory events. Pending delivery is retained across process restarts.

Quotas count logical event payload bytes, excluding SQLite page/index overhead.
Disk/WAL storage can exceed that logical limit; service filesystem quotas provide
the hard deployment boundary. Retention evicts acknowledged rows before pending
rows. Loss of pending records creates a gap event. Read-only history/export retains
acknowledged records until retention removes them.
