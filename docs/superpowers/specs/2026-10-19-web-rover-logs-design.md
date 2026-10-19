# Remote rover logs in the ARGOS web dashboard

Date: 2026-10-19
Status: Specification approved in chat, including offline buffering; implementation plan awaits review.

## Outcome and approved scope

The operator wants to diagnose unexpected vehicle disarms remotely from the ARGOS
web dashboard. The approved direction is a live log panel per rover, combined Pi,
TerraPhone, Bluetooth, and dashboard/router events, searchable retained history,
export, highlighted disarm causes, and replay after reconnecting. The Pi must
forward logs independently of TerraPhone so a Bluetooth failure remains visible.
The native Apple ARGOS dashboard is outside this change.

“All logs” means operational events produced by the Terra service and app, relevant
BlueZ service events, ARGOS server events, and fleet router service events. It does
not mean arbitrary files, the entire system journal, other applications' logs, or
all iOS operating-system logs. Existing OS logging remains available locally.

Observation must never arm, disarm, renew command freshness, replay a command, or
change the current safety policy. Changes to disarm thresholds are outside scope.

## Existing implementation

- Terra Pi is a compiled ARM64 Python service using BlueZ and Bluetooth. Its normal
  deployment has no Zenoh client. `SafetyController` exposes a current `stop_reason`
  but does not persist each safety transition; later states can overwrite it.
- TerraPhone logs through `OSLog` in `TerraLog.swift`, with useful control and
  Bluetooth messages. OSLog is not currently forwarded remotely.
- TerraPhone's Rust controller publishes hardware observations over Zenoh to
  `<prefix>/<rover_id>/hardware/status`. Latest-value telemetry cannot preserve
  individual log events.
- ARGOS's Rust web server already has authenticated browser sessions, SSE, and a
  SQLite database. The React dashboard selects a rover and displays its controls.
  It currently has no operational log history or log ingestion API.
- Work spans `/Users/yojan/git/Terra` and `/Users/yojan/git/ARGOS`. Preserve existing
  Terra ownership/rover-selection changes. ARGOS was clean during exploration.

## Collection and delivery

### Raspberry Pi

Add structured, transition-based safety logging to the rover service. Each actual
armed/arming-to-disarmed transition records the initiating reason immediately,
before a subsequent tick or connection change can overwrite it. Include the
previous armed/arming state, command age when known, session identifier, last
accepted sequence, and the initiating component. Do not emit another disarm event
on every already-disarmed safety tick. Log first faults and subsequent changes,
arming requests/outcomes, Bluetooth admission/disconnect, USR reset, pairing,
configuration changes, and startup/shutdown.

Add a separate compiled `terra-log-forwarder` system service. It follows JSON
journal entries for the explicit units `terra-rover.service` and
`bluetooth.service`, persists a journal cursor and bounded outgoing queue under
`/var/lib/terra-log-forwarder`, and uploads batches to ARGOS over HTTPS. Structured
rover events are decoded; ordinary service messages remain plain-text events. Derive
forwarded journal event IDs deterministically from the source instance and journal
cursor so replay after a collector crash cannot invent new IDs. On first setup,
collect at most the last 24 hours and 5,000 matching entries, marking this initial
history boundary.
The forwarder reads journals and its own queue; it receives no actuator access.
It does not depend on a phone, BLE connection, or rover service lifetime.

Configuration supplies the ARGOS HTTPS origin, the fleet rover ID, and a scoped
write-only ingestion token from a private environment/credential file. Do not
reuse Bluetooth ownership or copy a GitHub browser session. The rover ID must
match the phone's existing dashboard rover ID; provisioning checks this explicit
mapping rather than deriving an ID from the random rover name.

### TerraPhone

Introduce a shared operational event recorder alongside OSLog and route the app's
Bluetooth, control, tracking, configuration, and connection logging through it.
Retain the existing local messages. Capture disarm callers, app lifecycle changes,
leaving drive controls, readiness/tracking loss, router failures, and arm outcomes.
This makes `requested` on the Pi traceable to a specific app action or subsystem.

Buffer events on an independent serial utility queue and publish batches through
the current authenticated ARGOS Zenoh connection on
`<prefix>/<rover_id>/diagnostics/events`. A dedicated bounded event queue must be
used; the latest-value status map must not overwrite events. The web server
subscribes independently of command handling and acknowledges persisted event IDs
on `<prefix>/<rover_id>/diagnostics/acks/<source_instance>`.

Events leave the local queue only after acknowledgement. Reconnection republishes
unacknowledged events with unchanged IDs. ACK parsing accepts only the matching
source instance and queued IDs and never reaches command handlers. Existing bus
credential permissions must cover these diagnostic topics; the trust boundary
remains the authenticated fleet bus, not independent device attestation.

Freeze the configured rover ID and Bluetooth peripheral identity on event creation.
Switching rovers must not relabel buffered events. Replay earlier-rover events
through their original diagnostic topic only when the current credentials permit
that topic; otherwise keep them in the bounded local queue. Events with no established rover
association stay in local phone logs and are not assigned to an unrelated rover.
iOS suspension stops transmission; buffered events resume on the next active
session. This is not a promise of continuous iOS background execution.

### Server and router

Use the same journal forwarder on the ARGOS host with a separate token restricted
to the ARGOS server and router units. These are fleet-level events, shown as global
context rather than falsely attributed to an individual rover. Exclude access-log
noise and collector-internal errors from the forwarded unit allowlist. Collector
errors remain in its own local journal, avoiding recursive ingestion.

## Event contract and limits

Use a versioned JSON event containing:

- `schema_version`: 1.
- `event_id`: stable UUID retained through retries.
- `source_instance`: installation/device identifier; no secret or owner address.
- `boot_id` or app-run ID, plus a monotonically increasing per-instance sequence.
- `rover_id`: decimal string for a rover event, absent for fleet-global events.
- `source`: `pi`, `phone`, `bluetooth`, `server`, or `router`.
- `component`, `level` (`debug`, `info`, `warning`, `error`), `kind`, and `message`.
- Source UTC timestamp and source monotonic time when available.
- Bounded structured context, including disarm reason/command age where relevant.

The server adds an increasing receipt ID and receipt timestamp. Use receipt IDs for
pagination/reconnection and display receipt order by default. Label source times
separately: clock skew must not produce missing or endlessly replayed events.

Limits: 4 KiB per event, 32 events and 128 KiB per ingestion request; messages up to
2 KiB, component/kind up to 64 characters, and flat scalar context up to 16 keys.
Validate IDs, levels, sources, timestamps, and lengths before storage. Oversized
messages are truncated by collectors with a visible truncation marker; the server
rejects malformed/oversized envelopes. Export never includes ingestion credentials.

Each collector keeps at most 5,000 events and 20 MiB on disk, with 20 percent of
capacity reserved for warnings/errors. When full, discard oldest lower-priority
events first, then oldest events if necessary, and report a counted loss event on
recovery. Repeated state messages are coalesced at collection. No claim of lossless
logging under unbounded outage/load. Queue work and network delivery must never
block the Pi output worker or phone control queue.

Send batches at most every 500 ms under normal traffic; retry failures after 1, 2,
4, 8, 15, then 30 seconds, capped at 30 seconds with jitter. Authentication failures
pause delivery until configuration changes and remain visible locally. Disk and
network errors degrade log delivery, never control. Advance the journal cursor
only after durable queue insertion; a crash may replay a stable-ID event, which
server deduplication handles.

## Backend, storage, and access

Extend `argos-server` with:

- `POST /api/v1/log-ingest`: write-only token authentication, token-bound allowed
  rover/source/unit scope, bounded request validation, transactional insertion, and
  acknowledgement of committed or previously committed event IDs. It cannot issue
  any vehicle command. Store only token hashes server-side; support revocation.
  Provide an administrative CLI to create tokens into a private output file, list
  non-secret token metadata, and revoke tokens. No browser control claim or OAuth
  cookie is used for collector authentication.
- Fleet-bus ingestion of phone diagnostic events into that same validated storage
  path, with acknowledgement only after commit. Source attribution is reported by
  the authenticated bus producer, not proven by the event payload alone.
- `GET /api/v1/logs`: existing viewer/operator session authentication; filtered,
  cursor-based history with a maximum page size of 200.
- `GET /api/v1/logs/events`: a separate authenticated SSE stream with receipt IDs,
  cursor recovery, heartbeat, and a bounded per-client delivery window. Revalidate
  session expiry/revocation as existing fleet SSE does. Slow clients reconnect and
  read persisted history; they cannot block ingestion or fleet updates.
- `GET /api/v1/logs/export`: streamed JSON Lines for the current filters, capped at
  10,000 events per request, with a visible export range/limit.

Store events in a new indexed SQLite table, separate from command/audit state.
Deduplicate by source instance and event ID. Retain at most seven days and 10,000
events per rover/global group, with an overall cap of 100,000 events. Cleanup runs
in bounded batches. Receipt IDs are never reused. Return the earliest retained
cursor and an explicit history-gap indicator when retention has removed a client's
requested range. A replayed event pruned from history can be accepted again, but
must retain its original source timestamp; do not imply it is a new occurrence.

Apply an ingress budget of 200 events/second per credential and globally 1,000
accepted events/second; return a retriable limit response without acknowledging
uncommitted events. Avoid full event bodies in HTTP access logs. Redact credentials,
Bluetooth owner identity, and unrelated sensor payloads at collection. Browser
rendering treats every message as text. Logs require existing team access but do
not require a vehicle control claim.

## Web experience

Add a Logs section alongside the selected rover's details, accessible even when
that rover is offline. Also provide a fleet-level log view for server/router events
and rovers present only in history. Native ARGOS views remain unchanged.

The selected-rover panel defaults to that rover plus fleet-global context, clearly
labeling both. Controls are source, minimum severity, text search, live/pause,
load older, and Download JSONL. Each row shows time, severity, source/component,
and message; expanded details show context and source time. Disarms are identified
by their event kind and have an explicit reason label, with surrounding events
available by scrolling/search. Do not depend on color alone.

Pause freezes rendered rows while ingestion/history continue; show a new-event
count and resume at the current tail on request. Show log-stream state separately
from rover connectivity and command readiness. Keep the last loaded rows visible
on disconnect. Display queue loss and retention gaps explicitly. Cap rendered rows
at 1,000 with older events available through paging. Search/filter changes reset
the cursor and cancel obsolete requests so one rover's rows do not appear under
another selection.

Reuse the web dashboard's existing black-and-white design, typography, accessible
controls, responsive inspector, and event-stream conventions. Log stream errors
must not invoke the fleet stream's command-disconnect callback.

## Validation and rollout

Automated coverage must verify:

1. Real safety transitions log once with the original cause and command age;
   repeated disarmed ticks do not overwrite the retained event or flood logs.
2. Phone disarm caller and lifecycle events are recorded; buffered events remain
   attached to the original rover after switching.
3. Cursor persistence, crash replay, deduplication, acknowledgements, bounded queue
   eviction/loss reporting, and retry/authentication behavior.
4. Token/session authorization, source scope, malformed/oversized input, rate
   limits, retention, history gaps, pagination, and source clock skew.
5. SSE reconnect, expired sessions, slow clients, pause/resume, filtering,
   rover-selection races, safe rendering, and export boundaries.
6. Log traffic/failures do not change safety state, refresh drive freshness, replay
   commands, or disrupt the existing fleet stream.

Run Terra Pi tests, relevant Rust transport/mobile tests and binding checks, iOS
builds, ARGOS Rust tests, web tests/build, and browser end-to-end coverage. Verify
compiled ARM64 rover and collector packaging in a clean Linux environment.

Deploy the backend/schema and dashboard before enabling producers. Provision
separate write-only credentials without printing them, configure the Pi's actual
rover ID, and preserve owner/name/layout. Back up installed binaries/configuration
and the ARGOS database before rollout. Changing the existing Pi deployment or
hosted dashboard requires carrying out the normal reviewable deployment workflow;
prior authorization to deploy the ownership service does not itself identify the
hosted dashboard deployment destination.

Physical acceptance: observe a real arm/disarm, a Bluetooth disconnect, a Pi-service
restart, and an internet outage/reconnect from the web viewer; confirm the initiating
cause and buffered replay. Perform motion-related tests only under operator control.
Never trigger a disarm or ownership reset merely to validate log plumbing remotely.
Report hardware/network checks separately from software tests.

## Review record

The operator approved the web dashboard and the direct-Pi plus phone-event design
in conversation, then accepted the specification after reviewing offline behavior.
This document fixes the implementation contract; no log feature has been implemented
or deployed as part of writing it. Review the implementation plan before coding.
