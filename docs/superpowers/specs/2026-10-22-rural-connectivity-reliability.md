# Rural connectivity and reliable run recording

Date: 2026-10-22
Status: Planning requested; implementation has not started.

## Agreed outcome

The rover should execute an already accepted autonomous mission using onboard
information without requiring a continuous cloud/dashboard connection. Weak rural
connectivity must not hide a connected rover or prevent useful run recording.
Remote driving command expiry and onboard sensor/control health are distinct
from dashboard visibility and cloud delivery.

This work has three independently shippable parts, in this order:

1. Remove the confirmed local logging bottleneck.
2. Keep connected and previously observed rovers visible without fresh GPS.
3. Separate automatic network interruption from mission cancellation and local faults.

## Evidence and limits

Phone diagnostics show successful GCP batch acknowledgments followed by SQLite
`save_ack`/`sqlite_busy` failures. Retries back off while the pending queue grows.
`journal.rs::persist` scans event history and performs retention work inside each
event's write transaction. Two phone journals share the database. An offline
database-copy probe reproduced acknowledgment starvation with those scans;
removing the maintenance from the hot transaction made acknowledgment saves fast.
The dashboard also has a substantial queue using this same journal implementation.

The screen recording showed `healthy · 0 rovers` while the dashboard received
Rover 0 status messages. A read-only subscriber confirmed one online fleet member,
a fresh local pose, zero map markers, and no geographic coordinates. The phone
reported an 8.2-second-old GPS fix; its hard-coded three-second rule omitted the
coordinates. GPUI counts map markers instead of fleet members.

These findings explain the captured backlog and visibility failure. The earlier
brief Zenoh session loss has not been traced to a physical/network root cause.
Retain the new independent diagnostics to investigate a recurrence.

## Required behavior

### Recording

- Keep event IDs, original timestamps, pending records, attachments and current
  authenticated GCP/Rerun interfaces. Migrate existing databases in place.
- Share one storage writer per database path within a process. Acknowledgment
  updates must make progress during sustained event admission.
- Eliminate full-history accounting/retention scans from each event insert.
  Maintain transactional byte accounting and indexed retention/eviction queries.
- Preserve bounded admission, retention/quota loss markers, acknowledgment
  deduplication and independent diagnostic sidecars. Never wait on disk/network
  from control or sensor callbacks.
- Distinguish network delivery failure from local acknowledgment-save failure.
  A failed local acknowledgment save must not deliberately enter network backoff
  or immediately resend a batch already accepted by GCP.
- Do not clear a backlog or relax event retention to make health appear normal.

### Visibility and timing

- Count/list fleet members independently of map-marker eligibility. Show an online
  rover even before its first geographic fix, with `Position unavailable`.
- Keep the last measured position visible when GPS ages. Label its observation
  age and accuracy; stale position must never be presented as a fresh fix.
- On network interruption, retain the last-known rover and show connectivity age.
  Clear/relabel cached positions on a new localization frame/run so coordinates
  cannot silently migrate into a different reference frame.
- Keep raw GPS observation and its timestamp separate from navigation eligibility.
  Preserve permission/error/quality reasons in the report.
- Move GPS display freshness and network presentation timing into documented
  policy settings. Initial display freshness remains 3 seconds but becomes a
  label, not a visibility gate. Keep the existing 90-second fleet stale label
  initially. Neither value determines whether a mission may move.
- Do not invent a universal two-minute sensor cutoff. Existing onboard map,
  pose, IMU, controller and hardware limits remain until separately validated.
  Document which ages use source time and which use monotonic receipt time.

### Mission and reconnection

- Introduce an explicit distinction between automatic router loss and an operator
  cancel/disconnect action. Reconnection alone never arms, resumes a fault-paused
  mission, starts a new mission, or replays motor/arm/servo commands.
- Default automatic router-loss policy: continue an accepted locally executable
  autonomous mission while local hardware, required sensors, mission budget and
  existing admission conditions remain healthy. The policy is configurable;
  `pause` remains available for deployments requiring operator supervision.
- Apply this consistently to local/geographic waypoint execution and autonomous
  exploration/search. Supervised modes may finish an already approved action but
  cannot invent approval for the next proposal while the operator is absent.
- Expire remote manual intent using the existing short command leases. Preserve
  emergency-stop latches, explicit cancellation, hardware-link loss handling,
  and actual local sensor/control failures. Do not automatically rearm.
- Queue logs/evidence for eventual upload. After reconnection publish current
  state and reconcile mission identity; do not replay historical motion commands.
- A pause/cancel received after reconnection is a current validated command;
  offline delivery of a remote emergency-stop command is not assumed.

## Acceptance

Use a production-sized journal and sustained writes to prove acknowledgments
advance, queues drain when connectivity allows, and records remain deduplicated.
Prove stale GPS and router outages do not erase a rover from the dashboard.
In deterministic simulations, disconnect the router for 120 seconds while local
inputs remain fresh: accepted autonomy continues, manual intent expires, budgets
continue to elapse, and reconnect does not arm/replay/resume a latched stop.
Repeat with local sensor/hardware failure, explicit cancellation and new run IDs
to prove those distinct causes still take effect.

Deploy storage and visibility changes before mission-continuation behavior.
Preserve existing TLS, credentials, GCP collector, USB logging and Rerun routes.
Preserve the existing uncommitted diagnostic work in the shared workspace;
review/stage it separately rather than resetting it or including it accidentally.
For physical verification use a stationary/disarmed connectivity check first,
then an operator-supervised low-speed run with the rover's local stop available.

## Implementation plans

- [1. Log storage and delivery](../plans/2026-10-22-log-storage-reliability.md)
- [2. Rover visibility and GPS age](../plans/2026-10-22-rover-visibility.md)
- [3. Autonomous operation through network loss](../plans/2026-10-22-autonomy-network-loss.md)
