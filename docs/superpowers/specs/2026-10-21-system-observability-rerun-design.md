# System-wide observability and Rerun replay

Status: Operator requested the camera/ground-reference addition and implementation; implementation plan pending review.
Date: 2026-10-21

## Outcome

The operator wants logs for the whole robotics system, not only current disconnection problems, and a Rerun timeline to walk through events alongside robot state. Capture normal operation, state changes, decisions and faults across TerraPhone, Raspberry Pi, the GCP Zenoh router, ARGOS and Zorvane. Preserve evidence through disconnects and restarts; synchronize it when connectivity returns.

This supersedes the deployment assumptions of the October 19 web-rover-logs proposal. That document assumes an ARGOS web server and web UI. This design introduces an independent observability collector and downloadable Rerun recordings; it does not restore argos-server.

## Architecture

Each producer writes a bounded local structured journal through a background worker. Producers independently batch and upload committed records over authenticated HTTPS to a dedicated collector on the existing GCP host. The Pi uploads independently when it has network connectivity; without it, records remain local. Phone logging does not rely on the Zenoh connection under investigation. Existing OSLog and system journal output remain available.

The collector durably stores events and evidence attachments, provides authenticated session listing and export, and builds Rerun recordings in a separate worker. GCP deployment remains a subsequent step after local validation. Preserve existing Zenoh listeners, credentials, Caddy certificate renewal and router configuration. Add a separately authenticated HTTPS route and service; do not expose a public unauthenticated Rerun ingestion port.

The structured journal is the authoritative record. Rerun is its visual projection. Conversion failure leaves original evidence available and can be retried. Pin and record the tested Rerun SDK/viewer versions during implementation. No Rerun dependency is needed in the phone control loop: conversion runs on the collector or a local export tool.

## Coverage contract

Instrument every defined operational event and decision trigger, with a coverage inventory linking event kinds to call sites and tests. This does not promise arbitrary operating-system logs or every raw sensor sample.

- Lifecycle: boot, shutdown, foreground/background, controller/session start and reset, software/configuration versions.
- Connections: Bluetooth discovery, authentication, readiness, disconnect and retry; phone/router and dashboard/router transitions; router service and TLS renewal/restart events.
- Commands: creation, receipt, admission/rejection, expiration, acknowledgment, authority changes and operator actions, correlated without replaying control commands.
- Motion: intent, accepted target, measured velocity, output effort, hold/pause/disarm/arm transitions, initiating reason and recovery. Keep the original cause instead of overwriting it with a generic downstream label.
- Sensors: health changes, frame gaps, timestamps, actual age versus limit, localization quality, coordinate-frame/session changes and ground-reference establishment/loss.
- Mapping: accepted/rejected integrations, revision, ground classification summary, occupancy changes and referenced map snapshots.
- Navigation: mode, goal, route, proposals, progress, candidate/rejected maneuvers, constraint reasons and the map revision used.
- Recording: queue depth, upload results, storage pressure, retention gaps and dropped-event counts.

Log transitions and discrete triggers without sampling. Periodic state and heavy evidence have explicit configurable sampling; repeated identical per-tick decisions become counted intervals. Proposed initial rates: 10 Hz control summaries, 2 Hz map snapshots, downsampled point clouds at 1 Hz when available. These are logging rates, never control rates. Synchronized camera and depth evidence is included at sampled rates; full-rate camera video/raw depth is opt-in. Evidence sampling and dropped counts are visible in recordings.

## Event identity and time

Every event carries schema version, event ID, producer identity, boot ID, monotonically increasing source sequence, rover ID, session/run association, component, event kind, severity, message and typed attributes. Include causal parent/command IDs where available, wall time, source monotonic time and collector receipt time. Never use credentials as correlation IDs.

A restart creates a new boot identity. Offline events keep their original rover and session after selection changes. Sequence order within a producer is authoritative. Estimate cross-device clock offsets with uncertainty and retain raw timestamps. Do not imply precise cross-device order when uncertainty overlaps. Record session association changes explicitly; unassociated events remain discoverable rather than being assigned to an arbitrary mission.

## Reliable bounded delivery

Upload stable event IDs in bounded batches; collector acknowledges only durable commits. Persist acknowledgment cursors and retry unacknowledged batches with backoff and jitter. Collector deduplicates by producer, boot and sequence and rejects identity/payload conflicts. Restart or interrupted response must not produce duplicate visible events. Original event time remains unchanged after reconnect.

Proposed configurable defaults: phone journal 256 MiB, Pi 512 MiB, dashboard/simulator 256 MiB, seven-day local retention; collector 30-day retention and a 10 GiB quota. Separate heavy-attachment quotas from event storage. Evict acknowledged oldest records first; if unacknowledged records must be evicted, persist a gap marker and counts. Bound memory queues and expose drops during storage failures. Do not claim zero data loss under disk failure or exhausted retention.

Observability failure must not pause, disarm, arm, renew data freshness or block the control loop. Keep this pipeline independent of the existing experiment recorder's failure policy; changing that policy is outside this feature. This work does not extend safety timeouts or queue motion commands for later execution.

## Rerun experience

Export a replayable .rrd recording per selected rover/session and time range, with a manifest and original JSONL journal available alongside it. Offline conversion is supported. Late-arriving events cause a new export revision with the same event identities, not false current events.

Provide synchronized views for event text, connection states, sensor freshness, control target/measured/effort curves, pose/trajectory, occupancy, route/goal and maneuver candidates. Clearly identify coordinate frames and reset boundaries. Unknown cells remain unknown. Show rejected paths and reasons separately from executed motion.

Expose a session-time timeline with alignment uncertainty, source-sequence timelines where necessary, and a receipt-time view for transport delays. Scrubbing should reveal the map and state valid at the selected event. Provide a default viewer layout and an event index for locating arm/disarm, reconnect and planning rejection events. Do not promise a custom next-event control until supported by the pinned viewer or export index.

## Camera, depth and ground-reference replay

Physical and simulated runs must include sampled camera frames alongside matching
depth, camera calibration, exposure pose and the exact ground reference used for
mapping. An RGB frame alone cannot explain an occupied cell.

Verified baseline: TerraPhone establishes a fixed world offset and camera clearance
from a measured ground candidate of at least 0.4 square metres, stable within 3 cm
for one second. Its local supporting plane then follows the rover body up axis,
using that calibrated clearance. It assumes the rigidly mounted rover remains
supported on the terrain; it does not independently fit a ground plane from every
depth image. A changed mount or incorrect initial ground candidate can therefore
invalidate that assumption. These are possible causes, not diagnosed field faults.
Zorvane currently calls the mapper with the default world ground plane. The shared
mapper defaults classify endpoints at least 0.15 m and at most 2.0 m above its
supplied plane as obstacle evidence. Record actual configured values, not just
these defaults. The optical image center is not the ground reference.

For each captured mapping evidence bundle, record source frame ID and exposure
time, RGB image (if available), depth buffer and confidence/validity mask, RGB and
depth intrinsics/resolutions, image orientation/crop, camera-to-body and
camera-to-world transforms, body pose, frame convention, ground plane normal and
offset, calibrated clearance, world offset, mapping thresholds, map revision and
accepted/rejected integration reason. Link every attachment to its manifest with
content hash, encoding, dimensions and units. Preserve the exact sampled depth
and transform used by mapping. If RGB cannot be paired, mark it missing; do not
substitute the latest unrelated image.

Initial configurable evidence rate: one synchronized bundle per second, shared
with the sampled cloud. A bounded ten-second ring of these samples is retained
around a hold/fault, plus ten seconds afterward. No full-rate capture is implied.
Attachments use separate 512 MiB local and 5 GiB collector quotas; eviction emits
gap records. Simulation captures its rendered sensor view when available and
explicitly reports absent RGB instead of fabricating a camera image.

Rerun shows camera and depth images, camera frustum/axes, point cloud, supporting
ground plane and obstacle-height band beside occupancy. Color sampled points by
the mapping classification (free endpoint, obstacle endpoint, rejected/unknown).
Distinguish endpoint classification from accumulated occupancy evidence. Show the
actual optical principal point, image orientation and mounting transform so an
up/down offset can be inspected rather than inferred from the visual center.

Add fixture tests for flat ground, slopes, raised surfaces, changed camera height,
pitch/roll, mismatched RGB/depth timestamps and localization resets. Verify exported
geometry numerically against mapper inputs, then visually inspect a recording.
This feature observes calibration and classification; it does not silently change
the ground estimator, turning restrictions or occupancy thresholds.

## Security and resource isolation

Use scoped producer ingestion credentials and separate read credentials. Never embed a cloud service-account key in the app. Validate source/rover scope, payload sizes and schema; rate-limit ingestion. Exclude passwords, ownership secrets and authentication headers. Treat messages as plain text. Location and visual evidence remain authenticated with documented retention. Provision credentials through existing secure deployment mechanisms without printing them.

Background collection, uploads and Rerun conversion have bounded resources. Router restart events are observed, not induced for testing. No remote drive or disarm actions are taken merely to demonstrate logging.

## Acceptance

1. A normal run records lifecycle, configuration, connections, goals, commands and decision transitions from all instrumented producers.
2. Network loss and reconnection preserve offline event ordering and timestamps, with acknowledged replay and duplicate suppression.
3. Producer/collector crashes, response loss, disk exhaustion and quota eviction produce explicit recovery or gap records.
4. Every stop retains the initiating cause, relevant ages/limits and causal links rather than only a generic safety label.
5. Logging failures do not alter control outputs or command freshness.
6. A Rerun recording opens with maps, poses, control curves and events aligned; frame resets, delayed uploads and clock skew are represented honestly.
7. Authentication, scope isolation, redaction, rate limits and export access are tested.
8. Software tests and builds are reported separately from physical rover acceptance. Physical checks require an operator-controlled session.

## Implementation sequence

Define schema and coverage inventory; implement durable journals and fault tests; add producer instrumentation; implement collector and retry/ack protocol; add evidence attachments and Rerun exporter/layout; validate end-to-end replay; prepare deployment and credential provisioning; perform operator-led field acceptance. A detailed implementation plan follows written-spec approval.

## References

- https://rerun.io/docs/getting-started/data-in — explicit timelines, file saving and loading.
- https://rerun.io/docs/concepts/logging-and-ingestion/recordings — recording identity and merging.
- https://rerun.io/docs/concepts/logging-and-ingestion/rrd-format — recording persistence format.
