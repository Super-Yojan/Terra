# Phone visualization publication

Rust `terra-mobile` caches occupancy and point-cloud visualization content per
dashboard attachment. This is private to the phone crate; no shared protocol
dependency or simulator API change is required.

Existing full topics and schemas are unchanged. Occupancy compares every cell
(including unknown/free/occupied transitions), dimensions, resolution, origin and
run ID. Clouds compare the complete parsed JSON content excluding only `sequence`,
including every point and `frameID`. There is no spatial tolerance or sampling.
The original cloud string is forwarded for full publications; no full JSON is
serialized to detect changes. Swift still constructs its input JSON.

First observations, changed content and the first fresh observation at least two
seconds after the last full publication send a full baseline with the newest
source sequence. All dashboard install/connect/reconnect paths reset both gates.
Unchanged fresh observations can send:

* `<prefix>/<id>/map/occupancy/heartbeat`:
  `{"schema_version":1,"rover_id":<id>,"run_id":<last-full-run>,"sequence":<last-full-sequence>}`
* `<prefix>/<id>/pointcloud/heartbeat`:
  `{"version":1,"rover_id":<id>,"frameID":<last-full-frame>,"sequence":<last-full-sequence>}`

Heartbeat minimum intervals are 200ms for occupancy and 500ms for clouds.
Suppression never advances the last-full sequence. Occupancy source observations
are identified by strictly increasing revision within the explicit run ID;
clouds by strictly increasing sequence within `frameID`. Older samples and
same-sequence conflicts are ignored, including geometry/content changes. A
run/frame change allows a lower sequence and forces a baseline. Occupancy also requires finite, nonnegative source/control
timestamps and a source age in `[0, 0.5)` seconds before observing the gate, so
attachment or recovery cannot republish a stale cached map as fresh evidence.
A stopped sensor cannot renew freshness via cached samples or a
recovery timer. Recovery is sample-driven, not a promise to publish during a stall.
Legacy clouds without sequence/frame identity continue to send full payloads.

Authority, safety, hardware, transitions and command results are not gated. Pose
publication is unchanged: source-aware pose deduplication remains a follow-up,
not part of these heavy-stream caches.

Validate with `cargo test -p terra-mobile` and `cargo fmt -p terra-mobile --check`.
No Swift or UniFFI signature changes are required.

`terra-transport` consumes occupancy/cloud full payloads and their heartbeat
payloads once rather than replaying them on its 100ms worker timer. Other state
topics retain their existing periodic replay, and consumed visualization receipt
timestamps retain the existing 500ms fleet-liveness TTL. Pending observations
remain bounded/latest-wins as before. Validate the transport behavior with
`cargo test -p terra-transport`.

Full publications use acceptance-aware visualization enqueue. A rejected full
clears the gate baseline so the next newer source retries a full baseline, never a heartbeat
for a baseline that was not queued. Occupancy is capped at exactly 2,000,000 bytes on wire, clouds
at 262144 bytes, and ordinary state/heartbeats at 65536 bytes. Valid mobile maps
can exceed the occupancy cap; they remain full/retry-only until a baseline fits.
The source high-water mark survives rejection, avoiding repeated serialization
of an oversized cached source sample on controller timer ticks.

Visualization also matches the dashboard's content envelope before comparison
or serialization: dimensions 1..512, at most 250000 cells, cell values -1..100,
resolution 0.01..2, finite origins and extents, and a nonempty run ID of at most
256 bytes. Unsupported fresh samples invalidate the visualization baseline and
advance its observed high-water mark; the next newer valid sample sends a full
baseline. Cached source revisions skip validation/scanning. These restrictions
do not narrow the existing onboard `autonomy_map` inputs or planner map.

Phone map revisions use a dedicated monotonically increasing counter advanced
by `publishMap`, not the resettable control tick or `LocalOccupancyMap` internals.
It survives UI/control resets and the Bluetooth-manual to phone
`prepare`/preserve-runtime path for the entire `PhoneController` lifetime.
Rust reset drops the dashboard and cached map; reattachment resets gates.
No synthetic run/frame identifiers or shared sequence APIs are introduced here.

The source-level Swift regression checks that publication revision only
initializes once and increments at the map callsite, never in `prepare`:
`swift tests/ios/PhoneMapPublicationTests.swift` (from the repository root).
