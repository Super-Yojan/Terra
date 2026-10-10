# L2/L4 implementation ledger

L2 wire value: waypoint_direct; bypass map/planner only, retain sensor and motor authority safety. L4: local Vision person detection with measured scene depth, existing confirmation and bounded search.

Ruling: detector availability requires current tracked depth frames; no inferred distances, no target pursuit.

Verification: L2 regression observed fail before enum addition. Physical LiDAR tests require hardware.

## Operator instructions

Install the updated Terra phone app and ARGOS together. The actuator peripheral binary does not change for this release. Older phone runtimes do not advertise `waypoint_direct`, so ARGOS keeps L2 unavailable for them. All devices connect to the same fleet router as before.

L2 follows a waypoint directly and does not use mapped obstacles to avoid or stop. L3 continues using occupancy and avoidance. Both retain motor arming, sensor freshness, cancellation, and stop authority; dashboard communication tolerance does not relax onboard tracking freshness.

For L4, use a LiDAR-equipped iPhone/iPad with Terra camera tracking active. The mounted camera must see the search area, and scene depth must be available. Choose L4, target class `person`, local search bounds, and a time budget. A confirmed person stops search and produces a location report; the rover does not pursue or identify the person. Detector health is visible in Terra and availability is advertised to ARGOS. Missing or delayed detector data holds the existing search lifecycle and requires operator attention/resume. Other classes are unavailable until another detector is installed.

Vision receives camera orientation and its rectangles are transformed back into raw-camera pixel coordinates before depth projection. Confidence-high depth samples from the central torso region must agree; distance is never inferred from person size. Frames older than 0.5 seconds, failed tracking, missing depth, and previous sensor-session callbacks cannot provide confirmation evidence. One inference runs at a time, at most 5 Hz. Logs report detector health transitions and rejected observations; camera images are not uploaded by this component.

## Verification

Terra autonomy/mobile Rust regressions cover L2 no-map motion, occupied-map bypass versus L3, stale pose, cancellation, latched stop, L4 multi-frame confirmation, duplicate/stale/foreign observations, bounds, detector loss, timeout and report behavior. Swift projection/pacing and manual-route regressions pass. ARGOS tests cover distinct L2 authority and preserving L2 on a delayed-telemetry waypoint dispatch. Terra iOS and ARGOS Mac/iPad builds pass.

Physical LiDAR person-search accuracy, mounting orientations, forest lighting/occlusion, and real actuator behavior have not been tested here. The live ARGOS UI integration test requires a simulator fleet at port 7447 and was not completed; native unit tests run separately without sending real commands.

Final verification: 110 Terra Rust tests, 36 ARGOS Rust tests, 28 ARGOS Mac unit tests; Swift depth/orientation/pacing and manual-route executables passed. Terra iOS and ARGOS Mac/iPad builds succeeded. Independent whole-change review found no concrete defects.
