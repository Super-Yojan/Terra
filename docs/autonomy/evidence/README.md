# Acceptance evidence

The local two-rover run used mission seed 42, the portable arbiter, the simulated PI motor path, depth-derived occupancy, and an ARGOS Rust/Zenoh client on `tcp/127.0.0.1:7448`. It exercised every level, leased held input/release, waypoint arrival, supervised proposal approval, and individually acknowledged fleet takeover/stop. JSON evidence is shared with the companion ARGOS branch at `docs/autonomy/live-control-evidence.json`.

| Terra #17 requirement | Implementation and evidence |
|---|---|
| Identical shared authority in Bevy and phone | `terra-autonomy` is invoked by both adapters; Rust arbitration tests and `scripts/SwiftSmoke.swift` exercise four modes through generated UniFFI bindings. |
| Four levels selectable over Zenoh | Local two-rover live flow; token-correlated status and proposals are periodically published. |
| Safety at every level | Common arbiter holds, circumscribed swept/braking footprint, latched stop/reset, sensor/pose/map health, PI motor limits; core and simulator regressions. |
| Logs in simulator and phone | Versioned bounded JSONL writer with source/selected commands, goal/proposal state, decision events, mission outcomes, clearance episodes, and explicit incomplete/drop state. Phone contact sensing is unavailable, not zero. |
| Arbitration tests | Mode takeover, stale clocks/leases, sensor/map holds, stop/reset, idempotent goals, supervised approval and restart-scoped input/proposals. |

ARGOS #4 has native per-rover authority, takeover/stop, fleet fan-out, keyboard/touch input, proposal decisions, and export. Rust loopback tests cover correlated receipts, ordered release and stale-generation suppression; native model tests cover authority gating. Connected Rust-client acknowledgement latencies in the saved initial run were 0.11–0.54 s. These are transport/runtime timings, not native UI display timings.

The seeded mission component tests verify hidden targets remain absent from published observations, range/visibility gating, duplicate reports, all confirmations, return, and budget failure. A component replay is distinct from a completed operator study trial. No complete physical robot or comparative human study result is claimed.

## Review rulings

- Planner radius is 0.65 m to circumscribe the 0.9 × 0.8 m simulated chassis. Physical braking and geometry need calibration.
- Proposal approval revalidates the latest observed free map; it does not require the map revision to remain unchanged during sensing.
- Map-observed area is separate from confirmed survivor search effectiveness.
- Keyboard ordering is per key with a clear fence; captured authority/run generations prevent queued events crossing takeover or restart. All teleop releases advance the sender sequence. Distinct validated stops are retained and drained after ordinary intent.
- Both requested/effective authority and fixed trial assignment are recorded. Adaptive switching does not constitute an automatic optimal-autonomy policy.
- Automatic approval review rejected a new unauthenticated listener on all phone interfaces. The phone control-plane implementation enforces loopback; physical-phone LAN hosting remains unimplemented pending that boundary decision.

Native device focus/touch behavior, physical braking, complete fixed/adaptive trials, and workload questionnaires remain empirical study work. The implementation supplies mission outcome and interaction proxies without presenting them as measured human workload.
