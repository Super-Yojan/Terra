# Terra Level-of-Autonomy Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans after implementation is requested. Track steps with the checkboxes below. This plan is a proposed design; it does not authorize implementation.

**Goal:** Make four per-rover autonomy levels selectable at runtime, route every motion request through one shared Rust arbiter in Bevy and TerraPhone, and record enough experiment data to compare levels.

**Architecture:** Add portable navigation and autonomy crates alongside the existing waypoint, mapping, estimator, and velocity-controller crates. Platform adapters supply normalized input, invoke the same arbiter, and feed its sole selected output to the existing velocity controller. A shared event schema and off-thread Rust JSONL writer record decisions and platform observations; ARGOS supplies native level/proposal controls.

**Tech stack:** Rust, existing Zenoh and UniFFI conventions, SwiftUI for TerraPhone and ARGOS, Cargo tests, Swift binding/adapter tests. No Python tools or runtime are introduced.

**Spec:** https://github.com/Super-Yojan/Terra/issues/17. Companion requirements: https://github.com/Super-Yojan/ARGOS/issues/4. Dashboard implementation plan: ../../../ARGOS/docs/superpowers/plans/2026-10-06-autonomy-operator-controls.md. Related native dashboard: https://github.com/Super-Yojan/ARGOS/pull/5. Proposal topics and decisions below require review before implementation; the issue does not yet define a complete wire contract.

## Implementation status

Implemented on `codex/mission-autonomy`. The [acceptance evidence](../../autonomy/evidence/README.md) maps the delivered runtime, mission, logging, phone binding, and companion dashboard checks. Original task checkboxes combine implementation with broader physical/study demonstrations; those demonstrations are not inferred from builds and remain explicitly listed in the evidence document.

## Findings from the current code

Inspected Terra HEAD `79d6951`. `terra-control` provides a portable PI velocity controller with speed/effort limits, sensor-health stops, and target timeout. `terra-waypoint` supplies the shared geometric waypoint follower; `terra-mapping` supplies a rolling occupancy grid. There is no DWA planner or frontier explorer in these crates yet.

The simulator currently chooses motion through separate `apply_remote_commands` and `drive_goals` systems in `simulator/src/zenoh_bridge.rs`, then consumes `DriveCommand` in `simulator/src/velocity_controller.rs`. TerraPhone chooses teleop versus waypoint in Swift `PhoneController.waypointCommand`, then calls `MobileController`. Sharing the follower is already useful, but control-authority selection is currently duplicated. Both selection paths must be replaced, not supplemented with a third writer.

The simulator already maintains a per-rover occupancy map in `simulator/src/occupancy_map.rs`; phone mapping uses the same Rust mapper. Experiment-quality collision/near-miss logging and run manifests are new work. Simulator ground truth and phone observations must be distinguished in analysis.

## Mission-first simulation and autonomy evaluation

The research question is how much autonomy is needed to complete a mission under specific conditions, rather than whether four mode buttons work. Use a disaster search-and-rescue scenario as the initial reference mission: search assigned sectors, locate simulated survivors, confirm/report their locations, and return safely within a time/resource budget. Battlefield-relevant conditions can inform terrain, visibility, communication loss, and operator load; weapon engagement is outside this dashboard/navigation scope.

A mission definition records objectives, success/failure criteria, starting positions, search boundaries, survivor placements, traversability/hazards, visibility/depth availability, communication profile, rover count, time budget, and scenario seed/version. Use traversable corridors, rubble chokepoints, blocked routes, open search areas, and degraded visibility with reachable objectives. Separate the simulator's hidden ground truth from rover observations and operator-visible information; no planner or dashboard may discover survivors through ground-truth access. Define detection/confirmation rules and sensor range/occlusion explicitly before measuring search effectiveness.

Each autonomy level serves a mission need: teleop for precise operator-directed movement, assisted teleop for maneuvering with obstacle support, waypoint for transit to operator-selected locations, and supervised exploration for searching unobserved space while the operator supervises priorities. Frontier exploration is a navigation policy, not survivor detection or mission completion. Add mission observations/reporting as a distinct simulator capability; it must not silently alter the navigation arbiter or expose hidden information.

Compare two trial designs separately:

- **Fixed assigned-level trials:** identical mission conditions/configuration across levels, with multiple recorded seeds and balanced trial order. Takeover and emergency stop remain available and are logged as deviations from the assigned condition. These trials estimate the effect of a level; they do not prove an optimal switching policy.
- **Adaptive trials:** operators may switch levels to meet mission demands. Record requested/effective level, mission phase, observable conditions, and operator-stated reason when supplied. Switching remains immediate; reason entry is optional and never blocks takeover/stop. Compare adaptive runs against fixed-level baselines; distinguish operator choice from safety holds or automatic changes.

Evaluate objective completion, confirmed discoveries/false reports, time to first discovery and mission completion, searched observable area, distance/resource use, collision/near-miss exposure, safety stops, interventions, and operator active-control duration. Record denominators and unavailable measurements. Predefine success and an acceptable safety/performance threshold before trials; report the lowest tested autonomy level meeting it per condition, with uncertainty, rather than claiming one universal required level. Interaction counts alone do not measure workload; add a separate validated workload measure if that is a thesis outcome.

Add a mission configuration/manifest and deterministic search/report events in `simulator/src/mission.rs`, scenario fixtures under `tests/fixtures/missions`, and mission metadata/events in `terra-experiment`. Test reachable objectives, seeded reproducibility, occluded detection, duplicate reports, completion criteria, and separation of ground truth from telemetry. ARGOS displays the mission objective, progress, phase, constraints, and reported observations alongside autonomy controls. Mission machinery is a linked deliverable needed for the research study; it expands beyond the control-only acceptance criteria of Terra #17.


## Proposed behavior

| Level | Selected motion | Required inputs | Operator actions |
| --- | --- | --- | --- |
| Teleop | Fresh operator twist, subject to common safety | Healthy velocity estimate; fresh operator lease | Drive, stop, change level |
| Assisted teleop | Closest collision-admissible twist to operator intent through DWA | Teleop inputs plus fresh pose/map | Drive; see intervention reason; take over or change level |
| Waypoint autonomy | Shared waypoint intent passed through the shared local planner | Accepted goal, healthy estimate, fresh pose/map | Set/redirect goal, cancel, explicit takeover |
| Supervised autonomy | Frontier proposal, then approved target executed through the same waypoint/planner path | Waypoint inputs plus reachable frontiers and approval | Approve/reject proposal, redirect, cancel/pause, explicit takeover |

All modes use the same estimator, motor controller, speed/acceleration limits, e-stop behavior, clock validation, and controller tuning within a run. Selected/requested level, active command source, and safety state are separate fields. A safety stop does not silently relabel the assigned experiment condition as teleop.

### Authority and transition rules

- Safe startup is teleop with zero output. Existing goal-only behavior changes: explicitly select waypoint autonomy before sending a driving goal. Update demo/tests and ARGOS accordingly.
- Level changes acknowledge a request token and monotonically increasing revision. Reject unsupported/invalid levels; do not claim level 2 or 4 is available until its planner exists.
- Switching levels clears previous motion leases and unapproved proposals and resets velocity-controller integrators. Existing goals do not resume automatically on returning to an autonomous mode; a fresh goal/approval is required. Record discarded intent in the log.
- Teleop packets never implicitly preempt a waypoint. Takeover is an explicit set-level-to-teleop request; it clears autonomous intent. A subsequent fresh twist is required, preventing an old lease from moving the rover.
- In levels 1/2, a goal request is rejected with an explicit reason rather than moving or silently changing the level. In levels 3/4, an operator goal redirects the rover and is logged as operator intent; it does not change the level.
- Preserve existing waypoint cancellation expectations in level 3: cancel clears the goal, transitions to teleop, and emits both goal and level-change events. In level 4, cancel clears the goal and holds supervised autonomy paused until explicit resume or a new operator goal; no immediate replacement proposal/motion.
- Assisted teleop with no admissible trajectory outputs zero with `obstacle_blocked`. Missing/stale planner inputs stop levels 2–4 with a reason; teleop still requires healthy controller sensors but does not require an occupancy map.
- E-stop is latched across levels and reconnections. Reset is explicit and permitted only with healthy required safety inputs; it never restores old motion intent. The hardware watchdog remains independent and final.
- Invalid/nonmonotonic time, nonfinite inputs, stale estimates, and stale teleop leases never refresh a valid command. Source timestamps are recorded, but local monotonic receive times govern network freshness.

### Shared navigation

Add `terra-navigation` for DWA and frontier selection, using existing `MapSnapshot` (-1 unknown, 0–100 occupancy). Keep planner configuration identical across platforms and levels during a run. Candidate sampling, tie-breaking, and scoring are deterministic.

DWA samples reachable velocity pairs under acceleration limits, simulates swept robot footprints, and rejects collisions and trajectories that cannot brake safely within observed free space. Include the current-to-zero braking trajectory. Assisted teleop minimizes departure from operator intent; waypoint modes score progress/heading plus clearance. Both share the same admissibility checker. Unknown/out-of-grid space is not certified free; include the robot's known present footprint as traversable so an unobserved current cell does not deadlock every plan.

Proposed initial planning parameters for review, not empirically validated safety claims: 10 Hz replanning, 2 s rollout horizon, <=0.1 s rollout steps, map/pose freshness 0.5 s, occupancy >=65 considered blocked. Obtain footprint dimensions from the rover configuration. Record acceleration, braking, inflation, speed/yaw caps, map resolution, and every scoring weight in the run manifest. Calibrate stopping-distance and clearance margins before physical trials.

Frontiers are free cells adjacent to unknown cells. Choose a reachable free-space vantage near a frontier using a shared grid search; use footprint-inflated collision constraints. Rank by deterministic travel-cost/information-gain scoring; never put a goal inside unknown space. Emit one pending proposal at a time. Approval revalidates the proposal against current pose/map; stale, superseded, unreachable, or already-decided proposals cannot launch motion. Rejecting a frontier applies a configurable cooldown rather than proposing it continuously. No reachable frontier means hold with an explicit reason, not fabricate motion.

A depthless phone cannot support honest assisted/waypoint/supervised planning: report the capability/input failure and hold those levels. Do not substitute simulator ground truth for depth-derived maps in experiment runs.

## Proposed Zenoh contract

Treat `/autonomy` as the request topic; `/autonomy/status` is the authoritative retained-in-memory state, republished after changes and at least once per second for late subscribers. Application-level latching does not assume Zenoh history/replay storage is configured.

| Key suffix | Payload / role |
| --- | --- |
| `/autonomy` | `{"level":"assisted_teleop","token":"request-1"}`; levels `teleop`, `assisted_teleop`, `waypoint`, `supervised` |
| `/teleop` | `{"linear":1.0,"angular":0.2}`; finite twist with 500 ms receive lease; optional `operator_session_id` UUID and nonnegative `sequence` for log correlation; legacy two-field packets remain valid |
| `/autonomy/status` | `assigned_level` (optional run condition), `requested_level`, `effective_level` (null while held), `active_source`, `safety`, `reason`, `revision`, `token`, `result` (accepted/rejected), `supported_levels`, `paused` |
| `/goal` and `/goal/status` | Preserve existing local/WGS84/cancel encoding and goal correlation; add explicit command rejection feedback through autonomy status without pretending rejected goals arrived |
| `/goal/proposal` | Proposal ID, local target and optional geographic coordinates, map revision, reason, expiry time relative to this run |
| `/goal/decision` | `{"proposal_id":42,"decision":"approve","token":"decision-1"}`; also `reject`; resume uses `{"decision":"resume","token":"resume-1"}` with no proposal ID |
| `/experiment/status` | Run UUID, schema version, run-relative monotonic time, UTC sample time, recording state; publish on run changes and at 1 Hz for operator log alignment |
| `/safety` | `{"action":"stop","token":"stop-1"}` or `reset`; operator emergency stop and guarded reset |

Status `active_source` is `none`, `operator`, `assisted_operator`, `waypoint`, or `frontier`. Requested level remains latched during a safety hold; effective level is null and active source is none. Assigned level comes from the run manifest and does not change on an operator takeover. Every acknowledged request reports its token/result independently of current authority.

Fleet takeover and emergency stop fan out to per-rover keys; there is no atomic fleet acknowledgement. Takeover clears intent even when already in teleop. Emergency stop takes priority over all queued motion requests. Both acknowledge each target independently. ARGOS targets its last-known roster for emergency stop, including stale members, and reports missing acknowledgements as unknown. All keys are per rover under the configured prefix. Requests cap at 2048 bytes; state cap at 65,536 bytes; tokens follow the existing 1–64 ASCII goal-token alphabet. Reject unknown request fields, booleans masquerading as numbers/IDs, nonfinite values, and inactive rover IDs. Repeated request tokens are idempotent within a bounded cache; acknowledgement loss does not cause another authority transition or intervention count.

Compatibility: `/cmd_vel` remains a deprecated alias feeding the same leased operator inbox, subject to the selected level. It never bypasses the arbiter. `/teleop` is the new canonical key. Remove direct writes to motor intent from goal, keyboard, Swift, and transport callbacks. No automatic command replay on reconnect.

## Interfaces and files

Create `crates/terra-navigation/src/{lib,dwa,frontier}.rs` and focused tests. API: `LocalPlanner::plan(PlanningInput) -> PlanningDecision`; `FrontierExplorer::propose(ExplorationInput) -> Option<GoalProposal>`. Decisions include candidate motion, admissibility, clearance, and reason.

Create `crates/terra-autonomy/src/{lib,arbiter,contract,events}.rs`. API: `AutonomyArbiter::new(AutonomyConfig)`, `set_level(LevelRequest, now)`, `accept_operator(Twist, received_at)`, `accept_goal(GoalCommand, now)`, `decide_proposal(ProposalDecision, now)`, `set_emergency_stop(bool, now)`, and `step(ArbiterInput) -> ArbiterOutput`. Input carries current estimate, pose/map with timestamps/revision, safety state, and monotonic time. Output carries exactly one selected `VelocityTarget`, goal/proposal/status, and bounded decision events. No Bevy, Swift, filesystem, or Zenoh dependency in this crate.

Keep waypoint latching/following and planner selection inside the arbiter/composed runtime, so neither adapter recreates their ordering. Stop outputs reset controller integrators and use the existing stop paths; passing a zero setpoint must not disguise an e-stop or unhealthy estimate as normal active control.

Create `crates/terra-experiment/src/{lib,schema,writer,metrics}.rs`. `RunManifest`, `RunEvent`, and `RunRecorder` serialize versioned JSONL. Recorder owns a bounded off-thread queue and reports I/O/backpressure failures. Pure event generation stays in the shared autonomy/runtime; platform callbacks only supply normalized observations.

Modify workspace Cargo.toml, `crates/terra-types/src/lib.rs` as needed for neutral value types, `crates/terra-mobile/src/lib.rs`, add `crates/terra-mobile/src/autonomy.rs`, and extend `crates/terra-transport` for autonomy requests/status/proposals.

Modify `simulator/src/zenoh_bridge.rs`, `velocity_controller.rs`, `occupancy_map.rs`, `main.rs`, and add `simulator/src/autonomy.rs` plus `experiment.rs`. Modify `mobile/ios/TerraPhone/PhoneController.swift` and `ContentView.swift`; regenerate UniFFI bindings using existing build scripts. Add shared deterministic replay fixtures under `tests/fixtures/autonomy`.

ARGOS companion: extend Rust contract/state/Zenoh/FFI crates and shared SwiftUI rover detail/settings with the level selector, pending/confirmed mode, effective authority/safety reasons, proposal approval/rejection, pause/resume, takeover, and e-stop/reset. No planner or arbiter is implemented in ARGOS.

## Experiment log requirements

- Manifest: schema version, run UUID, rover ID, platform, git revisions, assigned experiment condition, task/seed, world/anchor, all planner/map/controller/safety settings, and sensor/collision-observation capabilities. Platform-specific calibration is explicit and fixed across LOA conditions.
- Correlate operator logs using command tokens and optional teleop session/sequence metadata. Publish `/experiment/status` so ARGOS records rover run IDs and paired time samples. Never compare monotonic clocks across machines directly; retain UTC and alignment uncertainty. Operator requests and rover receipts are distinct events, not duplicate interventions.
- Every event: run/rover ID, monotonic run-relative time, sequence, event kind, requested/effective level, active source, and reason. Wall-clock UTC is metadata for joining logs, not arbitration time.
- Record level request/accept/reject, operator input receipt, source commands (operator, waypoint, policy), chosen/safety-limited output, intervention/takeover/approval/rejection/redirect, goal lifecycle, proposal lifecycle, collision/contact observations, clearance-based near-misses, and sensor/map/logging faults.
- Log selected output at each control tick; log level/event transitions exactly once. Proposal rejection, takeover, and goal redirect are separate intervention categories; do not count every teleop packet as an intervention. Derive rates from event definitions and active-run exposure time.
- Use one clearance-based near-miss definition on both platforms, with configurable threshold, hysteresis, and onset/end events. Simulator contacts are ground truth; phone collision sensors/manual observations are separate tagged evidence. Missing phone collision sensing is `unavailable`, never zero collisions.
- Proposed bounded writer capacity: 8192 events, batched flush <=1 s, explicit flush/close at run end, recoverable truncated-tail handling. Control must never block on disk. In experiment mode, queue overflow/I/O failure marks the run incomplete and requests a safe hold; dropped events/counts must not be hidden.
- Phone records to its sandbox with a share/export action; simulator writes to a configured run directory. Core schemas must match. Workload proxies are interaction counts/approval latency/control duration, not a claim of measured human workload; collect subjective workload separately if the study requires it.

## Task 1: Freeze the control-authority contract

- [ ] Add Rust enum/record/codecs and protocol documentation for four levels, status, decisions, and stop/reset, optional teleop correlation metadata, and experiment status. Specify supported-level reporting and mode/goal transition rules above.
- [ ] Write tests for valid shapes, boundary values, tokens, malformed payloads, duplicate-request idempotence, and inactive IDs; observe failures before implementation.
- [ ] Verify no other adapter/API relies on goals silently selecting autonomous authority. Update documented startup/demo flow to explicit waypoint mode.
- [ ] Deliver independently testable codecs and transition specification; commit.

## Task 2: Shared collision-admissible local planner

- [ ] Write deterministic tests for clear space, frontal/side/reverse obstacles, swept footprint during turning, stopping distance, acceleration limits, unknown/out-of-bounds space, current footprint, and stale map/pose.
- [ ] Implement one admissibility checker plus assisted-intent and waypoint scoring. Identical inputs/config produce identical commands; no admissible command means zero and reason.
- [ ] Test candidate tie-breaking and computational bounds at the configured map/candidate sizes; benchmark on a target phone before claiming 10 Hz capacity.
- [ ] Run `cargo test -p terra-navigation`; deliver working assisted/waypoint planning with no platform integration yet; commit.

## Task 3: Frontier proposals and supervised goal decisions

- [ ] Test frontier detection, reachable free-space vantage selection, no-frontier hold, rejection cooldown, one pending proposal, expiry, stale-map revalidation, and duplicate/out-of-order approval.
- [ ] Implement deterministic proposal generation and approval/rejection/pause/resume state. Approved/operator-directed goals feed the existing shared waypoint follower and Task 2 planner.
- [ ] Test that proposals alone never generate motor motion and redirected goals cannot be resumed by old approvals.
- [ ] Run navigation/exploration tests; commit.

## Task 4: Shared arbiter and common safety gate

- [ ] Write a table-driven matrix covering all four levels, each command source, fresh/stale leases, missing estimates/maps, e-stop/reset, invalid clocks, and mode transitions.
- [ ] Add regression tests for takeover with an old teleop lease, stale proposal approval, cancel in waypoint versus supervised mode, stale goal output, disconnected operator during a latched autonomous goal, repeated takeover while already teleop, stop concurrent with queued motion, and no automatic resume after level change/reset.
- [ ] Implement one runtime composing authority, follower, planner, exploration, and common safety. Active source and safety reason remain distinct from requested/assigned LOA.
- [ ] Assert every output obeys configured speed/acceleration/effort boundaries through the controller, and stop reasons reach the final motor path. Test healthy input restoration never clears a latched e-stop.
- [ ] Run `cargo test -p terra-autonomy -p terra-navigation -p terra-control`; commit.

## Task 5: Versioned experiment events and recording

- [ ] Test manifest serialization, deterministic event order, monotonic time, exactly-once transitions, intervention categorization, near-miss hysteresis, unknown collision capability, and metric denominators.
- [ ] Implement the bounded JSONL writer and shared metrics reader. Test full queue, disk failure, interrupted final line, flush/close, and safe-hold/invalid-run behavior without blocking the control tick.
- [ ] Use a known synthetic trace to assert intervention count, active-run duration, task completion time, and approval latency. Reject incomplete runs from complete-data summaries.
- [ ] Run `cargo test -p terra-experiment`; commit.

## Task 6: Integrate Bevy with one arbiter writer

- [ ] Replace existing remote/goal output competition with per-rover input inboxes and a single fixed-step arbiter system feeding the velocity controller. Normalize keyboard/Space inputs through this path too; the disabled/ideal-drive debug path must not be used in experiment runs.
- [ ] Subscribe/publish agreed autonomy/teleop/proposal/decision/safety/status keys. Preserve cmd_vel alias without bypass; retain and periodically republish authoritative application state.
- [ ] Supply depth-derived occupancy, estimate/pose health, normalized contacts, and logging state. Record source timestamps and local control time; do not mix `Instant` values with Bevy simulation time without conversion.
- [ ] Integration-test all four runtime selections, stale-input stops, goal progress, takeover, rejection, cancelled-goal behavior, late client connection, and per-rover isolation. Test fleet fan-out with one unreachable rover and independent acknowledgements; publish experiment status/time anchors.
- [ ] Demonstrate an obstacle intervention in assisted mode and proposal -> approval -> motion in supervised mode. Run simulator tests from `simulator` and save logs; commit.

## Task 7: Integrate TerraPhone through UniFFI

- [ ] Add owned UniFFI autonomy/status/proposal/event records and a mobile runtime using the same arbiter. Preserve sensor ingress and controller tuning; delete Swift's `waypointCommand` authority decision.
- [ ] Phone simulation/physical control calls the Rust runtime; remote-supervisor mode sends intent over Zenoh and observes the simulator-owned arbiter, rather than applying a second phone arbiter to the same rover.
- [ ] Test generated Swift bindings against deterministic traces and the same config as Rust/Bevy: selected targets, safety states, authority transitions, and core decision events must match. Compare input-to-selected-target/controller behavior rather than claiming identical simulator and phone sensor/physics outputs.
- [ ] Add native mode/stop/proposal UI, sandbox run log creation, and export. Test tracking loss, missing depth capability, app suspension, logging failure, and guarded e-stop reset.
- [ ] Build device/simulator slices and run Swift checks. Physical-device validation is explicit evidence, not inferred from a simulator build; commit.

## Task 8: Mission scenarios and research instrumentation

- [ ] Define the reference search-and-rescue mission, detection/report rules, success/safety thresholds, and fixed versus adaptive trial protocols before implementing mission logic.
- [ ] Add seeded scenario fixtures and failing tests for reachability, occlusion, duplicate reports, observable search coverage, and completion/failure conditions.
- [ ] Implement `simulator/src/mission.rs` and mission observation/report telemetry with strict ground-truth separation; extend experiment manifests/events with scenario, phase, conditions, and outcome measurements.
- [ ] Run deterministic scenario/replay tests; demonstrate that an operator can complete the mission and export outcome evidence. Commit mission instrumentation separately from the arbiter.

## Task 9: ARGOS operator controls and acceptance evidence

Execute the companion ARGOS plan for issue #4 after the shared contract is finalized. It covers native mode controls, per-rover/fleet takeover and emergency stop, held-key teleop with release/focus-loss stops, supervised proposals, and session log export. Gamepad support is optional; keyboard teleop is required.

- [ ] Extend ARGOS contract/state/FFI for the finalized topics and token acknowledgements. Test rejected/stale mode requests, external level changes, missing capabilities, proposal replacement, and stale approval.
- [ ] Add SwiftUI level picker and separate assigned/requested/effective/source/safety display. Confirm level from Terra status rather than optimistically showing a submitted request as active. Expose takeover, stop/reset, and supervised decisions.
- [ ] Verify connected mode changes are acknowledged/displayed within approximately 1 s, fleet actions expose partial failures, keyboard release/connection loss stops teleop, and exported operator events join rover logs by token/session/run ID.
- [ ] Run native UI flows selecting each level and observing representative behavior on one simulated rover; save screenshot/log evidence. Rust core stays portable and Python-free.
- [ ] Replay shared fixtures through Bevy and UniFFI; compare authority output and core events. Record one complete run per level with identical task/map/controller configuration and documented initial state.
- [ ] Map all five issue acceptance criteria to code/tests/evidence. Update Terra README, docs/MOBILE_CONTROL.md, simulator/ZENOH.md, and ARGOS setup docs; commit separately in each repository.

## Delivery and completion

Suggested staged PRs: portable navigation/arbiter with tests; experiment recording plus Bevy/TerraPhone adapters; ARGOS controls and acceptance evidence. Do not close #17 after an enum-only mode selector: all four levels need operational behavior and logs on both platforms.

Completion requires one shared Rust authority implementation, all four runtime Zenoh levels, common safety at every level, matching run-log schemas, arbitration tests and adapter replay evidence. Physical collision observations and planner braking margins need explicit validation before physical experiment claims. Continuous blending and multi-operator arbitration are separate work. Study trial balancing and workload questionnaires belong to the research protocol; mission outcome instrumentation is part of the linked study deliverable.
