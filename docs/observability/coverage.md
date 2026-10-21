# Operational event coverage

This inventory distinguishes installed instrumentation from planned coverage.

| Source | Event | Capture site | Verification |
| --- | --- | --- | --- |
| Phone Rust | lifecycle.start | MobileController::start_observability | mobile observability integration tests |
| Phone Rust | autonomy.transition | observation tick after final status creation | stop_retains_first_cause |
| Phone Rust | control.summary (10 Hz) | observation tick | logging_failure_does_not_change_control |
| Phone Swift | motion.pause/disarm | initiating pauseHardwareMotion/revokeHardwareMotion callers | Swift event sink tests; physical check pending |
| Phone Swift | hardware.state | refreshBluetooth | changed-state suppression Swift test |
| Phone Swift | connection.requested/state | connectDashboard/updateAuthority | integration build and field check pending |
| Phone Swift | lifecycle.prepare/stop/failure | controller lifecycle | build and field check pending |
| Pi | motion.disarm/pause/armed | SafetyController transition boundary | Pi observability tests + existing safety tests |
| Pi | command.accepted/rejected | SafetyController::accept | existing protocol/safety regressions |

ARGOS/simulator/router sources, synchronized imaging, delivery, export and cloud
packaging are implemented below. Hardware acceptance has not run.
Phone logging initializes on a utility queue; early events before initialization
can be unavailable. Diagnostic logging never changes controller health.

Completed instrumentation also includes:

| Source | Event / evidence | Capture site |
| --- | --- | --- |
| Phone | command.received/admission | dashboard action consumption; retains age and correlation fields |
| Phone | command.dispatched/rejected | local autonomy request outcome |
| Phone | decision.event | every drained arbiter decision event |
| Phone | control.output | final selected motor output, sampled alongside summaries |
| Phone | mapping.evidence | accepted ARKit map update: exact depth, confidence-derived validity, pose, calibration and ground |
| Phone | occupancy-only evidence | publishMap snapshot at configured map rate |
| ARGOS | operator/session/state/command events | existing session evidence projected into the independent durable journal |
| Zorvane | autonomy.transition/event and control.summary | final simulator control-tick evidence |
| Zorvane | mapping.evidence and snapshots | accepted exposure-aligned integration and configured map sampling |
| Zorvane | camera.receipt_only | existing GPU RGB readback, explicitly unpaired with depth |
| Router host | service.journal | allowlisted router/TLS service journal, stable cursor identity |
| Pi forwarder | service.journal / original structured events | independent allowlisted journal reader; own queue and HTTPS uploader |
| Uploader | upload.failed/recovered; observability.health; clock.alignment | background upload worker, independent of driving |

The inventory covers operational events, decision transitions and sampled state.
It does not claim instruction-level tracing, all OS logs, full-rate image/depth or
an explanation of every rejected planner candidate beyond the emitted planner
trace. Complete source journals and explicit gap records remain authoritative.
