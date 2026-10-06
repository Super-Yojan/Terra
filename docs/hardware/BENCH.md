# Optional future Bluetooth actuator bench

Not performed. The user authorized implementation only and prohibited tests,
builds, smoke scripts, screenshots and hardware checks. This procedure is a future
commissioning outline requiring separate authorization; it is not an acceptance
result. See [evidence status](evidence/README.md).

## Prerequisites and wiring

Record the Git revision, Pi/board revision, Linux/BlueZ/Python/dbus-next versions,
phone/iOS version and installed SunFounder `fusion_hat` source. The adapter accepts
1.14.0 by default; API/source checks do not prove electrical compatibility. A future
build phase must generate ignored UniFFI artifacts and establish app compilation
before a phone session. Follow [owner setup](BLUETOOTH.md) for the encrypted
read-only provisioning service, local numeric confirmation, owner persistence,
setup disconnect and restart into normal service.

Use raised wheels, restrained equipment, appropriate fused supplies and a reachable
independent physical cutoff. Start with propulsion power isolated. Follow the board
and actuator wiring specifications for power, logic voltage, signal reference,
current capacity and servo travel. Do not power propulsion from a signal pin. The
cutoff must remove propulsion power or independently disable the driver even if
the Pi process stops; a gate file is only input to software. Supervise its producer:
`0` explicitly means open/isolated, `1` closed; unavailable/invalid input forbids
motion and configuration. Do not fabricate a software gate for real equipment.

Before attaching an ESC, use an instrument to establish actual 50 Hz pulse width,
voltage and approved stop/neutral range with propulsion disconnected. Zero effort
requires continuous neutral/stop pulses, not zero pulse width. Pulse continuity
after service termination is unproven and cannot be guaranteed by this process.
Isolate the downstream equipment before shutdown/restart. Servo safe position can
itself cause movement; choose it with linkage and travel constraints in mind.

## Proposed sequence, with no recorded results

1. Inventory exposed connectors and all resource owners. M0 aliases P11/P10,
   M1 P9/P8, M2 P6/P7, M3 P4/P5; motors use 100 Hz. PWM timer groups are P0–P3,
   P4–P7, P8–P11. Reject aliases and mixed 100/50 Hz users of a group. Confirm
   `--pwm-ports` against physical connectors rather than the numeric channel range.
2. With cutoff open and service disarmed, read capabilities and layout. Replace
   `SELECT_*` markers in [examples](../../crates/terra-actuators/presets/) and use
   the current active revision. Stage with a fresh numeric request ID, then commit
   referencing that exact `staged_request_id` and `staged_revision`. Record replies,
   active revision and `hardware_gate_open_confirmed`; re-read with fresh IDs.
3. With power isolated, measure DC safe output, ESC stop/neutral and servo safe
   position/disabled PWM. Measure configured endpoints without powering propulsion;
   determine approved actuator calibration before enabling power. Confirm inversion
   exactly once and shared-timer behavior. Example 1000/1500/2000 µs values are
   placeholders for calibration decisions, not universal actuator specifications.
4. Under controlled power, close the independent gate without arming. Observe
   whether outputs remain safe. Explicitly arm after a fresh complete safe DRIVE;
   distinguish transport acknowledgement from command acceptance and armed status.
   Keep fresh safe commands during the configured ESC interval, then apply small
   fresh complete commands and record sequence/revision/session and measured output.
5. Separately observe command loss, incomplete/invalid owner frames, disconnect,
   status unsubscribe, phone background, stop and tracking loss. The designed target
   is expiry at age >=200 ms and a safe output request within 210 ms while service
   and I/O run. Measure receiver/output timing; source review does not establish it.
6. Exercise independently opened cutoff, unavailable gate input, partial backend
   failure and emergency stop only with a controlled fault-injection arrangement.
   Record every channel's safe behavior and fault/status changes. Reset while
   disarmed and confirm reset/gate closure does not itself arm.
7. Isolate propulsion before process restart or owner replacement. Observe startup,
   re-admission, fresh session, explicit rearm and rejection of old commands/stages.
   Keep physical output measurements distinct from Linux mock and transport records.

For each authorized future step record equipment, configuration, timestamp method,
expected behavior, actual observation, pass/fail or blocked reason, raw evidence
location and operator. Do not mark skipped steps passed. No procedure here certifies
hardware, radio operation, timing, or issue acceptance criteria.
