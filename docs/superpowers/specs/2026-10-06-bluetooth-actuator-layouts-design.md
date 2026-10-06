# Terra Bluetooth actuator layouts

## Purpose and scope

TerraPhone controls a user-designed Terra chassis over BLE without Bevy. The phone computes actuator commands; the Raspberry Pi applies them through Fusion HAT and independently enforces safety. Keep the existing Rust sensing, control, autonomy, and Zenoh simulator path. Use a Python Pi service to access SunFounder's native library.

Support configurable DC motor, ESC, and positional servo outputs in the first version. terra-mini uses motor ports; NEXT uses ESCs on servo pins. Neither preset defines Terra's required geometry. TurboPi, cameras over BLE, general sensor peripherals, and a new arbitrary-geometry autonomy controller are outside this version.

Status: Tasks 1–8 implemented and independently reviewed by source; Task 9 is
documentation-only. The user explicitly prohibited tests, builds, check scripts,
binding generation, screenshots and hardware checks. The architecture below
describes intended behavior; compilation, runtime safety/timing, radio behavior and
physical compatibility remain unverified. Acceptance criteria are not proven.
Historical test/bench requirements describe a future separately authorized phase.

Execution rulings: vendor source/API support targets `fusion_hat` 1.14.0; PWM
connectors must be explicitly supplied. Real configuration requires confirmed open
physical gate, exposed as `hardware_gate_open_confirmed`. Unknown gate input never
counts as open. Requests use numeric u32 IDs; commit identifies both exact staged
request and revision. Provisioning exports only encrypted read-only status, then
normal service requires explicit restart/reconnect and owner admission. See
[deployment protocol](../../hardware/BLUETOOTH.md) and [evidence](../../hardware/evidence/README.md).

## Components and ownership

- Rust shared code: configuration models, validation, actuator routing, command encoding, and existing phone control calculations. Expose new functions through the existing UniFFI boundary.
- Swift: CoreBluetooth central, discovery and selection, connection lifecycle, configuration editor, arm/disarm controls, and actuator status. Serialize hardware state and reject callbacks from previous connections.
- Python `hardware/raspberry-pi`: BlueZ GATT peripheral using `dbus-next`, configuration persistence, protocol validation, monotonic watchdog, and Fusion HAT adapters. Hardware imports are optional in mock mode.

The Python service mirrors the safety semantics of `terra-motors`; it does not call the Rust crate or infer velocity from effort. Shared fixtures and equivalent timeout/enable tests prevent behavioral drift. The BLE layer hands complete validated frames to a bounded latest-command mailbox; no unbounded command replay queue.

## User-defined layouts

A versioned JSON layout contains a revision and up to 16 actuator entries. Each entry has a unique numeric ID (0–255), name, backend port, output kind, inversion, command limits, and phone routing. The rover advertises usable resources; the phone offers only those resources.

| Kind | Command | Calibration | Disarmed or stale output |
| --- | --- | --- | --- |
| DC motor | Signed effort in [-1, 1] | Maximum power fraction and inversion | Zero power |
| Bidirectional ESC | Signed effort in [-1, 1] | Reverse, neutral, forward pulse widths and neutral arming duration | Continuous neutral pulse |
| Unidirectional ESC | Effort in [0, 1] | Stop and full-power pulse widths and arming duration | Continuous stop pulse |
| Positional servo | Normalized position in [-1, 1] | Minimum, center, maximum pulse widths and safe position | Configured safe position or PWM disabled |

Require a user-selected safe policy for each positional servo. Reject hold-last as the loss-of-link policy in version 1. Writing a safe position can cause motion; bench verification must confirm it is appropriate. All servo/ESC channels use 50 Hz in this version. Do not interpret zero ESC effort as zero pulse width.

The phone supports explicit routing from existing left/right efforts (including duplication across several actuators), configurable bounded forward/turn coefficients for manual effort control, and individual servo sliders. Rust clamps the final commands; the rover independently validates them. Existing differential feedback/autonomy modes are offered only for compatible left/right propulsion routing. Custom layouts can use manual effort mode without claiming closed-loop velocity or autonomous navigation support. Commands carry normalized actuator values, never chassis velocity targets. Servo position is distinguished from motor effort by the committed layout.

Example presets: terra-mini assigns M2/M3 to left effort and M0/M1 to right effort; an ESC template lets the user choose exposed PWM ports and calibration. Presets start disarmed with conservative effort limits. No automatic ESC endpoint calibration or assumed bidirectional ESC behavior.

## Fusion HAT resources

Use `fusion_hat.motor.Motor` for motor power and the supported PWM API for calibrated pulses. Validate the installed library version and expose its board resource map. The published map associates M0 with P11/P10, M1 with P9/P8, M2 with P6/P7, and M3 with P4/P5. Reject overlapping resources, even when names differ (for example M0 and P10). Also reject unsupported channels and incompatible shared timer frequencies. Never infer available connector pins solely from a library's numeric channel range.

The backend adapter converts microsecond pulse widths using the actual supported API. Published PWM examples use 1500 for servo center, while some method descriptions label units as milliseconds; inspect the installed source and verify the waveform on the bench before enabling ESC output. Hardware exceptions latch a fault, trigger best-effort safe output on all channels, and require explicit fault reset while disarmed.

## BLE profile

Use BLE GATT, which fits iOS CoreBluetooth and the Pi's BlueZ stack. Advertise a configurable name (default `Terra Rover`) and a new Terra service; do not reuse the archived RV v2 service for incompatible packets.

UUID prefix: `7e5a`, suffix: `-4c2b-4f91-9e3a-1d8c6b2a0f10`.

| UUID first group | Characteristic | Properties |
| --- | --- | --- |
| 7e5a0010 | Service | Primary |
| 7e5a0011 | Drive | Write with response |
| 7e5a0012 | Status | Read, notify |
| 7e5a0013 | Configuration and control | Write with response |
| 7e5a0014 | Configuration replies and capabilities | Read, notify |

Require encrypted bonded access for drive and configuration. Provision an owner through an explicit local setup command with a BlueZ agent; setup is time limited, disarmed, and verifies the selected phone. Outside setup, accept only the configured owner and one active control connection. Do not assume a peripheral name is identity. The archived unpaired RV v2 controllers are reference code, not the shipping safety/security model.

## Payloads and transactions

Binary drive frames are little-endian: ASCII `TA` (2 bytes), protocol version 1 (u8), kind (u8: drive 1, arm 2, disarm 3, emergency stop 4), session token (u32), layout revision (u32), sequence (u32), then zero or more `(actuator_id u8, normalized_value f32)` records. Drive records must cover every configured actuator exactly once. Reject nonfinite values, unknown or repeated IDs, out-of-range commands, incorrect lengths, wrong revisions, sessions, and non-increasing sequences. Control frames have no records. Retire the session before sequence exhaustion.

The rover issues a fresh session token on connection and publishes it in status. Tokens identify sessions; encrypted owner access provides authentication. Never accept an arm frame until configuration is valid, the hardware gate is closed, status is subscribed, and all propulsion values have been confirmed safe. Arming holds ESCs at neutral/stop for the configured interval and reports arming until completed. Subsequent fresh drive frames activate outputs. Disarm and emergency stop invalidate pending motion; emergency stop remains latched until an explicit reset while disarmed.

Frames may exceed the minimum ATT payload. Use application fragmentation on all writes/replies: message ID u16, fragment index u8, count u8, followed by bytes. Maximum logical drive size is 96 bytes; maximum JSON configuration/reply size is 16 KiB. Permit one assembly per characteristic, expire after 100 ms, and reject duplicate/conflicting/out-of-order fragments. Disarm/stop takes priority and clears incomplete drive assembly. Incomplete frames never refresh the watchdog. Check negotiated write size; never assume a larger MTU. Dispatch only the latest complete frame, stamped at completion, with a connection generation and bounded age.

Configuration/control JSON uses schema version 1 and request IDs. Operations: capabilities, read layout, stage layout, commit layout, reset fault, and reset emergency stop. All replies include request ID, result, active revision, and structured validation errors. Stage and commit require disarmed state. Commit revalidates resources, initializes safe outputs, atomically persists the layout, and increments revision; failure preserves the previous valid layout or faults if hardware rollback fails. Phone routing is saved with the layout so another phone can reproduce it. Drive remains inhibited until the phone reads the committed revision. No partial configuration is applied.

Status at 10 Hz and on transitions includes session, revision, last accepted sequence, armed/arming state, hardware gate, watchdog, emergency stop, fault, and command age. Battery is nullable with an explicit unavailable reason until supported measurement is implemented. Notifications report applied service state, not proof that a physical wheel moved. GATT write completion alone is not an actuator acknowledgement.

## Safety and timing

Send complete actuator frames at 20 Hz with at most one write outstanding and one replaceable pending frame. Keep the original production time in the phone queue; discard samples older than 100 ms instead of retransmitting them as new effort. If transport completion or telemetry stalls, stop sending motion, disarm best-effort, and require operator recovery. Never refresh an old phone effort just to keep the rover enabled.

The Pi independently checks command freshness every 10 ms. A valid frame expires at age >=200 ms measured on the Pi monotonic clock; safe outputs must be requested within 210 ms of the last accepted frame. Delayed local mailbox entries keep their original receive timestamp. Watchdog expiry disarms, and fresh commands alone cannot rearm. Disconnect, open hardware gate, stop, malformed active drive frame, and backend fault request safe outputs immediately on the next motor tick. Configuration traffic and notifications never refresh drive freshness.

The phone stops on background, tracking loss in feedback mode, explicit stop, mode switch, and connection loss. Each reconnect creates a fresh session and requires configuration synchronization and explicit arming; targets reset to safe values. The hardware gate must be able to cut propulsion power independently of Python. Without a configured/readable gate, real-hardware arming is blocked; mock mode supplies a simulated gate. ESC neutral is distinct from physically removing motor power. A Python watchdog cannot cover OS/process hangs or power loss; the 210 ms bound applies to a running service and excludes device I/O stalls. Require an external cutoff and document the actual hardware timing during bench verification.

## Verification and acceptance

1. Rust and Python consume the same wire and layout fixtures: valid layouts, resource conflicts, pulse mapping, turn signs, packet rejection, fragments, revision/session/sequence checks.
2. Fake clock/backend tests exercise boot, gate opening, ESC arming, stop latch, malformed frames, stale samples, exact 200 ms expiry, reconnection, configuration failures, and backend faults.
3. Swift transport tests exercise discovery, notifications, fragmented writes, write backpressure, stale callbacks, background stop, and explicit rearming. Build TerraPhone and run the existing Rust/Swift checks to protect Zenoh.
4. A Python BLE mock peripheral exposes the same GATT service on a Linux BlueZ host with fake outputs. TerraPhone must configure a mixed layout, arm, send commands, receive matching status, and demonstrate watchdog/disconnect stop without Bevy or a Fusion HAT.
5. Physical iPhone/Pi bench: verify port ownership, motor direction, servo limits, measured ESC pulses, neutral arming, hardware cutoff, and timeout with raised wheels. Record library/board versions and measured latency. Host tests do not count as this physical verification.

Issue #10 is complete when the profile is documented, TerraPhone drives a named peripheral, the documented Pi mock/backend applies safety rules, status round-trips without Bevy, and verification clearly distinguishes automated checks from hardware results.

## References

- [Terra issue 10](https://github.com/Super-Yojan/Terra/issues/10)
- [Archived Python Fusion HAT controller](https://github.com/Super-Yojan/RoverApp/tree/archive/rover-controller-python)
- [Archived Rust controller](https://github.com/Super-Yojan/RoverApp/tree/archive/rover-controller)
- [Fusion HAT motor API and PWM resource map](https://docs.sunfounder.com/projects/fusion-hat/en/latest/api/fusion_hat.motor.html)
- [Fusion HAT PWM API](https://docs.sunfounder.com/projects/fusion-hat/en/latest/api/fusion_hat.pwm.html)
- [Fusion HAT Python library](https://github.com/sunfounder/fusion-hat)
- Existing `crates/terra-motors/README.md` and `docs/MOBILE_CONTROL.md`.
