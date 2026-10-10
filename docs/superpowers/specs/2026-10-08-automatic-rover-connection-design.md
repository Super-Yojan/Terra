# Automatic rover pairing, reconnection, and router connection

Date: 2026-10-08
Status: Approved; implemented and software-verified. Physical acceptance pending.

## Intended outcome

Replace the normal Find rovers → select rover → connect workflow. While Terra is in the foreground, a nearby new rover in its physically enabled pairing window produces a Connect / Not now popup. Terra reconnects to the last successfully paired rover on launch and foreground return. Once authenticated Bluetooth access and configuration synchronization complete, Terra connects to the saved ARGOS Zenoh router automatically.

The user approved this behavior in chat. This specification makes the lifecycle, failure behavior, and implementation boundaries explicit. Work is limited to this connection flow and its stability; it does not redesign unrelated navigation or autonomy.

## Existing behavior and constraints

- `TerraDashboard` already starts discovery on launch and foreground return.
- `TerraAutoConnectionPolicy` can silently select a lone unknown rover. Multiple candidates stop discovery and require manual selection.
- `BluetoothLink` recognizes setup only after characteristic discovery. Pairing confirmation closes the link and asks the user to scan again.
- Physical pairing and normal operation currently use the same Terra service and rover name. Discovery cannot reliably distinguish their modes.
- `PhoneController` saves the preferred rover only when motion configuration becomes ready. A paired rover with an invalid layout must still remain remembered.
- Dashboard connection currently requires a running phone or simulation controller. The default Bluetooth manual path creates neither.
- Backgrounding stops the controller and connections. Inactive transitions disarm, including during system pairing prompts.
- Terra contains existing uncommitted dashboard and simulator work. Preserve it and integrate with its current interfaces.

## Proposed responsibilities

### Rover advertisement

Advertise the local name `terra-<existing rover suffix>-pair` only during the physically authorized enrollment window, using the existing Bless transport name argument. Keep the same Terra service UUID. Normal operation retains `terra-<existing rover suffix>`. iOS recognizes only Terra-service advertisements with the pairing suffix as enrollment candidates and removes the suffix for display. Apply the same naming rule to physical-button and terminal enrollment. Verify the advertised local name on hardware and verify that the peripheral identifier stays stable across setup and operation. Normal operation cannot trigger a new-owner popup.

The indicator invites enrollment; it does not authorize ownership. Existing physical-window checks, encrypted enrollment read, durable owner storage, and authenticated normal-operation checks remain authoritative. After pairing, normal advertising resumes as today.

### Foreground connection coordinator

Use one lifecycle owner in `PhoneController`, with a small pure policy/state helper where useful. It coordinates discovery, pending consent, selected rover, enrollment completion, Bluetooth authentication, and router startup. Keep CoreBluetooth operations on `BluetoothLink`'s serial queue, Rust/controller work on `controlQueue`, and observable UI state on the main queue.

Track a generation for every foreground session and connection attempt. Canceled timers and late callbacks must not restart discovery, change the selected rover, or publish success for an obsolete attempt. Only one Bluetooth connection and one router attempt may be active at a time.

## Discovery and consent

1. Foreground launch/return starts Terra-service discovery when automatic connection is enabled.
2. A remembered rover advertising normal operation is reconnected without a popup. Unknown normal-operation rovers are never selected automatically.
3. An unknown rover advertising pairing mode creates a single app-wide popup showing its name, with Connect and Not now actions. Bluetooth's system pairing prompt may follow after Connect.
4. Repeated advertisements cannot duplicate the popup. Multiple pairing candidates are handled one at a time in discovery order.
5. Not now suppresses that rover for the current foreground session. A later foreground session can offer it again. Do not persist refusal as an ownership decision.
6. A popup does not interrupt an active connection to the remembered rover. Offer new enrollment while idle; switching a connected rover remains an explicit user action.
7. After Connect, a bounded attempt connects to that exact candidate. If it disappears or its pairing window expires, report the failure and return to discovery without repeatedly prompting in the same foreground session.

Discovery continues while idle, including when the remembered rover is absent. Repeated advertisement delivery must be enabled where necessary to observe a pairing-to-operational transition. Bound resource usage through one scan and cancellation on background, user stop, or disabled automatic connection.

## Enrollment and reconnection

Remember the rover after the encrypted setup response confirms durable pairing, or after authenticated normal-operation status is accepted. Do not wait for a valid actuator layout to remember it. An advertisement or successful physical BLE connection alone is insufficient.

After setup confirmation, close the setup connection, wait for teardown, and rediscover/reconnect to the same identifier as normal operation becomes available. Do not reuse stale setup characteristics. Allow the existing synchronization sequence to refresh capabilities, layout, and status.

Use connection deadlines and retry delays of 1, 2, 4, 8, then 15 seconds, capped at 15 seconds. Retry transient disconnects and unavailable devices while foreground automation remains enabled. Reset backoff after a successful authenticated connection. Permission denial or disabled Bluetooth waits for state change instead of repeatedly attempting a connection. Authentication rejection and malformed protocol messages remain visible errors and stop that attempt until a new foreground session or explicit retry. Layout/fault conditions remain connected for repair and do not create reconnect loops.

Manual stop, explicit disconnect, and disabling automatic connection suspend reconnection for the current session. Returning from background starts a fresh session if automatic connection is enabled. Merely becoming inactive for a system permission or pairing dialog must not cancel enrollment; it continues to revoke motion permission.

## Automatic Zenoh router connection

Use the existing saved `dashboardRouterEndpoint`, `dashboardTopicPrefix`, and `dashboardRoverID`. This refers to the ARGOS router, not the simulator-only Bevy connection. A blank endpoint shows “Set router endpoint in Settings” and performs no connection attempts. Invalid endpoint or ID settings show an actionable status and wait for correction.

After Bluetooth owner authentication and configuration availability, ensure a Rust `MobileController` exists and connect its dashboard transport using the existing interface. Router startup must also work in Bluetooth manual mode: it must not require AR tracking, enable feedback, or change manual effort routing. Service transport polling and connection-failure observation in that mode so the new session actually remains live; preserve the existing behavior for phone feedback mode. Avoid replacing a live controller just to connect its dashboard.

Retry transient router failures with the same capped backoff, independently of Bluetooth discovery. A router failure must not tear down a healthy Bluetooth connection. Bluetooth disconnect, backgrounding, or stop disconnects the router and clears remote intent. Explicit dashboard disconnect suppresses router retries for the current foreground session. Settings changes invalidate stale attempts and permit a fresh attempt when valid.

All automatic connections start with zero local targets and disarmed hardware. Reconnection never restores arm permission, joystick values, or an earlier mission. Router connection alone cannot arm hardware. Preserve the current explicit arming, emergency-stop, session/revision validation, and fresh-status requirements.

## User interface

- Attach the consent popup at the dashboard root so it works from every tab and does not appear behind the splash screen.
- Replace the normal Find rovers button, picker, and Connect selected rover button with connection status and first-setup instructions: hold USR until the LED blinks, then accept the popup.
- Keep the automatic connection setting, router settings, actuator configuration, and explicit safety controls.
- Retain an explicit retry action for stopped/error states and an explicit disconnect action. Provide a deliberate remembered-rover reset/switch action; do not silently replace the remembered owner.
- Keep phone-feedback selection separate from discovering and pairing hardware. Feedback still requires compatible configuration and available phone tracking.
- Show Bluetooth and router status separately so “rover connected” does not imply the router is connected.
- Update documentation that currently tells users to scan again after pairing or manually connect after foreground return.

## Validation

Automated policy tests cover unknown normal advertisements, preferred-rover selection, pairing-only consent, duplicate suppression, declined candidates, multiple candidates, pairing completion, background cancellation, late callbacks, explicit disconnect suppression, deadlines, and capped backoff. Replace the existing test expectation that a lone unknown rover is silently selected.

Verify advertisement mode differentiation and enrollment cleanup with the existing Raspberry Pi tests. Add targeted tests for any advertisement helper introduced. Verify router lifecycle without a sensor session, failure/retry behavior, and stale generation cancellation using testable policy or transport seams. Run the existing Swift checks, relevant Rust/dashboard tests, and iOS build checks supported by the environment.

Physical acceptance requires an iPhone and rover: first-owner popup and iOS pairing, automatic transition to operation, cold-launch reconnection, router arrival after BLE readiness, temporary radio/router loss, declined popup, two nearby rovers, background/foreground, and disarmed outputs throughout. Report these as unverified if hardware is unavailable; compilation and policy tests cannot establish radio behavior.

## Scope boundary

No background Bluetooth execution, automatic router endpoint discovery, multi-rover simultaneous connections, ownership transfer protocol, firmware deployment, or restoration of previous motion is included. Router endpoint configuration remains a one-time settings task. Implementation follows written-spec approval and a reviewed implementation plan.
