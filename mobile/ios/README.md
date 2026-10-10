# Terra Mobile

Home focuses on connecting your rover and the ARGOS fleet dashboard. The rover
preview stays beside the real connection state, remembered name, and configured
actuator names. Open Settings from the lower-left gear button for Fleet connection,
Rover setup, Diagnostics, and Debug tools. Manual drive, simulation, map inspection,
and waypoint tools live under Debug tools. All pages share one controller; moving
between pages keeps healthy connections running. The timed launch splash is removed.

The model is bundled as `TerraPhone/Models/rover.usdz` (approximately 6.3 MB).
Drag to orbit, pinch to zoom, and tap Reset View to restore the overview camera.
`materials.json` retains the CAD palette, including black materials that Apple's
OBJ converter otherwise replaces with defaults. The viewer converts CAD Z-up to
SceneKit Y-up, loads the model off the UI thread, and renders on demand.

To rebuild the model, install `numpy`, `trimesh`, and `fast-simplification` in a
Python environment, then run:

```sh
python3 scripts/prepare-ios-rover.py /path/to/robot.usdc
```

USD, USDC, USDA, USDZ and OBJ are accepted. For OBJ, keep its MTL file beside it. Apple's `usdcat` and `usdzip` must be available.
Disconnected CAD components can prevent the simplifier reaching its target.
The current preview retains approximately 853,000 of the original 3.5 million
triangles; the original CAD export is not included in the app.

## Automatic hardware connection

On physical iPhones, automatic connection is enabled by default. Terra scans for
its Bluetooth service while in the foreground and reconnects only to the last
confirmed owner rover. Unknown operational rovers are never selected silently.
For first setup, hold the rover’s USR button for three seconds until its LED blinks.
Its pairing advertisement triggers a **Connect / Not now** popup; iOS may then
request Bluetooth pairing. After confirmation, Terra remembers the rover and
waits for its normal service to restart, then reconnects automatically. Not now
suppresses that rover until the next foreground session.

In **Settings → Fleet connection**, save the TCP router endpoint, topic prefix, and
rover ID once. Terra connects to this router after Bluetooth authentication and
configuration synchronization, including in manual mode without phone tracking.
Bluetooth and router failures retry independently with delays of 1, 2, 4, 8,
then 15 seconds. A blank endpoint asks you to configure Settings.

Connection never arms motors or restores a mission. After ARGOS connects, phone
tracking starts automatically in every autonomy mode, including Manual/Teleop.
It stops when the fleet session ends. Camera/sensor availability and tracking health
are shown separately; Rover setup offers Retry phone tracking after a sensor error.
Tracking starts without replacing the router session or accepted autonomy. Feedback
control additionally requires a compatible layout and healthy tracking. Backgrounding
disconnects and disarms; foreground return reconnects. Inactive transitions zero
and disarm without canceling pairing. Stop or explicit Disconnect suppresses
reconnection until Retry or a new foreground session. Forget rover clears the
saved rover and starts discovery for another pairing candidate. Disable
**Automatic connection** to stop discovery and automatic retries. Simulator
builds retain the existing Bevy and local simulation flows.

Run the automatic selection checks:

```sh
swiftc mobile/ios/TerraPhone/TerraAutoConnectionPolicy.swift \
  tests/ios/TerraAutoConnectionTests.swift -o /tmp/terra-auto-tests
/tmp/terra-auto-tests
```

Fleet settings are validated and applied together when you tap Save. Cancel discards
the draft. Fleet Retry leaves the rover Bluetooth link connected. Home exposes
Emergency Stop while authenticated, and connection detail sheets provide Disconnect
and Forget rover. Forget changes the phone's remembered rover without erasing the
rover's owner record.

## Manual Drive

**Settings → Debug tools → Manual drive** opens a touch joystick and Arm/Disarm and Emergency Stop controls.
Push up/down for forward/reverse and left/right for steering. Input is clamped to
a circular range with an 8% center dead zone. Release or gesture cancellation sends
zero. Hardware input requires an explicit Arm and the rover's armed acknowledgement;
the joystick stays locked during autonomy. Stop immediately inhibits touch input and
requests the existing emergency stop. Reset leaves output disarmed. App interruptions,
leaving Drive, connection loss, and disarming invalidate the current gesture.

An explicit Arm can resume manual control over a synchronized Bluetooth connection
retained after navigation. It does not reconnect or restore a previous motion target.
Emergency Stop and Reset Stop also reach that retained link while output is stopped.

Run the joystick checks:

```sh
swiftc mobile/ios/TerraPhone/DriveJoystickCommand.swift \
  tests/ios/DriveJoystickTests.swift -o /tmp/terra-joystick-tests
/tmp/terra-joystick-tests
```

Bench-capable rovers advertise `gate_mode=bench` and `bench_enabled` in status.
Drive displays **Enable Bench Control** separately from Arm, explains the absence
of a physical power cutoff, and uses rover acknowledgement to enable Arm. Stop,
Disarm and disconnect clear the rover's session-only bench permission. Rovers
using an external battery cutoff omit `--gate-file`; they report
`gate_mode=external_power_cutoff` and `configuration_allowed` without claiming
to measure a switch. Explicit physical-gate installations retain their interlock
checks. See [rover1 configuration](../../docs/phone/rover1-configuration.md).

Bluetooth transport acknowledgements have a separate 2-second liveness timeout.
Fresh command packets still expire at 100 ms, and the rover disarms after 200 ms
without fresh drive input. The app synchronizes retained subscriptions on relaunch
and persists separate request-ID ranges for synchronization and configuration.

### Phone mapping ground reference

Phone tracking detects horizontal planes and waits for an ARKit-classified floor
or the lowest unclassified horizontal ground candidate of at least 0.4 m², stable within 3 cm for one second. Its measured height establishes
the fixed ground reference for both the occupancy map and the published point cloud;
no camera mounting height is assumed. Point the camera toward the floor during startup.
Until the floor is established, cloud publication and depth integration wait rather
than classify returns against an invented ground plane. Restart tracking to recalibrate.
The point cloud includes floor samples; occupancy treats returns below 15 cm above
that measured floor as free-space evidence rather than obstacle endpoints.

Terra disables idle auto-lock while its scene is active and restores normal idle behavior
when inactive or in the background. Manual actuator layouts receive only the arbiter’s
accepted teleop/assisted-teleop twist through normalized manual routes; explicit Arm
and Bluetooth freshness checks remain required.

ARGOS hardware arming: the selected rover panel now exposes Arm and Disarm. Terra stays on Home and forwards an explicit, fresh, current-run request over its existing Bluetooth safe-frame/arm handshake. The dashboard receives hardware readiness, actual armed/arming state and stop reason. Disarm remains independent of authority freshness. Debug drive is only needed for local troubleshooting. Backgrounding, emergency stop, network loss and the rover watchdog still inhibit motion.

The rover phone mount has its left edge facing forward. The shared phone-to-body transform maps phone -X to rover +X and phone -Y to rover +Y for both AR heading and IMU feedback. Camera depth remains in its measured optical/world frame.

Accepted active L3 waypoints continue locally across an ARGOS router outage (operator-approved). Automatic link recovery retains the mission and existing arming state; connection setup/teardown runs away from the local control loop. Teleop does not get this exemption. Explicit disconnect/stop, backgrounding, Bluetooth loss/watchdog, and unhealthy local tracking/maps still inhibit motion. During an outage, new dashboard orders and remote stop commands cannot be delivered until connectivity returns.


## USB debug logging

Connect the iPhone by USB, select it in Xcode, and Run TerraPhone using the Debug configuration. Show the debug console (Shift-Command-Y) and filter for TerraPhone or the `com.terra.phone` subsystem. macOS Console can also stream the connected iPhone's logs; select the device and enable Info and Debug messages.

Use `TerraLog.bluetooth`, `.control`, `.tracking`, or `.configuration` when adding diagnostics. For example:

```swift
TerraLog.control.debug("Manual control ready=\(self.hardwareReady)")
```

Use `notice` for safety transitions, `info` for normal lifecycle events, `debug` for temporary diagnostics, and `error` for failures. Bluetooth logs include rover armed/arming state, `stop_reason`, fault, gate mode and configuration eligibility whenever these change. Control logs identify the caller requesting motion revocation. Avoid logs on every frame or drive tick, and never log credentials or complete sensor/network payloads. Reinstall the updated app before testing on the phone.
