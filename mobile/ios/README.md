# Terra Mobile

The interactive rover remains the main overview on Home both before and after connection.
The local depth map is accessible through Live View.
The SwiftUI home screen presents the robot overview, manual control, missions,
depth map, telemetry, and configuration shortcuts. All views share one controller.
The mountain splash artwork is generated; the interactive rover is derived from
the supplied CAD `robot.usdc`.

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

On physical iPhones, automatic connection is enabled by default. The app waits
for Bluetooth readiness, scans only for Terra's service, and reconnects to the
last successfully synchronized rover. Without a saved rover it waits two seconds
after discovery and selects only a single candidate. Multiple candidates require
manual selection in Robots. It does not select a different rover when a saved
rover is absent. Initial pairing still requires accepting iOS's pairing prompt
and putting the rover into pairing mode.

Connection never arms the motors or starts a mission. Automatic connection uses
manual hardware mode; phone feedback can be enabled through the existing setup
flow. Leaving controller details stops and disarms output while preserving the Bluetooth link. Backgrounding disconnects and disarms. Inactive transitions disarm and zero
the target without interrupting pairing prompts. Disable **Connect Automatically**
in Robots to prevent future automatic attempts. Simulator builds retain the Bevy
and local simulation paths and do not attempt physical Bluetooth auto-connection.

Run the automatic selection checks:

```sh
swiftc mobile/ios/TerraPhone/TerraAutoConnectionPolicy.swift \
  tests/ios/TerraAutoConnectionTests.swift -o /tmp/terra-auto-tests
/tmp/terra-auto-tests
```

Battery, radio strength, temperature, video, and mission completion percentages
are not currently exposed by the controller, so the connected dashboard displays
its real depth map, velocity, turn rate, motor effort, and waypoint distance.

## Manual Drive

Drive starts with a touch joystick and Arm/Disarm and Emergency Stop controls.
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
Disarm and disconnect clear the rover's session-only bench permission. The normal
physical gate remains required for rovers that do not explicitly use bench mode.

Bluetooth transport acknowledgements have a separate 2-second liveness timeout.
Fresh command packets still expire at 100 ms, and the rover disarms after 200 ms
without fresh drive input. The app synchronizes retained subscriptions on relaunch
and persists separate request-ID ranges for synchronization and configuration.
