# TerraPhone

TerraPhone is the iOS app in `mobile/ios/`. SwiftUI draws the rover, the drive joystick, the waypoint section, and the occupancy grid. Shared control, mapping, waypoints, actuators, and Zenoh go through `terra-mobile`.

| Topic | Page |
| --- | --- |
| Xcode build and the UniFFI framework | [Building in Xcode](build.md) |
| Which link exists in the Simulator and on an iPhone | [Simulator and iPhone](simulator.md) |
| Pairing with the Pi | [Bluetooth pairing](bluetooth.md) |
| 3D tap-to-configure | [Tap to configure](tap-to-configure.md) — **planned** |

The long-form sensor contract, motor-adapter notes, and Bluetooth command rules are in [Phone controller](../MOBILE_CONTROL.md). The app README in the tree is [`mobile/ios/README.md`](https://github.com/Super-Yojan/Terra/blob/main/mobile/ios/README.md).

## What the home screen does

One controller backs every view. Home shows the rover model, with shortcuts for manual control, missions, the depth map, telemetry, and configuration. The model is `TerraPhone/Models/rover.usdz`. Drag orbits, pinch zooms, and Reset View restores the camera. The viewer converts CAD Z-up to SceneKit Y-up. Tapping a part does not open actuator settings. That interaction is the planned tap-to-configure mode.

Drive starts disarmed. The joystick is a circular pad with an 8% dead zone. Hardware output needs an explicit Arm and the rover's armed acknowledgement. Emergency Stop inhibits touch input. Backgrounding disconnects and disarms. Connection never arms the motors or starts a mission.

**Simulated rover** is a local motor plant inside the app. It is available in the Simulator and on an iPhone. It is not Zorvane.

**Phone IMU + VIO** needs a physical iPhone with ARKit world tracking. The starter mount is the phone flat, screen up, top edge forward. Core Motion is converted into rover axes. Loss of tracking commands neutral effort. That mount and the 0.5 m camera-height assumption still need calibration on a real vehicle.

Battery, radio strength, temperature, video, and mission-completion percentage are not on the controller, so the connected dashboard shows the depth map, velocity, turn rate, motor effort, and waypoint distance that do exist.
