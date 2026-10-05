# Phone velocity controller

The reusable pipeline is IMU + VIO → estimated body velocity → velocity controller → signed left/right motor effort. Both efforts are normalized to [-1, 1]; positive effort drives forward. Hardware must translate magnitude to PWM and sign to direction. This project currently displays the output; it does not send PWM to hardware or implement an iOS Zenoh transport.

## Build and run

Run `./scripts/build-ios.sh` on a Mac with Xcode, Cargo and Rust targets `aarch64-apple-ios`, `aarch64-apple-ios-sim`, and `x86_64-apple-ios` installed. It generates the Swift bindings and an XCFramework for iPhone and both simulator architectures. Generated artifacts are ignored by Git. Open `mobile/ios/TerraPhone.xcodeproj`, select a signing team and run TerraPhone. The deployment target is iOS 17.

The app offers a simulated sensor mode and a phone sensor mode, plus a repeatable Rust benchmark. Simulator mode exercises the same UniFFI controller without requiring a camera. Phone mode reads Core Motion's gravity-free acceleration and angular rate and estimates world velocity from ARKit poses. Camera permission and a device supporting AR world tracking are required for phone mode. Loss of tracking produces neutral effort. Leaving the foreground stops the controller.

`./scripts/check-swift.sh` validates a real Swift → UniFFI → Rust call and runs the controller benchmark. `cargo test --workspace` checks the Rust modules; the simulator's headless physics test checks acceleration, turning and stopping through Avian motor forces.

## Sensor contract

All timestamps are seconds on the same monotonic clock. IMU and VIO samples must increase independently; delayed VIO is replayed against recent IMU history. Body axes are X forward, Y left, Z up. Acceleration is in m/s² with gravity removed; gyro is in rad/s. VIO supplies position and velocity in a consistent world frame and an XYZW quaternion rotating body vectors into that world frame. Samples and configuration are validated at the Rust boundary.

The estimator anchors velocity to VIO and integrates IMU only between fresh VIO updates. It is not a visual odometry implementation or a full bias-estimating filter. The default IMU timeout is 100 ms and VIO timeout 350 ms; the target timeout is 500 ms. Missing, stale, invalid or untracked inputs yield zero motor effort. The controller accepts forward velocity and yaw rate, limits targets, mixes the two PI outputs into wheel efforts, preserves their ratio on saturation, and uses back-calculation anti-windup.

## Mounting and tuning

The starter app assumes the phone lies flat, screen upward, with its top edge pointing forward. Core Motion phone axes are converted to rover axes; ARKit poses are converted into a Z-up world and the same body frame. The app treats the sensor origin as the rover origin. Before physical motor integration, calibrate the mount rotation and sensor offset, account for offset-induced rotational velocity, and tune gains, feedforward, limits and sensor filtering for the actual rover. Defaults are tuned for the included simulated motor plant.

Zero effort means neutral motor output, not guaranteed mechanical braking. A hardware enable switch and independent motor-command watchdog belong in the motor adapter. Physical phone sensing and hardware actuation have not been validated on a rover.

## Simulation

Each rover gets its own estimator and controller. Avian velocity and orientation provide synthetic IMU and 20 Hz VIO feedback, while controller effort produces forces and yaw torque. This tests the control loop rather than calculating VIO from rendered images. `VelocitySimulationConfig` exposes sensor enable switches, sensor frequency, motor force and drag for experiments. Disabling its `enabled` field restores the existing ideal drive model. Existing fleet changes and Zenoh velocity commands remain supported.

## iOS occupancy view

The app shows the Rust local occupancy grid, rover position/heading, map scale and a clear action. Simulated mode generates room-wall depth observations at 10 Hz through the same UniFFI mapping object. Phone mode requests ARKit `sceneDepth` only when supported, rescales camera intrinsics to depth resolution, filters low-confidence returns and passes each depth map with that ARFrame's camera pose and timestamp. Mapping pauses while tracking is lost. Unsupported devices retain velocity control and show that depth is unavailable.

The map is world-aligned: +X right and +Y upward on screen. Free cells are green, occupied cells use the primary foreground, unknown cells are faint gray and uncertain observed cells are darker gray. The blue marker indicates rover pose. Initial phone camera height is assumed to be 0.5 m over a flat ground reference; physical mounting and ground height require calibration. Phone depth comes from the rear camera's optical pose, independently of the rover-body mounting rotation. Stop preserves the map; starting either mode creates a new map.

Bevy Zenoh mode uses the same grid. It does not subscribe to an occupancy topic. Each simulator depth packet carries the exposure-aligned optical camera pose and the rover body pose in the robotics frame (the same conversion the simulator uses for its own per-rover map). The phone recenters on the body position, integrates axial depth through `MobileOccupancyMap`, and draws the body heading. Ground is robotics Z = 0, which is the simulator floor (Bevy Y = 0); the 0.5 m phone-height assumption is not used. Depth packets without an exposure pose are ignored. Body pose is sampled with the camera at exposure time, so the marker matches the depth frame rather than a later odometry estimate. There is still no published map snapshot for ARGOS or other operators.

## Drive Bevy from TerraPhone over Zenoh

TerraPhone now exposes the Rust `terra-transport` client through UniFFI. Use the **Bevy simulator · Zenoh** section to configure an endpoint and rover ID, then connect. This mode publishes the velocity sliders directly as simulator `cmd_vel` requests; the Bevy rover runs its own velocity feedback loop. Local phone IMU/VIO/motor effort readouts do not provide remote feedback. Connecting starts at zero. Stop, switching modes and leaving the foreground close the session with a final zero. The Rust publisher also expires its 250 ms command lease if Swift stops refreshing it; the simulator's independent 500 ms watchdog remains active.

Demo on one Mac:

```sh
cd simulator
TERRA_ROVER_COUNT=2 cargo run
```

Build/run TerraPhone in iOS Simulator, select `tcp/127.0.0.1:7447` and rover `0`, connect, then move the velocity sliders. Stop and check that the rover stops. Choose rover `1` to drive the second rover. To connect a physical iPhone, start Bevy with `TERRA_ZENOH_LISTEN=tcp/0.0.0.0:7447`, put the Mac and phone on the same network, enter `tcp/MAC_LAN_IP:7447`, allow local-network access, and allow incoming traffic to the simulator if prompted. Endpoint and ID are stored on the device; there is no automatic reconnect or automatic startup motion.

The default topic prefix is `terra/rover`. Session-open status confirms a transport session, not rover discovery or movement acknowledgement. The connection status counts posed depth frames as they arrive. Fleet/state and RGB subscriptions remain follow-ups. Remote mode builds the local occupancy map from `terra/rover/<id>/camera/depth` only; it does not mix in simulated-room or ARKit observations.

One-Mac occupancy demo:

```sh
cd simulator
TERRA_ROVER_COUNT=1 cargo run
```

Build and run TerraPhone in the iOS Simulator (`./scripts/build-ios.sh`, then open `mobile/ios/TerraPhone.xcodeproj`). Select `tcp/127.0.0.1:7447` and rover `0`, then connect. The occupancy section starts at “Waiting for simulator depth and exposure pose”. After the simulator publishes a depth frame, the grid fills with free and occupied cells as the rover sees the world. Move the velocity sliders; the blue marker and the map window follow the published body pose. Stop disconnects and keeps the last grid. Rover `1` maps that rover only. A physical iPhone uses the same LAN setup as velocity control; the depth stream is about 1.9 MiB/s at the default 256×192 resolution, before protocol overhead.

Verification:

```sh
./scripts/build-ios.sh
python3 scripts/check-zenoh-swift.py # requires eclipse-zenoh Python package
cargo test -p terra-transport
cargo test -p terra-transport -- --ignored
cargo test --manifest-path simulator/Cargo.toml depth_packet_pose_round_trips_into_the_phone_decoder
cargo test --manifest-path simulator/Cargo.toml mobile_adapter_drives_avian -- --ignored
```

The Swift test exercises Swift → UniFFI → Rust → Zenoh against a real Python peer, checking the selected topic, payload, lease expiry and final zero. It also publishes one posed depth packet and checks that Swift integrates it into an occupied cell. `depth_packet_pose_round_trips_into_the_phone_decoder` checks that a simulator depth packet decodes to the same robotics pose the in-sim map uses, then occupies the expected cell. The ignored transport test checks that the phone client consumes each posed depth sequence once. The Bevy integration test uses the same transport to move an Avian rover and verifies stopping on disconnect. Physical iPhone networking has not been tested on a device. Seeing the grid in Simulator still requires the Bevy app to be running so the GPU depth camera can publish frames.
