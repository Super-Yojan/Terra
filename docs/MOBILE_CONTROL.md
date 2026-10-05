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

The map is world-aligned: +X right and +Y upward on screen. Free cells are green, occupied cells use the primary foreground, unknown cells are faint gray and uncertain observed cells are darker gray. The blue marker indicates rover pose. Initial phone camera height is assumed to be 0.5 m over a flat ground reference; physical mounting and ground height require calibration. Phone depth comes from the rear camera's optical pose, independently of the rover-body mounting rotation. Stop preserves the map; starting either mode creates a new map. No Bevy/Zenoh map subscription is included.
