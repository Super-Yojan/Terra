# terra-mobile

UniFFI boundary for TerraPhone. The robotics crates stay free of UniFFI. This crate wraps them in objects Swift can call. Bindings are generated on a Mac and are not committed.

[API](https://super-yojan.dev/Terra/api/terra_mobile/index.html) · [iOS build](../phone/build.md)

## What Swift calls

| Export | Wraps |
| --- | --- |
| `MobileController` | Estimator, velocity controller, autonomy arbiter, experiment recorder. |
| `default_control_settings` | The combined control and estimator configuration. |
| `MobileWaypoint` | `terra-waypoint`, including `setOrigin`, `setGoal`, and `step`. |
| `MobileOccupancyMap` | `terra-mapping`. |
| `actuator_validate_layout`, `actuator_route`, `actuator_encode_frame` | `terra-actuators`. |
| `MobileZenohClient` | `RoverConnection` in `terra-transport`. |
| `host_dashboard` | Loopback `ControlPlane`. The endpoint must be `tcp/127.0.0.1`. |

`run_velocity_benchmark` runs the controller against the in-crate plant and returns a `BenchmarkReport`. `./scripts/check-swift.sh` calls through the generated Swift bindings into that path.

## Dashboard publish

When `host_dashboard` has opened the loopback plane, each control tick can publish:

- `experiment/status`, `autonomy/status`, `goal/status`, `goal/proposal`
- `map/occupancy` at most every 0.2 s, via `occupancy_telemetry`
- `pose` with rover id, sequence, `x`, `y`, and `yaw`

`autonomy_request` accepts `autonomy`, `safety`, `goal`, and `goal/decision` and feeds the arbiter. Other kinds are rejected.

## Build

`./scripts/build-ios.sh` builds `terra-mobile` for the host, runs `terra-bindgen` for Swift, then builds static libraries for `aarch64-apple-ios`, `aarch64-apple-ios-sim`, and `x86_64-apple-ios` and packs them into `mobile/ios/Generated/TerraCore.xcframework`. That script needs Xcode (`xcodebuild`, `xcrun lipo`) and the three Rust targets. Generated artifacts are gitignored.
