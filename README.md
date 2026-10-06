# Terra

See the [mission-autonomy guide](docs/autonomy/README.md) for four shared-core levels, explicit waypoint authority, safety controls, and experiment logs.

Terra combines a Bevy/Avian rover simulator with reusable Rust robotics modules and an iOS controller app.

- `terra-types`: sensor samples, coordinate conventions, commands and motor outputs.
- `terra-state`: VIO velocity anchors with bounded IMU prediction.
- `terra-control`: differential-drive velocity PI control, feedforward, anti-windup and freshness checks.
- `terra-waypoint`: lat/lon go-to-waypoint follower. The phone imports it through `terra-mobile` as `MobileWaypoint`. TerraPhone's Waypoint section selects the goal and runs that follower.
- `terra-motors`: signed wheel effort to PWM duty and direction, with a hardware enable gate and a command watchdog. See [motor adapter](docs/MOBILE_CONTROL.md#motor-adapter).
- `terra-mapping`: rolling local occupancy grids from depth and camera poses. See [mapping API](crates/terra-mapping/README.md).
- `terra-transport`: leased Zenoh velocity publishing for remote simulator control.
- `terra-mobile`: UniFFI interface shared by Swift and Rust.

See [mobile controller setup](docs/MOBILE_CONTROL.md) for the sensor contract, iOS build and validation. The simulator lives in `simulator/`; its existing fleet and Zenoh velocity commands now feed the shared controller, which applies motor forces through Avian.

The simulator also runs without a Mac. The dev container works in GitHub Codespaces and in VS Code on Windows or Linux, and shows the Bevy window on a virtual desktop. See [Develop without a Mac](docs/DEVELOP-WITHOUT-MAC.md).

[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/Super-Yojan/Terra)

The simulator still boots into the flat practice world. `TERRA_TILES=1` builds that square from a real Terrarium elevation tile instead (bundled George Mason University Fairfax sample, centered on the Johnson Center, or another lat/lon when the network is available). High-level goals use `terra/rover/<id>/goal`; `cmd_vel` stays a debug twist. See [world design](simulator/WORLD.md#real-world-tiles), [how to run and test](simulator/WORLD.md#reproduce-and-test), and the [Zenoh waypoint contract](simulator/ZENOH.md#go-to-waypoint).

```sh
cargo test --workspace
cargo test --manifest-path simulator/Cargo.toml
./scripts/build-ios.sh
./scripts/check-swift.sh
open mobile/ios/TerraPhone.xcodeproj
```
