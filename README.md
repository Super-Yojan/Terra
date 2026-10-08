# Terra

Terra is the vehicle body and onboard autonomy: shared Rust crates, the TerraPhone
iOS app, and the Raspberry Pi rover. The world simulator is
[Zorvane](https://github.com/Super-Yojan/Zorvane). ARGOS is the fleet operator.

Configurable Bluetooth actuator control is implemented for TerraPhone and a
standalone Raspberry Pi peripheral. See [Bluetooth setup and protocol](docs/hardware/BLUETOOTH.md),
[compiled Pi executable and installation](docs/hardware/INSTALL.md),
[Fusion HAT resource requirements](docs/hardware/FUSION_HAT.md), and
[mobile hardware controls](docs/MOBILE_CONTROL.md#bluetooth-actuator-control).
The compiled packaging path has separate build and installation checks; see
[evidence status](docs/hardware/evidence/README.md) for the original implementation
history and current verification limits. Radio sessions, actuator timing and
physical compatibility remain unverified. The [optional bench procedure](docs/hardware/BENCH.md)
requires separate authorization.

See the [mission-autonomy guide](docs/autonomy/README.md) for four shared-core levels, explicit waypoint authority, safety controls, and experiment logs.

- `terra-types`: sensor samples, coordinate conventions, commands and motor outputs.
- `terra-actuators`: configurable actuator layouts, routing, and Bluetooth framing;
  [editable examples](crates/terra-actuators/presets/) require capability validation
  and explicit port selection for the ESC/servo templates.
- `terra-state`: VIO velocity anchors with bounded IMU prediction.
- `terra-control`: differential-drive velocity PI control, feedforward, anti-windup and freshness checks.
- `terra-waypoint`: lat/lon go-to-waypoint follower. The phone imports it through `terra-mobile` as `MobileWaypoint`. TerraPhone's Waypoint section selects the goal and runs that follower.
- `terra-motors`: signed wheel effort to PWM duty and direction, with a hardware enable gate and a command watchdog. See [motor adapter](docs/MOBILE_CONTROL.md#motor-adapter).
- `terra-mapping`: rolling local occupancy grids from depth and camera poses. See [mapping API](crates/terra-mapping/README.md).
- `terra-transport`: leased Zenoh velocity publishing. The default prefix is `terra/rover`.
- `terra-mobile`: UniFFI interface shared by Swift and Rust.

See [mobile controller setup](docs/MOBILE_CONTROL.md) for the sensor contract, iOS build and validation.

## Simulation

World, cameras, physics, and the Zenoh bridge live in
[Zorvane](https://github.com/Super-Yojan/Zorvane). From a checkout of that repo:

```sh
cargo run -p zorvane
```

The Zenoh prefix stays `terra/rover` (`cmd_vel`, `goal`, cameras, fleet). The
`TERRA_*` variables still select the fleet, tiles, mission, and listen address.
`TERRA_ZENOH_LISTEN` defaults to `tcp/127.0.0.1:7447`. Tile worlds and the
go-to-waypoint contract are documented in Zorvane:
[simulator/WORLD.md](https://github.com/Super-Yojan/Zorvane/blob/main/simulator/WORLD.md)
and
[simulator/ZENOH.md](https://github.com/Super-Yojan/Zorvane/blob/main/simulator/ZENOH.md).

```sh
cargo test --locked --workspace
./scripts/build-ios.sh
./scripts/check-swift.sh
open mobile/ios/TerraPhone.xcodeproj
```
