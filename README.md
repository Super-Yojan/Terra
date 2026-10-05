# Terra

Terra combines a Bevy/Avian rover simulator with reusable Rust robotics modules and an iOS controller app.

- `terra-types`: sensor samples, coordinate conventions, commands and motor outputs.
- `terra-state`: VIO velocity anchors with bounded IMU prediction.
- `terra-control`: differential-drive velocity PI control, feedforward, anti-windup and freshness checks.
- `terra-mapping`: rolling local occupancy grids from depth and camera poses. See [mapping API](crates/terra-mapping/README.md).
- `terra-mobile`: UniFFI interface shared by Swift and Rust.

See [mobile controller setup](docs/MOBILE_CONTROL.md) for the sensor contract, iOS build and validation. The simulator lives in `simulator/`; its existing fleet and Zenoh velocity commands now feed the shared controller, which applies motor forces through Avian.

```sh
cargo test --workspace
cargo test --manifest-path simulator/Cargo.toml
./scripts/build-ios.sh
./scripts/check-swift.sh
open mobile/ios/TerraPhone.xcodeproj
```
