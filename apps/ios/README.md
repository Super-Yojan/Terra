# TerraPhone

The phone is Terra's sensor head and motor link. It does two things:
read the phone's sensors, and send wheel commands to the rover. Every
decision in between happens in the Rust crate `terra-av`.

```text
Planner loop, 20 Hz:   PhoneSensors ─► SensorFrame ─► TerraAv.tick ─► DriveCommand (target + measured twist)
                                           ▲ e-stop        ▲ Goal (joystick)
Velocity loop, 50 Hz:  latest DriveCommand ─► VelocityController ─► BluetoothLink ─► Pi
```

The two loops share only the latest command, so a slow planner tick never
delays the motors. The velocity loop stops if the target is older than 0.3 s.

| Folder | Job |
| --- | --- |
| `Sense/` | ARKit pose, gyro, GPS, thinned LiDAR depth. Raw, in each sensor's own frame. |
| `TerraAV/` | Phone helpers for the generated terra-av types (`Vec3`, `VioPose` from ARKit). |
| `../Generated/` | Built by the script, not committed: `TerraAV.swift` bindings + `TerraAV.xcframework`. |
| `Drive/` | Joystick → `Goal.velocity`. |
| `Motors/` | `VelocityController` (twist → wheel effort, P + feedforward) and the Bluetooth link to the Pi. |
| `App/` | `RoverLoop` (the two loops) and the one screen. |
| `Resources/terra-av.toml` | Which algorithm runs in each stage, plus the phone mount. |

The phone mount (rotation and camera offset) lives in `terra-av.toml`, not in
the app.

## Status

The app calls the real Rust `TerraAv.tick` (crates/terra-av/src/ffi.rs). It
returns a zero twist until the pipeline stages are implemented. Bluetooth, sensors,
joystick and e-stop are real.

## Build

```sh
rustup target add aarch64-apple-ios aarch64-apple-ios-sim x86_64-apple-ios   # once
./scripts/ios/build-ios.sh
```

The script generates the Swift bindings from `crates/terra-av/src/ffi.rs`,
builds the static library for device and simulator, bundles
`apps/ios/Generated/TerraAV.xcframework`, and regenerates the Xcode project.
Re-run it after any Rust change.

Then open `apps/ios/TerraPhone.xcodeproj` in Xcode and run on an iPhone with LiDAR.
