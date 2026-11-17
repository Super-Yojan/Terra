# TerraPhone

The phone is Terra's sensor head and motor link. It does two things:
read the phone's sensors, and send wheel commands to the rover. Every
decision in between happens in the Rust crate `terra-av`.

```text
PhoneSensors ─► SensorFrame ─► TerraAV.tick ─► WheelCommand ─► BluetoothLink ─► Pi
                    ▲               ▲
                 e-stop        Goal (joystick)
```

| Folder | Job |
| --- | --- |
| `Sense/` | ARKit pose, gyro, GPS, thinned LiDAR depth. Raw, in each sensor's own frame. |
| `TerraAV/` | The Swift ↔ Rust boundary: `SensorFrame`, `Goal`, `WheelCommand`, `tick`. |
| `Drive/` | Joystick → `Goal.velocity`. |
| `Motors/` | Bluetooth link to the rover's Raspberry Pi (pairing, actuator layout, drive frames). |
| `App/` | `RoverLoop` (100 Hz tick) and the one screen. |
| `Resources/terra-av.toml` | Which algorithm runs in each stage, plus the phone mount. |

The phone mount (rotation and camera offset) lives in `terra-av.toml`, not in
the app.

## Status

`TerraAV.tick` is a stub that returns stop until `terra-av` is implemented
and exposes Swift bindings. Bluetooth, sensors, joystick and e-stop are real.

## Build

```sh
./scripts/ios/build-ios.sh   # regenerates TerraPhone.xcodeproj
```

Then open `apps/ios/TerraPhone.xcodeproj` in Xcode and run on an iPhone with LiDAR.
