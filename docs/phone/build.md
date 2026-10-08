# Building TerraPhone in Xcode

The app builds on a Mac. This repository's Linux CI does not run `xcodebuild`.

## Tools

- Xcode, with the iOS 17 SDK. The deployment target is iOS 17.
- Rust, and the targets `aarch64-apple-ios`, `aarch64-apple-ios-sim`, and `x86_64-apple-ios`.
- A signing team for a physical iPhone. The Simulator does not need a device profile.

Install the Rust targets once:

```sh
rustup target add aarch64-apple-ios aarch64-apple-ios-sim x86_64-apple-ios
```

## Generate the UniFFI framework

From the repository root:

```sh
./scripts/build-ios.sh
```

The script:

1. Builds `terra-mobile` for the host and runs `terra-bindgen` to emit Swift and a C header under `mobile/ios/Generated/`.
2. Builds the static library for the device and both simulator architectures.
3. Lipos the two simulator libraries together.
4. Writes `mobile/ios/Generated/TerraCore.xcframework`.

`Generated/` is gitignored. Re-run the script when the Rust API changes. The script only replaces that generated bundle.

Check the Swift-to-Rust call and the controller benchmark:

```sh
./scripts/check-swift.sh
```

That links `scripts/SwiftSmoke.swift` against the host debug library. It is a Mac check. It does not launch the GUI.

Two smaller Swift checks do not need the XCFramework:

```sh
swiftc mobile/ios/TerraPhone/TerraAutoConnectionPolicy.swift \
  tests/ios/TerraAutoConnectionTests.swift -o /tmp/terra-auto-tests
/tmp/terra-auto-tests

swiftc mobile/ios/TerraPhone/DriveJoystickCommand.swift \
  tests/ios/DriveJoystickTests.swift -o /tmp/terra-joystick-tests
/tmp/terra-joystick-tests
```

## Run

```sh
open mobile/ios/TerraPhone.xcodeproj
```

Select the TerraPhone scheme. For a Zenoh session against Zorvane, choose an iOS Simulator destination. For Bluetooth, choose a physical iPhone. The two destinations compile different connection UI. See [Simulator and iPhone](simulator.md).

Set the signing team on the app target before installing on a phone. Camera permission is required for Phone IMU + VIO. Bluetooth permission is required to scan for a rover.

## Rebuild the rover model

The bundled model is about 6.3 MB at `TerraPhone/Models/rover.usdz`, simplified from a CAD `robot.usdc` that is not in the repository. To regenerate it, install `numpy`, `trimesh`, and `fast-simplification`, and have Apple's `usdcat` and `usdzip` on the path:

```sh
python3 scripts/prepare-ios-rover.py /path/to/robot.usdc
```

USD, USDC, USDA, USDZ, and OBJ are accepted. For OBJ, keep the MTL file beside it. The current preview keeps about 853,000 of the original 3.5 million triangles. `materials.json` keeps the CAD palette, including blacks that Apple's OBJ converter would otherwise replace.
