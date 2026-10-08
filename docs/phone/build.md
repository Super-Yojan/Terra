# Building in Xcode

!!! tip "TL;DR"
    Needs a Mac, Xcode, and iOS 17.
    `./scripts/build-ios.sh` makes the XCFramework.
    Linux CI does not run `xcodebuild`.

![App icon produced for the phone target](../assets/terra-icon.png){ width="180" }

*Generated Rust bindings are gitignored under `mobile/ios/Generated/`.*

```mermaid
flowchart LR
  Host[host cargo build] --> Bind[terra-bindgen Swift]
  Bind --> Dev[aarch64-apple-ios]
  Bind --> Sim[simulator lipo]
  Dev --> XC[TerraCore.xcframework]
  Sim --> XC
  XC --> Xcode[TerraPhone scheme]
```

*Three Rust targets: device, arm64 simulator, x86_64 simulator.*

```sh
rustup target add aarch64-apple-ios aarch64-apple-ios-sim x86_64-apple-ios
./scripts/build-ios.sh
./scripts/check-swift.sh
open mobile/ios/TerraPhone.xcodeproj
```

![Which scheme destination you pick changes the link](../assets/phone-home.svg){ width="240" }

*Simulator destination for Zenoh. Physical iPhone for Bluetooth. Set a signing team for the phone.*

Rebuild the mesh on a Mac with `usdcat` and `usdzip`:

```sh
python3 scripts/prepare-ios-rover.py /path/to/robot.usdc
```

The preview keeps about 853,000 of 3.5 million triangles. The original CAD file is not in git.
