## Build and run the iOS app, and log the output.
##
##   just dev                 Rust + Swift: rebuild terra-av, then run on the phone
##   just fast                same, but a quick debug build of terra-av for the phone only
##   just run                 Swift only: build, install, launch, stream logs
##   just device=Rover1 run   another phone (name or UDID; see `just devices`)
##
## TerraPhone needs a real iPhone (ARKit, LiDAR, Bluetooth), not a simulator.

# Phone to run on: a name or UDID from `just devices`. Or set TERRA_DEVICE.
device := env("TERRA_DEVICE", "Rover1")

project := "apps/ios/TerraPhone.xcodeproj"
derived := "target/ios-build"
app := derived / "Build/Products/Debug-iphoneos/TerraPhone.app"
bundle := "org.terra.robotics.phone"

default:
    @just --list

# Rust → Swift: regenerate the bindings, rebuild TerraAV.xcframework and the Xcode project.
rust:
    scripts/ios/build-ios.sh

# Build the app for iOS devices (Debug).
build:
    xcodebuild -project {{project}} -scheme TerraPhone -configuration Debug \
        -destination 'generic/platform=iOS' -derivedDataPath {{derived}} \
        -allowProvisioningUpdates -quiet build

# Build, install on the phone, launch, and stream the app's stdout/stderr (Ctrl-C to stop).
run: build
    xcrun devicectl device install app --device "{{device}}" {{app}}
    xcrun devicectl device process launch --device "{{device}}" --terminate-existing --console {{bundle}}

# After Rust changes: rebuild terra-av for iOS, then run.
dev: rust run

# No simulator, no release, no Xcode project regeneration: run `just rust`
# after adding Swift files, or before a release or simulator build.

# Quick Rust → Swift: debug terra-av for the phone only, plus fresh bindings.
rust-fast:
    #!/bin/sh
    set -eu
    generated=apps/ios/Generated
    # Bindings, from the host build (incremental).
    cargo build -p terra-av
    cargo run -q -p terra-av --features bindgen --bin uniffi-bindgen -- generate \
        --library target/debug/libterra_av.dylib --language swift \
        --config crates/terra-av/uniffi.toml --out-dir "$generated"
    cp "$generated/TerraAVFFI.h" "$generated/headers/TerraAVFFI.h"
    # The phone's library only, debug.
    cargo build -p terra-av --target aarch64-apple-ios
    rm -rf "$generated/TerraAV.xcframework"
    xcodebuild -create-xcframework \
        -library target/aarch64-apple-ios/debug/libterra_av.a -headers "$generated/headers" \
        -output "$generated/TerraAV.xcframework"

# After Rust changes, the quick way: `rust-fast`, then run.
fast: rust-fast run

# A local Zenoh router (no login, no TLS) instead of the cloud one; see examples/router.rs.
router port="7447":
    @echo "Phone [comms]: endpoint = \"tcp/$(ipconfig getifaddr en0 || echo '<LAN IP>'):{{port}}\"  (no user)"
    @echo "Base  [comms]: endpoint = \"tcp/127.0.0.1:{{port}}\""
    cargo run -q -p terra-comms --example router -- tcp/0.0.0.0:{{port}}

# Screenshot the phone into target/screenshots/ (pass a name, or get a timestamp).
screenshot name="":
    #!/bin/sh
    set -eu
    mkdir -p target/screenshots
    name="{{name}}"
    [ -n "$name" ] || name="{{device}}-$(date +%Y%m%d-%H%M%S)"
    out="target/screenshots/${name%.png}.png"
    xcrun devicectl device capture screenshot --device "{{device}}" --destination "$out" --quiet
    echo "$out"

# Phones and simulators that devicectl can see.
devices:
    xcrun devicectl list devices

# Regenerate the Excalidraw diagrams and push them to their Excalidraw+ scenes (docs/diagrams/scenes.json).
diagrams:
    #!/bin/sh
    set -eu
    cd docs/diagrams
    python3 -c 'import json; [print(k, v["scene"], v["generator"]) for k, v in json.load(open("scenes.json")).items() if not k.startswith("_") and v["push"]]' |
    while read -r name scene generator; do
        python3 "$generator" "$name.excalidraw"
        excalidraw render "$name.excalidraw" --out "$name.png" --scale 2 >/dev/null
        excalidraw scenes content put "$scene" --file "$name.excalidraw" >/dev/null
        echo "pushed $name → https://app.excalidraw.com (scene $scene)"
    done
    rm -rf __pycache__
