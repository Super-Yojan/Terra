#!/bin/sh
# Build terra-av for iOS and wire it into TerraPhone:
#   1. generate Swift bindings from crates/terra-av/src/ffi.rs (UniFFI)
#   2. cross-compile the static library for device and simulator
#   3. bundle them into apps/ios/Generated/TerraAV.xcframework
#   4. regenerate TerraPhone.xcodeproj
# Needs macOS with Xcode and: rustup target add aarch64-apple-ios aarch64-apple-ios-sim x86_64-apple-ios
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
cd "$repo_root"
generated="$repo_root/apps/ios/Generated"
headers="$generated/headers"
mkdir -p "$headers"

# 1. Bindings (read from the host build of the library).
cargo build -p terra-av
cargo run -p terra-av --features bindgen --bin uniffi-bindgen -- generate \
    --library target/debug/libterra_av.dylib --language swift \
    --config crates/terra-av/uniffi.toml --out-dir "$generated"
cp "$generated/TerraAVFFI.h" "$headers/TerraAVFFI.h"
cat > "$headers/module.modulemap" <<'MODULE'
module TerraAVFFI {
    header "TerraAVFFI.h"
    export *
}
MODULE

# 2. Static libraries.
for target in aarch64-apple-ios aarch64-apple-ios-sim x86_64-apple-ios; do
    cargo build -p terra-av --release --target "$target"
done
mkdir -p target/ios-simulator/release
xcrun lipo -create target/aarch64-apple-ios-sim/release/libterra_av.a \
    target/x86_64-apple-ios/release/libterra_av.a \
    -output target/ios-simulator/release/libterra_av.a

# 3. XCFramework (only this script's output is replaced).
rm -rf "$generated/TerraAV.xcframework"
xcodebuild -create-xcframework \
    -library target/aarch64-apple-ios/release/libterra_av.a -headers "$headers" \
    -library target/ios-simulator/release/libterra_av.a -headers "$headers" \
    -output "$generated/TerraAV.xcframework"

# 4. Xcode project.
ruby scripts/ios/create-ios-project.rb
echo "Open apps/ios/TerraPhone.xcodeproj in Xcode."
