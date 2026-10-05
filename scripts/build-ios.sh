#!/bin/sh
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$repo_root"
generated="$repo_root/mobile/ios/Generated"
headers="$generated/headers"
mkdir -p "$headers"
cargo build -p terra-mobile
cargo run -p terra-mobile --features bindgen --bin terra-bindgen -- generate \
    --library target/debug/libterra_mobile.dylib --language swift \
    --config crates/terra-mobile/uniffi.toml --out-dir "$generated"
cp "$generated/TerraCoreFFI.h" "$headers/TerraCoreFFI.h"
# A plain C module map works with static-library XCFrameworks and Xcode 27.
cat > "$headers/module.modulemap" <<'MODULE'
module TerraCoreFFI {
    header "TerraCoreFFI.h"
    export *
}
MODULE
for ios_target in aarch64-apple-ios aarch64-apple-ios-sim x86_64-apple-ios; do
    cargo build -p terra-mobile --release --target "$ios_target"
done
mkdir -p target/ios-simulator/release
xcrun lipo -create target/aarch64-apple-ios-sim/release/libterra_mobile.a \
    target/x86_64-apple-ios/release/libterra_mobile.a \
    -output target/ios-simulator/release/libterra_mobile.a
# Only replace this script's generated bundle, never project sources.
if [ -d "$generated/TerraCore.xcframework" ]; then
    rm -rf "$generated/TerraCore.xcframework"
fi
xcodebuild -create-xcframework \
    -library target/aarch64-apple-ios/release/libterra_mobile.a -headers "$headers" \
    -library target/ios-simulator/release/libterra_mobile.a -headers "$headers" \
    -output "$generated/TerraCore.xcframework"
