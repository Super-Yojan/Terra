#!/bin/sh
# Build TerraPhone.app on Linux with xtool. Stops before compile when the
# Darwin Swift SDK (from an Xcode .xip) is not installed.
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$repo_root"
ios="$repo_root/mobile/ios"
generated="$ios/Generated"
support="$ios/LinuxSupport"

need() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "build-ios-linux: missing '$1'. See docs/phone/linux.md." >&2
        exit 2
    fi
}

need swift
need xtool
need cargo
need rustup

"$repo_root/scripts/check-xtool-ios.sh"

if [ -n "${XDG_CONFIG_HOME:-}" ]; then
    swiftpm_dir="$XDG_CONFIG_HOME/swiftpm"
else
    swiftpm_dir="$HOME/.swiftpm"
fi
sdk_bundle="$swiftpm_dir/swift-sdks/darwin.artifactbundle"

sdk_status=$(xtool sdk status || true)
echo "$sdk_status"
if ! printf '%s\n' "$sdk_status" | grep -q "Darwin SDK is installed"; then
    if [ -n "${XCODE_XIP:-}" ]; then
        if [ ! -f "$XCODE_XIP" ]; then
            echo "build-ios-linux: XCODE_XIP is set but not a file: $XCODE_XIP" >&2
            exit 2
        fi
        echo "Installing Darwin SDK from $XCODE_XIP"
        xtool sdk install "$XCODE_XIP"
        sdk_status=$(xtool sdk status || true)
        echo "$sdk_status"
    fi
fi
if ! printf '%s\n' "$sdk_status" | grep -q "Darwin SDK is installed"; then
    cat >&2 <<'EOF'
build-ios-linux: Darwin Swift SDK is not installed, so xtool cannot compile for iOS.

Download Xcode 27 as Xcode.xip (Apple ID required):
  https://developer.apple.com/download/all/?q=Xcode
Then install it and rerun this script:
  xtool sdk install /absolute/path/to/Xcode.xip
  ./scripts/build-ios-linux.sh

Or set XCODE_XIP to that path and rerun. This script does not prompt, because
`xtool dev build` waits for the path on stdin when the SDK is missing.
EOF
    exit 2
fi

echo "Generating UniFFI Swift bindings"
cargo build -p terra-mobile
host_lib="$repo_root/target/debug/libterra_mobile.so"
if [ ! -f "$host_lib" ]; then
    host_lib="$repo_root/target/debug/libterra_mobile.dylib"
fi
[ -f "$host_lib" ] || {
    echo "build-ios-linux: host libterra_mobile was not produced" >&2
    exit 1
}
mkdir -p "$generated"
cargo run -p terra-mobile --features bindgen --bin terra-bindgen -- generate \
    --library "$host_lib" \
    --language swift \
    --config "$repo_root/crates/terra-mobile/uniffi.toml" \
    --out-dir "$generated"
[ -f "$generated/TerraCore.swift" ] || {
    echo "build-ios-linux: bindgen did not write TerraCore.swift" >&2
    exit 1
}
[ -f "$generated/TerraCoreFFI.h" ] || {
    echo "build-ios-linux: bindgen did not write TerraCoreFFI.h" >&2
    exit 1
}

echo "Building terra-mobile for aarch64-apple-ios"
rustup target add aarch64-apple-ios
sdk_root=$(find "$sdk_bundle/Developer/Platforms/iPhoneOS.platform/Developer/SDKs" -maxdepth 1 -type d -name 'iPhoneOS*.sdk' | head -n 1)
linker="$sdk_bundle/toolset/bin/ld64.lld"
if [ -z "$sdk_root" ] || [ ! -d "$sdk_root" ]; then
    echo "build-ios-linux: iPhoneOS SDK not found under $sdk_bundle" >&2
    exit 1
fi
if [ ! -x "$linker" ]; then
    echo "build-ios-linux: ld64.lld not found at $linker" >&2
    exit 1
fi
export SDKROOT="$sdk_root"
export CARGO_TARGET_AARCH64_APPLE_IOS_LINKER="$linker"
export CARGO_TARGET_AARCH64_APPLE_IOS_RUSTFLAGS="-C link-arg=-syslibroot -C link-arg=$sdk_root"
if command -v clang >/dev/null 2>&1; then
    export CC_aarch64_apple_ios=clang
    export CFLAGS_aarch64_apple_ios="-target arm64-apple-ios -isysroot $sdk_root -miphoneos-version-min=17.0"
fi
# The package also emits a cdylib. Point the link at ld64.lld from the
# Darwin SDK. The host GNU ld stops with "unrecognised emulation mode: llvm".
cargo build -p terra-mobile --release --target aarch64-apple-ios
ios_lib="$repo_root/target/aarch64-apple-ios/release/libterra_mobile.a"
[ -f "$ios_lib" ] || {
    echo "build-ios-linux: missing $ios_lib" >&2
    exit 1
}

mkdir -p "$support/include" "$support/lib"
cp "$generated/TerraCore.swift" "$ios/TerraPhone/TerraCore.swift"
cp "$generated/TerraCoreFFI.h" "$support/include/TerraCoreFFI.h"
cp "$ios_lib" "$support/lib/libterra_mobile.a"

echo "Packing with xtool"
cd "$ios"
xtool dev build
echo "Wrote $ios/xtool/TerraPhone.app"
