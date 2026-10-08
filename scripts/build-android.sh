#!/bin/sh
# Host library, Kotlin UniFFI bindings, and Android cdylibs for the emulator and arm64 phones.
# Generated sources and .so files are gitignored, same as the iOS XCFramework.
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$repo_root"

if ! command -v cargo-ndk >/dev/null 2>&1; then
    echo "cargo-ndk is required: cargo install cargo-ndk --version 3.5.4 --locked" >&2
    exit 1
fi

if [ -z "${ANDROID_NDK_HOME:-}" ]; then
    if [ -n "${ANDROID_NDK_ROOT:-}" ]; then
        ANDROID_NDK_HOME=$ANDROID_NDK_ROOT
    elif [ -n "${ANDROID_HOME:-}" ] && [ -d "$ANDROID_HOME/ndk" ]; then
        ANDROID_NDK_HOME=$(find "$ANDROID_HOME/ndk" -mindepth 1 -maxdepth 1 -type d | sort | tail -n 1)
    fi
fi
if [ -z "${ANDROID_NDK_HOME:-}" ] || [ ! -d "$ANDROID_NDK_HOME" ]; then
    echo "Set ANDROID_NDK_HOME to an installed NDK (or ANDROID_HOME with ndk/<version>)." >&2
    exit 1
fi
export ANDROID_NDK_HOME

rustup target add aarch64-linux-android x86_64-linux-android

# Host cdylib: UniFFI metadata for bindgen, and the JVM smoke test.
cargo build -p terra-mobile
mkdir -p mobile/android/app/src/main/java
cargo run -p terra-mobile --features bindgen --bin terra-bindgen -- generate \
    --library target/debug/libterra_mobile.so --language kotlin \
    --config crates/terra-mobile/uniffi.toml \
    --out-dir mobile/android/app/src/main/java \
    --no-format

# UniFFI 0.31 names exception fields `message`, which clashes with Throwable.message
# under Kotlin 2. The workspace pins uniffi, so rename the generated property.
python3 - <<'PY'
from pathlib import Path
root = Path("mobile/android/app/src/main/java")
for path in root.rglob("*.kt"):
    text = path.read_text()
    updated = (
        text.replace("val `message`:", "val errorMessage:")
        .replace("${ `message` }", "${ errorMessage }")
        .replace("value.`message`", "value.errorMessage")
    )
    if updated != text:
        path.write_text(updated)
PY

jni=mobile/android/app/src/main/jniLibs
mkdir -p "$jni"
cargo ndk --target arm64-v8a --target x86_64 -o "$repo_root/$jni" \
    build -p terra-mobile --release
