#!/bin/sh
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$repo_root"
swiftc mobile/ios/Generated/TerraCore.swift scripts/SwiftSmoke.swift \
    -module-cache-path target/swift-module-cache \
    -I mobile/ios/Generated/headers -L target/debug -lterra_mobile \
    -Xlinker -rpath -Xlinker "$repo_root/target/debug" -o target/debug/terra-swift-smoke
target/debug/terra-swift-smoke
