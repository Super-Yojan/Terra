#!/bin/sh
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
argos_root=${ARGOS_ROOT:-"$repo_root/../ARGOS"}
cd "$repo_root"
bundle="$repo_root/target/dashboard-swift-smoke"
mkdir -p "$bundle"
swiftc -emit-library -emit-module -module-name TerraCore mobile/ios/Generated/TerraCore.swift \
    -module-cache-path target/swift-module-cache -I mobile/ios/Generated/headers \
    -L target/debug -lterra_mobile -Xlinker -rpath -Xlinker "$repo_root/target/debug" \
    -emit-module-path "$bundle/TerraCore.swiftmodule" -o "$bundle/libTerraCore.dylib"
swiftc -emit-library -emit-module -module-name ArgosCore "$argos_root/apps/apple/Generated/ArgosCore.swift" \
    -module-cache-path target/swift-module-cache -I "$argos_root/apps/apple/Generated/headers" \
    -L "$argos_root/target/debug" -largos_ffi -Xlinker -rpath -Xlinker "$argos_root/target/debug" \
    -emit-module-path "$bundle/ArgosCore.swiftmodule" -o "$bundle/libArgosCore.dylib"
swiftc -parse-as-library scripts/DashboardSwiftSmoke.swift -I "$bundle" -I mobile/ios/Generated/headers \
    -I "$argos_root/apps/apple/Generated/headers" -L "$bundle" -lTerraCore -lArgosCore \
    -module-cache-path target/swift-module-cache -Xlinker -rpath -Xlinker "$bundle" \
    -o "$bundle/dashboard-swift-smoke"
python3 - "$bundle/dashboard-swift-smoke" <<'PY'
import json
import socket
import subprocess
import sys
import zenoh
with socket.socket() as socket_reservation:
    socket_reservation.bind(('127.0.0.1', 0))
    endpoint = f'tcp/127.0.0.1:{socket_reservation.getsockname()[1]}'
config = zenoh.Config()
config.insert_json5('mode', '"router"')
config.insert_json5('listen/endpoints', json.dumps([endpoint]))
config.insert_json5('scouting/multicast/enabled', 'false')
with zenoh.open(config):
    subprocess.run([sys.argv[1], endpoint], check=True, timeout=30)
PY
