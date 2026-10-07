#!/bin/sh
# Build on ARM64 Linux, directly on a Pi or inside the supplied Docker builder.
set -eu

fail() { echo "build-rover: $*" >&2; exit 1; }
if [ "${1:-}" = --help ]; then
    echo 'usage: ./scripts/build-rover.sh [output-directory]'
    echo 'Requires ARM64 Linux, Python 3.11+, a C compiler, patchelf and curl.'
    exit 0
fi
[ "$#" -le 1 ] || fail 'usage: build-rover.sh [output-directory]'
[ "$(uname -s)" = Linux ] && [ "$(uname -m)" = aarch64 ] || fail 'build on ARM64 Linux; see docs/hardware/INSTALL.md for Docker on macOS'
for tool in python3 cc patchelf curl tar sha256sum; do
    command -v "$tool" >/dev/null 2>&1 || fail "missing build tool: $tool"
done
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
package="$root/hardware/raspberry-pi"
output=${1:-"$package/dist"}
mkdir -p "$output"
output=$(CDPATH= cd -- "$output" && pwd)
build="$package/.build"
mkdir -p "$build"
export NUITKA_CACHE_DIR="$build/nuitka-cache"
python3 -m venv "$build/venv"
python="$build/venv/bin/python"
"$python" -m pip install -r "$package/requirements-build.txt"

fusion_commit=4bd1018ad5a70ee113160536f969cdadd9f22918
fusion_archive="$build/fusion-hat-$fusion_commit.tar.gz"
if [ ! -f "$fusion_archive" ]; then
    curl --fail --location --retry 3 \
        "https://codeload.github.com/sunfounder/fusion-hat/tar.gz/$fusion_commit" \
        --output "$fusion_archive"
fi
echo "37ea53b9b72b6431a4a2ee8b752804ad74f0f10071c416f856991814347d9c2d  $fusion_archive" | sha256sum -c -
tar -xzf "$fusion_archive" -C "$build"
vendor="$build/fusion-hat-$fusion_commit"
# Motor/PWM and their imports use the standard library. Avoid unrelated vendor
# voice/audio dependencies and never run the vendor's system installer here.
"$python" -m pip install --no-deps --no-build-isolation "$vendor" "$package"
cd "$package"
"$python" rover.py --check-bundle
"$python" -m unittest discover -s tests -v
"$python" -m nuitka \
    --mode=onefile \
    --output-filename=terra-rover \
    --output-dir="$build/compiled" \
    --include-package=terra_rover \
    --include-package=bless \
    --include-distribution-metadata=bless \
    --include-distribution-metadata=bleak \
    --include-module=fusion_hat \
    --include-module=fusion_hat.motor \
    --include-module=fusion_hat.pwm \
    --include-module=_json \
    --include-module=_bisect \
    --include-distribution-metadata=fusion_hat \
    --include-distribution-metadata=dbus-next \
    --python-flag=unbuffered \
    --update-check=never \
    --jobs="${TERRA_BUILD_JOBS:-2}" \
    --assume-yes-for-downloads \
    --report="$build/compilation-report.xml" \
    rover.py

binary="$build/compiled/terra-rover"
# Execute from elsewhere with a clean module path: catch accidental dependencies
# on the source checkout, developer virtual environment, or current directory.
cd "$build"
env -u PYTHONPATH -u PYTHONHOME "$binary" --help
env -u PYTHONPATH -u PYTHONHOME "$binary" --check-bundle
version=$("$python" -c 'from importlib.metadata import version; print(version("terra-rover"))')
name="terra-rover-$version-linux-arm64"
release="$output/$name"
mkdir -p "$release"
install -m 0755 "$binary" "$release/terra-rover"
install -m 0755 "$root/packaging/rover/install.sh" "$release/install.sh"
install -m 0644 "$root/packaging/terra-rover.service" "$release/terra-rover.service"
install -m 0644 "$root/packaging/rover/terra-rover.env" "$release/terra-rover.env"
install -m 0644 "$root/packaging/rover/terra-rover.conf" "$release/terra-rover.conf"
install -m 0644 "$root/docs/hardware/INSTALL.md" "$release/INSTALL.md"
install -m 0644 "$build/compilation-report.xml" "$release/compilation-report.xml"
mkdir -p "$release/sources"
cp "$fusion_archive" "$release/sources/"
# Retain the exact rover source, build recipe and GPL vendor source/license.
tar --exclude=.venv --exclude=.build --exclude=dist --exclude='*.egg-info' \
    --exclude=__pycache__ -czf "$release/sources/terra-rover-source.tar.gz" \
    -C "$root" hardware/raspberry-pi scripts/build-rover.sh packaging/rover \
    packaging/terra-rover.service docs/hardware/INSTALL.md
cp "$vendor/LICENSE" "$release/sources/FUSION-HAT-LICENSE"
"$python" - "$release/build-info.json" "$fusion_commit" <<'PY'
import json, platform, sys
from importlib.metadata import version
from pathlib import Path
Path(sys.argv[1]).write_text(json.dumps({
    'platform': platform.system(), 'architecture': platform.machine(),
    'libc': platform.libc_ver(), 'python': platform.python_version(),
    'terra-rover': version('terra-rover'), 'nuitka': version('Nuitka'),
    'bless': version('bless'), 'bleak': version('bleak'),
    'dbus-next': version('dbus-next'), 'fusion_hat': version('fusion_hat'),
    'fusion_hat_commit': sys.argv[2],
}, indent=2) + '\n')
PY
cd "$release"
sha256sum terra-rover install.sh terra-rover.service terra-rover.env terra-rover.conf \
    INSTALL.md build-info.json compilation-report.xml sources/* > SHA256SUMS
cd "$output"
tar -czf "$name.tar.gz" "$name"
sha256sum "$name.tar.gz" > "$name.tar.gz.sha256"
echo "Built $output/$name.tar.gz"
