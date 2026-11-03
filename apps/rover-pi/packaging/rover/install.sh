#!/bin/sh
# Install an extracted release, or stage its files into a Raspberry Pi image.
set -eu

fail() { echo "terra-rover: $*" >&2; exit 1; }
destination=
case "${1:-}" in
    '') [ "$#" -eq 0 ] || fail 'unexpected arguments' ;;
    --root)
        [ "$#" -eq 2 ] || fail 'usage: install.sh [--root /absolute/image/root]'
        destination=$2
        case "$destination" in /*) ;; *) fail '--root must be absolute' ;; esac
        [ "$destination" != / ] || fail '--root must not be /'
        ;;
    --help) echo 'usage: sudo ./install.sh (live Pi) or ./install.sh --root /absolute/image/root'; exit 0 ;;
    *) fail 'usage: install.sh [--root /absolute/image/root]' ;;
esac

bundle=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$bundle"
for filename in terra-rover terra-rover.service terra-rover.env terra-rover.conf SHA256SUMS; do
    [ -f "$filename" ] || fail "release is missing $filename"
done
if command -v sha256sum >/dev/null 2>&1; then
    sha256sum -c SHA256SUMS || fail 'release checksum verification failed'
else
    shasum -a 256 -c SHA256SUMS || fail 'release checksum verification failed'
fi
# Check the ELF class, byte order and AArch64 machine code without executing it.
magic=$(od -An -tx1 -N6 terra-rover | tr -d ' \n')
machine=$(od -An -tx1 -j18 -N2 terra-rover | tr -d ' \n')
[ "$magic" = 7f454c460201 ] && [ "$machine" = b700 ] || fail 'requires an ARM64 Linux ELF executable'

if [ -z "$destination" ]; then
    [ "$(uname -s)" = Linux ] && [ "$(uname -m)" = aarch64 ] || fail 'requires 64-bit Raspberry Pi OS'
    [ "$(id -u)" -eq 0 ] || fail 'run sudo ./install.sh'
    command -v systemctl >/dev/null 2>&1 || fail 'systemd is required'
    command -v dbus-send >/dev/null 2>&1 || fail 'install dbus first: sudo apt install dbus'
    command -v bluetoothctl >/dev/null 2>&1 || fail 'install BlueZ first: sudo apt install bluez'
    if systemctl is-active --quiet terra-rover.service; then
        fail 'isolate actuator power and stop terra-rover.service before upgrading'
    fi
    chmod 0755 terra-rover
    ./terra-rover --check-bundle || fail 'binary dependency check failed'
    getent group terra-rover >/dev/null || groupadd --system terra-rover
    if ! id terra-rover >/dev/null 2>&1; then
        useradd --system --gid terra-rover --home-dir /var/lib/terra-rover --no-create-home --shell /usr/sbin/nologin terra-rover
    fi
    for group in i2c bluetooth; do
        getent group "$group" >/dev/null || groupadd --system "$group"
        usermod --append --groups "$group" terra-rover
    done
fi

install -d -m 0755 "$destination/usr/local/bin" "$destination/etc/systemd/system" \
    "$destination/etc/dbus-1/system.d" "$destination/etc/terra-rover"
# Replace an executable through rename, so a failed copy cannot truncate it.
install -m 0755 terra-rover "$destination/usr/local/bin/.terra-rover.new"
mv -f "$destination/usr/local/bin/.terra-rover.new" "$destination/usr/local/bin/terra-rover"
install -m 0644 terra-rover.service "$destination/etc/systemd/system/terra-rover.service"
install -m 0644 terra-rover.conf "$destination/etc/dbus-1/system.d/terra-rover.conf"
if [ ! -e "$destination/etc/terra-rover/rover.env" ]; then
    install -m 0644 terra-rover.env "$destination/etc/terra-rover/rover.env"
fi
install -d -m 0700 "$destination/var/lib/terra-rover"

if [ -z "$destination" ]; then
    chown terra-rover:terra-rover /var/lib/terra-rover
    systemctl daemon-reload
    # Reload the bus policy without restarting the system bus or BlueZ.
    dbus-send --system --type=method_call --print-reply \
        --dest=org.freedesktop.DBus /org/freedesktop/DBus org.freedesktop.DBus.ReloadConfig
    if [ ! -e /var/lib/terra-rover/owner.json ] && [ ! -L /var/lib/terra-rover/owner.json ]; then
        systemctl enable --now terra-rover.service
        echo 'Installed and waiting for the Fusion HAT USR button. Hold 3 seconds, then pair in TerraPhone.'
    else
        echo 'Installed. Existing owner preserved; restart the service after checking hardware configuration.'
    fi
    echo 'See INSTALL.md for the Fusion HAT kernel driver and physical interlock requirements.'
else
    echo "Staged files in $destination; no host users or services were changed."
    echo 'Create the terra-rover user and i2c/bluetooth groups in the image before boot.'
fi
