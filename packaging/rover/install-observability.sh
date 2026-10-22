#!/bin/bash
# Install only the independent observer. Run from a reviewed Terra source bundle.
set -euo pipefail
cd "$(dirname "$0")/../.."
[[ $EUID -eq 0 ]] || { echo 'Run this installer as root' >&2; exit 1; }
[[ -f /etc/terra-observability-pi/uploader.env ]] || {
    echo 'Provision the scoped uploader.env and token file first; see operations.md' >&2
    exit 1
}
getent group terra-observability >/dev/null || groupadd --system terra-observability
id -u terra-observability >/dev/null 2>&1 || useradd --system --gid terra-observability --home-dir /var/lib/terra-observability-pi --shell /usr/sbin/nologin terra-observability
install -d -o root -g root -m 755 /opt/terra-observability-pi/terra_rover
install -o root -g root -m 644 hardware/raspberry-pi/terra_rover/observability.py hardware/raspberry-pi/terra_rover/log_forwarder.py hardware/raspberry-pi/terra_rover/usb_queue.py /opt/terra-observability-pi/terra_rover/
touch /opt/terra-observability-pi/terra_rover/__init__.py
chmod 644 /opt/terra-observability-pi/terra_rover/__init__.py
install -d -o terra-observability -g terra-observability -m 700 /var/lib/terra-observability-pi
chown root:terra-observability /etc/terra-observability-pi /etc/terra-observability-pi/uploader.env /etc/terra-observability-pi/upload.token
chmod 750 /etc/terra-observability-pi
chmod 640 /etc/terra-observability-pi/uploader.env /etc/terra-observability-pi/upload.token
install -o root -g root -m 644 packaging/rover/terra-log-forwarder.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now terra-log-forwarder
systemctl restart terra-log-forwarder
# Does not restart, pair, arm or drive the rover service.
