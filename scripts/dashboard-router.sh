#!/bin/sh
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$repo_root"
if [ "${1:-}" = "--local" ]; then
    exec cargo run --locked -p terra-transport --example dashboard-router
fi
address=${1:-}
if [ -z "$address" ]; then
    if ! command -v tailscale >/dev/null 2>&1; then
        echo 'Install and enable Tailscale, or pass the Mac Tailscale IPv4 address.' >&2
        exit 1
    fi
    status=$(tailscale status 2>&1) || { printf '%s\n' "$status" >&2; exit 1; }
    case "$status" in
        *'Tailscale is stopped'*|*'Logged out'*)
            echo 'Enable Tailscale on this Mac before starting the phone router.' >&2
            exit 1 ;;
    esac
    address=$(tailscale ip -4)
fi
exec cargo run --locked -p terra-transport --example dashboard-router -- "$address"
