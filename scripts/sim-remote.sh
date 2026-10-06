#!/usr/bin/env bash
# Launch the Terra simulator on the dev container virtual desktop.
# The Bevy window appears in the noVNC desktop (forwarded port 6080).
#
#   ./scripts/sim-remote.sh
#   TERRA_TILES=1 ./scripts/sim-remote.sh
#   ./scripts/sim-remote.sh --release
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../.devcontainer/cargo-jobs.sh
source "${ROOT}/.devcontainer/cargo-jobs.sh"

export DISPLAY="${DISPLAY:-:1}"
export WGPU_BACKEND="${WGPU_BACKEND:-vulkan}"
export WGPU_POWER_PREFERENCE="${WGPU_POWER_PREFERENCE:-low_power}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"

if [[ -z "${VK_ICD_FILENAMES:-}" || ! -f "${VK_ICD_FILENAMES}" ]]; then
  icd=""
  for candidate in \
    /etc/vulkan/terra-lvp.json \
    /usr/share/vulkan/icd.d/lvp_icd.json \
    /usr/share/vulkan/icd.d/lvp_icd.x86_64.json
  do
    if [[ -f "${candidate}" ]]; then
      icd="${candidate}"
      break
    fi
  done
  if [[ -z "${icd}" ]]; then
    echo "No lavapipe Vulkan ICD found. Install mesa-vulkan-drivers or set VK_ICD_FILENAMES." >&2
    exit 1
  fi
  export VK_ICD_FILENAMES="${icd}"
fi
export VK_DRIVER_FILES="${VK_DRIVER_FILES:-${VK_ICD_FILENAMES}}"

if command -v xdpyinfo >/dev/null 2>&1; then
  ready=0
  for _ in $(seq 1 60); do
    if xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
      ready=1
      break
    fi
    sleep 0.5
  done
  if [[ "${ready}" -ne 1 ]]; then
    echo "X display ${DISPLAY} is not available." >&2
    echo "In the dev container, open forwarded port 6080 and wait for the desktop, then retry." >&2
    exit 1
  fi
else
  echo "xdpyinfo is not installed; starting without a display check." >&2
fi

echo "DISPLAY=${DISPLAY}"
echo "WGPU_BACKEND=${WGPU_BACKEND}"
echo "VK_ICD_FILENAMES=${VK_ICD_FILENAMES}"
echo "Open the desktop on forwarded port 6080 (noVNC). Password: vscode."
echo "From another terminal, in simulator/: python3 tools/zenoh_client.py fleet"

cd "${ROOT}/simulator"
exec cargo run "$@"
