#!/usr/bin/env bash
set -euo pipefail

export DISPLAY=:0
export XDG_RUNTIME_DIR=/tmp/terra-runtime
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

Xvfb "$DISPLAY" \
  -screen 0 "${TERRA_DISPLAY_MODE:-1600x900x24}" \
  -nolisten tcp -noreset +extension GLX +render &

for _ in $(seq 1 40); do
  if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    break
  fi
  sleep 0.25
done
xdpyinfo -display "$DISPLAY" >/dev/null

export WGPU_BACKEND="${WGPU_BACKEND:-vulkan}"

if [[ "${TERRA_SOFTWARE_RENDERING:-1}" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  lavapipe_icd="$(find /usr/share/vulkan/icd.d -maxdepth 1 -name 'lvp_icd*.json' -print -quit || true)"
  if [[ -n "$lavapipe_icd" ]]; then
    export VK_ICD_FILENAMES="$lavapipe_icd"
  else
    echo "Lavapipe Vulkan ICD was not found; Vulkan device selection will use installed drivers." >&2
  fi
fi

x11vnc -display "$DISPLAY" -forever -shared -nopw -listen 0.0.0.0 -rfbport 5900 -quiet &
websockify --web=/usr/share/novnc/ 6080 127.0.0.1:5900 &

exec /usr/local/bin/terra-simulator
