#!/usr/bin/env bash
# Warm Cargo and compile the simulator so the first `cargo run` is incremental.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
# shellcheck source=cargo-jobs.sh
source .devcontainer/cargo-jobs.sh

git config --global --add safe.directory "$(pwd)" || true
if command -v git-lfs >/dev/null 2>&1; then
  git lfs install --skip-repo
  git lfs pull || echo "git lfs pull failed; rover.glb stays a pointer until Git LFS can authenticate."
fi

if [[ -n "${CARGO_TARGET_DIR:-}" ]]; then
  sudo mkdir -p "${CARGO_TARGET_DIR}"
  sudo chown -R "$(id -u):$(id -g)" "${CARGO_TARGET_DIR}"
fi
if [[ -n "${CARGO_HOME:-}" ]]; then
  sudo mkdir -p "${CARGO_HOME}/registry" "${CARGO_HOME}/git"
  sudo chown -R "$(id -u):$(id -g)" "${CARGO_HOME}/registry" "${CARGO_HOME}/git"
fi

echo "Rust: $(rustc --version)"
echo "CARGO_BUILD_JOBS=${CARGO_BUILD_JOBS:-<default>}"
free -h

if [[ ! -f /etc/vulkan/terra-lvp.json ]]; then
  echo "lavapipe ICD /etc/vulkan/terra-lvp.json is missing" >&2
  exit 1
fi
# DISPLAY is set for the desktop. vulkaninfo then tries to create an X
# window and exits non-zero on lavapipe; the headless query is the check.
echo "Vulkan devices:"
env -u DISPLAY vulkaninfo --summary

echo "Installing Python Zenoh client dependencies..."
python3 -m pip install -r simulator/tools/requirements.txt

echo "Fetching crates..."
cargo fetch --locked --manifest-path Cargo.toml
cargo fetch --locked --manifest-path simulator/Cargo.toml

echo "Building the workspace and the simulator..."
cargo build --locked --workspace
cargo build --locked --manifest-path simulator/Cargo.toml

echo
echo "Terra dev container is ready."
echo "Desktop: forwarded port 6080 (noVNC), password vscode."
echo "Start the simulator: ./scripts/sim-remote.sh"
