# Develop the simulator without a Mac

The Bevy rover simulator runs in a Linux dev container. GitHub Codespaces opens that container in the browser. VS Code Dev Containers opens the same container on Windows or Linux. The container is the stand-in for the phone: it builds the simulator, shows the Bevy window on a virtual desktop, and speaks Zenoh on `tcp/127.0.0.1:7447`.

The iOS app under `mobile/ios` needs Xcode and is not part of this container.

Real-terrain tiles (`TERRA_TILES`) and the Zenoh go-to-waypoint command (`terra/rover/<id>/goal`) arrived with [PR #14](https://github.com/Super-Yojan/Terra/pull/14). This container does not add that protocol. Current `main` includes it, so the commands below work in this environment. Details live in [world design](../simulator/WORLD.md#real-world-tiles) and the [Zenoh waypoint contract](../simulator/ZENOH.md#go-to-waypoint).

[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/Super-Yojan/Terra)

## What the container includes

- Rust stable, new enough for edition 2024 (1.85 or newer), with rustfmt, clippy, rust-src, and rust-analyzer
- Bevy’s system libraries (`pkg-config`, `build-essential`, Wayland, X11, XKB, Vulkan, ALSA, udev)
- Mesa lavapipe / llvmpipe, selected with `WGPU_BACKEND=vulkan` and `VK_ICD_FILENAMES=/etc/vulkan/terra-lvp.json`
- Python 3 and the Zenoh client packages from `simulator/tools/requirements.txt` (`eclipse-zenoh>=1,<2`, `numpy`)
- A Fluxbox desktop on noVNC port **6080** (VNC port 5901, 1440×768, 24-bit color), password `vscode`
- Cargo registry, git, and target directories on Docker volumes, so rebuilding the container keeps the compile cache
- `lld` as the linker, and dev builds with line tables only (`CARGO_PROFILE_DEV_DEBUG=line-tables-only`), so a Bevy debug build fits in memory

The simulator is excluded from the root Cargo workspace. Build it with `cargo build --manifest-path simulator/Cargo.toml`, or use the helper below.

## Open in GitHub Codespaces

The dev container asks for the 8-core, 32 GB RAM, 64 GB disk Codespace. That is the smallest GitHub machine with enough RAM and disk for this Bevy graph (Bevy, Avian, voxel hills, procedural trees, water).

1. Click **Open in GitHub Codespaces** above, or open <https://codespaces.new/Super-Yojan/Terra>, and create the codespace. Accept the 8-core machine.
2. Wait until the post-create script finishes. The first session compiles the workspace and the simulator. Later `cargo run` calls are incremental. The log ends with `Terra dev container is ready.`
3. In the terminal, start the simulator and open the desktop:

   ```sh
   ./scripts/sim-remote.sh
   ```

   In the Ports panel, open **noVNC desktop** (port 6080). Click **Connect** and enter the password `vscode`. The Bevy window is on that desktop.

Keep port 6080 private to the codespace. The desktop password is the only extra gate if the port is shared.

## Open locally in VS Code (Windows or Linux)

1. Install [Docker](https://docs.docker.com/get-docker/) and [VS Code](https://code.visualstudio.com/). On Windows, use Docker Desktop with the WSL2 backend. Install the **Dev Containers** extension.
2. Clone the repository and open the folder in VS Code. Run **Dev Containers: Reopen in Container** from the command palette. VS Code builds `.devcontainer` and runs the same post-create script.
3. Start `./scripts/sim-remote.sh`. Open forwarded port 6080 (**noVNC desktop**), click **Connect**, and enter `vscode`.

A local machine with 16 GB of RAM can compile. `scripts/sim-remote.sh` and the post-create script cap `CARGO_BUILD_JOBS` at 1 below 20 GB, and at 2 below 28 GB. The first compile is still long. 32 GB is the comfortable size and is what Codespaces requests.

## View the simulator

`scripts/sim-remote.sh` waits for the virtual display (`DISPLAY=:1`), forces software Vulkan, and runs `cargo run` in `simulator/`. Extra arguments are passed to Cargo:

```sh
./scripts/sim-remote.sh
TERRA_ROVER_COUNT=1 ./scripts/sim-remote.sh
TERRA_TILES=1 ./scripts/sim-remote.sh
./scripts/sim-remote.sh --release
```

`TERRA_TILES=1` is the real-elevation world from PR #14. Leave it unset for the practice town (roads, buildings, trees, pond, one rover).

The window title is the Bevy app. Software rendering draws the town slowly; the window can sit on a black frame for a bit while shaders compile on lavapipe. Right-click the Fluxbox desktop for the window menu if the window is behind the terminal.

A native VNC client can use forwarded port 5901 with the same password, at 24-bit color.

## Send commands from another terminal

Open a second terminal in the same container (the Zenoh peer listens on `tcp/127.0.0.1:7447`; no separate router). From `simulator/`:

```sh
python3 -m pip install 'eclipse-zenoh>=1,<2' 'numpy>=1.26,<3'   # already installed in the container
python3 tools/zenoh_client.py fleet
python3 tools/zenoh_client.py --rover 0 drive --linear 1.0 --angular 0.3 --seconds 5
```

The client repeats the twist at 20 Hz and sends zero when it finishes. The rover stops on its own if commands stop for 500 ms.

PR #14’s goal command is one publish, not a stream. On the practice town, a local goal is metres from the spawn:

```sh
python3 tools/zenoh_client.py --rover 0 goto --x 12 --y -4
```

With the tile world already running (`TERRA_TILES=1 ./scripts/sim-remote.sh`), a WGS84 goal about 12 m north of the George Mason Johnson Center is:

```sh
python3 tools/zenoh_client.py --rover 0 goto --lat 38.82981 --lon -77.3075 --token gmu-north
```

That publishes once on `terra/rover/0/goal`. The client prints `goal/status` until `state` is `arrived`. Cancel with `python3 tools/zenoh_client.py --rover 0 goto --cancel`. While a goal is latched, twists do not preempt it.

Port 7447 is forwarded and labeled **Zenoh peer**. VS Code and Codespaces reach a listener on `127.0.0.1` through that forwarder, so the default bind works for a terminal on your laptop that targets the forwarded port. A client that arrives through Docker’s published port (the packet hits the container’s network interface, not loopback) needs:

```sh
TERRA_ZENOH_LISTEN=tcp/0.0.0.0:7447 ./scripts/sim-remote.sh
```

and `--endpoint tcp/HOST:7447` on the client. There is no authentication on this peer.

## Performance

Lavapipe is a CPU Vulkan implementation. Expect the simulator to run slower than realtime, often a few frames per second, with a long pause the first time shaders compile. Use one rover. Camera readback and extra rovers cost more. This is enough to see the world, drive, and exercise Zenoh. It is not a realtime hardware-in-the-loop target.

Debug builds in the container keep line tables and link with `lld`. Unset `CARGO_PROFILE_DEV_DEBUG` when you want full debug info. `./scripts/sim-remote.sh --release` is faster at runtime and slower to compile.

## Out of scope

- The iOS app, `./scripts/build-ios.sh`, `./scripts/check-swift.sh`, and `mobile/ios/TerraPhone.xcodeproj`. Those need a Mac with Xcode.
- GPU passthrough. The container always prefers lavapipe.
- An always-on hosted simulator. A possible follow-up is a server with a real GPU, a browser video stream, and an authenticated Zenoh router so ARGOS can connect without opening this dev container. This repository does not run that service.

## Troubleshooting

**Post-create seems stuck.** The first Bevy compile often takes most of an hour on a cold cache. The step is done when the log prints `Terra dev container is ready.` A later container rebuild reuses the Cargo volumes.

**`cargo` runs out of memory.** Use the 8-core / 32 GB Codespace. Locally, leave `CARGO_BUILD_JOBS` to the helper (1 under 20 GB, 2 under 28 GB). The dev container also raises `/dev/shm` to 2 GB, which Bevy and the desktop need.

**noVNC shows a connection error.** Wait until the desktop entrypoint has started, then open port 6080 again. The password is `vscode`. `echo $DISPLAY` should print `:1`.

**The Bevy window never appears, or the process exits.** Run `vulkaninfo --summary` in the container. The device name should mention lavapipe or llvmpipe. Confirm `VK_ICD_FILENAMES` is `/etc/vulkan/terra-lvp.json` and that the file exists. `scripts/sim-remote.sh` exits if `DISPLAY=:1` is not up yet.

**The window is black for a long time.** Shader compilation on lavapipe is slow the first run. Leave the process running. stderr from `cargo run` shows progress.

**The Python client cannot connect.** Start the simulator first. From inside the container the endpoint is `tcp/127.0.0.1:7447`. If you published the port with Docker rather than the editor’s forwarder, set `TERRA_ZENOH_LISTEN=tcp/0.0.0.0:7447`.

**`python3` cannot import `zenoh`.** The image installs the client into `/opt/terra-venv` and puts that interpreter on `PATH` as `python3`. Re-run `python3 -m pip install -r simulator/tools/requirements.txt`.

**Tile or goal commands are missing.** They arrived with PR #14. `python3 tools/zenoh_client.py goto --help` lists them on a checkout that includes that PR. `TERRA_TILES=1` selects the elevation world; the practice town is the default.

**iOS scripts fail.** Expected on this container. Use a Mac for `mobile/ios`.

## Continuous integration

[`.github/workflows/devcontainer.yml`](../.github/workflows/devcontainer.yml) builds this dev container with [devcontainers/ci](https://github.com/devcontainers/ci) and, inside it, runs `vulkaninfo`, `cargo build` and `cargo test` for the simulator, and the Python client unit tests. GPU render tests stay `#[ignore]` and are not part of that job. The job adds swap on the GitHub-hosted runner and sets `CARGO_BUILD_JOBS=2`.
