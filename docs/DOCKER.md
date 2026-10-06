# Run the simulator in Docker

The container installs and builds the simulator, its Rust/Bevy native
dependencies, the Python Zenoh client dependencies, and the virtual display
stack. It uses Mesa Lavapipe software Vulkan and streams the simulator window
to a browser with noVNC. The host needs Docker with the Compose plugin; Rust,
Cargo, Python, and the Bevy system packages are installed inside the image.

## Build and start

From the repository root:

```sh
docker compose -f docker/compose.yaml up --build
```

The first build downloads Rust and Python dependencies and compiles the
simulator. It may take several minutes. Open
<http://localhost:6080/vnc.html?autoconnect=1&resize=scale> to see the
simulator window. Click in the display to send keyboard input; WASD and Space
control the simulator. Stop the container with Ctrl-C.

The virtual display defaults to 1600 × 900 at 24-bit color. Change
`TERRA_DISPLAY_MODE` in `docker/compose.yaml` if you need another size.
Software rendering works without an NVIDIA driver or host GPU passthrough and
may be slower than hardware rendering.

## Python client

The image installs the packages in `simulator/tools/requirements.txt`. With
the simulator running, run client commands inside its container:

```sh
docker compose -f docker/compose.yaml exec terra-simulator python tools/zenoh_client.py fleet
docker compose -f docker/compose.yaml exec terra-simulator python tools/zenoh_client.py --rover 0 drive --linear 1.0 --angular 0.3 --seconds 5
```

The Zenoh peer is forwarded on `localhost:7447` for a client running on the
host.

## Tests

The build stage contains the Rust toolchain and native development packages:

```sh
docker build --file docker/Dockerfile --target build -t terra-simulator-build .
docker run --rm terra-simulator-build cargo test --workspace
docker run --rm terra-simulator-build cargo test --manifest-path simulator/Cargo.toml
docker compose -f docker/compose.yaml exec terra-simulator python -m unittest discover -s tools -p 'test_*.py'
```

The ordinary simulator test command skips tests marked `#[ignore]`, including
GPU/display-dependent checks.
