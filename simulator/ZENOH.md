# Terra fleet, camera frames, and Zenoh control

The simulator starts one rover by default. Choose an initial count from 0 through
32 with `TERRA_ROVER_COUNT`:

```sh
cd /Users/yojan/git/Terra/simulator
TERRA_ROVER_COUNT=3 cargo run
```

Every rover has its own physics body, differential-drive controller, odometry,
front RGB camera, and depth camera. New rovers spawn along the clear roads with
spacing between their chassis. Live spawning waits if the available road positions
are occupied. Reducing the count removes the newest rovers and their camera/model
children. IDs start at zero, remain stable for surviving rovers, and are never
reused during a run. After shrinking and growing, IDs may have gaps.

The overview camera and on-screen depth preview follow the primary rover in
spawn slot zero. The current limit is `terra::MAX_ROVERS = 32`; rendering costs
increase with two camera views per rover.

## Python client

Install dependencies in your preferred Python environment:

```sh
python3 -m pip install 'eclipse-zenoh>=1,<2' 'numpy>=1.26,<3'
```

From the simulator directory, list current rover IDs:

```sh
python3 tools/zenoh_client.py fleet
```

Increase or decrease the fleet during the run, including reducing it to zero:

```sh
python3 tools/zenoh_client.py fleet --count 5
python3 tools/zenoh_client.py fleet --count 2
```

Receive RGB and depth frames for rover zero, or all rovers:

```sh
python3 tools/zenoh_client.py --rover 0 frames --output /tmp/terra-frames
python3 tools/zenoh_client.py frames --all --output /tmp/terra-frames
```

Each rover's output directory contains its latest `rgb.npy`, `depth.npy`, JSON
metadata, and an RGB PPM image. `rgb.npy` has shape `(height, width, 4)` and uint8
sRGB RGBA channels. `depth.npy` has shape `(height, width)` and float32 axial
metres; NaN means no valid depth return.

Drive a particular rover for five seconds:

```sh
python3 tools/zenoh_client.py --rover 0 drive --linear 1.0 --angular 0.3 --seconds 5
```

The client repeats commands at 20 Hz and sends zero on completion or Ctrl-C.
Forward velocity is in m/s; positive angular velocity turns left in rad/s. Wheel
speed saturation and Avian collisions still apply. Each rover stops when it has
no fresh command for 500 ms, or the transport fails. Holding Space stops all
remote-controlled rovers. Invalid commands do not renew the timeout.

Zenoh owns driving while enabled. For local W/A/S/D control of the primary rover,
start with `TERRA_ZENOH=0`; fleet size can still be changed through the Bevy
resource API below, but network operations are disabled.

## Topics and wire formats

Default prefix: `terra/rover`. There is no shared command topic that drives every
rover. The per-rover keys are:

| Key | Payload |
| --- | --- |
| `terra/rover/<id>/cmd_vel` | JSON `{"linear":1.0,"angular":0.3}` |
| `terra/rover/<id>/camera/rgb` | Frame packet, `RGBA8_SRGB` |
| `terra/rover/<id>/camera/depth` | Frame packet, `32FC1_LE` |
| `terra/rover/fleet/size` | JSON `{"count":3}` |
| `terra/rover/fleet/state` | JSON `{"count":3,"max_count":32,"ids":[0,1,2]}` |

Fleet state is published on changes and every second for clients that connect
later. Count is the actual number of spawned rovers; it acknowledges a size
request once the spawn/removal completes. Commands for inactive IDs are ignored.
Drive and fleet requests must be valid JSON with exactly the fields above and
fit within 2048 bytes. Count must be an integer from 0 through 32.

A frame packet is a UTF-8 JSON header, a single newline byte, then contiguous
pixel bytes. The header contains:

```json
{"rover_id":0,"version":1,"width":256,"height":192,"sequence":12,"received_at":2.5,"encoding":"32FC1_LE","vertical_fov":1.0471976,"near":0.05,"far":30.0,"exposure_time":2.4,"camera":{"x":0.3,"y":0.0,"z":0.4,"qx":0.0,"qy":0.0,"qz":0.0,"qw":1.0},"body":{"x":0.0,"y":0.0,"yaw":0.0}}
```

`exposure_time`, `camera`, and `body` are present on depth packets that were paired with a GPU exposure. They are omitted otherwise, including every RGB packet. Version stays 1; older clients ignore the extra fields. `camera` is the optical pose (X right, Y down, Z forward) in the robotics world, Z up, using the same Bevy conversion as the simulator’s local occupancy map. `body` is the rover origin in that frame and the yaw of body forward, sampled with the camera at exposure time. This is not an occupancy-grid topic. A phone or other client integrates the depth image itself.

Pixels are row-major, starting at the top left, without GPU row padding. Depth
uses one little-endian float32 per pixel. RGB uses four uint8 channels per pixel.
Dimensions and lens parameters come from each rover's depth-camera configuration.
For pixel centres `(u + 0.5, v + 0.5)`, the pinhole focal length in pixels is
`fx = fy = height / (2 * tan(vertical_fov / 2))`, with principal point
`(width / 2, height / 2)`. `terra-mapping` addresses integer pixels, so the phone
decoder passes principal point `(width / 2 - 0.5, height / 2 - 0.5)`, matching
the simulator’s own occupancy integration.

RGB and depth cameras share their pose, resolution, and projection, but GPU
readbacks complete independently. Sequences are per-camera readback counters;
matching sequence numbers do not guarantee matching exposures. `received_at` is
CPU receipt time in seconds since the simulation started, not exposure time.
This interface does not promise synchronized RGB-D pairs.

Frames publish at up to 10 Hz per stream per rover, with only the latest unsent
frame retained. Older unsent frames are replaced, and congestion may drop frames.
At the default 256×192 resolution, the two raw streams total about 3.75 MiB/s per
rover before protocol overhead. Rendering/readback rates depend on the GPU and
fleet size.

## Configuration and Bevy API

The default Zenoh peer listens on `tcp/127.0.0.1:7447`, without multicast discovery.
The supplied client connects directly; no separate router is required.

| Environment variable | Purpose |
| --- | --- |
| `TERRA_ROVER_COUNT` | Initial count, default 1 |
| `TERRA_ZENOH` | Set to `0` to disable the bridge |
| `TERRA_ZENOH_LISTEN` | Listener endpoint |
| `TERRA_ZENOH_PREFIX` | Topic prefix |
| `TERRA_ZENOH_CONFIG` | Full Zenoh JSON5 configuration file; replaces default session configuration |

For a client on another computer, set the listener to `tcp/0.0.0.0:7447` and pass
`--endpoint tcp/SIMULATOR_IP:7447` to the client. Match any custom prefix with
`--prefix`. Restart after correcting a session startup error; the bridge logs
errors and keeps remote drive commands stopped.

Other Bevy systems can change the fleet with `ResMut<RoverFleet>::set_count(n)`
and inspect the requested count with `count()`. Fleet state reports the actual count. `TerraPlugin` reconciles the requested count in `PreUpdate`.
Query rover entities with `(With<Rover>, &RoverId)` to find a specific controller;
`DriveCommand` and `Odometry` remain per-rover components. Camera entities also
carry `RoverId`, along with `RgbFrame` or `DepthFrame`.

`ZenohBridgeConfig` configures publication rate and command timeout in code.
Camera resolution, field of view, and range use `DepthCameraConfig`.

## Verification

Ordinary tests: `cargo test`. Graphics and networking checks:

```sh
cargo test -- --include-ignored --test-threads=1
TERRA_ZENOH_TEST_PYTHON=python3 cargo test zenoh_round_trip -- --ignored --test-threads=1
python3 -m unittest discover -s tools -p 'test_*.py'
cargo clippy --all-targets -- -D warnings
```

The optional Python test uses the installed Zenoh and NumPy packages. Tests cover
live resizing, ID retirement, command isolation, timeout stopping, RGB/depth
payload decoding, real TCP sessions, and two simultaneous GPU camera pairs with
cleanup after a rover is removed.

## iOS client

TerraPhone can publish velocity commands directly to this bridge and build a local occupancy map from the depth stream. Configure the endpoint and rover ID in the app’s **Bevy simulator · Zenoh** section. Use localhost in iOS Simulator or the Mac’s LAN address on a physical phone; for LAN access bind this simulator to `tcp/0.0.0.0:7447`. See [phone demo and verification](../docs/MOBILE_CONTROL.md#drive-bevy-from-terraphone-over-zenoh). The app refreshes a leased Rust publisher, sends at 20 Hz, and sends zero when stopped or backgrounded. While connected it subscribes to `terra/rover/<id>/camera/depth`, keeps the latest posed frame, and integrates it with `MobileOccupancyMap`. The on-screen grid is phone-local. Fleet/state, RGB, and a Zenoh occupancy snapshot for other operators are follow-ups.
