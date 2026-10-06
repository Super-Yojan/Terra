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

Drive a particular rover for five seconds. This is the debug twist path:

```sh
python3 tools/zenoh_client.py --rover 0 drive --linear 1.0 --angular 0.3 --seconds 5
```

The client repeats commands at 20 Hz and sends zero on completion or Ctrl-C.
Forward velocity is in m/s; positive angular velocity turns left in rad/s. Wheel
speed saturation and Avian collisions still apply. Each rover stops when it has
no fresh command for 500 ms, or the transport fails. Holding Space stops all
remote-controlled rovers. Invalid commands do not renew the timeout.

Send one go-to-waypoint goal instead of streaming twists. Terra latches it and
drives with the onboard velocity loop. See [Go to waypoint](#go-to-waypoint).

```sh
python3 tools/zenoh_client.py --rover 0 goto --x 12 --y -4
python3 tools/zenoh_client.py --rover 0 goto --lat 38.8299 --lon -77.3075
python3 tools/zenoh_client.py --rover 0 goto --cancel
```

Zenoh owns driving while enabled. A latched goal overrides debug twists until
it is cancelled. For local W/A/S/D control of the primary rover, start with
`TERRA_ZENOH=0`; fleet size can still be changed through the Bevy resource API
below, but network operations are disabled. Real-world tiles are independent of
Zenoh; see [WORLD.md](WORLD.md#real-world-tiles).

## Topics and wire formats

Default prefix: `terra/rover`. There is no shared command topic that drives every
rover. The per-rover keys are:

| Key | Payload |
| --- | --- |
| `terra/rover/<id>/cmd_vel` | JSON `{"linear":1.0,"angular":0.3}` (debug twist) |
| `terra/rover/<id>/goal` | JSON go-to-waypoint, published once and latched |
| `terra/rover/<id>/goal/status` | JSON progress for the latched goal |
| `terra/rover/<id>/camera/rgb` | Frame packet, `RGBA8_SRGB` |
| `terra/rover/<id>/camera/depth` | Frame packet, `32FC1_LE` |
| `terra/rover/fleet/size` | JSON `{"count":3}` |
| `terra/rover/fleet/state` | JSON `{"count":3,"max_count":32,"ids":[0,1,2]}` |

Fleet state is published on changes and every second for clients that connect
later. Count is the actual number of spawned rovers; it acknowledges a size
request once the spawn/removal completes. Commands for inactive IDs are ignored.
Drive and fleet requests must be valid JSON with exactly the fields above and
fit within 2048 bytes. Count must be an integer from 0 through 32. Goal
requests use the same size cap; the accepted shapes are below.

## Go to waypoint

`cmd_vel` remains the debug teleop path: ARGOS or TerraPhone may stream twists,
and the 500 ms watchdog still applies. The operator contract for a mission goal
is one message on `terra/rover/<id>/goal`. Terra does not require a refresh.
The follower itself is the `terra-waypoint` crate. TerraPhone imports it as
`MobileWaypoint` from the UniFFI bundle and runs `step` on the phone, then
passes the twist to `MobileController`. The simulator does not contain a second
follower: its Zenoh bridge calls that same crate and writes the resulting
`DriveCommand` into the velocity loop. A fresh `cmd_vel` does not preempt a
latched goal. Publish `{"cancel":true}` before using debug teleop again.
Holding Space zeros the wheels and leaves the goal latched.

The goal body is one of these objects and nothing else:

```json
{"frame":"local","x":12.0,"y":-4.0}
{"frame":"local","x":12.0,"y":-4.0,"yaw":0.4,"token":"goal-1"}
{"frame":"wgs84","latitude":38.8299,"longitude":-77.3075}
{"cancel":true}
```

`x` and `y` are metres in the same robotics frame as depth `body`: `x` is
`-Bevy Z`, `y` is `-Bevy X`, and yaw 0 faces +x. In the flat world that is
simply the practice square, spawn at `(0, 0)`. With `TERRA_TILES=1`, `+x` is
north and `+y` is west of the configured latitude and longitude. Optional `yaw`
is radians in `[-2π, 2π]` and is the heading to hold after the position is
reached. Optional `token` is 1 to 64 characters from `[A-Za-z0-9._:-]` and is
echoed so a client can correlate a goal. `wgs84` is accepted only while a tile
anchor is loaded; Terra converts it into local metres. Targets outside the
ground square (one metre inside the edge) are ignored and do not replace the
current goal. Commands for an id that is not spawned are ignored.

Terra publishes `terra/rover/<id>/goal/status` when the state changes and a few
times a second while a goal is active:

```json
{"state":"active","goal_id":4,"token":"goal-1","distance":6.2,"x":12.0,"y":-4.0,"yaw":0.4,"latitude":38.8299,"longitude":-77.3075}
```

`state` is `idle`, `active`, or `arrived`. `goal_id` increases for each accepted
goal and is `0` when idle. `latitude` and `longitude` are the goal that was
sent. `x` and `y` are that same goal after the tangent-plane projection
(`x` north, `y` west), not the rover pose. `distance` is the remaining
horizontal metres. `token`, `yaw`, `latitude`, and `longitude` are omitted when
the goal did not set them. Arrival is within 0.75 m, and
within about 0.12 rad when a final yaw was set. The follower slows inside 3 m,
turns in place when the heading error is large, and writes a body twist into
the existing velocity loop. It does not plan around obstacles; the chassis
stops on Avian collisions.

`python3 tools/zenoh_client.py goto` publishes that JSON once and prints status
until `arrived`, or until `idle` after `--cancel`. Step-by-step build, tile,
and test commands are in [WORLD.md](WORLD.md#reproduce-and-test). The phone-facing
goal is latitude and longitude. About 12 m north of the Johnson Center stays
clear on the bundled Fairfax patch:

```sh
python3 tools/zenoh_client.py --rover 0 goto --lat 38.82981 --lon -77.3075 --token gmu-north
```

Key `terra/rover/0/goal`, body `{"frame":"wgs84","latitude":38.82981,"longitude":-77.3075,"token":"gmu-north"}`.
Watch `terra/rover/0/goal/status` for `"state":"arrived"`. `x` and `y` in that
status are the projected metres, not the command.

### ARGOS follow-up

ARGOS does not speak this topic yet. `docs/DESIGN.md` there already says goal
keys land in `src/argos/contract.py` after Terra specifies them. The follow-up
in `Super-Yojan/ARGOS` is:

- Add `TerraTopics.goal(rover_id)` → `<prefix>/<id>/goal` and `goal_status` → `<prefix>/<id>/goal/status`.
- Add `encode_goal` for the three bodies above, with the same 2048-byte cap as `encode_twist`.
- Add a CLI (and later a supervisor call) that publishes **once**, then watches `goal/status` until `goal_id` advances and `state` is `arrived`. Do not refresh the goal at 20 Hz.
- Leave `cmd_vel` / `encode_twist` marked as the debug path. A latched Terra goal ignores twists until `{"cancel":true}`.

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
