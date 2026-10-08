# Simulation

The world simulator is [Zorvane](https://super-yojan.dev/Zorvane/). Terra PR #28 removed `simulator/` from this repository. Cameras, physics, tiles, and the Zenoh bridge live there. Terra keeps the crates those components call: state, control, waypoint following, mapping, local planning, the autonomy arbiter, and experiment logs.

From a Zorvane checkout:

```sh
cargo run -p zorvane
```

The Zenoh prefix stays `terra/rover`. `TERRA_*` variables still select the fleet, tiles, mission, and listen address. `TERRA_ZENOH_LISTEN` defaults to `tcp/127.0.0.1:7447`.

Tile worlds and the bridge contract are documented in that repo:

- [simulator/WORLD.md](https://github.com/Super-Yojan/Zorvane/blob/main/simulator/WORLD.md)
- [simulator/ZENOH.md](https://github.com/Super-Yojan/Zorvane/blob/main/simulator/ZENOH.md)

## Drive it from TerraPhone

Only the **iOS Simulator** build shows **Bevy simulator · Zenoh**. A physical iPhone uses Bluetooth and does not join this peer over the LAN.

On the Mac that is running the Simulator:

```sh
TERRA_ROVER_COUNT=1 TERRA_ZENOH_LISTEN=tcp/0.0.0.0:7447 cargo run -p zorvane
```

In TerraPhone, connect to `tcp/127.0.0.1:7447` and rover `0`. Details are in [Simulator and iPhone](phone/simulator.md).

## Mission scenario

The reference search runs in Zorvane. The arbiter stays in Terra.

```sh
TERRA_MISSION=1 TERRA_MISSION_SEED=42 TERRA_ROVER_COUNT=2 cargo run -p zorvane
```

`TERRA_HEADLESS=1` is for background rendering and local checks. The depth renderer still runs. Levels, safety, and log fields are in the [mission guide](autonomy/README.md). Summarize a JSONL log with `cargo run -p terra-experiment --bin terra-run-summary` from this checkout.

## What moved, and what did not

The Docker image and dev container that used to launch the Bevy world moved with the simulator. This repo's container is the Pi rover image. See [Containers](DOCKER.md) and [Develop without a Mac](DEVELOP-WITHOUT-MAC.md).

**Simulated rover** inside TerraPhone is a local motor plant. It does not start Zorvane. **Phone IMU + VIO** reads the handset. Neither one is a substitute for the Zorvane process.
