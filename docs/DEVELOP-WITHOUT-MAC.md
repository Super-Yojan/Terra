# Develop the world without a Mac

The Bevy world simulator is [Zorvane](https://github.com/Super-Yojan/Zorvane), not this repository. Terra keeps the vehicle body, onboard autonomy, TerraPhone, and the Pi rover.

From a Zorvane checkout:

```sh
cargo run -p zorvane
```

The Zenoh prefix is still `terra/rover`. `TERRA_*` variables still apply, including `TERRA_TILES`, `TERRA_ROVER_COUNT`, `TERRA_MISSION`, and `TERRA_ZENOH_LISTEN` (default `tcp/127.0.0.1:7447`). How to build, run headless, and test is in the [Zorvane README](https://github.com/Super-Yojan/Zorvane/blob/main/README.md).

TerraPhone still needs a Mac with Xcode. Build it from this repo with `./scripts/build-ios.sh`.
