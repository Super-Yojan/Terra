# Simulator and a physical iPhone

!!! tip "TL;DR"
    Compile-time split. Not a setting.
    Simulator: Zenoh to Zorvane.
    iPhone: Bluetooth. Saved Zenoh keys are deleted.

![Simulator wireframe: endpoint, rover id, Connect](../assets/phone-simulator.svg){ width="260" }

*Placeholder. Labels match the `#if targetEnvironment(simulator)` section in `ContentView`.*

![iPhone wireframe: Bluetooth, no Bevy section](../assets/phone-iphone.svg){ width="260" }

*Placeholder. `TerraPhoneApp` calls `discardOnDevice()` at launch.*

| | Simulator | iPhone |
| --- | --- | --- |
| Bevy · Zenoh | Shown | Hidden |
| Bluetooth Discover | Hidden | Shown |
| Simulated rover plant | Shown | Shown |
| Phone IMU + VIO | Needs a real phone | Shown |

[Terra #25](https://github.com/Super-Yojan/Terra/issues/25) asked for this. [PR #26](https://github.com/Super-Yojan/Terra/pull/26) landed it.

## Drive Zorvane

```sh
TERRA_ROVER_COUNT=1 TERRA_ZENOH_LISTEN=tcp/0.0.0.0:7447 cargo run -p zorvane
```

In the Simulator, connect to `tcp/127.0.0.1:7447`, rover `0`.

![Grid that fills after the first depth frame](../assets/occupancy.svg)

*Stop disconnects and keeps the last grid. There is no auto-reconnect.*

An iPhone does not join this peer over the LAN.

```sh
cargo test -p terra-transport -- --ignored
python3 scripts/check-zenoh-swift.py
```
