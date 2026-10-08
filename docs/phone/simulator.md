# Simulator and a physical iPhone

[Terra #25](https://github.com/Super-Yojan/Terra/issues/25) asked for one link per destination. [PR #26](https://github.com/Super-Yojan/Terra/pull/26) implemented it, and that change is on `main`. The split is compile-time `#if targetEnvironment(simulator)` in `ContentView` and `PhoneController`. There is no setting that shows the Bevy section on an iPhone.

`TerraPhoneApp` calls `TerraBevySessionStore.discardOnDevice()` at launch. On a device that deletes `zenohEndpoint` and `zenohRoverID` from `UserDefaults`. A stale Bevy endpoint from an older build does not reconnect.

| Target | Bevy simulator · Zenoh | Bluetooth actuators |
| --- | --- | --- |
| iOS Simulator | Shown. Endpoint and rover id persist in the Simulator's defaults. Connect publishes `cmd_vel` and subscribes to that rover's depth camera. | Hidden. The Simulator has no Bluetooth radio, so Discover and Connect cannot reach a rover. |
| Physical iPhone | Hidden. The section is not in the device UI. Launch deletes any stored endpoint and rover id. `startBevy` does not open a session. | Shown. This is the path to a real rover. |

**Simulated rover** in the Controller section is the local motor plant. It stays on both targets. **Phone IMU + VIO** still needs a physical iPhone with ARKit.

Bluetooth is omitted from the Simulator because CoreBluetooth cannot scan or connect there (`CBCentralManager` has no radio). Those controls would sit beside the link that does work: Zorvane at `tcp/127.0.0.1:7447` on the same Mac.

## Drive Zorvane from the Simulator

From a [Zorvane](https://super-yojan.dev/Zorvane/) checkout, on the same Mac as the Simulator:

```sh
TERRA_ROVER_COUNT=1 TERRA_ZENOH_LISTEN=tcp/0.0.0.0:7447 cargo run -p zorvane
```

Build and run TerraPhone in the Simulator (`./scripts/build-ios.sh`, then the Xcode project). In **Bevy simulator · Zenoh**, use `tcp/127.0.0.1:7447` and rover `0`, then connect. The occupancy section starts at "Waiting for simulator depth and exposure pose". After a depth frame arrives, the grid fills and the blue marker follows the published body pose. Stop disconnects and keeps the last grid. Rover `1` maps that rover only, which needs `TERRA_ROVER_COUNT` of at least 2.

Connecting starts at zero. The velocity sliders, or the waypoint follower after **Go to waypoint**, publish `cmd_vel`. Stop, switching modes, and leaving the foreground close the session with a final zero. The Rust publisher also expires its 250 ms lease if Swift stops refreshing it. Zorvane's 500 ms watchdog is independent. There is no automatic reconnect and no motion at startup.

The phone's local IMU, VIO, and motor-effort readouts are not remote feedback from Zorvane. Session-open status counts posed depth frames. It does not mean the rover moved.

A physical iPhone does not show this section, does not read a previously saved endpoint, and does not join Zorvane over the LAN. Use Bluetooth on that phone.

## Checks that do not need the GUI

```sh
cargo test -p terra-transport
cargo test -p terra-transport -- --ignored
python3 scripts/check-zenoh-swift.py
```

`check-zenoh-swift.py` needs the Eclipse Zenoh Python package. It checks topic, payload, lease expiry, a final zero, and one posed depth frame integrated into an occupied cell. In a Zorvane checkout, the moved world tests are `cargo test -p zorvane`. Seeing the grid in the Simulator still requires Zorvane to be running so the depth camera can publish.
