# Real-phone ARGOS connection over Tailscale

This prototype uses the existing native Zenoh transport, with a router on the Mac:

```
ARGOS on Mac -> localhost:7448 -> Zenoh router <- Mac Tailscale IP:7448 <- TerraPhone
                                                                          |
                                                                     Bluetooth rover
```

## Mac setup

1. Enable Tailscale on the Mac and iPhone, signed into the same tailnet. Tailnet access rules must permit the phone to reach the Mac on TCP port 7448. Allow the router's incoming connection if the macOS firewall prompts.
2. From Terra, run `./scripts/dashboard-router.sh`. It listens only on localhost and the Mac's Tailscale IPv4 address. The port is 7448 to avoid the simulator's 7447 listener. Pass an address explicitly with `./scripts/dashboard-router.sh 100.x.y.z`, or use `--local` for loopback testing without Tailscale.
3. Leave the router process running. In ARGOS Connection, use endpoint `tcp/127.0.0.1:7448`, prefix `terra/phone`, and turn **Geographic map** off for initial ARKit local-coordinate testing.

The launcher uses the Rust dependency already pinned by Terra; a separate zenohd installation, MQTT broker, or AWS account is unnecessary. No public port forwarding is required.

## Phone setup

Build updated bindings with `./scripts/build-ios.sh`, then build/run TerraPhone on the physical iPhone using your Apple signing team.

1. Enable Tailscale and keep TerraPhone in the foreground.
2. For telemetry without actuators, choose **More → Controller → Phone IMU + VIO** and permit camera/motion access. For hardware, accept the rover pairing popup or let Terra reconnect to its remembered rover. Manual Bluetooth mode can attach to ARGOS without sensors; tracked pose and waypoint control require enabling phone feedback with a compatible actuator layout.
3. In **Settings → Fleet connection**, enter `tcp/<Mac Tailscale IP>:7448`, topic prefix `terra/phone`, and rover ID `0`. Terra connects automatically when the Bluetooth rover is authenticated and its configuration is available. Tracking then starts automatically in every ARGOS autonomy mode. ARGOS should discover that ID and show pose only as tracking becomes healthy.
4. Connection does not arm the Bluetooth rover. Check tracking/layout and arm explicitly on the phone before physical motion. Choose **Waypoint** autonomy on the phone before sending a waypoint from ARGOS; the current native ARGOS UI has no autonomy-level selector.
5. For a bench test, send a small local waypoint in ARGOS. **Sent** means publication; a matching phone goal token confirms acceptance. Cancel waits for an idle status. Keep the physical rover disarmed until telemetry and acknowledgements have been verified.

ARKit establishes a local frame on each tracking run. Do not assume it is compass-aligned or tied to the simulator's GMU geographic anchor. Initial tests use local coordinates only.

## Lifecycle and scope

- A ready Bluetooth rover connects to the saved router automatically. The endpoint still requires one-time configuration; Retry and manual Connect remain available.
- Explicit Disconnect suppresses retries for the current foreground session and clears remote goal/teleop intent, returns requested authority to teleop, and disarms Bluetooth. It preserves an emergency stop latch.
- Router loss is latched when Zenoh observes the connection disappearing. The phone clears remote intent and retries with delays of 1, 2, 4, 8, then 15 seconds. Network failure detection is not instantaneous. The rover's local watchdogs and sensor gates remain independent of this network.
- Backgrounding/stopping Terra closes its dashboard session and stops/disarms the controller. Returning reconnects the remembered rover and saved router, while leaving hardware disarmed. Browsing phone screens keeps telemetry running but revokes manual driving intent and disarms hardware.
- Telemetry expires after 500 ms without control publications. Pose is published only with healthy, fresh tracked VIO; ARGOS marks observations stale after 2.5 seconds without reception.
- **One phone per topic prefix in this slice.** Each phone emits a singleton fleet membership record, so multiple phones sharing a prefix would overwrite fleet membership. Use a separate prefix for a simulator, and do not mix it with the physical phone. Fleet aggregation is follow-up work.
- Physical phones publish lightweight `<prefix>/<id>/pose` JSON. ARGOS also keeps its existing simulator depth-header subscription. This adds no image/video streaming.
- Automated loopback tests verify routing, telemetry expiry, disconnect and command acknowledgement. They do not establish real iPhone signing, foreground sensor performance, Tailscale latency, firewall access, or physical rover motion. Verify these on hardware before a study.

## Troubleshooting

Use `tailscale status` and `tailscale ip -4` on the Mac. If Tailscale is stopped, enable it before starting the router. The phone needs the Mac's Tailscale address, never phone localhost. If the router cannot bind the Tailscale address, check that the tunnel is enabled. If the session opens but no rover appears, check the matching topic prefix, active phone sensor controller, and app foreground state.

## Repeating local verification

Run `cargo test --workspace --locked` in Terra and ARGOS. After rebuilding both Apple bindings/libraries, run `./scripts/check-dashboard-swift.sh` from Terra. It expects ARGOS beside Terra (override `ARGOS_ROOT` otherwise), Swift on macOS, and the existing Python `eclipse-zenoh` package. It starts a temporary loopback router, exercises both generated Swift clients, and closes the router on completion.

On October 8, 2026, Terra's 68 workspace tests passed (two existing live remote tests ignored), ARGOS's 15 passed, the paired Swift smoke passed, and TerraPhone device/Simulator plus ARGOS Mac builds succeeded. Builds used `CODE_SIGNING_ALLOWED=NO`; they do not establish device installation. This Mac's recorded Tailscale address was `100.112.151.78`, but Tailscale was stopped and its registered physical iPhone was unavailable. Enable Tailscale and confirm the current address before hardware testing.


### Scene telemetry and remote driving

While phone tracking is active, Terra publishes `localization` (1 Hz), `pointcloud` (about 3 Hz), and the existing observed `map/occupancy` snapshots (up to 5 Hz) below the configured `prefix/<rover-id>`. Point clouds use LiDAR depth on supported phones and sparse ARKit features otherwise. Occupancy requires scene depth; feature points alone are not advertised as observed free space. Location permission affects geographic alignment only.

ARGOS requests `autonomy` with a token and reads `autonomy/status` before enabling manual input. `teleop` carries bounded linear/angular speed, run ID, authority revision, operator session and increasing sequence. Idle phone controls do not continuously republish neutral over remote intent; a held local input retains priority and release publishes neutral once. The arbiter retains its existing 0.5-second operator timeout, sensor-health gating and emergency stop. Physical movement still requires the phone's normal hardware setup/arming; ARGOS takeover does not arm hardware.

The dashboard presents L0 teleop, L1 assisted teleop and L3 obstacle-aware waypoint mode. L4 target search is enabled only when the rover advertises a fresh target detector; the simulator survivor detector is the first adapter. Physical phones have no detector registered by default. L2 obstacle-free waypoint and L5 fleet decision making remain unavailable. The backend's supervised frontier exploration remains a separate approval-based mode. See [target search](autonomy/TARGET_SEARCH.md).


Cloud and occupancy messages have a dedicated 256 KiB publisher budget (matching ARGOS), while ordinary telemetry remains bounded to 64 KiB. Dense clouds and 200×200 occupancy grids exceed 64 KiB; older Terra builds silently dropped them, producing a connected dashboard with fresh pose but no cloud/map. The loopback regression sends real-sized JSON snapshots and checks exact receipt and oversized-message rejection.
