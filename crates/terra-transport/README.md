# terra-transport

Zenoh sessions for Terra, independent of Bevy and UniFFI.

`ControlPlane` is what the phone runs. `listen` opens a loopback-only dashboard endpoint; `connect` and `connect_authenticated` open an outbound client to a fleet router. It queues operator requests from `terra_types::requests` for the controller and publishes the topics in `terra_types::telemetry` with their byte caps. Transport status reports a session, not that a rover exists or moved. Reliable actuator stopping remains the receiver's responsibility if the link is unavailable.

`ControlPlane` also carries WebRTC signaling on the topics defined by `fpv-signaling`. Those samples are capped at 65536 bytes and are not control actions. `publish` will not latch them as telemetry. The session machine is not in this crate.

`RoverConnection` is the simulator client Zorvane's bridge tests use. `RoverConnection::connect(endpoint, prefix, rover_id)` opens a client with a 2-second connection deadline and multicast scouting disabled. Keep this call and disconnect off the UI thread. Refresh `set_target(linear, angular)` while driving: a worker publishes the latest target at 20 Hz as JSON with `linear` and `angular` on `<prefix>/<rover_id>/teleop`. Targets must be finite and within ±2 m/s and ±2 rad/s. Invalid input revokes the previous target. A 250 ms monotonic lease forces zero if refresh stops. Disconnect and Drop send a final zero before closing; Zorvane independently expires incoming commands after 500 ms. The default prefix simulator clients use is `terra/rover`.

```sh
cargo test -p terra-transport
```

The tests open local TCP sockets.
