# terra-transport

Direct TCP Zenoh client for Terra velocity commands, independent of Bevy and UniFFI. `RoverConnection::connect(endpoint, prefix, rover_id)` opens a client with a 2-second connection deadline and multicast scouting disabled. Keep this call and disconnect off the UI thread.

Refresh `set_target(linear, angular)` while driving. A worker publishes the latest target at 20 Hz as JSON with exactly `linear` and `angular` on `<prefix>/<rover_id>/cmd_vel`. Targets must be finite and within ±2 m/s and ±2 rad/s. Invalid input revokes the previous target. No command queue retains old movement instructions. A 250 ms monotonic lease forces zero if refresh stops. Disconnect and Drop send a final zero before closing; the simulator independently expires incoming commands after 500 ms.

Transport status reports a session, not an acknowledgement that a rover exists or moved. Fleet/state subscriptions, RGB/depth subscriptions and reconnection UI are follow-ups. Commands sent to inactive IDs are ignored by the simulator. Reliable actuator stopping remains the receiver's responsibility if the link is unavailable.

```sh
cargo test -p terra-transport
cargo test -p terra-transport -- --ignored
```

The ignored test requires local TCP sockets and checks topic selection, wire format, lease expiry and zero on disconnect.
