# terra-transport

Zenoh I/O for Terra, with no Bevy and no UniFFI types of its own. Two entry points serve two roles.

[API](https://super-yojan.dev/Terra/api/terra_transport/index.html) · [crate readme](https://github.com/Super-Yojan/Terra/blob/main/crates/terra-transport/README.md)

## `RoverConnection`

`RoverConnection::connect(endpoint, prefix, rover_id)` opens a Zenoh **client**. The endpoint must be `tcp/...`, at most 512 bytes, and a valid Zenoh endpoint. UDP is rejected. The prefix must be a key expression without `*`, at most 256 bytes. The usual prefix is `terra/rover`.

The client disables multicast scouting, sets a 2 s connect timeout, and exits on failure. Call `connect` and `disconnect` off the UI thread.

A worker publishes the latest target at 20 Hz (a 50 ms sleep) on `<prefix>/<rover_id>/cmd_vel`. The payload is JSON with exactly `linear` and `angular`. Values must be finite and within ±2 m/s and ±2 rad/s. An invalid target drops the previous one. The lease is **250 ms**: if `set_target` stops, the worker publishes zeros. `disconnect` and `Drop` publish a final zero and join the worker. Zorvane expires incoming commands on its own 500 ms watchdog.

The same session subscribes to `<prefix>/<rover_id>/camera/depth` and keeps the newest frame whose `sequence` increases. `decode_depth_frame` requires a version-1 header, encoding `32FC1_LE`, a newline, then little-endian `f32` metres. The header carries `exposure_time`, `vertical_fov`, and a camera pose (`x,y,z,qx,qy,qz,qw`). An optional `body` pose supplies rover `x`, `y`, and `yaw`; without it the body yaw is the optical forward axis. Intrinsics use a principal point of `(width/2, height/2) - 0.5`, matching `terra-mapping`. Frames wider or taller than 2048 pixels are dropped.

It also subscribes to `<prefix>/<rover_id>/autonomy/status` and stores the latest UTF-8 payload up to 64 KiB.

`send_action` queues `autonomy`, `safety`, `goal`, or `goal/decision` when the payload is JSON of at most 2048 bytes. The queue holds 64 actions.

`status` reports a session and a depth-frame count. That string means the TCP session is open. It does not mean a rover acknowledged motion.

## `ControlPlane`

`ControlPlane::listen` binds a Zenoh peer that **only** accepts `tcp/127.0.0.1:...`. `tcp/0.0.0.0:7447` returns `InvalidSettings`. This is the phone dashboard endpoint. LAN exposure is a separate, unbuilt decision.

It subscribes to `<prefix>/<id>/**` and keeps `autonomy`, `safety`, `goal`, `goal/decision`, `teleop`, and `cmd_vel` when the payload is at most 2048 bytes. Every 100 ms it publishes `<prefix>/fleet/state` as `{"count":1,"max_count":1,"ids":[id]}` and republishes any status strings the caller has stored.

## Tests

```sh
cargo test -p terra-transport
cargo test -p terra-transport -- --ignored
```

The ignored test checks topic selection, the wire format, lease expiry, and a zero on disconnect. It needs local TCP sockets.

## Follow-ups

The crate readme marks fleet/state subscriptions on the client, RGB subscriptions, and a reconnection UI as follow-ups. The client already receives depth and autonomy status. Commands sent to an inactive rover id are ignored by Zorvane. Stopping actuators when the link is gone remains the receiver's job: the Pi watchdog and the motor adapter, not this publisher.
