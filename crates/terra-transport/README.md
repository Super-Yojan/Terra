# terra-transport

Zenoh control plane for one rover, independent of Bevy and UniFFI. `ControlPlane::listen` opens a loopback-only dashboard endpoint; `ControlPlane::connect` and `connect_authenticated` open an outbound client session to an explicitly configured router. Multicast scouting is disabled.

Operator intent arrives on `<prefix>/<rover_id>/<suffix>` and is queued in a bounded inbox (256 entries, 2048 bytes each). Distinct safety stops and hardware disarms are never evicted and are drained last, so a stop dominates every other request in the same control tick. Transport carries intent only; the onboard runtime validates payloads and decides whether anything moves.

Telemetry is latched with a 500 ms TTL and replayed at 10 Hz while fresh. A stalled producer stops fleet advertisements rather than looking freshly observed. A lost router or failed publish latches the session failed with a named `failure_reason` in `diagnostics()`; the service does not reconnect.

```sh
cargo test -p terra-transport
```

`ControlPlane` also carries WebRTC signaling on the topics defined by `fpv-signaling`. Those samples are capped at 65536 bytes and are not control actions. `publish` will not latch them as telemetry. The session machine is not in this crate.
