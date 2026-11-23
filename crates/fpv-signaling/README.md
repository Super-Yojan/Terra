# fpv-signaling

Shared WebRTC FPV signaling for TerraPhone and the ARGOS dashboard. The crate encodes and decodes the six `webrtc/…` topics, builds their Zenoh keys, and runs one session state machine for the phone (offerer) or the dashboard (answerer).

It does not depend on Zenoh or on a WebRTC media stack. A signaling effect cannot arm the rover or publish a velocity command. Video capture stays in the iOS app. Playback stays in `argos-gpui`.

```sh
cargo test -p fpv-signaling
```

ARGOS `main` does not depend on Terra crates. The GPUI branch checks Terra out as a sibling and uses a path dependency (the same pattern as `terra-observability`). From `crates/argos-gpui/Cargo.toml` or `crates/argos-zenoh/Cargo.toml`:

```toml
fpv-signaling = { path = "../../../Terra/crates/fpv-signaling" }
```

The crate is not published to crates.io. See `docs/phone/fpv.md` for the topic contract, timeouts, and the sibling checkout.
