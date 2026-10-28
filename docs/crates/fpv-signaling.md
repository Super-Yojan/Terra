# fpv-signaling

> [!TIP]
> **TL;DR**
> Shared FPV signaling for the phone and ARGOS.
> Six JSON topics, one session machine, no Zenoh and no video.

The crate is `crates/fpv-signaling`. TerraPhone reaches it through `MobileController`. The dashboard should construct `SignalingSession::dashboard()` and play the video itself.

Signaling cannot arm or drive. Topic keys look like `terra/phone/{id}/webrtc/offer`. The byte cap is 65536. Timeouts are 15 seconds for the offer, 20 seconds for the answer, and 30 seconds to connect.

ARGOS consumes it as a sibling path, not from crates.io:

```toml
fpv-signaling = { path = "../../../Terra/crates/fpv-signaling" }
```

That line belongs in `crates/argos-gpui/Cargo.toml` or `crates/argos-zenoh/Cargo.toml`, with Terra checked out next to the ARGOS repo. The contract and the git alternative are in [FPV video](../phone/fpv.md).

```sh
cargo test -p fpv-signaling
```
