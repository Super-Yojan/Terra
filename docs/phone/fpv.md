# FPV video

> [!TIP]
> **TL;DR**
> The iPhone sends one rear-camera video track to ARGOS over WebRTC.
> Signaling rides the existing fleet Zenoh session.
> Drive commands stay on `teleop` and `hardware`. The video path cannot arm or move the rover.

The phone and the dashboard are often on different networks. WebRTC uses ICE, with a public STUN server by default and TURN when you configure it. There is no Tailscale dependency for the video.

## Topics

`P` is the fleet prefix (`terra/phone` on the hosted router) and `{id}` is the rover id.

| Key | Direction | JSON |
| --- | --- | --- |
| `P/{id}/webrtc/request` | dashboard to phone | `{"v":1,"action":"start"\|"stop","session":"<uuid>"}` |
| `P/{id}/webrtc/offer` | phone to dashboard | `{"v":1,"session","sdp"}` |
| `P/{id}/webrtc/answer` | dashboard to phone | `{"v":1,"session","sdp"}` |
| `P/{id}/webrtc/ice/phone` | phone to dashboard | `{"v":1,"session","candidate","sdpMid","sdpMLineIndex"}` |
| `P/{id}/webrtc/ice/dashboard` | dashboard to phone | same candidate object |
| `P/{id}/webrtc/state` | phone to dashboard | `{"v":1,"session","state"}` |

`state` is `connecting`, `connected`, `failed`, or `closed`. An empty `candidate` means the sender is done gathering. `sdpMid` may be an empty string.

The phone creates the offer after `start`. One call is active at a time. A `start` with a new session id replaces the current call. A repeated `start` for the live session is ignored. After a failure, the same session id can start again. Messages for any other session id are ignored.

The offer must appear within **15 seconds** of `start`, the answer within **20 seconds** of the offer, and the connection within **30 seconds** of the answer. A timeout fails the call. The dashboard's timeout also sends `request` `stop`, so the phone does not keep a live session that would ignore the next `start`. A phone timeout publishes `state` `failed` and keeps the session until `stop` or a new `start`. Connected calls do not time out from signaling; a later ICE failure is a media state.

Unknown JSON fields on these six topics are ignored. The required fields above are still required. Control topics are unchanged and still drop unknown fields.

Session ids are UUIDs. Comparison ignores letter case, and the phone echoes the spelling from the `start` that created the call. `v` must be the JSON number `1`. A JSON boolean is rejected. `sdpMLineIndex` must be an integer from 0 through 2147483647.

## Size limit

Control payloads stay capped at **2048 bytes**. Signaling payloads have their own cap of **65536 bytes**. A larger sample is dropped. WebRTC topics are not control topics: they never enter the autonomy inbox, so an SDP cannot evict a stop or be decoded as `teleop`, `hardware`, or `safety`.

Offers and candidates are published once. They are not replayed with the 500 ms telemetry cache.

## Media

One video track, send only, from the rear camera. No audio. No data channel is created, and a data channel opened by the dashboard is ignored.

The encoder list is H.264 first (hardware encoder on iPhone), then VP8. The sender asks for about 720p30, with bitrate from 150 kbit/s to 1.5 Mbit/s and balanced degradation when the link tightens.

While phone tracking is running, frames come from the AR session so a second camera capture is not opened. If tracking is off, the app opens the rear camera itself. If that camera is busy or permission is denied, FPV reports failure and leaves arming and velocity alone.

The image is rotated for the portrait mount (top edge up, rear camera forward). Confirm the picture on a device.

## ICE configuration

| Setting | Store | Default |
| --- | --- | --- |
| STUN URL | UserDefaults `fpvStunURL` | `stun:stun.l.google.com:19302` |
| TURN URL | UserDefaults `fpvTurnURL` | empty (TURN off) |
| TURN username | UserDefaults `fpvTurnUsername` | empty |
| TURN credential | Keychain service `Terra.ARGOS`, account `fpv-turn` | empty |

Set them in **Settings → Fleet connection → FPV video**. Saving those fields does not reconnect the fleet or arm the rover. Blank or invalid STUN falls back to the public server. TURN is included only when the URL starts with `turn:` or `turns:`.

Do not commit TURN credentials.

## What you see in the app

The home screen shows **FPV** with Idle, Connecting, Live · streaming, or Failed. **Stop FPV** ends the call. The dashboard's `stop` does the same. Neither changes arming. Disconnecting the fleet, or leaving the app, also ends the call.

## Shared crate

Protocol types, JSON rules, topic keys, and the session machine live in [`fpv-signaling`](../crates/fpv-signaling.md) (`crates/fpv-signaling`). The iPhone uses that crate through the existing `MobileController` FFI (`fpvReceive`, `fpvNoteOffer`, `fpvNoteIce`, `fpvNoteMedia`, `fpvTick`, `fpvStop`). Google's native WebRTC stays in the app and only carries media. ARGOS should use the same crate for the dashboard role (`SignalingSession::dashboard`) and its own player for display.

The crate depends on `serde_json` only. It does not depend on Zenoh or WebRTC. Zenoh ingress and egress stay in `terra-transport`: the signaling queue, the 65536-byte cap, and the refusal to latch these topics as telemetry.

ARGOS `main` does not link Terra. The GPUI dashboard branch is the precedent: CI checks Terra out as a sibling of the ARGOS repo, and a crate manifest points at that checkout. `fpv-signaling` is not on crates.io. From `crates/argos-gpui/Cargo.toml` or `crates/argos-zenoh/Cargo.toml`:

```toml
fpv-signaling = { path = "../../../Terra/crates/fpv-signaling" }
```

Pin the sibling checkout to this branch until it is on `main`, the same way `.github/scripts/checkout-observability.sh` pins a Terra revision. A git dependency also resolves the crate, because it is a workspace member:

```toml
fpv-signaling = { git = "https://github.com/Super-Yojan/Terra.git", branch = "cursor/webrtc-fpv-ios-c3c2" }
```

Use the path line. It matches the sibling checkout ARGOS already uses.

## Try it with the ARGOS dashboard

1. On a Mac, regenerate bindings and open the project. This VM cannot compile the iOS app.

    ```sh
    rustup target add aarch64-apple-ios aarch64-apple-ios-sim x86_64-apple-ios
    ./scripts/build-ios.sh
    open mobile/ios/TerraPhone.xcodeproj
    ```

    Xcode resolves the Swift package `https://github.com/stasel/WebRTC.git` at version **155.0.0**.

2. Run TerraPhone on an iPhone. The simulator has no rear camera and no ARKit world tracking, so FPV fails cleanly there.
3. Join the same fleet router and prefix as the ARGOS dashboard, using the ARGOS FPV branch.
4. From the dashboard, publish `webrtc/request` with `action` `start` and a UUID `session`.
5. The phone publishes `webrtc/offer`, trickle candidates on `webrtc/ice/phone`, and `webrtc/state`. The dashboard sends `webrtc/answer` and `webrtc/ice/dashboard` for that same session.
6. Confirm the rover's armed state does not change when the video connects, stops, or fails. Drive from the existing teleop controls.

Signaling checks on a desktop:

```sh
swiftc mobile/ios/TerraPhone/FpvIceSettings.swift \
  tests/ios/FpvSignalingTests.swift -o /tmp/terra-fpv-tests
/tmp/terra-fpv-tests
cargo test -p fpv-signaling
cargo test -p terra-transport --test signaling
cargo test -p terra-mobile --test webrtc_signaling
```
