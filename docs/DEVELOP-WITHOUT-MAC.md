# Develop the world without a Mac

!!! tip "TL;DR"
    Zorvane runs on Linux.
    TerraPhone still needs a Mac.
    The Pi image also builds on Linux, with Docker.

![Splash from the phone, which you cannot compile here](assets/mountain-banner.jpg)

*World loop does not need this splash. The app does.*

```mermaid
flowchart LR
  Linux[Linux] --> Z[Zorvane]
  Linux --> Pi[Pi package]
  Mac[Mac] --> Phone[TerraPhone]
  Phone -->|Simulator| Z
```

*From a [Zorvane](https://super-yojan.dev/Zorvane/) checkout: `cargo run -p zorvane`.*

`TERRA_TILES`, `TERRA_ROVER_COUNT`, `TERRA_MISSION`, and `TERRA_ZENOH_LISTEN` still apply. Default listen is `tcp/127.0.0.1:7447`.

Build notes: [Zorvane README](https://github.com/Super-Yojan/Zorvane/blob/main/README.md).

Phone: `./scripts/build-ios.sh` on a Mac. See [Xcode](phone/build.md).
