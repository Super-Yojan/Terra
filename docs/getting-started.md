# Getting started

Rust checks run on Linux. TerraPhone needs a Mac with Xcode. The world needs a separate Zorvane checkout.

## Rust workspace

From this repository:

```sh
cargo test --locked --workspace
```

The workspace members are the crates under `crates/`. Edition is 2024. The shared package version is `0.1.0`. The Pi service is a separate Python package at version `0.2.0`.

Ignored transport tests open local TCP sockets:

```sh
cargo test -p terra-transport -- --ignored
```

Motor mapping, the enable gate, and the watchdog, including `software_bench_sequence`, run with the motors unit tests. They do not power a driver.

```sh
cargo test -p terra-motors
```

## TerraPhone

On a Mac, with Rust targets `aarch64-apple-ios`, `aarch64-apple-ios-sim`, and `x86_64-apple-ios`:

```sh
./scripts/build-ios.sh
./scripts/check-swift.sh
open mobile/ios/TerraPhone.xcodeproj
```

`build-ios.sh` generates Swift bindings and an XCFramework. Generated files are gitignored. The deployment target is iOS 17. Pick a signing team in Xcode, then run TerraPhone. The Simulator and a physical iPhone compile different connection sections; see [Simulator and iPhone](phone/simulator.md).

## World

From a [Zorvane](https://super-yojan.dev/Zorvane/) checkout:

```sh
cargo run -p zorvane
```

To drive it from the iOS Simulator on the same Mac:

```sh
TERRA_ROVER_COUNT=1 TERRA_ZENOH_LISTEN=tcp/0.0.0.0:7447 cargo run -p zorvane
```

In TerraPhone's Bevy section, use `tcp/127.0.0.1:7447` and rover `0`.

## Raspberry Pi

Build and install the ARM64 executable with the [Pi guide](hardware/INSTALL.md). Pair from TerraPhone with the USR button, described in [Bluetooth pairing](phone/bluetooth.md). Treat the [bench procedure](hardware/BENCH.md) as future commissioning: it has not been executed as acceptance evidence.

## Docs site

This site is MkDocs Material. Sources are `docs/`, and `mkdocs.yml` is at the repository root.

```sh
pip install -r docs/requirements.txt
mkdocs serve
```

The published site also hosts workspace rustdoc at [`/api/`](https://super-yojan.dev/Terra/api/). CI builds it with `cargo doc --no-deps --locked --workspace` and copies `target/doc` into the site. Pull requests run `mkdocs build --strict`. Pushes to `main` deploy with GitHub Pages actions.

Local API docs, after a workspace doc build:

```sh
cargo doc --no-deps --locked --workspace
mkdir -p site/api
cp -a target/doc/. site/api/
```

`mkdocs serve` does not copy rustdoc. Open `site/api/index.html` for the local API tree, or follow the links above once Pages has deployed.
