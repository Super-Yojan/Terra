# Compiled Raspberry Pi release validation

Date: 2026-10-07. Scope: compiled packaging and single-button customer enrollment.
This updates packaging evidence, without changing the historical record for the
original actuator implementation or claiming physical commissioning.

## Built artifact

`hardware/raspberry-pi/dist/terra-rover-0.1.1-linux-arm64.tar.gz`

SHA256:
`592d221b32b56f3a910149953dd555900c58e4f37af7df4df20012da5b7b1c85`

The artifact is generated locally and excluded from Git. Rebuilding can change
the archive digest. It contains the ARM64 ELF executable, installer, service,
default configuration, D-Bus policy, build metadata, compilation report, exact
rover source/build recipe, and pinned Fusion HAT GPLv3 source/license.

Build environment: ARM64 Debian Bookworm, glibc 2.36, Python 3.11.17,
Nuitka 4.2.2, dbus-next 0.2.3, fusion_hat 1.14.0 at commit
`4bd1018ad5a70ee113160536f969cdadd9f22918`.

## Executed checks

- `docker buildx build --platform linux/arm64 -f packaging/rover/Dockerfile --output type=local,dest=hardware/raspberry-pi/dist .`: passed. The actual ARM64 onefile executable ran `--help` and `--check-bundle` outside the source directory.
- The Docker verification stage ran the executable, staged the release with `install.sh --root /image`, and ran the installed executable in clean `debian:bookworm-slim` with no Python installation: passed.
- `python3 -m unittest discover -s hardware/raspberry-pi/tests -v`: 27 tests passed locally and in the ARM64 builder. They exercise button debounce/hold/release, owner validation/private atomic persistence, single-candidate authorization and disconnect revocation, setup read/commit/cleanup failures, customer lifecycle and clean shutdown, as well as executable/service installation, private state permissions, upgrade preservation of environment configuration/owner/name/layout, and rejection of corrupt, missing, or wrong-architecture files before destination writes.
- Live installation path in a disposable ARM64 Debian container: account creation, i2c/bluetooth memberships, private state ownership, executable permissions, real D-Bus policy reload, and execution as `terra-rover` all passed. `systemctl is-active`, `daemon-reload`, and `enable --now` were stubbed because the container had no systemd PID 1. No real boot/startup test is claimed.
- `systemd-analyze verify /etc/systemd/system/terra-rover.service` in that container: passed.
- iOS simulator app build (`xcodebuild`, no signing): passed. Swift → UniFFI → Rust smoke: passed.
- Compiled customer waiting mode with temporary button/LED files and no gate: startup, persistent name, no owner creation, and SIGTERM shutdown passed in the clean no-Python runtime.
- Independent review identified failure handling issues; regression tests cover the fixes.
- Archive checksum verification: passed.
- Shell syntax, Python syntax compilation, and `git diff --check`: passed.
- `cargo test --workspace --locked --offline`: passed, with the two existing TCP-socket-dependent terra-transport tests marked ignored by the suite. Rust source was not changed by this packaging work.

## Remaining verification limits

No physical Raspberry Pi, Fusion HAT board, independent gate producer, Bluetooth
radio pairing, iOS interaction, actuator output, or timing measurements were used.
The Fusion HAT kernel module/device-tree overlay must be installed for the Pi's
running kernel. Actual systemd startup and sysfs write access require a Pi check.
The GitHub workflow was added but has not been dispatched or published.

See [installation instructions](../INSTALL.md) and the separately authorized
[bench procedure](../BENCH.md).

## Advertisement correction — version 0.1.2

Physical rover2 logs exposed an interface-name collision: the advertising local
name overwrote `ServiceInterface.name`. Corrected Advertisement to store
`local_name`, and the same collision in normal GATT characteristics to use `role`.
29 Python tests passed, including regressions for exported interface identity,
advertisement properties and status-read routing. Independent focused review
found no remaining issues. ARM64 compilation, no-Python bundle/startup checks and
archive verification passed. Retained source matches the reviewed fix.

Release: `hardware/raspberry-pi/dist/terra-rover-0.1.2-linux-arm64.tar.gz`.
Actual Bluetooth discovery after upgrading rover2 still needs verification;
rover2 is on a separate network and could not be reached from this computer.

## Bless migration — 0.2.0 (2026-10-07)

GATT service/characteristic creation and advertisement registration now use the
pinned Bless 0.3.0 BlueZ backend, with Bleak 1.1.1. A narrow characteristic
adapter preserves BlueZ's device/MTU/offset options for encrypted owner checks;
the library's published callbacks omit those options. BlueZ pairing agents and
bond inspection remain necessary for the physical enrollment flow.

The 37 source tests pass, including enrollment through the actual Bless backend
objects on a simulated BlueZ bus, advertisement interface/name/UUID inspection,
partial registration cleanup, owner persistence before confirmation, encrypted
request identity, prepared-write rejection/disarm callbacks, and subscription
reset. Independent code review found two startup cleanup/adapter-selection issues;
both were corrected and rechecked. Phone discovery and radio behavior on rover2
remain unverified until the new package is installed there.

The final ARM64 Docker build passed all 37 tests and compiled with Nuitka 4.2.2.
A clean Debian Bookworm ARM64 verification stage with no Python installed passed
`--help`, `--check-bundle`, staging install, and unowned startup/SIGTERM checks.
The final binary reports Bless 0.3.0, Bleak 1.1.1, dbus-next 0.2.3 and Fusion HAT
1.14.0. Its retained source archive matches the reviewed source files.

An isolated ARM64 container also passed live installer account/group creation,
mode-0700 state ownership, mode-0755 installed executable, D-Bus configuration
reload, service-account bundle check, and systemd unit validation. The container
has no systemd PID 1, so service-manager calls were stubbed; this does not prove
actual service startup or Bluetooth radio operation on rover2.

Archive SHA-256:
`a1e9ede449f3bcfa25682f3623236c67fe901b9c8936190b66bec87eec4f2a3c`
