# Hardware

The body side of Terra is a 64-bit Raspberry Pi running `terra-rover`, a compiled ARM64 executable, and a Fusion HAT motor and PWM board. TerraPhone on a physical iPhone is the controller. The Rust crates do not run on the Pi. The Python package in `hardware/raspberry-pi/` implements the Bluetooth protocol and the HAT backend.

| Guide | What it covers |
| --- | --- |
| [Install on a Pi](INSTALL.md) | Docker or on-Pi build, `install.sh`, `rover.env`, USR-button enrollment, bench mode. |
| [Bluetooth peripheral](BLUETOOTH.md) | GATT services, framing, owner admission, configuration operations. |
| [Fusion HAT](FUSION_HAT.md) | Library 1.14.0, motor pin map, timer groups, gate file. |
| [Bench procedure](BENCH.md) | Wheels-up commissioning outline. **Not performed.** It needs separate authorization. |
| [Evidence](evidence/README.md) | What was checked in the compiled bundle, and what remains unverified. |

The software stand-in for the motor adapter, `software_bench_sequence`, and the matching wheels-up checklist are in [terra-motors](../crates/terra-motors.md). That unit test does not power a driver.

Radio sessions, pulse timing, and electrical compatibility are unverified. A gate file is an input to software. The cutoff that removes propulsion power has to work when this process is not running. Zero effort and a dropped enable are coast, which is not a brake.
