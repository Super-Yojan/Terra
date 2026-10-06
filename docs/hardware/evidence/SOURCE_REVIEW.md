# Bluetooth implementation source review

Implementation is retained locally on `codex/bluetooth-actuators`, forked at `557523d`. Product implementation and review fixes end at `fe2c231`.

Independent source reviews covered portable Rust routing/protocol, Python layout/safety/Fusion HAT/configuration/BlueZ, Swift BLE/UI integration, and documentation. A whole-branch review and scoped correction review reported all identified Important and Critical findings addressed. This is source inspection, not executed verification.

The user requested implementation without testing. No completed test run, build, binding generation, radio run, timing measurement, or physical hardware result is claimed. An initial baseline test command was interrupted when that instruction arrived. Compilation and runtime behavior remain unverified; issue acceptance requiring demonstrated round-trip/hardware behavior is not claimed.

## Rulings made during implementation

- Ruling: versioned capability maps will be backend-specific and unsupported hardware fails closed — documentation differs across library versions — cost if wrong: additional adapter work.

- Ruling: use numeric u32 request IDs and commit staged_request_id plus staged_revision — bind exact staged layout and prevent delayed commit selecting replacement — cost if wrong: wire schema migration before release.

- Ruling: require physical gate open for real output topology/calibration reconfiguration — old PWM objects can disable newly opened channels during destruction; release old owners before opening new — cost if wrong: extra cutoff step for mobile configuration or adapter lifecycle rework.

- Ruling: correct this load-bearing interaction within same final fix scope and re-review it — required firstconfig/faultrepair must remain usable; no new broad work or testing — cost if wrong: extra source revision/review.

## Local delivery

The feature remains on its isolated branch; it has not been merged, pushed, deployed, or used to close issue 10. Setup and protocol are documented in [Bluetooth](../BLUETOOTH.md), hardware mapping in [Fusion HAT](../FUSION_HAT.md), and unperformed future bench steps in [Bench](../BENCH.md).
