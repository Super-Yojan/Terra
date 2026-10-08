# Rover1: two bidirectional ESCs

Rover1 uses two bidirectional ESCs on Fusion HAT PWM outputs P0 and P1.
The phone offers a ready-to-edit preset with Left ESC on P0 and Right ESC on P1.
The port assignments and inversion can be edited before connecting. Applying
requires authenticated Bluetooth access, synchronized output capabilities and
a disarmed rover. Validate and stage, then commit the acknowledged draft.

The preset starts with 1500 µs neutral, 1000/2000 µs reverse/forward endpoints,
and a 2000 ms arming interval. Neutral and ports were confirmed by the user;
endpoint and arming values remain editable defaults rather than measured calibration.
Use inversion to correct the installed motor direction.

The battery switch provides the external power cutoff. The Pi does not measure
its position. Omitting `--gate-file` starts the service in
`gate_mode=external_power_cutoff`, with no Pi-connected interlock dependency.
Status reports `configuration_allowed` separately and does not claim
`hardware_gate_open_confirmed`. The updated phone understands this status.
Optional explicit `--gate-file` and `--bench-mode` installations retain their
existing policies.

Connecting, configuring, and resetting faults do not arm. Arming is explicit;
ESCs receive neutral while disarmed. The 200 ms command-loss watchdog,
emergency stop, authenticated owner access and disconnect behavior remain active.

This first version uses the existing actuator editor. CAD body selection and
3D tap-to-configure remain future work.

## Installed development runtime

On 2026-10-08 rover1 was switched to the tested Python runtime at
`/opt/terra-rover1-config-20261008/venv` using
`/etc/systemd/system/terra-rover.service.d/rover1-battery-cutoff.conf`.
Its original executable remains installed. The override preserves owner pairing
and uses P0/P1 with no gate file. This is a development runtime, not a newly
compiled release binary.

The phone had cached status handle `0x0024`, while BlueZ relocated the service
after restart. The override sets `TERRA_GATT_SERVICE_HANDLE=0x001b` to retain
this rover's commissioned service address (status at `0x0024`). The adapter
exposes BlueZ's optional read/write service `Handle` property; registration fails
if that range conflicts rather than silently moving the service. Other devices
must use their own commissioned range, or leave this setting unset to allocate
automatically. Pairing and encryption policies are unchanged.

The initial encrypted status read uses a compact admission document bounded to
512 bytes, preserving session, revision, arming/fault and configuration permission
fields. Complete diagnostics remain in status notifications. This avoids an
oversized attribute read being truncated into invalid JSON on iOS.

Live iPhone Mirroring verification reached authenticated configuration access,
confirmed the P0/P1 port picker, and received `Staged revision 0 · commit explicitly`
for the restored two-ESC preset. The draft was left staged, not committed; no
arming or drive command was issued. The Pi test suite passed all 51 tests.
