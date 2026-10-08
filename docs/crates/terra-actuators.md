# terra-actuators

Portable actuator layouts, routing, and Bluetooth frame helpers. The phone validates a draft here before it stages the layout. The Pi service validates again in Python. This crate does not arm outputs and it does not apply inversion. The hardware adapter applies `inverted` once while it maps normalized effort or position onto power or pulses.

[API](https://super-yojan.dev/Terra/api/terra_actuators/index.html) · [crate readme](https://github.com/Super-Yojan/Terra/blob/main/crates/terra-actuators/README.md)

## Two validators

`validate_structure` checks the portable JSON: ids, kinds, limits, and calibration fields. `validate_layout` also checks the backend capability advertisement: ports, resource ownership, supported kinds, and shared timer frequencies.

JSON enums use a `type` discriminator, except `kind`, which is a snake_case string. Pulse fields are integer microseconds. Arming durations are integer milliseconds. Limits, coefficients, power fractions, and commands are normalized floats.

Capabilities supply `board`, `library`, `library_version`, `supported_kinds`, `ports`, and `occupied_resources`. Each port is `{kinds, resources, timer, frequency_hz}`. The documented Fusion HAT map is M0 on P11/P10, M1 on P9/P8, M2 on P6/P7, and M3 on P4/P5. This crate does not infer physical ports. Pass the backend's ownership map.

## Presets

Editable drafts live in [`crates/terra-actuators/presets`](https://github.com/Super-Yojan/Terra/tree/main/crates/terra-actuators/presets):

- `terra-mini.json`
- `esc-template.json`
- `mixed-servo-template.json`

The PWM templates contain `SELECT_*_PWM_PORT` markers so they stay invalid until a real advertised port is chosen. They are not a claim that a given ESC or servo is compatible. Example pulse widths are placeholders.

Positive manual yaw turns left when the left turn coefficient is negative and the right coefficient is positive. Servo routes default to their safe position. A disabled safe policy has no positional target and defaults to a bounded center. No preset stores arming state. Revision zero is a draft. The configuration service owns committed revision increments.

Unidirectional ESC inversion is rejected, because that channel cannot represent reverse.

## Routing

`ActuatorValue` is an id (`u8`) and a normalized `f32`. `route_commands` maps a `RoutingInput` through the layout coefficients onto those values. The phone calls the UniFFI wrapper `actuator_route`. The Pi applies the resulting command only after its own gate, arming, and watchdog checks. See [Bluetooth](../hardware/BLUETOOTH.md).
