# Portable actuator layouts

This crate routes normalized commands and validates layouts against backend-supplied
capabilities. It does not arm outputs or apply inversion. The hardware adapter applies
`inverted` once while mapping normalized effort/position to power or calibrated pulses.
`validate_structure` checks portable configuration; `validate_layout` additionally checks
advertised ports, resource ownership, supported kinds, and shared timer frequencies.

All JSON enums use a `type` discriminator except `kind`, which is a snake_case string.
Pulse calibration fields are integer microseconds; arming durations are integer
milliseconds. Limits, coefficients, power fractions, and commands are normalized floats.
Capabilities supply `board`, `library`, `library_version`, `supported_kinds`, `ports`, and
`occupied_resources`. `ports` maps names to `{kinds, resources, timer, frequency_hz}`.
Use the actual backend ownership map (M0 occupies P11/P10, M1 P9/P8, M2 P6/P7,
M3 P4/P5 on the documented Fusion HAT map). No physical ports are inferred by this crate.

Editable JSON presets are in `presets`. PWM templates deliberately require selecting
an advertised port and checking calibration before use. They are not hardware
compatibility claims. Positive manual yaw turns left when the left turn coefficient is
negative and the right coefficient is positive. Servo routes default to their safe
position; a disabled safe policy has no positional target and defaults to bounded center.
No preset stores arming state. Revision zero is a draft; the configuration service owns
committed revision increments.
