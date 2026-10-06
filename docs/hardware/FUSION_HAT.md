# Fusion HAT actuator backend

Hardware operation and calibration remain physically unverified. Source compatibility
is based on SunFounder fusion_hat 1.14.0:
[version](https://github.com/sunfounder/fusion-hat/blob/HEAD/fusion_hat/_version.py),
[motor](https://github.com/sunfounder/fusion-hat/blob/HEAD/fusion_hat/motor.py),
[PWM](https://github.com/sunfounder/fusion-hat/blob/HEAD/fusion_hat/pwm.py).
HEAD links can change; deployment must retain the reviewed installed source.

The adapter lazily loads fusion_hat at configure, checks exact version against
`supported_library_versions` (default only 1.14.0), Motor.MOTOR_PINS,
Motor.DEFAULT_FREQ=100, PWM.CHANNEL_NUM=12 and required callable APIs. A version
string/API shape does not certify electrical compatibility. No dependency install
runs implicitly. Mock mode imports without fusion_hat or Raspberry Pi support.

Motor.power accepts signed percent: normalized effort 0.5 at maximum power fraction
1 maps to 50. Backend applies inversion exactly once with library reversal disabled.
PWM.pulse_width takes integer microseconds; PWM.enable controls output. Bidirectional
ESC zero maps to neutral_us; unidirectional zero maps to stop_us, continuously enabled.
Unidirectional inversion is rejected because it cannot represent reverse direction.
Servos map around center_us, with configured position or disabled safe policy.

## Physical resources and deployment inputs

M0 uses P11/P10, M1 P9/P8, M2 P6/P7, M3 P4/P5. Each motor uses 100 Hz.
Numerical PWM channels P0–P11 exist in library source but are **not** all assumed
physically exposed. Provide `pwm_ports` as explicit canonical names verified against
the actual board and wiring. Default is empty. Pulse outputs use 50 Hz. Timer groups
are channel integer divided by four: P0–P3, P4–P7, P8–P11. Mixed motor/pulse use
within one timer group is rejected even when channels do not overlap. Direct aliases
such as M0 and P10 are rejected. `occupied_resources` reserves channels owned by
other hardware clients; deployment must also prevent external timer reconfiguration.

Provide an independent physical `gate_reader` returning exact bool True only when
closed. Missing reader, read exception, non-bool result, or closed adapter returns
False. `file_gate_reader(path)` reads an independently maintained ASCII input file;
only trimmed `1` means closed. The GPIO/interface producer must actively report
current input, including failure/open states. A stale file is unsafe: supervision
and electrical fail-open wiring belong to deployment. Gate is separate from BLE
and must not be a simulated software enable in hardware mode.

Service CLI should expose explicit PWM port list and gate input path and construct
FusionHatBackend(pwm_ports=..., gate_reader=file_gate_reader(...)). Layout changes require disarmed service state and physical gate open. Reconfigure
safes and explicitly closes old PWM resources before opening new owners; rollback
reopens the previous layout at safe outputs after failure. Service owns disarmed enforcement.
Apply errors attempt safe output independently on every channel and return
BackendError(port, operation, message). Worker must latch fault and continue applying
safe batches each tick. Motor leg zero attempts are independent after partial power
errors. Constructor failures may leave vendor-created resources inaccessible;
physical gate and power isolation remain necessary. Close requests safe outputs,
retains objects and keeps ESC safe pulses enabled; do not call vendor PWM.close
until downstream equipment is independently isolated.

## Required physical commissioning (not performed)

With propulsion isolated, confirm installed source/version, board exposed pins and
resource ownership; measure shared timer periods and pulses with an instrument.
Check min/center/max or stop/neutral/full calibration against each actuator's approved
range. Confirm inversion once, safe output on gate opening, disconnect/watchdog,
write failure, restart and shutdown. Record board/library identities and measured
results separately. Source capability checks and saved JSON are not physical
verification or calibration certification.

Ordinary close retains safe ESC output objects; process exit and object destruction
can disable pulses, so continuity cannot be guaranteed after service termination.
