# Task 4 implementation report

Added backend.py, package metadata and docs/hardware/FUSION_HAT.md. Source-read
self-review only; no tests, syntax checks, builds, smoke or hardware checks run or
written. Hardware compatibility and calibration remain physically unverified.

Backend protocol: capabilities(), configure(layout), apply(OutputBatch), read_gate(),
close(). BackendError exposes port and operation. MockBackend(pwm_ports=(),
occupied_resources=()) exposes gate=False, applied[id] mapped actual output, history
(id, port, mapped_value), fail_ports=set() for persistent injected failures.
FusionHatBackend(pwm_ports=(), gate_reader=None, *,
supported_library_versions=('1.14.0',), motor_factory=None, pwm_factory=None,
library_version=None, occupied_resources=()) loads factories lazily and checks
version, source resource map, timer assumptions and required APIs. Injection factories
must expose the same metadata and APIs. Hardware default advertises only M0–M3;
PWM ports must be explicitly supplied from physically confirmed exposed wiring.

Service integration: use explicit PWM port list and independent gate input file via
file_gate_reader(path), or injected physical GPIO reader. Reader must return bool;
exceptions/missing reader fail closed to motion. Enforce independent ownership and
fresh gate producer supervision. capabilities reports library_version='unloaded'
until successful configure. Validate against capabilities before configuration;
configure repeats validation before output construction. Repeated configure is supported. Real resource transfer requires gate open, safes
and explicitly closes old outputs before constructing new owners, and attempts
previous-layout rollback at safe outputs after a failure. Service must enforce disarmed state. Worker catches BackendError,
latches controller fault and continues safe batches each tick. Close keeps ESC
neutral/stop PWM continuously enabled and retains vendor objects; lifecycle shutdown
must independently isolate downstream equipment before vendor object destruction.

Review traced full batch membership validation before active writes, gate failure,
all-channel fallback after partial write errors, independent motor leg zero attempts,
resource aliases and 100/50 Hz timer conflicts, single inversion and continuous
ESC zero. Vendor constructor failures can leave inaccessible partially-created
resources; adapter attempts remaining safe channels, returns error, and physical
interlock remains required. Unidirectional inversion is explicitly unsupported
(zero must remain stop); configure rejects it before constructing outputs.

Ordinary close retains safe ESC output objects; process exit and object destruction
can disable pulses, so continuity cannot be guaranteed after service termination.

## Review fixes after a969438

Initial and repeated real configuration now check gate open before output construction.
Configure captures only failure text/port, recursively clears vendor exception
traceback frames and exception chains, and exits the exception handler before
closing candidate resources and constructing rollback replacements. Clearing frames
retires inaccessible partially-constructed vendor PWM owners before replacements
can exist. Explicitly closed known output owners are released before reopening.
Rollback errors are likewise sanitized; final configure BackendError has no vendor
exception cause. Source-read review traced write and constructor failure ownership,
retirement before rollback, and initial gate ordering. No execution checks performed.
