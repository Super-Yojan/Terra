# terra-robot

One rover's control loop, with no transport or recording I/O of its own
beyond the attached fleet link and recorder.

- `Robot::update(&SensorData) -> MotorCommand`: the phone's single per-tick
  entry point. The level module proposes, the safety gate decides and the
  actuator frame is routed here.
- `Core`: arbiter, estimator, velocity loop, map and request admission.
  Zorvane drives it directly with `ingest` and `step`.
- `level`, `levels`: the `LevelModule` trait, L0 `Teleop` and the `Planned`
  adapter that L1 to L5 use until each gets its own module.
- `safety`: the motor gate (link, e-stop, lease, tracking, routes, mount).

Full page: `docs/crates/terra-robot.md`.

## Verification

- `cargo test -p terra-robot -p terra-mobile`
- Zorvane `cargo test -p zorvane simulator_sensor_adapter`

Zorvane's `terra-source.json` pins the sibling Terra checkout. Update it only
after the corresponding Terra revision is pushed.
