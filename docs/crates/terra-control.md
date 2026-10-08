# terra-control

`VelocityController` turns a `VelocityEstimate` and a `VelocityTarget` into left and right wheel effort.

[API](https://super-yojan.dev/Terra/api/terra_control/index.html) · [source](https://github.com/Super-Yojan/Terra/blob/main/crates/terra-control/src/lib.rs)

## Defaults

| Field | Default |
| --- | --- |
| `linear_kp`, `linear_ki` | 0.8, 0.6 |
| `yaw_kp`, `yaw_ki` | 0.4, 0.3 |
| `linear_feedforward` | 1/3 |
| `yaw_feedforward` | 0.2 |
| `max_forward`, `max_yaw_rate` | 2 m/s, 2 rad/s |
| `max_effort` | 1 |
| `target_timeout` | 0.5 s |
| `max_step` | 0.1 s |
| `anti_windup` | 4 |

`validate` rejects non-finite gains, a non-positive limit, `max_effort` above 1, forward or yaw limits above 20, and `max_step` above 1 s.

## Step

`set_target` clamps forward speed and yaw rate to the configured limits. A non-finite target clears the latched target and the integrators.

`step` returns `MotorOutput::stopped` when health is anything other than `Ready`, the target is missing or older than `target_timeout`, the timestamp goes backwards, or the step is outside `(0, max_step]`. The first step assumes `dt` of 0.01 s.

Otherwise the command is feedforward plus proportional error plus integral. Differential mix is `left = linear - yaw` and `right = linear + yaw`, then both are scaled down so the larger magnitude stays within `max_effort`. Integrators use back-calculation against that scaled command. A settled zero target and a near-zero estimate clear the integrators and command exact zero.

The 0.5 s target timeout is independent of the motor watchdog in `terra-motors`, which defaults to 0.2 s.
