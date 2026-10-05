# terra-motors

PWM adapter between `MotorOutput` effort and a motor driver. It depends only on `terra-types`.

`map_effort` turns one signed effort in `[-1, 1]` into a PWM duty in `[0, 1]` and a direction. Positive effort is forward. The chassis API is `MotorAdapter`: it will not assert driver enable until the hardware switch is closed and a command has been applied while it is closed, and an independent watchdog coasts both wheels when the producer timestamp goes stale.

```sh
cargo test -p terra-motors
```

This crate does not toggle GPIO and does not speak Zenoh. The iOS app still only displays effort. The simulator still turns effort into Avian forces.

## What to write to the driver

Use the `ChassisPwm` returned by `set_hardware_enable`, `apply`, `apply_output`, `apply_effort_now`, or `poll`. Do not keep a previous struct after a later call.

- `drive_enabled` → driver EN or nSLEEP, in the chip's active level. `false` means asleep / not enabled.
- Sign-magnitude: `WheelPwm::sign_magnitude` → `(DIR, duty)`. DIR `true` is forward. Quantize duty with `duty_counts`.
- IN1/IN2: `WheelPwm::in1_in2` → `(in1_duty, in2_duty)`. At most one input is nonzero.

Zero effort is coast: duty 0, no direction, IN1 and IN2 both 0. It is not a mechanical brake and it is not the both-inputs-high electrical brake some H-bridges use. `drive_enabled` can stay true for a fresh zero so the bridge stays in coast instead of a chip-specific disable mode. Confirm the chosen driver's disable state coasts or disconnects; if it brakes, that is the chip, not this adapter.

## Enable switch

Boot treats the switch as open. `set_hardware_enable(false, now)` coasts and drops `drive_enabled` on that call. Closing the switch returns `AwaitCommand` and does not replay a latched effort; the next `apply` while the switch stays closed is what starts PWM.

Wire the same switch so it can cut motor power or the driver enable pin without this process. The software gate only helps while the motor tick is running.

## Watchdog

Default `command_timeout` is **200 ms**, capped at 1 s by config validation. It is not the velocity controller's 500 ms target timeout.

`commanded_at` is when the effort was computed. The watchdog age is `now - commanded_at`. Calling `apply` again with the same stamp does not extend the window. `apply_effort_now` stamps the effort at `now`; use it only for an effort produced on that tick. A bridge that repeats the last phone or controller sample must keep the original stamp and otherwise call `poll`.

On expiry the hold is `Watchdog`, both wheels coast, and `drive_enabled` is false. A later fresh command may drive again without recycling the switch. An older stamp than the one already latched is `InvalidTime` and also coasts.

## Loss of phone or link

- The phone backgrounds, tracking drops, or the velocity target goes stale, and the control loop is still applying each new `MotorOutput`: efforts are zero, the adapter coasts, `drive_enabled` stays true while the switch is closed. The rover is not braked and can roll.
- The loop stops calling `apply`, or it keeps forwarding the last sample under the original `commanded_at`: after the watchdog window, `drive_enabled` drops and the outputs stay in coast. The rover is still not braked. Open the hardware switch before the wheels touch the ground.

## Bench (wheels off the ground)

No autonomy, map, or radio stack is required. Logic power only until the enable checks pass. The hardware switch must be able to kill driver enable or motor power by itself.

1. Switch open. Apply `+1` and `-1`. Both duties stay 0, `drive_enabled` is false, hold is `EnableOpen`. The wheels must not twitch.
2. Close the switch and do not send a new command. Hold is `AwaitCommand`. Outputs stay in coast. Closing the switch must not replay the last effort.
3. Apply left `+0.5`, right `-0.25`. Left PWM is half-scale forward, right is quarter-scale reverse, `drive_enabled` is true. The wheels spin in opposite directions.
4. Apply `0, 0` with a new producer time. Duties go to 0 and `drive_enabled` may stay true. The wheels coast down. They are not actively braked. Confirm IN1 and IN2 are both low, not both high.
5. Stop applying new commands (or keep applying the step-4 stamp). Within 200 ms, or whatever `command_timeout` was set to, hold is `Watchdog`, duties stay 0, and `drive_enabled` is false. A wheel turned by hand should coast.
6. Apply a new stamp. PWM may resume while the switch stays closed. Open the switch and confirm PWM drops immediately, even if the command is fresh.
7. Kill the phone link or stop the control process and repeat step 5. The chassis coasts. It does not brake. Open the hardware switch before putting the wheels down.

`software_bench_sequence` in the crate tests is the same sequence without power. It does not replace this check. Phone sensing and this adapter have not been validated on a rover.
