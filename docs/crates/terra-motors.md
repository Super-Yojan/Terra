# terra-motors

PWM adapter between `MotorOutput` effort and a motor driver. It depends only on `terra-types`. It does not toggle GPIO and it does not speak Zenoh. The iOS UI displays effort. Zorvane turns effort into physics forces. A rover driver would write the `ChassisPwm` this crate returns.

[API](https://super-yojan.dev/Terra/api/terra_motors/index.html) · [crate readme](https://github.com/Super-Yojan/Terra/blob/main/crates/terra-motors/README.md)

## Effort to PWM

`map_effort` turns one signed effort in `[-1, 1]` into a duty in `[0, 1]` and a direction. Positive is forward. A small deadband coasts.

Use the `ChassisPwm` from the latest `set_hardware_enable`, `apply`, `apply_output`, `apply_effort_now`, or `poll`.

- `drive_enabled` is the driver EN or nSLEEP line, in the chip's active level. `false` means asleep.
- `WheelPwm::sign_magnitude` is `(DIR, duty)`. DIR `true` is forward. `duty_counts` quantizes duty onto a timer period.
- `WheelPwm::in1_in2` is `(in1_duty, in2_duty)`. At most one input is nonzero.

Zero effort is coast: duty 0, no direction, both bridge inputs 0. It is not a mechanical brake and it is not the both-inputs-high electrical brake some H-bridges use. `drive_enabled` can stay true for a fresh zero so the bridge coasts instead of entering a chip-specific disable mode. If the chosen chip brakes when disabled, that behavior belongs to the chip.

## Enable switch

Boot treats the switch as open. `set_hardware_enable(false, now)` coasts and drops `drive_enabled`. Closing the switch returns `AwaitCommand` and does not replay a latched effort. The next `apply` while the switch stays closed starts PWM.

Wire the same switch so it can cut motor power or the driver enable pin without this process. The software gate only helps while the motor tick is running.

## Watchdog

Default `command_timeout` is **200 ms**. Config validation caps it at 1 s. This is separate from the velocity controller's 500 ms target timeout.

`commanded_at` is when the effort was computed. Age is `now - commanded_at`. Calling `apply` again with the same stamp does not extend the window. `apply_effort_now` stamps the effort at `now`. Use it only for an effort produced on that tick. A bridge that repeats the last sample must keep the original stamp and otherwise call `poll`.

On expiry the hold is `Watchdog`, both wheels coast, and `drive_enabled` is false. A later fresh command may drive again without recycling the switch. An older stamp than the one already latched is `InvalidTime` and also coasts.

A stream of controller zeros (phone backgrounded, tracking lost, or a stale velocity target) keeps the watchdog refreshed, so `drive_enabled` stays true while the switch is closed and the wheels coast. The chassis can roll. If the loop stops, or it keeps forwarding the last sample under the original stamp, the watchdog drops enable after the timeout. That is still coast, which is not a brake.

## Software bench test

`software_bench_sequence` in the crate tests is the wheels-up sequence without power. It does not spin hardware and it does not replace a physical check.

With `command_timeout` of 0.25 s the test asserts:

1. Switch open. `apply_effort_now(0.0, 1.0, -1.0)` stays neutral with hold `EnableOpen`.
2. `set_hardware_enable(true, 0.0)` stays coast with hold `AwaitCommand`. Closing the switch does not replay effort.
3. `apply_effort_now(0.0, 0.5, -0.25)` drives. Left `in1_in2` is `(0.5, 0.0)`. Right is `(0.0, 0.25)`. `drive_enabled` is true.
4. `apply_effort_now(0.125, 0.0, 0.0)` is hold `Live`, enabled, and coasting.
5. `poll(0.375)` and `poll(0.5)` are hold `Watchdog` and neutral. The 0.25 s timeout has elapsed from the zero command stamped at 0.125 s.
6. `apply_effort_now(0.5, 0.25, 0.25)` is live again, both directions forward.
7. `set_hardware_enable(false, 0.5)` and a later full effort stay `EnableOpen`.

```sh
cargo test -p terra-motors software_bench_sequence
```

## Wheels-up bench on a driver

This is the same sequence with the wheels off the ground. No autonomy, map, or radio stack is required. Logic power only until the enable checks pass. The hardware switch must be able to kill driver enable or motor power by itself. Phone sensing and this adapter have not been validated on a rover. The Bluetooth and Fusion HAT commissioning outline in [Bench](../hardware/BENCH.md) is a separate procedure and has not been run.

1. Switch open. Apply `+1` and `-1`. Both duties stay 0, `drive_enabled` is false, hold is `EnableOpen`. The wheels must not twitch.
2. Close the switch and do not send a new command. Hold is `AwaitCommand`. Outputs stay in coast. Closing the switch must not replay the last effort.
3. Apply left `+0.5`, right `-0.25`. Left PWM is half-scale forward, right is quarter-scale reverse, `drive_enabled` is true. The wheels spin in opposite directions.
4. Apply `0, 0` with a new producer time. Duties go to 0 and `drive_enabled` may stay true. The wheels coast down. They are not actively braked. Confirm IN1 and IN2 are both low.
5. Stop applying new commands, or keep applying the step-4 stamp. Within 200 ms, or whatever `command_timeout` was set to, hold is `Watchdog`, duties stay 0, and `drive_enabled` is false. A wheel turned by hand should coast.
6. Apply a new stamp. PWM may resume while the switch stays closed. Open the switch and confirm PWM drops immediately, even if the command is fresh.
7. Kill the phone link or stop the control process and repeat step 5. The chassis coasts. Open the hardware switch before putting the wheels down.
