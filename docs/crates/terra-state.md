# terra-state

`VelocityEstimator` keeps a short IMU history and the latest VIO sample, then reports body forward speed and yaw rate. The crate describes itself as a VIO velocity anchor with bounded IMU prediction.

[API](https://super-yojan.dev/Terra/api/terra_state/index.html) · [source](https://github.com/Super-Yojan/Terra/blob/main/crates/terra-state/src/lib.rs)

## Defaults

`EstimatorConfig` defaults to an IMU timeout of **0.1 s** and a VIO timeout of **0.35 s**. `vio_timeout` must be finite, positive, and at most 2 s.

## What `estimate` returns

`estimate(now)` returns zero speed and a `Health` value when the clock is invalid, VIO is missing or untracked, IMU is missing, either sample is stale, or no IMU sample exists at or before the VIO timestamp.

When the inputs are fresh, the estimator replays IMU samples from the VIO timestamp up to `now`: it rotates the gravity-free acceleration into the world, integrates velocity, and then projects that velocity back into the body. Reported `forward` is body X. Reported `yaw_rate` is the latest IMU's body Z rate.

Samples must be finite and increasing in time. Acceleration above 1000 m/s², angular rate above 100 rad/s, or VIO speed above 100 m/s is `OutOfRange`. The IMU buffer keeps 512 samples.

Phone mode is expected to fill these samples from Core Motion and ARKit. Zorvane can feed synthetic IMU and VIO from the physics body. Neither path is a visual-odometry implementation inside this crate.
