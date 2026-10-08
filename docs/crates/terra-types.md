# terra-types

Shared geometry and control samples. Later crates depend on this one and do not redefine the axes.

[API](https://super-yojan.dev/Terra/api/terra_types/index.html) · [source](https://github.com/Super-Yojan/Terra/blob/main/crates/terra-types/src/lib.rs)

## Frames

Rover body axes are **+X forward, +Y left, +Z up**. Timestamps are seconds on one monotonic clock. `Vector3` and `Quaternion` are the only geometry types. `Quaternion::rotate` expects a normalized quaternion. `from_rotation_vector` builds one from an axis-angle vector in radians.

## Samples

| Type | Fields that matter |
| --- | --- |
| `ImuSample` | Gravity-removed acceleration in body axes, m/s², and body angular velocity, rad/s. |
| `VioSample` | Position, body-to-world orientation, world-frame velocity, and `tracked`. World is metric and +Z up. |
| `VelocityTarget` | `forward` in m/s and `yaw_rate` in rad/s. |
| `VelocityEstimate` | Body forward speed, yaw rate, and a `Health` value. |
| `MotorOutput` | Signed `left` and `right` effort in `[-1, 1]`. Positive is forward. Zero is coast, which is not a mechanical brake. `stop_reason` explains a neutral output. |

`Health` covers a missing or stale IMU, a missing or stale VIO sample, lost tracking, and an invalid clock. `StopReason` is the controller's view of the same failures, plus a stale target. `InputError` is what push and configure methods return for non-finite values, a bad quaternion, out-of-order timestamps, out-of-range magnitudes, or an invalid configuration.
