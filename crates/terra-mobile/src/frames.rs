//! Shared phone-frame conversions and the local simulated plant.
//!
//! Swift and Kotlin only sample platform sensors. Axis changes, the Y-up camera
//! pose, the velocity smoother, the flat-ground height latch, the simulated
//! motor plant, and the simulated room depth all live here so both apps call
//! the same functions.
use crate::ControllerError;
use std::f64::consts::{FRAC_PI_2, PI};
use std::sync::{Arc, Mutex};
use terra_types::{InputError, Quaternion, Vector3};

#[derive(Clone, Copy, Debug, PartialEq, uniffi::Record)]
pub struct AxisSample {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

fn finite3(x: f64, y: f64, z: f64) -> Result<(), ControllerError> {
    if x.is_finite() && y.is_finite() && z.is_finite() {
        Ok(())
    } else {
        Err(InputError::NonFinite.into())
    }
}

/// Device axes, screen up: +X right, +Y toward the top edge, +Z out of the screen.
/// Body axes: +X forward, +Y left, +Z up.
///
/// Core Motion and Android `SensorManager` use this device frame. The returned
/// sample is `(forward, left, up)`.
#[uniffi::export]
pub fn device_vector_to_body(x: f64, y: f64, z: f64) -> Result<AxisSample, ControllerError> {
    finite3(x, y, z)?;
    let rotated = phone_to_body().rotate(Vector3 { x, y, z });
    Ok(AxisSample {
        x: rotated.x,
        y: rotated.y,
        z: rotated.z,
    })
}

#[derive(Clone, Copy, Debug, PartialEq, uniffi::Record)]
pub struct BodyPose {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub quaternion_x: f64,
    pub quaternion_y: f64,
    pub quaternion_z: f64,
    pub quaternion_w: f64,
}

fn pose_from(position: Vector3, orientation: Quaternion) -> BodyPose {
    BodyPose {
        x: position.x,
        y: position.y,
        z: position.z,
        quaternion_x: orientation.x,
        quaternion_y: orientation.y,
        quaternion_z: orientation.z,
        quaternion_w: orientation.w,
    }
}

fn camera_quaternion(
    x: f64,
    y: f64,
    z: f64,
    qx: f64,
    qy: f64,
    qz: f64,
    qw: f64,
) -> Result<Quaternion, ControllerError> {
    finite3(x, y, z)?;
    finite3(qx, qy, qz)?;
    if !qw.is_finite() {
        return Err(InputError::NonFinite.into());
    }
    Quaternion {
        x: qx,
        y: qy,
        z: qz,
        w: qw,
    }
    .normalized()
    .ok_or_else(|| InputError::InvalidQuaternion.into())
}

/// Y-up camera pose (ARKit `camera.transform`, ARCore camera pose) into the
/// robotics body pose: Z-up world, X-forward / Y-left body.
///
/// `quaternion_*` rotates camera vectors into the Y-up world. Translation is
/// the camera origin in that world, metres.
#[uniffi::export]
pub fn y_up_camera_to_body_pose(
    x: f64,
    y: f64,
    z: f64,
    quaternion_x: f64,
    quaternion_y: f64,
    quaternion_z: f64,
    quaternion_w: f64,
) -> Result<BodyPose, ControllerError> {
    let camera = camera_quaternion(
        x,
        y,
        z,
        quaternion_x,
        quaternion_y,
        quaternion_z,
        quaternion_w,
    )?;
    let orientation = yup_to_zup()
        .compose(camera)
        .compose(phone_to_body().conjugate())
        .normalized()
        .ok_or(InputError::InvalidQuaternion)?;
    Ok(pose_from(
        yup_point_to_zup(Vector3 { x, y, z }),
        orientation,
    ))
}

/// Heading of a body quaternion from [`y_up_camera_to_body_pose`], radians from +X.
#[uniffi::export]
pub fn body_yaw(
    quaternion_x: f64,
    quaternion_y: f64,
    quaternion_z: f64,
    quaternion_w: f64,
) -> Result<f64, ControllerError> {
    let orientation = Quaternion {
        x: quaternion_x,
        y: quaternion_y,
        z: quaternion_z,
        w: quaternion_w,
    }
    .normalized()
    .ok_or(InputError::InvalidQuaternion)?;
    let forward = orientation.rotate(Vector3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    });
    Ok(forward.y.atan2(forward.x))
}

/// Same camera pose, expressed as an optical frame (X right, Y down, Z forward)
/// for `MobileOccupancyMap`.
#[uniffi::export]
pub fn y_up_camera_to_optical_pose(
    x: f64,
    y: f64,
    z: f64,
    quaternion_x: f64,
    quaternion_y: f64,
    quaternion_z: f64,
    quaternion_w: f64,
) -> Result<BodyPose, ControllerError> {
    let camera = camera_quaternion(
        x,
        y,
        z,
        quaternion_x,
        quaternion_y,
        quaternion_z,
        quaternion_w,
    )?;
    let orientation = yup_to_zup()
        .compose(camera)
        .compose(optical_from_camera())
        .normalized()
        .ok_or(InputError::InvalidQuaternion)?;
    Ok(pose_from(
        yup_point_to_zup(Vector3 { x, y, z }),
        orientation,
    ))
}

/// First tracked camera height is shifted onto `reference` metres (0.5 for a
/// phone held as the rover origin). Later samples keep that offset.
#[uniffi::export]
pub fn latch_ground_offset(
    latched: Option<f64>,
    position_z: f64,
    reference: f64,
) -> Result<f64, ControllerError> {
    if !position_z.is_finite() || !reference.is_finite() {
        return Err(InputError::NonFinite.into());
    }
    if let Some(offset) = latched {
        if !offset.is_finite() {
            return Err(InputError::NonFinite.into());
        }
        return Ok(offset);
    }
    Ok(reference - position_z)
}

/// Low-pass on finite differences of a tracked pose. Matches the phone VIO
/// smoother: samples closer than 200 ms, time constant 30 ms. Tracking loss
/// clears the filter so the next sample does not invent a velocity.
#[derive(uniffi::Object)]
pub struct PoseVelocityFilter {
    state: Mutex<VelocityFilter>,
}

struct VelocityFilter {
    previous: Option<(Vector3, f64)>,
    filtered: Vector3,
}

#[uniffi::export]
impl PoseVelocityFilter {
    #[uniffi::constructor]
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            state: Mutex::new(VelocityFilter {
                previous: None,
                filtered: Vector3::ZERO,
            }),
        })
    }

    pub fn reset(&self) -> Result<(), ControllerError> {
        let mut state = self.state.lock().map_err(|_| ControllerError::Internal)?;
        state.previous = None;
        state.filtered = Vector3::ZERO;
        Ok(())
    }

    /// World-frame velocity in m/s. `None` until two tracked poses land inside
    /// the 200 ms window.
    pub fn push(
        &self,
        x: f64,
        y: f64,
        z: f64,
        timestamp: f64,
        tracked: bool,
    ) -> Result<Option<AxisSample>, ControllerError> {
        let mut state = self.state.lock().map_err(|_| ControllerError::Internal)?;
        if !tracked {
            state.previous = None;
            state.filtered = Vector3::ZERO;
            return Ok(None);
        }
        finite3(x, y, z)?;
        if !timestamp.is_finite() {
            return Err(InputError::NonFinite.into());
        }
        let position = Vector3 { x, y, z };
        let mut ready = None;
        if let Some((last, previous_time)) = state.previous {
            let dt = timestamp - previous_time;
            if dt > 0.0 && dt < 0.2 {
                let raw = Vector3 {
                    x: (position.x - last.x) / dt,
                    y: (position.y - last.y) / dt,
                    z: (position.z - last.z) / dt,
                };
                let alpha = dt / (0.03 + dt);
                let delta = Vector3 {
                    x: raw.x - state.filtered.x,
                    y: raw.y - state.filtered.y,
                    z: raw.z - state.filtered.z,
                };
                state.filtered = state.filtered.plus(delta.scaled(alpha));
                ready = Some(AxisSample {
                    x: state.filtered.x,
                    y: state.filtered.y,
                    z: state.filtered.z,
                });
            }
        }
        state.previous = Some((position, timestamp));
        Ok(ready)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, uniffi::Record)]
pub struct PlantState {
    pub forward: f64,
    pub yaw_rate: f64,
    pub yaw: f64,
    pub x: f64,
    pub y: f64,
    /// Body velocity expressed in the Z-up world, m/s.
    pub velocity_x: f64,
    pub velocity_y: f64,
    pub acceleration_forward: f64,
    pub quaternion_x: f64,
    pub quaternion_y: f64,
    pub quaternion_z: f64,
    pub quaternion_w: f64,
}

struct Plant {
    forward: f64,
    yaw_rate: f64,
    yaw: f64,
    x: f64,
    y: f64,
    acceleration_forward: f64,
}

impl Plant {
    fn state(&self) -> PlantState {
        let half = self.yaw * 0.5;
        PlantState {
            forward: self.forward,
            yaw_rate: self.yaw_rate,
            yaw: self.yaw,
            x: self.x,
            y: self.y,
            velocity_x: self.forward * self.yaw.cos(),
            velocity_y: self.forward * self.yaw.sin(),
            acceleration_forward: self.acceleration_forward,
            quaternion_x: 0.0,
            quaternion_y: 0.0,
            quaternion_z: half.sin(),
            quaternion_w: half.cos(),
        }
    }

    fn step(&mut self, left: f64, right: f64) {
        const DT: f64 = 0.01;
        self.acceleration_forward = 3.0 * (left + right) * 0.5 - self.forward;
        self.forward += self.acceleration_forward * DT;
        self.yaw_rate += (5.0 * (right - left) * 0.5 - self.yaw_rate) * DT;
        self.x += self.yaw.cos() * self.forward * DT;
        self.y += self.yaw.sin() * self.forward * DT;
        self.yaw += self.yaw_rate * DT;
    }
}

/// Local motor plant used by Simulated rover. One `step` is 10 ms.
/// Read [`SimulatedPlant::state`] for the pose that feeds IMU and VIO, then
/// `step` with the controller effort. The map for that tick uses the pose
/// from before `step`.
#[derive(uniffi::Object)]
pub struct SimulatedPlant {
    plant: Mutex<Plant>,
}

#[uniffi::export]
impl SimulatedPlant {
    #[uniffi::constructor]
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            plant: Mutex::new(Plant {
                forward: 0.0,
                yaw_rate: 0.0,
                yaw: 0.0,
                x: 0.0,
                y: 0.0,
                acceleration_forward: 0.0,
            }),
        })
    }

    pub fn reset(&self) -> Result<(), ControllerError> {
        *self.plant.lock().map_err(|_| ControllerError::Internal)? = Plant {
            forward: 0.0,
            yaw_rate: 0.0,
            yaw: 0.0,
            x: 0.0,
            y: 0.0,
            acceleration_forward: 0.0,
        };
        Ok(())
    }

    pub fn state(&self) -> Result<PlantState, ControllerError> {
        Ok(self
            .plant
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .state())
    }

    pub fn step(&self, left_effort: f64, right_effort: f64) -> Result<PlantState, ControllerError> {
        if !left_effort.is_finite() || !right_effort.is_finite() {
            return Err(InputError::NonFinite.into());
        }
        let mut plant = self.plant.lock().map_err(|_| ControllerError::Internal)?;
        plant.step(left_effort, right_effort);
        Ok(plant.state())
    }
}

#[derive(Clone, Debug, PartialEq, uniffi::Record)]
pub struct SimulatedDepthFrame {
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub camera_x: f64,
    pub camera_y: f64,
    pub camera_z: f64,
    pub quaternion_x: f64,
    pub quaternion_y: f64,
    pub quaternion_z: f64,
    pub quaternion_w: f64,
    pub depth_metres: Vec<f32>,
}

/// Axial depths of a square room whose walls sit at ±6 m, from a camera 0.5 m
/// above the floor looking along the rover heading. 81 columns, one row.
#[uniffi::export]
pub fn simulated_room_depth(
    x: f64,
    y: f64,
    yaw: f64,
) -> Result<SimulatedDepthFrame, ControllerError> {
    finite3(x, y, yaw)?;
    let mut depths = vec![f32::NAN; 81];
    for (index, depth) in depths.iter_mut().enumerate() {
        let right = (index as f64 - 40.0) / 60.0;
        let dx = yaw.cos() + right * yaw.sin();
        let dy = yaw.sin() - right * yaw.cos();
        let mut nearest = f64::INFINITY;
        for wall in [-6.0, 6.0] {
            if dx.abs() > 1e-9 {
                let t = (wall - x) / dx;
                if t > 0.0 && (y + t * dy).abs() <= 6.0 {
                    nearest = nearest.min(t);
                }
            }
            if dy.abs() > 1e-9 {
                let t = (wall - y) / dy;
                if t > 0.0 && (x + t * dx).abs() <= 6.0 {
                    nearest = nearest.min(t);
                }
            }
        }
        if nearest.is_finite() {
            *depth = nearest as f32;
        }
    }
    let half = yaw * 0.5;
    let yaw_q = Quaternion {
        x: 0.0,
        y: 0.0,
        z: half.sin(),
        w: half.cos(),
    };
    let optical = yaw_q.compose(OPTICAL_TO_BODY);
    Ok(SimulatedDepthFrame {
        width: 81,
        height: 1,
        fx: 60.0,
        fy: 60.0,
        cx: 40.0,
        cy: 0.0,
        camera_x: x,
        camera_y: y,
        camera_z: 0.5,
        quaternion_x: optical.x,
        quaternion_y: optical.y,
        quaternion_z: optical.z,
        quaternion_w: optical.w,
        depth_metres: depths,
    })
}

/// Rotation by −90° about Z. Device +Y (top edge) becomes body +X (forward).
fn phone_to_body() -> Quaternion {
    Quaternion::from_rotation_vector(Vector3 {
        x: 0.0,
        y: 0.0,
        z: -FRAC_PI_2,
    })
}

/// ARKit / ARCore world (+Y up) into robotics (+Z up), right-handed.
/// Columns of the matrix are the images of the Y-up basis.
fn yup_to_zup() -> Quaternion {
    // m rows: [0, 0, -1], [-1, 0, 0], [0, 1, 0]
    quaternion_from_matrix([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
}

fn yup_point_to_zup(point: Vector3) -> Vector3 {
    Vector3 {
        x: -point.z,
        y: -point.x,
        z: point.y,
    }
}

/// 180° about camera X: right/up/back becomes optical right/down/forward.
fn optical_from_camera() -> Quaternion {
    Quaternion::from_rotation_vector(Vector3 {
        x: PI,
        y: 0.0,
        z: 0.0,
    })
}

/// Body yaw 0, optical axes of a forward-looking camera. Same constants the
/// Swift simulated map used (`simd_quatd(ix: -0.5, iy: 0.5, iz: -0.5, r: 0.5)`).
const OPTICAL_TO_BODY: Quaternion = Quaternion {
    x: -0.5,
    y: 0.5,
    z: -0.5,
    w: 0.5,
};

fn quaternion_from_matrix(m: [[f64; 3]; 3]) -> Quaternion {
    let (m00, m01, m02) = (m[0][0], m[0][1], m[0][2]);
    let (m10, m11, m12) = (m[1][0], m[1][1], m[1][2]);
    let (m20, m21, m22) = (m[2][0], m[2][1], m[2][2]);
    let trace = m00 + m11 + m22;
    let q = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        Quaternion {
            w: 0.25 * s,
            x: (m21 - m12) / s,
            y: (m02 - m20) / s,
            z: (m10 - m01) / s,
        }
    } else if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        Quaternion {
            w: (m21 - m12) / s,
            x: 0.25 * s,
            y: (m01 + m10) / s,
            z: (m02 + m20) / s,
        }
    } else if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        Quaternion {
            w: (m02 - m20) / s,
            x: (m01 + m10) / s,
            y: 0.25 * s,
            z: (m12 + m21) / s,
        }
    } else {
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        Quaternion {
            w: (m10 - m01) / s,
            x: (m02 + m20) / s,
            y: (m12 + m21) / s,
            z: 0.25 * s,
        }
    };
    q.normalized().expect("rotation matrix")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-9, "{a} != {b}");
    }

    #[test]
    fn device_axes_match_the_flat_phone_mount() {
        let forward = device_vector_to_body(0.0, 1.0, 0.0).unwrap();
        approx(forward.x, 1.0);
        approx(forward.y, 0.0);
        approx(forward.z, 0.0);
        let left = device_vector_to_body(-1.0, 0.0, 0.0).unwrap();
        approx(left.x, 0.0);
        approx(left.y, 1.0);
        let up = device_vector_to_body(0.0, 0.0, 2.0).unwrap();
        approx(up.z, 2.0);
        assert!(device_vector_to_body(f64::NAN, 0.0, 0.0).is_err());
    }

    #[test]
    fn yup_basis_lands_on_the_robotics_basis() {
        let q = yup_to_zup();
        let x = q.rotate(Vector3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        });
        let y = q.rotate(Vector3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        });
        let z = q.rotate(Vector3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        });
        approx(x.y, -1.0);
        approx(y.z, 1.0);
        approx(z.x, -1.0);
        let point = yup_point_to_zup(Vector3 {
            x: 3.0,
            y: 4.0,
            z: 5.0,
        });
        approx(point.x, -5.0);
        approx(point.y, -3.0);
        approx(point.z, 4.0);
    }

    #[test]
    fn body_yaw_reads_the_forward_axis() {
        approx(body_yaw(0.0, 0.0, 0.0, 1.0).unwrap(), 0.0);
        let half = FRAC_PI_2 * 0.5;
        approx(
            body_yaw(0.0, 0.0, half.sin(), half.cos()).unwrap(),
            FRAC_PI_2,
        );
        assert!(body_yaw(0.0, 0.0, 0.0, 0.0).is_err());
    }

    #[test]
    fn identity_camera_pose_is_a_unit_body_rotation() {
        let pose = y_up_camera_to_body_pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0).unwrap();
        approx(pose.x, -3.0);
        approx(pose.y, -1.0);
        approx(pose.z, 2.0);
        let q = Quaternion {
            x: pose.quaternion_x,
            y: pose.quaternion_y,
            z: pose.quaternion_z,
            w: pose.quaternion_w,
        };
        let n = q.x.hypot(q.y).hypot(q.z).hypot(q.w);
        approx(n, 1.0);
        assert!(y_up_camera_to_body_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).is_err());
    }

    #[test]
    fn velocity_filter_ramps_then_drops_on_tracking_loss() {
        let filter = PoseVelocityFilter::new();
        assert!(filter.push(0.0, 0.0, 0.0, 0.0, true).unwrap().is_none());
        let first = filter.push(1.0, 0.0, 0.0, 0.1, true).unwrap().unwrap();
        let alpha = 0.1 / 0.13;
        approx(first.x, 10.0 * alpha);
        assert!(filter.push(1.0, 0.0, 0.0, 0.2, false).unwrap().is_none());
        assert!(filter.push(2.0, 0.0, 0.0, 0.3, true).unwrap().is_none());
    }

    #[test]
    fn ground_offset_locks_the_first_camera_height_to_half_a_metre() {
        let offset = latch_ground_offset(None, 1.2, 0.5).unwrap();
        approx(offset, -0.7);
        approx(1.2 + offset, 0.5);
        approx(latch_ground_offset(Some(offset), 1.4, 0.5).unwrap(), -0.7);
    }

    #[test]
    fn plant_coasts_forward_when_both_efforts_are_positive() {
        let plant = SimulatedPlant::new();
        let before = plant.state().unwrap();
        approx(before.forward, 0.0);
        let after = plant.step(1.0, 1.0).unwrap();
        approx(after.acceleration_forward, 3.0);
        approx(after.forward, 0.03);
        approx(after.yaw_rate, 0.0);
        approx(after.x, 0.0003);
        approx(after.quaternion_w, 1.0);
    }

    #[test]
    fn simulated_room_center_ray_hits_the_north_wall() {
        let frame = simulated_room_depth(0.0, 0.0, 0.0).unwrap();
        assert_eq!((frame.width, frame.height), (81, 1));
        approx(frame.depth_metres[40] as f64, 6.0);
        approx(frame.camera_z, 0.5);
        let turned = simulated_room_depth(0.0, 0.0, FRAC_PI_2).unwrap();
        approx(turned.depth_metres[40] as f64, 6.0);
    }
}
