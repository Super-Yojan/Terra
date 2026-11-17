//! The local simulated rover: a planar motor plant and a synthetic room depth
//! image, so the iPhone app can drive and map with no hardware.
use crate::ControllerError;
use std::sync::{Arc, Mutex};
use terra_types::{InputError, Quaternion};

fn finite3(x: f64, y: f64, z: f64) -> Result<(), ControllerError> {
    if x.is_finite() && y.is_finite() && z.is_finite() {
        Ok(())
    } else {
        Err(InputError::NonFinite.into())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, uniffi::Record)]
/// Planar simulated rover state after a fixed 10 ms plant tick.
pub struct PlantState {
    /// Body forward speed, m/s.
    pub forward: f64,
    /// Body yaw rate, rad/s.
    pub yaw_rate: f64,
    /// World heading from +X toward +Y, radians.
    pub yaw: f64,
    /// X component; world metres for poses and plant state.
    pub x: f64,
    /// Y component; world metres for poses and plant state.
    pub y: f64,
    /// Body velocity expressed in the Z-up world, m/s.
    pub velocity_x: f64,
    /// World +Y velocity, m/s.
    pub velocity_y: f64,
    /// Body forward acceleration, m/s².
    pub acceleration_forward: f64,
    /// Returned frame-to-world quaternion X component.
    pub quaternion_x: f64,
    /// Returned frame-to-world quaternion Y component.
    pub quaternion_y: f64,
    /// Returned frame-to-world quaternion Z component.
    pub quaternion_z: f64,
    /// Returned frame-to-world quaternion scalar component.
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
    /// Construct a stationary simulated rover at world (0, 0) with zero heading.
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

    /// Restore the stationary origin state. Returns `Internal` for a poisoned mutex.
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

    /// Copy the current simulated pose and sensor state without advancing time.
    /// Returns `Internal` for a poisoned mutex.
    pub fn state(&self) -> Result<PlantState, ControllerError> {
        Ok(self
            .plant
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .state())
    }

    /// Advance the plant exactly 0.01 s and return the resulting state.
    /// Efforts must be finite; this simulator does not clamp their magnitude.
    /// Returns `InvalidInput` for nonfinite efforts or `Internal` for a poisoned mutex.
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
/// Synthetic axial depth image with optical intrinsics and camera-to-world pose.
pub struct SimulatedDepthFrame {
    /// Depth image width, pixels.
    pub width: u32,
    /// Depth image height, pixels.
    pub height: u32,
    /// Horizontal focal length, pixels.
    pub fx: f64,
    /// Vertical focal length, pixels.
    pub fy: f64,
    /// Horizontal principal point in integer-pixel coordinates.
    pub cx: f64,
    /// Vertical principal point in integer-pixel coordinates.
    pub cy: f64,
    /// Camera origin world +X coordinate, metres.
    pub camera_x: f64,
    /// Camera origin world +Y coordinate, metres.
    pub camera_y: f64,
    /// Camera origin world +Z coordinate, metres.
    pub camera_z: f64,
    /// Returned frame-to-world quaternion X component.
    pub quaternion_x: f64,
    /// Returned frame-to-world quaternion Y component.
    pub quaternion_y: f64,
    /// Returned frame-to-world quaternion Z component.
    pub quaternion_z: f64,
    /// Returned frame-to-world quaternion scalar component.
    pub quaternion_w: f64,
    /// Row-major axial optical depths in metres; length must equal width × height.
    pub depth_metres: Vec<f32>,
}

/// Axial depths of a square room whose walls sit at ±6 m, from a camera 0.5 m
/// above the floor looking along the rover heading. 81 columns, one row.
#[uniffi::export]
/// `x` and `y` are world metres, `yaw` is radians. Rays without a forward
/// wall intersection remain NaN. Returns `InvalidInput` for nonfinite inputs.
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

/// Body yaw 0, optical axes of a forward-looking camera. Same constants the
/// Swift simulated map used (`simd_quatd(ix: -0.5, iy: 0.5, iz: -0.5, r: 0.5)`).
const OPTICAL_TO_BODY: Quaternion = Quaternion {
    x: -0.5,
    y: 0.5,
    z: -0.5,
    w: 0.5,
};

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn approx(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-9, "{a} != {b}");
    }

    #[test]
    /// Applies equal positive efforts for one plant tick and checks it accelerates and moves forward without turning.
    fn plant_coasts_forward_when_both_efforts_are_positive() {
        let plant = SimulatedPlant::new();
        let before = plant.state().unwrap();
        approx(before.forward, 0.0);
        let after = plant.step(1.0, 1.0).unwrap();
        assert!(after.acceleration_forward > 0.0);
        assert!(after.forward > 0.0);
        assert!(after.x > before.x);
        approx(after.yaw_rate, 0.0);
        approx(after.quaternion_w, 1.0);
    }

    #[test]
    /// Checks the synthetic 81×1 depth image and six-metre center ray at zero and quarter-turn headings.
    fn simulated_room_center_ray_hits_the_north_wall() {
        let frame = simulated_room_depth(0.0, 0.0, 0.0).unwrap();
        assert_eq!((frame.width, frame.height), (81, 1));
        approx(frame.depth_metres[40] as f64, 6.0);
        approx(frame.camera_z, 0.5);
        let turned = simulated_room_depth(0.0, 0.0, FRAC_PI_2).unwrap();
        approx(turned.depth_metres[40] as f64, 6.0);
    }
}
