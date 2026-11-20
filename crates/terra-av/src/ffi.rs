//! The phone boundary (UniFFI). Plain records that mirror `sense`, `planning`
//! and `pipeline`, plus one object that owns the pipeline. Swift sees only
//! what is in this file.
use std::sync::{Arc, Mutex};

use crate::config::PipelineConfig;
use crate::pipeline::Pipeline;
use crate::{geometry, planning, sense, vehicle};

#[derive(uniffi::Record)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// ARKit `camera.transform`: position plus row-major 3×3 rotation (9 values).
#[derive(uniffi::Record)]
pub struct VioPose {
    pub position: Vec3,
    pub rotation: Vec<f64>,
    pub tracked: bool,
}

#[derive(uniffi::Record)]
pub struct GpsFix {
    pub latitude: f64,
    pub longitude: f64,
    pub accuracy: f64,
}

#[derive(uniffi::Record)]
pub struct SensorFrame {
    pub t: f64,
    pub estop: bool,
    pub vio: Option<VioPose>,
    pub gyro: Option<Vec3>,
    pub gps: Option<GpsFix>,
    pub depth: Vec<Vec3>,
}

#[derive(uniffi::Enum)]
pub enum Goal {
    None,
    Velocity { v: f64, w: f64 },
    Pose { x: f64, y: f64, yaw: f64 },
}

/// Body velocity: forward m/s, yaw rad/s (+ = left).
#[derive(uniffi::Record)]
pub struct Twist {
    pub v: f64,
    pub w: f64,
}

/// Target twist for the phone's velocity loop, plus the measured twist it
/// feeds back on (`None` until the pose is tracked).
#[derive(uniffi::Record)]
pub struct DriveCommand {
    pub target: Twist,
    pub measured: Option<Twist>,
}

#[derive(uniffi::Object)]
pub struct TerraAv {
    pipeline: Mutex<Option<Pipeline>>,
}

#[uniffi::export]
impl TerraAv {
    /// Build the pipeline from the app's terra-av.toml. `router_password`
    /// (from the Keychain) replaces any `[comms] password` in the file.
    /// An invalid config leaves no pipeline, so every tick stops the rover;
    /// the parse error is printed to the console.
    #[uniffi::constructor]
    pub fn new(config_toml: String, router_password: Option<String>) -> Arc<Self> {
        let pipeline = match PipelineConfig::from_toml(&config_toml) {
            Ok(config) => Some(config.with_router_password(router_password).build()),
            Err(e) => {
                eprintln!("terra-av: invalid config, rover will not drive: {e}");
                None
            }
        };
        Arc::new(Self {
            pipeline: Mutex::new(pipeline),
        })
    }

    /// One control tick. Stops when there is no pipeline.
    pub fn tick(&self, frame: SensorFrame, goal: Goal) -> DriveCommand {
        let stop = DriveCommand {
            target: Twist { v: 0.0, w: 0.0 },
            measured: None,
        };
        let mut pipeline = self.pipeline.lock().unwrap();
        match pipeline.as_mut() {
            None => stop,
            Some(pipeline) => {
                let out = pipeline.tick(&frame.into(), &goal.into());
                DriveCommand {
                    target: out.target.into(),
                    measured: out.measured.map(Into::into),
                }
            }
        }
    }
}

// Conversions between the FFI records and the crate's internal types.

impl From<Vec3> for geometry::Vec3 {
    fn from(v: Vec3) -> Self {
        Self { x: v.x, y: v.y, z: v.z }
    }
}

impl VioPose {
    /// `None` if `rotation` is not 9 values, so a bad pose from Swift is
    /// treated as untracked instead of panicking across the FFI.
    fn into_sense(self) -> Option<sense::VioPose> {
        let r: [f64; 9] = self.rotation.try_into().ok()?;
        Some(sense::VioPose {
            position: self.position.into(),
            rotation: geometry::Rotation3::from_matrix([
                [r[0], r[1], r[2]],
                [r[3], r[4], r[5]],
                [r[6], r[7], r[8]],
            ]),
            tracked: self.tracked,
        })
    }
}

impl From<GpsFix> for sense::GpsFix {
    fn from(g: GpsFix) -> Self {
        Self { latitude: g.latitude, longitude: g.longitude, accuracy: g.accuracy }
    }
}

impl From<SensorFrame> for sense::SensorFrame {
    fn from(f: SensorFrame) -> Self {
        Self {
            t: f.t,
            estop: f.estop,
            vio: f.vio.and_then(VioPose::into_sense),
            gyro: f.gyro.map(Into::into),
            gps: f.gps.map(Into::into),
            depth: f.depth.into_iter().map(Into::into).collect(),
        }
    }
}

impl From<Goal> for planning::Goal {
    fn from(g: Goal) -> Self {
        match g {
            Goal::None => Self::None,
            Goal::Velocity { v, w } => Self::Velocity(vehicle::Twist { v, w }),
            Goal::Pose { x, y, yaw } => Self::Pose(geometry::Pose2 { x, y, yaw }),
        }
    }
}

impl From<vehicle::Twist> for Twist {
    fn from(t: vehicle::Twist) -> Self {
        Self { v: t.v, w: t.w }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn without_a_pipeline_tick_stops() {
        let av = TerraAv::new(String::new(), None);
        let frame = SensorFrame {
            t: 0.0,
            estop: false,
            vio: None,
            gyro: None,
            gps: None,
            depth: vec![],
        };
        let out = av.tick(frame, Goal::Velocity { v: 0.5, w: 0.0 });
        assert_eq!((out.target.v, out.target.w), (0.0, 0.0));
        assert!(out.measured.is_none());
    }

    #[test]
    fn bad_rotation_becomes_no_vio() {
        let frame = SensorFrame {
            t: 0.0,
            estop: false,
            vio: Some(VioPose {
                position: Vec3 { x: 0.0, y: 0.0, z: 0.0 },
                rotation: vec![1.0, 0.0, 0.0],
                tracked: true,
            }),
            gyro: None,
            gps: None,
            depth: vec![],
        };
        let frame: sense::SensorFrame = frame.into();
        assert!(frame.vio.is_none());
    }
}
