//! The phone boundary (UniFFI). Plain records that mirror `sense`, `planning`
//! and `pipeline`, plus one object that owns the pipeline. Swift sees only
//! what is in this file.
use std::sync::{Arc, Mutex};

use crate::pipeline::Pipeline;

#[derive(uniffi::Record)]
pub struct Vec3 { pub x: f64, pub y: f64, pub z: f64 }

/// ARKit `camera.transform`: position plus row-major 3×3 rotation (9 values).
#[derive(uniffi::Record)]
pub struct VioPose { pub position: Vec3, pub rotation: Vec<f64>, pub tracked: bool }

#[derive(uniffi::Record)]
pub struct GpsFix { pub latitude: f64, pub longitude: f64, pub accuracy: f64 }

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
pub struct Twist { pub v: f64, pub w: f64 }

/// Target twist for the phone's velocity loop, plus the measured twist it
/// feeds back on (`None` until the pose is tracked).
#[derive(uniffi::Record)]
pub struct DriveCommand { pub target: Twist, pub measured: Option<Twist> }

#[derive(uniffi::Object)]
pub struct TerraAv {
    pipeline: Mutex<Option<Pipeline>>,
}

#[uniffi::export]
impl TerraAv {
    /// Build the pipeline from the app's terra-av.toml.
    #[uniffi::constructor]
    pub fn new(config_toml: String) -> Arc<Self> {
        // TODO: PipelineConfig::from_toml(&config_toml)?.build() once the
        // stages are implemented. Until then there is no pipeline.
        Arc::new(Self { pipeline: Mutex::new(None) })
    }

    /// One control tick. Stops when there is no pipeline.
    pub fn tick(&self, frame: SensorFrame, goal: Goal) -> DriveCommand {
        let stop = DriveCommand { target: Twist { v: 0.0, w: 0.0 }, measured: None };
        let mut pipeline = self.pipeline.lock().unwrap();
        match pipeline.as_mut() {
            None => stop,
            // TODO: convert frame/goal to crate::sense / crate::planning
            // types and return pipeline.tick(..).
            Some(_) => stop,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn without_a_pipeline_tick_stops() {
        let av = TerraAv::new(String::new());
        let frame = SensorFrame { t: 0.0, estop: false, vio: None, gyro: None, gps: None, depth: vec![] };
        let out = av.tick(frame, Goal::Velocity { v: 0.5, w: 0.0 });
        assert_eq!((out.target.v, out.target.w), (0.0, 0.0));
        assert!(out.measured.is_none());
    }
}
