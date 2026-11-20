//! Pick one implementation per stage from a TOML file. `type` selects the
//! implementation; the other keys are its parameters.
//!
//! ```toml
//! [vehicle]
//! type = "diff_drive"            # diff_drive
//! track = 0.5                    # m
//! max_wheel_speed = 0.8          # m/s
//! max_accel = 2.0                # m/s², optional
//! max_yaw_accel = 0.5            # rad/s², optional
//!
//! [mount]                        # phone on the rover; degrees, metres
//! roll = -90
//! pitch = 0
//! yaw = 0
//! offset = [0.10, 0.0, 0.23]
//!
//! [estimator]
//! type = "vio"                   # vio | localization
//!
//! [mapper]
//! type = "occupancy_grid"        # none | occupancy_grid
//! resolution = 0.1
//!
//! [path_planner]
//! type = "hybrid_astar"          # straight_line | hybrid_astar
//! resolution = 0.1
//! yaw_bins = 72
//! step = 0.3
//! heuristic_weight = 1.5
//!
//! [local_planner]
//! type = "dwa"                   # teleop | drive_pose | dwa
//! horizon = 2.0
//! samples = 20
//! [local_planner.weights]
//! heading = 1.0
//! clearance = 1.0
//! speed = 0.5
//!
//! [comms]                        # optional; absent = none
//! type = "zenoh"                 # none | zenoh
//! rover_id = "0"                 # publishes terra/0/depth
//! endpoint = "tcp/192.168.1.10:7447"
//! user = "terra"                 # optional router login
//! password = "..."               # optional; the app passes it from the Keychain instead
//! ```
use serde::Deserialize;

use crate::{
    comms::{BaseStationLink, NoLink},
    estimation::{Localization, StateEstimator, Vio},
    execution::SafetyGate,
    geometry::{Rotation3, Vec3},
    mapping::{Mapper, NoMap, OccupancyGrid},
    pipeline::Pipeline,
    planning::{
        global::{HybridAStar, PathPlanner, StraightLine},
        local::{DrivePose, Dwa, DwaWeights, LocalPlanner, Teleop},
    },
    sense::Mount,
    vehicle::{DiffDrive, Limits, VehicleModel},
};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PipelineConfig {
    pub vehicle: VehicleConfig,
    pub mount: MountConfig,
    pub estimator: EstimatorConfig,
    pub mapper: MapperConfig,
    pub path_planner: PathPlannerConfig,
    pub local_planner: LocalPlannerConfig,
    #[serde(default)]
    pub comms: CommsConfig,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum VehicleConfig {
    DiffDrive {
        track: f64,
        max_wheel_speed: f64,
        #[serde(default = "default_max_accel")]
        max_accel: f64,
        #[serde(default = "default_max_yaw_accel")]
        max_yaw_accel: f64,
    },
}

fn default_max_accel() -> f64 {
    2.0
}
fn default_max_yaw_accel() -> f64 {
    0.5
}

/// Phone mount: Z-Y-X Euler angles in degrees, camera offset in the body
/// frame in metres.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MountConfig {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
    pub offset: [f64; 3],
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum EstimatorConfig {
    Vio,
    Localization,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum MapperConfig {
    None,
    OccupancyGrid { resolution: f64 },
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum PathPlannerConfig {
    StraightLine,
    HybridAstar {
        resolution: f64,
        yaw_bins: usize,
        step: f64,
        heuristic_weight: f64,
    },
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum LocalPlannerConfig {
    Teleop,
    DrivePose { k_rho: f64, k_alpha: f64, k_beta: f64 },
    Dwa { horizon: f64, samples: usize, weights: DwaWeightsConfig },
}

#[derive(Debug, Default, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum CommsConfig {
    #[default]
    None,
    Zenoh {
        rover_id: String,
        endpoint: String,
        user: Option<String>,
        password: Option<String>,
    },
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DwaWeightsConfig {
    pub heading: f64,
    pub clearance: f64,
    pub speed: f64,
}

impl PipelineConfig {
    /// Parse a `terra-av.toml`. Missing sections, unknown keys and unknown
    /// `type`s are errors, so a typo cannot silently fall back to a default.
    pub fn from_toml(text: &str) -> Result<Self, String> {
        let config: Self = toml::from_str(text).map_err(|e| e.to_string())?;
        config.comms.check()?;
        Ok(config)
    }

    /// Use this router password instead of the one in the file, so the real
    /// one can live in the phone's Keychain. No effect without `[comms]`.
    pub fn with_router_password(mut self, password: Option<String>) -> Self {
        if let (CommsConfig::Zenoh { password: slot, .. }, Some(password)) = (&mut self.comms, password) {
            *slot = Some(password);
        }
        self
    }

    pub fn build(&self) -> Pipeline {
        Pipeline {
            vehicle: self.vehicle.build(),
            estimator: self.estimator.build(&self.mount),
            mapper: self.mapper.build(&self.mount),
            path_planner: self.path_planner.build(),
            local_planner: self.local_planner.build(),
            safety: SafetyGate,
            link: self.comms.build(),
        }
    }
}

impl CommsConfig {
    /// Errors that serde cannot catch: `rover_id` becomes one segment of the
    /// topic, and `zenoh` needs the crate feature of the same name.
    fn check(&self) -> Result<(), String> {
        let Self::Zenoh { rover_id, .. } = self else {
            return Ok(());
        };
        if !cfg!(feature = "zenoh") {
            return Err("[comms] type = \"zenoh\", but terra-av was built without the zenoh feature".into());
        }
        if rover_id.is_empty() || rover_id.contains(['/', '*', '$', '?', '#']) {
            return Err(format!("[comms] rover_id {rover_id:?} must be non-empty, without / * $ ? #"));
        }
        Ok(())
    }

    fn build(&self) -> Box<dyn BaseStationLink> {
        match self {
            Self::None => Box::new(NoLink),
            #[cfg(feature = "zenoh")]
            Self::Zenoh { rover_id, endpoint, user, password } => {
                Box::new(crate::comms::ZenohLink::new(crate::comms::ZenohSettings {
                    endpoint: endpoint.clone(),
                    rover_id: rover_id.clone(),
                    user: user.clone(),
                    password: password.clone(),
                }))
            }
            #[cfg(not(feature = "zenoh"))]
            Self::Zenoh { .. } => unreachable!("rejected by check()"),
        }
    }
}

impl VehicleConfig {
    fn build(&self) -> Box<dyn VehicleModel> {
        match *self {
            Self::DiffDrive { track, max_wheel_speed, max_accel, max_yaw_accel } => {
                Box::new(DiffDrive {
                    track,
                    limits: Limits { max_wheel_speed, max_accel, max_yaw_accel },
                })
            }
        }
    }
}

impl MountConfig {
    fn build(&self) -> Mount {
        let [x, y, z] = self.offset;
        Mount {
            camera_to_body: Rotation3::from_euler(
                self.roll.to_radians(),
                self.pitch.to_radians(),
                self.yaw.to_radians(),
            ),
            camera_offset: Vec3 { x, y, z },
        }
    }
}

impl EstimatorConfig {
    fn build(&self, mount: &MountConfig) -> Box<dyn StateEstimator> {
        match self {
            Self::Vio => Box::new(Vio { mount: mount.build() }),
            Self::Localization => Box::new(Localization),
        }
    }
}

impl MapperConfig {
    fn build(&self, mount: &MountConfig) -> Box<dyn Mapper> {
        match *self {
            Self::None => Box::new(NoMap),
            Self::OccupancyGrid { resolution } => {
                Box::new(OccupancyGrid { mount: mount.build(), resolution })
            }
        }
    }
}

impl PathPlannerConfig {
    fn build(&self) -> Box<dyn PathPlanner> {
        match *self {
            Self::StraightLine => Box::new(StraightLine),
            Self::HybridAstar { resolution, yaw_bins, step, heuristic_weight } => {
                Box::new(HybridAStar { resolution, yaw_bins, step, heuristic_weight })
            }
        }
    }
}

impl LocalPlannerConfig {
    fn build(&self) -> Box<dyn LocalPlanner> {
        match self {
            Self::Teleop => Box::new(Teleop),
            &Self::DrivePose { k_rho, k_alpha, k_beta } => {
                Box::new(DrivePose { k_rho, k_alpha, k_beta })
            }
            Self::Dwa { horizon, samples, weights } => Box::new(Dwa {
                horizon: *horizon,
                samples: *samples,
                weights: DwaWeights {
                    heading: weights.heading,
                    clearance: weights.clearance,
                    speed: weights.speed,
                },
            }),
        }
    }
}
