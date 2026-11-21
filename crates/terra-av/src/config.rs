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
//! [comms]                        # optional; absent = no base station
//! endpoint = "tcp/192.168.1.10:7447"   # tcp/host:port or tls/host:port
//! prefix = "terra"               # optional; keys are terra/<rover_id>/<topic>
//! rover_id = 0                   # required on the rover
//! user = "terra"                 # optional router login
//! # The password is never in the file: the app passes it from the Keychain.
//! ```
//!
//! `[comms]` is `terra_comms::CommsConfig`; see `crates/terra-comms/API.md`.
use serde::Deserialize;

use terra_comms::{Capabilities, GoalKind, StageOption};

use crate::{
    comms::Link,
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
    /// The base-station link; `None` when there is no `[comms]` table.
    #[serde(default)]
    pub comms: Option<terra_comms::CommsConfig>,
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
        if let Some(comms) = &config.comms {
            if !cfg!(feature = "zenoh") {
                return Err("[comms] is set, but terra-av was built without the zenoh feature".into());
            }
            if comms.rover_id.is_none() {
                return Err("[comms] rover_id is required on the rover".into());
            }
        }
        Ok(config)
    }

    /// Use this router password instead of the one in the file, so the real
    /// one can live in the phone's Keychain. No effect without `[comms]`.
    pub fn with_router_password(mut self, password: Option<String>) -> Self {
        self.comms = self.comms.map(|c| c.with_password(password));
        self
    }

    /// Build the pipeline and, with `[comms]`, start the base-station link
    /// (in the background; this returns at once). Fails only on a bad
    /// `[comms]`.
    pub fn build(&self) -> Result<Pipeline, String> {
        Ok(Pipeline {
            vehicle: self.vehicle.build(),
            estimator: self.estimator.build(&self.mount),
            mapper: self.mapper.build(&self.mount),
            path_planner: self.path_planner.build(),
            local_planner: self.local_planner.build(),
            safety: SafetyGate,
            link: self.link()?,
        })
    }

    fn link(&self) -> Result<Link, String> {
        match &self.comms {
            None => Ok(Link::none()),
            #[cfg(feature = "zenoh")]
            Some(comms) => Link::connect(comms, &self.capabilities()),
            #[cfg(not(feature = "zenoh"))]
            Some(_) => Err("[comms] needs the zenoh feature".into()),
        }
    }

    /// What this rover can run, for the base station. Until there is a
    /// controller registry, that's just the configured controller and
    /// planner, with no run-time parameters.
    pub fn capabilities(&self) -> Capabilities {
        let option = |id: &str, description: &str, goals: Vec<GoalKind>| StageOption {
            id: id.into(),
            version: env!("CARGO_PKG_VERSION").into(),
            description: description.into(),
            params: vec![],
            goals,
        };
        let controller = match self.local_planner {
            LocalPlannerConfig::Teleop => option("teleop", "Drive from a stick", vec![]),
            LocalPlannerConfig::DrivePose { .. } => {
                option("drive_pose", "Steer onto the next path pose", vec![GoalKind::Local])
            }
            LocalPlannerConfig::Dwa { .. } => {
                option("dwa", "Dynamic window approach", vec![GoalKind::Local])
            }
        };
        let planner = match self.path_planner {
            PathPlannerConfig::StraightLine => {
                option("straight_line", "The goal itself, no search", vec![GoalKind::Local])
            }
            PathPlannerConfig::HybridAstar { .. } => {
                option("hybrid_astar", "A* over drivable arcs", vec![GoalKind::Local])
            }
        };
        Capabilities {
            t: 0.0,
            default_controller: controller.id.clone(),
            controllers: vec![controller],
            planners: vec![planner],
            software: env!("CARGO_PKG_VERSION").into(),
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
            Self::Vio => Box::new(Vio::new(mount.build())),
            Self::Localization => Box::new(Localization),
        }
    }
}

impl MapperConfig {
    fn build(&self, mount: &MountConfig) -> Box<dyn Mapper> {
        match *self {
            Self::None => Box::new(NoMap),
            Self::OccupancyGrid { resolution } => {
                Box::new(OccupancyGrid::new(mount.build(), resolution))
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

#[cfg(test)]
mod tests {
    use super::*;

    const APP_CONFIG: &str = include_str!("../../../apps/ios/TerraPhone/Resources/terra-av.toml");

    fn with_comms(comms: &str) -> String {
        let base = APP_CONFIG.split("[comms]").next().unwrap();
        format!("{base}[comms]\n{comms}")
    }

    #[test]
    #[cfg(feature = "zenoh")]
    fn the_app_config_parses() {
        let config = PipelineConfig::from_toml(APP_CONFIG).unwrap();
        let comms = config.comms.as_ref().expect("the app ships a [comms] table");
        assert_eq!(comms.rover_id, Some(0));
        assert_eq!(comms.prefix, "terra");
        let config = config.with_router_password(Some("secret".into()));
        assert_eq!(config.comms.unwrap().password.as_deref(), Some("secret"));
    }

    #[test]
    fn comms_is_optional() {
        let config = PipelineConfig::from_toml(APP_CONFIG.split("[comms]").next().unwrap()).unwrap();
        assert!(config.comms.is_none());
    }

    #[test]
    #[cfg(not(feature = "zenoh"))]
    fn comms_needs_the_zenoh_feature() {
        let err = PipelineConfig::from_toml(APP_CONFIG).unwrap_err();
        assert!(err.contains("zenoh feature"), "{err}");
    }

    #[test]
    fn rejects_bad_comms() {
        for comms in [
            "endpoint = \"tcp/h:1\"",                                   // no rover_id
            "type = \"zenoh\"\nrover_id = 0\nendpoint = \"tcp/h:1\"",   // old format
            "rover_id = \"0\"\nendpoint = \"tcp/h:1\"",                 // string id
            "rover_id = 0\nendpoint = \"tcp/h:1\"\npassword = \"x\"",   // password in the file
            "rover_id = 0\nendpoint = \"udp/h:1\"",                     // bad endpoint
        ] {
            assert!(PipelineConfig::from_toml(&with_comms(comms)).is_err(), "accepted {comms:?}");
        }
    }

    #[test]
    fn capabilities_list_the_configured_stages() {
        let without_comms = APP_CONFIG.split("[comms]").next().unwrap();
        let caps = PipelineConfig::from_toml(without_comms).unwrap().capabilities();
        assert_eq!(caps.default_controller, "teleop");
        assert_eq!(caps.controllers.len(), 1);
        assert_eq!(caps.planners[0].id, "straight_line");
    }
}
