//! UniFFI waypoint follower. The phone owns the goal and feeds the twist into `MobileController`.
use crate::ControllerError;
use std::sync::{Arc, Mutex};
use terra_types::InputError;
use terra_waypoint::{GeoOrigin, GoalCommand, WaypointConfig, WaypointController};

#[derive(Clone, Copy, Debug, uniffi::Record)]
/// Local tangent-plane point relative to a WGS84 origin.
pub struct TangentMetres {
    /// Distance north of the origin, metres (+X).
    pub north: f64,
    /// Distance west of the origin, metres (+Y).
    pub west: f64,
}

#[derive(Clone, Copy, Debug, uniffi::Record)]
/// WGS84 point in decimal degrees.
pub struct GeographicPosition {
    /// WGS84 latitude, decimal degrees.
    pub latitude: f64,
    /// WGS84 longitude, decimal degrees.
    pub longitude: f64,
}

fn geographic_range(latitude: f64, longitude: f64) -> Result<(), ControllerError> {
    if !latitude.is_finite()
        || !longitude.is_finite()
        || !(-85.0..=85.0).contains(&latitude)
        || !(-180.0..=180.0).contains(&longitude)
    {
        Err(InputError::OutOfRange.into())
    } else {
        Ok(())
    }
}

/// Project a WGS84 point into the follower frame: `north` is +x, `west` is +y.
#[uniffi::export]
/// Inputs are decimal degrees. Returns `InvalidInput` for nonfinite or invalid
/// coordinates (latitude must lie in ±85°, longitude in ±180°).
pub fn tangent_metres(
    origin_latitude: f64,
    origin_longitude: f64,
    latitude: f64,
    longitude: f64,
) -> Result<TangentMetres, ControllerError> {
    geographic_range(latitude, longitude)?;
    let (north, west) =
        GeoOrigin::new(origin_latitude, origin_longitude)?.to_local(latitude, longitude);
    Ok(TangentMetres { north, west })
}

/// Inverse of [`tangent_metres`]. A map tap in north/west metres becomes the goal coordinate.
#[uniffi::export]
/// Origin coordinates are decimal degrees. Rejects nonfinite metre offsets or
/// invalid geographic coordinates with `InvalidInput`.
pub fn geographic_position(
    origin_latitude: f64,
    origin_longitude: f64,
    north: f64,
    west: f64,
) -> Result<GeographicPosition, ControllerError> {
    if !north.is_finite() || !west.is_finite() {
        return Err(InputError::OutOfRange.into());
    }
    let (latitude, longitude) =
        GeoOrigin::new(origin_latitude, origin_longitude)?.from_local(north, west);
    geographic_range(latitude, longitude)?;
    Ok(GeographicPosition {
        latitude,
        longitude,
    })
}

#[derive(Clone, Debug, uniffi::Record)]
/// Waypoint speed, heading, slowing, and arrival parameters.
pub struct WaypointSettings {
    /// Cruise forward speed, m/s.
    pub cruise: f64,
    /// Heading-error proportional gain, per second.
    pub yaw_gain: f64,
    /// Maximum yaw-rate magnitude, rad/s.
    pub max_yaw_rate: f64,
    /// Position arrival tolerance, metres.
    pub arrive_radius: f64,
    /// Final heading tolerance, radians.
    pub align_yaw: f64,
    /// Distance over which forward speed tapers, metres.
    pub slow_radius: f64,
    /// Heading-error threshold for permitting forward travel, radians.
    pub heading_gate: f64,
}

impl WaypointSettings {
    fn config(&self) -> WaypointConfig {
        WaypointConfig {
            cruise: self.cruise,
            yaw_gain: self.yaw_gain,
            max_yaw_rate: self.max_yaw_rate,
            arrive_radius: self.arrive_radius,
            align_yaw: self.align_yaw,
            slow_radius: self.slow_radius,
            heading_gate: self.heading_gate,
        }
    }
}

#[uniffi::export]
/// Return the portable waypoint follower defaults.
pub fn default_waypoint_settings() -> WaypointSettings {
    let config = WaypointConfig::default();
    WaypointSettings {
        cruise: config.cruise,
        yaw_gain: config.yaw_gain,
        max_yaw_rate: config.max_yaw_rate,
        arrive_radius: config.arrive_radius,
        align_yaw: config.align_yaw,
        slow_radius: config.slow_radius,
        heading_gate: config.heading_gate,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, uniffi::Enum)]
/// Lifecycle phase reported by the phone waypoint follower.
pub enum WaypointPhase {
    /// No active goal; requested motion is zero.
    Idle,
    /// Follower is approaching the goal or aligning to its requested heading.
    Active,
    /// Goal position and any requested final heading have been reached.
    Arrived,
}

#[derive(Clone, Debug, uniffi::Record)]
/// Velocity intent and goal status from one waypoint-follower evaluation.
pub struct WaypointStep {
    /// Requested body forward speed, m/s.
    pub forward: f64,
    /// Requested body yaw rate, rad/s.
    pub yaw_rate: f64,
    /// Distance to the goal, metres.
    pub distance: f64,
    /// Goal lifecycle phase.
    pub phase: WaypointPhase,
    /// Follower-assigned goal identifier.
    pub goal_id: u64,
    /// Goal north coordinate relative to the origin, metres.
    pub local_x: f64,
    /// Goal west coordinate relative to the origin, metres.
    pub local_y: f64,
    /// WGS84 latitude, decimal degrees.
    pub latitude: Option<f64>,
    /// WGS84 longitude, decimal degrees.
    pub longitude: Option<f64>,
    /// Optional caller-provided goal correlation token.
    pub token: Option<String>,
}

impl From<terra_waypoint::WaypointOutput> for WaypointStep {
    fn from(output: terra_waypoint::WaypointOutput) -> Self {
        let status = output.status;
        Self {
            forward: output.linear,
            yaw_rate: output.angular,
            distance: status.distance,
            phase: match status.state {
                terra_waypoint::GoalState::Idle => WaypointPhase::Idle,
                terra_waypoint::GoalState::Active => WaypointPhase::Active,
                terra_waypoint::GoalState::Arrived => WaypointPhase::Arrived,
            },
            goal_id: status.goal_id,
            local_x: status.x,
            local_y: status.y,
            latitude: status.latitude,
            longitude: status.longitude,
            token: status.token,
        }
    }
}

#[derive(uniffi::Object)]
/// Mutex-protected geographic waypoint follower; does not directly drive motors.
pub struct MobileWaypoint {
    follower: Mutex<WaypointController>,
}

#[uniffi::export]
impl MobileWaypoint {
    #[uniffi::constructor]
    /// Construct an idle follower with validated settings.
    /// Returns `InvalidInput` for invalid configuration.
    pub fn new(settings: WaypointSettings) -> Result<Arc<Self>, ControllerError> {
        Ok(Arc::new(Self {
            follower: Mutex::new(WaypointController::new(settings.config())?),
        }))
    }

    /// Latitude and longitude of the pose origin. Geographic goals are measured from here.
    /// Coordinates are decimal degrees; returns `InvalidInput` for invalid origin
    /// coordinates and `Internal` for a poisoned mutex.
    pub fn set_origin(&self, latitude: f64, longitude: f64) -> Result<(), ControllerError> {
        self.follower
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .set_origin(latitude, longitude)?;
        Ok(())
    }

    /// Latch one WGS84 goal. `half_extent` is how far from the origin, in metres, a goal may lie.
    /// Returns false when the point is outside that square or the origin has not been set.
    /// Coordinates are decimal degrees and optional final yaw is radians. Optional
    /// tokens contain 1..=64 ASCII alphanumeric or `._:-` bytes. Callers must validate
    /// geographic bounds and final yaw: this method does not enforce them.
    /// Returns `InvalidInput` for invalid tokens or nonpositive/nonfinite extent,
    /// and `Internal` for a poisoned mutex. Nonfinite projected coordinates return false.
    pub fn set_goal(
        &self,
        latitude: f64,
        longitude: f64,
        yaw: Option<f64>,
        token: Option<String>,
        half_extent: f64,
    ) -> Result<bool, ControllerError> {
        if let Some(token) = &token {
            let bytes = token.as_bytes();
            if !(1..=64).contains(&bytes.len())
                || !bytes.iter().all(|byte| {
                    byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b':' | b'-')
                })
            {
                return Err(InputError::OutOfRange.into());
            }
        }
        self.follower
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .accept(
                &GoalCommand::Wgs84 {
                    latitude,
                    longitude,
                    yaw,
                    token,
                },
                half_extent,
            )
            .map_err(ControllerError::from)
    }

    /// Clear the active goal so subsequent steps return idle motion.
    /// Returns `Internal` for a poisoned mutex.
    pub fn cancel(&self) -> Result<(), ControllerError> {
        self.follower
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .cancel();
        Ok(())
    }

    /// Rover pose in the origin frame: `x` metres north, `y` metres west, `yaw` radians from north.
    /// Returns a velocity request and copied goal status; does not advance a clock
    /// or actuate hardware. Returns `Internal` for a poisoned mutex.
    pub fn step(&self, x: f64, y: f64, yaw: f64) -> Result<WaypointStep, ControllerError> {
        let follower = self
            .follower
            .lock()
            .map_err(|_| ControllerError::Internal)?;
        Ok(follower.step(x, y, yaw).into())
    }
}

/// Deterministic contract and regression scenarios for this module.
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    /// Sets a geographic goal north of the origin, checks forward steering and goal metadata,
    /// then cancels and checks idle zero motion.
    fn phone_facade_steers_toward_a_lat_lon_north_of_the_origin() {
        let follower = MobileWaypoint::new(default_waypoint_settings()).unwrap();
        follower.set_origin(38.8297, -77.3075).unwrap();
        assert!(
            follower
                .set_goal(38.82981, -77.3075, None, Some("gmu-north".into()), 49.0,)
                .unwrap()
        );
        let step = follower.step(0.0, 0.0, 0.0).unwrap();
        assert_eq!(step.phase, WaypointPhase::Active);
        assert_eq!(step.goal_id, 1);
        assert!(step.forward > 0.5, "{}", step.forward);
        assert!(step.yaw_rate.abs() < 0.05, "{}", step.yaw_rate);
        assert!(step.distance > 10.0, "{}", step.distance);
        assert_eq!(step.latitude, Some(38.82981));
        assert_eq!(step.longitude, Some(-77.3075));
        assert_eq!(step.token.as_deref(), Some("gmu-north"));
        follower.cancel().unwrap();
        let idle = follower.step(0.0, 0.0, 0.0).unwrap();
        assert_eq!(idle.phase, WaypointPhase::Idle);
        assert_eq!(idle.forward, 0.0);
    }

    #[test]
    /// Projects a nearby WGS84 point into north/west metres and checks inverse projection within 1e-9 degrees.
    fn map_tap_round_trips_to_the_same_latitude() {
        let projected = tangent_metres(38.8297, -77.3075, 38.82981, -77.3075).unwrap();
        assert!(projected.north > 12.0 && projected.west.abs() < 1e-6);
        let geographic =
            geographic_position(38.8297, -77.3075, projected.north, projected.west).unwrap();
        assert!((geographic.latitude - 38.82981).abs() < 1e-9);
        assert!((geographic.longitude + 77.3075).abs() < 1e-9);
    }
}
