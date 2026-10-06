//! UniFFI waypoint follower. The phone owns the goal and feeds the twist into `MobileController`.
use crate::ControllerError;
use std::sync::{Arc, Mutex};
use terra_types::InputError;
use terra_waypoint::{GeoOrigin, GoalCommand, WaypointConfig, WaypointController};

#[derive(Clone, Copy, Debug, uniffi::Record)]
pub struct TangentMetres {
    pub north: f64,
    pub west: f64,
}

#[derive(Clone, Copy, Debug, uniffi::Record)]
pub struct GeographicPosition {
    pub latitude: f64,
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
pub struct WaypointSettings {
    pub cruise: f64,
    pub yaw_gain: f64,
    pub max_yaw_rate: f64,
    pub arrive_radius: f64,
    pub align_yaw: f64,
    pub slow_radius: f64,
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
pub enum WaypointPhase {
    Idle,
    Active,
    Arrived,
}

#[derive(Clone, Debug, uniffi::Record)]
pub struct WaypointStep {
    pub forward: f64,
    pub yaw_rate: f64,
    pub distance: f64,
    pub phase: WaypointPhase,
    pub goal_id: u64,
    pub local_x: f64,
    pub local_y: f64,
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
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
pub struct MobileWaypoint {
    follower: Mutex<WaypointController>,
}

#[uniffi::export]
impl MobileWaypoint {
    #[uniffi::constructor]
    pub fn new(settings: WaypointSettings) -> Result<Arc<Self>, ControllerError> {
        Ok(Arc::new(Self {
            follower: Mutex::new(WaypointController::new(settings.config())?),
        }))
    }

    /// Latitude and longitude of the pose origin. Geographic goals are measured from here.
    pub fn set_origin(&self, latitude: f64, longitude: f64) -> Result<(), ControllerError> {
        self.follower
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .set_origin(latitude, longitude)?;
        Ok(())
    }

    /// Latch one WGS84 goal. `half_extent` is how far from the origin, in metres, a goal may lie.
    /// Returns false when the point is outside that square or the origin has not been set.
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

    pub fn cancel(&self) -> Result<(), ControllerError> {
        self.follower
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .cancel();
        Ok(())
    }

    /// Rover pose in the origin frame: `x` metres north, `y` metres west, `yaw` radians from north.
    pub fn step(&self, x: f64, y: f64, yaw: f64) -> Result<WaypointStep, ControllerError> {
        let follower = self
            .follower
            .lock()
            .map_err(|_| ControllerError::Internal)?;
        Ok(follower.step(x, y, yaw).into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
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
    fn map_tap_round_trips_to_the_same_latitude() {
        let projected = tangent_metres(38.8297, -77.3075, 38.82981, -77.3075).unwrap();
        assert!(projected.north > 12.0 && projected.west.abs() < 1e-6);
        let geographic =
            geographic_position(38.8297, -77.3075, projected.north, projected.west).unwrap();
        assert!((geographic.latitude - 38.82981).abs() < 1e-9);
        assert!((geographic.longitude + 77.3075).abs() < 1e-9);
    }
}
