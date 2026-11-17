//! Local tangent-plane conversions for the goal map picker.
use crate::ControllerError;
use terra_types::InputError;
use terra_waypoint::GeoOrigin;

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
