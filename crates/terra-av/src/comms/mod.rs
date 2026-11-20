//! Link to the base station, chosen in the `[comms]` section of the config.
//! Not a pipeline stage: the pipeline hands it data each tick and carries on.
//!
//! - [`NoLink`]: sends nothing. The default when `[comms]` is absent.
//! - [`ZenohLink`]: publishes to the fleet's Zenoh router (feature `zenoh`).
//!
//! Topics, all JSON:
//!
//! | key                     | payload                                                       |
//! |-------------------------|---------------------------------------------------------------|
//! | `terra/{rover_id}/depth`| `{"rover_id", "t", "frame": "camera", "points": [[x, y, z], ...]}` |
//!
//! Depth points are the thinned LiDAR points exactly as sensed: camera frame,
//! metres, `t` in the phone's sensor clock (s).
mod none;
#[cfg(feature = "zenoh")]
mod zenoh;

pub use none::NoLink;
#[cfg(feature = "zenoh")]
pub use self::zenoh::{ZenohLink, ZenohSettings};

use crate::geometry::Vec3;

/// A message from the base station. Empty until the rover subscribes to
/// anything.
pub struct BaseStationMsg {}

pub trait BaseStationLink: Send {
    /// Publish this frame's depth points. Must not block: it is called from
    /// the control tick. A link may drop frames when the network is slower
    /// than the tick.
    fn publish_depth(&mut self, t: f64, points: &[Vec3]);
    /// Latest message from the base station, if any.
    fn poll(&mut self) -> Option<BaseStationMsg>;
}
