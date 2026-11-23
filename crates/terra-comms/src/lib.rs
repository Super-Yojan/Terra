//! The wire contract between the rover (`terra-av`) and the base station.
//!
//! One crate owns the config, the key names and the message encodings, so the
//! two sides can't drift apart. See `API.md` for the design.
//!
//! - [`CommsConfig`]: the `[comms]` table.
//! - [`Topics`] / [`Topic`]: every key is `{prefix}/{rover_id}/{topic}`.
//! - Messages ([`Task`], [`TaskStatus`], [`Pose`], …) and the [`Message`]
//!   trait that encodes them.
//! - [`ParamSpec`] and [`validate`], shared so the UI and the rover reject
//!   exactly the same inputs.
//! - With the `zenoh` feature (default): [`connect`], [`RoverLink`] and
//!   [`BaseLink`].
use std::time::Duration;

mod codec;
mod config;
mod messages;
mod params;
mod topics;

#[cfg(feature = "zenoh")]
mod link;

pub use codec::{DecodeError, Downlink, Message, Uplink, WIRE_VERSION};
pub use config::{CommsConfig, ConfigError};
pub use messages::{
    Attention, Capabilities, Depth, Estop, Goal, GoalKind, GpsAnchor, MeshChunk, Occupancy,
    PlannedPath, Pose, Pose2, Rejection, RemoteCmd, RemoteState, StageOption, Status, Task,
    TaskAction, TaskCommand, TaskState, TaskStatus, Teleop, Twist, WorldMap,
};
pub use params::{ParamError, ParamKind, ParamSpec, ParamValue, Params, check_specs, validate};
pub use topics::{Direction, Topic, Topics};

#[cfg(feature = "zenoh")]
pub use link::{BaseLink, Link, RoverLink, Stats, Subscription, TopicStats, connect};

/// How long a command over the link (`teleop`, `remote/cmd`) stays valid
/// after it was received. Covers the trip through the cloud-hosted router.
pub const COMMAND_LEASE: Duration = Duration::from_secs(1);

/// `BaseLink` re-sends the last e-stop (engaged or released) to each rover
/// this often, so a lost message or a restarted rover catches up.
pub const ESTOP_REPEAT: Duration = Duration::from_secs(1);

/// The base station lists a rover in `BaseLink::rovers` for this long after
/// its last new message (the link's automatic repeats don't count).
pub const ROVER_SEEN_WINDOW: Duration = Duration::from_secs(3);
