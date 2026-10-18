//! Experiment records, search-mission observations, and offline metrics.
//!
//! `mission` defines the recorded mission data; `writer` moves file I/O onto a
//! bounded background queue; `metrics` summarizes recorded outcomes. Recording
//! failure must remain visible and must not block the control loop.
//!
//! Mission coordinates and ranges use metres. Mission and recorded control-tick
//! timestamps are seconds from run start. Ground-truth survivor placements belong
//! to simulator evaluation; published mission status includes only observed
//! sightings. Confirmation alone does not complete a mission: every observing
//! rover must return within 2 m of the origin after all survivors are confirmed.
//!
//! ```
//! use terra_experiment::{Mission, MissionConfig};
//!
//! let mut config = MissionConfig::rescue(42);
//! config.survivors.truncate(1);
//! let target = config.survivors[0].clone();
//! let mut mission = Mission::new(config).unwrap();
//! assert!(mission.status(0.0).observations.is_empty());
//! assert_eq!(mission.observe(7, target.x, target.y, 1.0, true).len(), 1);
//! assert!(mission.report(7, target.id, 2.0));
//! assert_eq!(mission.status(2.0).phase, "return");
//! mission.update_pose(7, 0.0, 0.0, 3.0);
//! assert!(mission.status(3.0).complete);
//! ```

mod mission;
mod writer;
pub use mission::*;
pub use writer::*;
mod metrics;
pub use metrics::*;
