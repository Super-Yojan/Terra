//! Experiment records, search-mission observations, and offline metrics.
//!
//! `mission` defines the recorded mission data; `writer` moves file I/O onto a
//! bounded background queue; `metrics` summarizes recorded outcomes. Recording
//! failure must remain visible and must not block the control loop.

mod mission;
mod writer;
pub use mission::*;
pub use writer::*;
mod metrics;
pub use metrics::*;
