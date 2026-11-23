//! Stage 5, Execution, Rust half: the fixed chain every target twist goes
//! through before it leaves the pipeline, whatever produced it:
//!
//! ```text
//! twist ─► vehicle.clamp ─► limit_acceleration ─► SafetyGate ─► DriveCommand
//! ```
//!
//! The velocity loop (twist → wheel effort) runs on the platform that owns
//! the motors; on TerraPhone that is `Motors/VelocityController.swift`.
mod limiter;
mod safety;
pub use limiter::limit_acceleration;
pub use safety::SafetyGate;
