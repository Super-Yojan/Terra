//! Stage 5, Execution, Rust half: the last check on the target twist before
//! it leaves the pipeline. The velocity loop (twist → wheel effort) runs on
//! the platform that owns the motors; on TerraPhone that is
//! `Motors/VelocityController.swift`.
mod safety;
pub use safety::SafetyGate;
