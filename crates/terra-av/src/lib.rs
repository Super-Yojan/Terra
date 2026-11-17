//! A modular autonomy pipeline for a ground vehicle, one module per stage:
//!
//! ```text
//! sense ─► estimation ─► mapping ─► planning (global ─► local) ─► execution ─► motors
//!                ▲                      ▲            ▲
//!                └──── vehicle (shared) ┴────────────┘
//!                      geometry (shared)
//! ```
//!
//! Each stage is a trait; each algorithm is one implementation of it.
//! `config` picks one implementation per stage, so swapping a planner or a
//! vehicle is a configuration change.
pub mod geometry;
pub mod vehicle;

pub mod sense;
pub mod estimation;
pub mod mapping;
pub mod planning;
pub mod execution;

pub mod config;
pub mod pipeline;

/// Swift bindings for the phone.
pub mod ffi;

uniffi::setup_scaffolding!();
