//! Localization (roadmap 6): VIO for the pose, GPS for where that pose is
//! on Earth.
//!
//! The pose itself is exactly [`Vio`]'s: smooth, and in the world frame that
//! started where the phone was switched on. GPS is not mixed into the pose
//! (a 5 m GPS jump must never reach the controllers). Instead, every new GPS
//! fix refines the [`GpsAnchor`], which says where the world frame sits on
//! Earth. A GPS goal is converted to the world frame with the newest anchor
//! on every tick, so slow VIO drift moves the converted goal, not the pose.
//!
//! This is a least-squares fit, not an EKF: see `gps.rs` for the maths.
use super::gps::AnchorFit;
use super::{GpsAnchor, State, StateEstimator, Vio};
use crate::sense::{GpsFix, Mount, SensorFrame};
use crate::vehicle::VehicleModel;

pub struct Localization {
    vio: Vio,
    fit: AnchorFit,
    /// The newest fix already given to `fit`. The phone repeats its latest
    /// fix in every frame (GPS is ~1 Hz, frames are 20 Hz); each fix must
    /// count once.
    last_fix: Option<GpsFix>,
}

impl Localization {
    pub fn new(mount: Mount) -> Self {
        Self {
            vio: Vio::new(mount),
            fit: AnchorFit::default(),
            last_fix: None,
        }
    }
}

impl StateEstimator for Localization {
    fn update(&mut self, frame: &SensorFrame, vehicle: &dyn VehicleModel) -> Option<State> {
        // No tracked VIO pose: no state, and no sample either, because a GPS
        // fix is only useful paired with the world position it was taken at.
        let state = self.vio.update(frame, vehicle)?;

        if let Some(fix) = frame.gps
            && self.last_fix != Some(fix)
        {
            self.last_fix = Some(fix);
            self.fit.add(state.pose.position(), &fix);
        }
        Some(state)
    }

    fn gps_anchor(&self) -> Option<GpsAnchor> {
        self.fit.anchor()
    }
}
