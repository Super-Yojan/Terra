use crate::*;
use outdoor::{CourseFix, GpsFix, HeadingFix};
fn invalid(s: &str) -> ControllerError {
    ControllerError::InvalidInput { message: s.into() }
}
pub(crate) use terra_runtime::outdoor::evidence;
#[uniffi::export]
impl MobileController {
    /// Invalidate geographic guidance when the platform starts a new local AR frame.
    pub fn reset_outdoor_frame(&self) -> Result<(), ControllerError> {
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        b.outdoor.cancel();
        b.outdoor.invalidate_sensors();
        b.pose = None;
        b.map = None;
        b.imu_time = None;
        b.estimator.reset();
        b.outdoor.revision = b.outdoor.revision.saturating_add(1);
        if b.outdoor.selected {
            b.autonomy.clear_guidance();
            b.controller.reset();
        }
        Ok(())
    }

    pub fn invalidate_outdoor_sensors(&self) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .outdoor
            .invalidate_sensors();
        Ok(())
    }
    /// Called only by platforms implementing measured GPS and mount-aware true heading ingestion.
    pub fn enable_outdoor_gps(&self) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .outdoor
            .supported = true;
        Ok(())
    }
    pub fn push_gps(&self, sample: GpsFix, now: f64) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .outdoor
            .gps(sample, now)
            .map_err(invalid)
    }
    pub fn push_heading(&self, sample: HeadingFix, now: f64) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .outdoor
            .heading(sample, now)
            .map_err(invalid)
    }
    pub fn push_course(&self, sample: CourseFix, now: f64) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .outdoor
            .course(sample, now)
            .map_err(invalid)
    }
}
