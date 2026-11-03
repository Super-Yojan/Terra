use crate::*;
use outdoor::{CourseFix, GpsFix, HeadingFix};
pub(crate) fn invalid(s: &str) -> ControllerError {
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
        self.brain.inbox.clear_outdoor();
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
    /// Queue a fix for the next `step`; `now` is the clock its freshness is judged against.
    pub fn push_gps(&self, sample: GpsFix, now: f64) -> Result<(), ControllerError> {
        self.brain
            .inbox
            .push(crate::inbox::Sample::Gps(sample, now))
    }
    /// Queue a fix for the next `step`; `now` is the clock its freshness is judged against.
    pub fn push_heading(&self, sample: HeadingFix, now: f64) -> Result<(), ControllerError> {
        self.brain
            .inbox
            .push(crate::inbox::Sample::Heading(sample, now))
    }
    /// Queue a fix for the next `step`; `now` is the clock its freshness is judged against.
    pub fn push_course(&self, sample: CourseFix, now: f64) -> Result<(), ControllerError> {
        self.brain
            .inbox
            .push(crate::inbox::Sample::Course(sample, now))
    }
}
