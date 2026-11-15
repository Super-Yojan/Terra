use crate::*;
use outdoor::{CourseFix, GpsFix, HeadingFix};
#[uniffi::export]
impl MobileController {
    /// Invalidate geographic guidance when the platform starts a new local AR frame.
    pub fn reset_outdoor_frame(&self) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .runtime
            .reset_outdoor_frame();
        Ok(())
    }

    pub fn invalidate_outdoor_sensors(&self) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .runtime
            .invalidate_outdoor_sensors();
        self.brain.inbox.clear_outdoor();
        Ok(())
    }
    /// Called only by platforms implementing measured GPS and mount-aware true heading ingestion.
    pub fn enable_outdoor_gps(&self) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .runtime
            .enable_outdoor_gps();
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
