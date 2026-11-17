use crate::*;
#[uniffi::export]
impl MobileController {
    /// Invalidate geographic guidance when the platform starts a new local AR frame.
    pub fn reset_outdoor_frame(&self) -> Result<(), ControllerError> {
        self.robot()?.core_mut().reset_outdoor_frame();
        Ok(())
    }
    /// Drop GPS, heading and course evidence, for example when location access ends.
    pub fn invalidate_outdoor_sensors(&self) -> Result<(), ControllerError> {
        self.robot()?.core_mut().invalidate_outdoor_sensors();
        Ok(())
    }
    /// Called only by platforms implementing measured GPS and mount-aware true heading ingestion.
    pub fn enable_outdoor_gps(&self) -> Result<(), ControllerError> {
        self.robot()?.core_mut().enable_outdoor_gps();
        Ok(())
    }
}
