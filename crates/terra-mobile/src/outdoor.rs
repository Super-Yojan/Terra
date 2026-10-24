//! Phone FFI records around the shared geographic waypoint controller.
pub use terra_autonomy::outdoor::{Evidence, OutdoorController, distance_bearing};
#[derive(Clone, Copy, Debug, uniffi::Record)]
pub struct GpsFix {
    pub timestamp: f64,
    pub latitude: f64,
    pub longitude: f64,
    pub accuracy: f64,
}
#[derive(Clone, Copy, Debug, uniffi::Record)]
pub struct HeadingFix {
    pub timestamp: f64,
    pub true_heading: f64,
    pub accuracy: f64,
}
/// GPS course over ground; the core pairs it with local travel direction.
#[derive(Clone, Copy, Debug, uniffi::Record)]
pub struct CourseFix {
    pub timestamp: f64,
    pub course: f64,
    pub accuracy: f64,
    pub speed: f64,
}

impl From<GpsFix> for terra_autonomy::outdoor::GpsFix {
    fn from(sample: GpsFix) -> Self {
        Self {
            timestamp: sample.timestamp,
            latitude: sample.latitude,
            longitude: sample.longitude,
            accuracy: sample.accuracy,
        }
    }
}
impl From<HeadingFix> for terra_autonomy::outdoor::HeadingFix {
    fn from(sample: HeadingFix) -> Self {
        Self {
            timestamp: sample.timestamp,
            true_heading: sample.true_heading,
            accuracy: sample.accuracy,
        }
    }
}
impl From<CourseFix> for terra_autonomy::outdoor::CourseFix {
    fn from(sample: CourseFix) -> Self {
        Self {
            timestamp: sample.timestamp,
            course: sample.course,
            accuracy: sample.accuracy,
            speed: sample.speed,
        }
    }
}
