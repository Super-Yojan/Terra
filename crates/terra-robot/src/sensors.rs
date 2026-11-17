//! What the host knows this tick: the latest value of each sensor, each with
//! its own timestamp, `None` until the first one arrives.
use terra_autonomy::outdoor::{CourseFix, GpsFix, HeadingFix};
use terra_types::{ImuSample, VioSample};

/// One control tick's inputs from the phone. Swift fills it from
/// its own sensor callbacks and hands it to [`crate::Robot::update`].
/// A sample already seen (same timestamp) is not applied twice.
#[derive(Clone, Debug, Default)]
pub struct SensorData {
    /// Control-clock time of this tick, monotonic seconds.
    pub now: f64,
    pub imu: Option<ImuSample>,
    pub vio: Option<VioSample>,
    pub gps: Option<GpsFix>,
    pub heading: Option<HeadingFix>,
    pub course: Option<CourseFix>,
    /// The phone's own joystick, if the person is touching it.
    pub stick: Option<Stick>,
    pub motors: MotorLink,
    /// Where motion evidence comes from.
    pub source: DriveSource,
}

/// Normalized local joystick: `forward` and `turn` in `[-1, 1]`, positive
/// turn is left.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stick {
    pub forward: f64,
    pub turn: f64,
    pub t: f64,
}

/// The phone's link to the motor board, and the facts only the phone knows.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MotorLink {
    /// A rover Bluetooth link exists.
    pub connected: bool,
    /// The link is armed and accepts drive frames.
    pub ready: bool,
    /// The person confirmed the phone sits in its mount.
    pub mount_confirmed: bool,
    /// AR tracking is normal on the latest camera frame.
    pub tracked: bool,
    /// The rover reports a layout usable for velocity-feedback actuation.
    pub feedback_compatible: bool,
}

/// What the phone is driving from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DriveSource {
    /// Phone sensors (ARKit / ARCore + IMU).
    #[default]
    Phone,
    /// The local simulated plant.
    Simulation,
    /// Bluetooth effort with no phone sensors: the debug joystick.
    BluetoothManual,
}
