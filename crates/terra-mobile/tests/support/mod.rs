//! The phone's tick in tests: samples pushed between ticks are held as the
//! latest of each kind (what the apps keep) and handed to
//! `MobileController::update` by `step`.
#![allow(dead_code)]
use std::cell::RefCell;
use std::collections::HashMap;
use terra_mobile::*;

/// Local velocity intent in m/s and rad/s; sent as the phone stick.
pub struct TwistSetpoint {
    pub timestamp: f64,
    pub forward: f64,
    pub yaw_rate: f64,
}

thread_local! {
    /// Latest samples per controller, and whether any arrived since the last tick.
    static PENDING: RefCell<HashMap<usize, (PhoneSensors, bool)>> = RefCell::new(HashMap::new());
}

thread_local! {
    static LAST: RefCell<HashMap<usize, f64>> = RefCell::new(HashMap::new());
}

fn with(c: &MobileController, f: impl FnOnce(&mut PhoneSensors)) {
    PENDING.with(|p| {
        let mut p = p.borrow_mut();
        let s = p
            .entry(c as *const MobileController as usize)
            .or_insert_with(|| (idle(0.), false));
        f(&mut s.0);
        s.1 = true;
    });
}

fn take(c: &MobileController, now: f64) -> (PhoneSensors, bool) {
    PENDING.with(|p| {
        let mut p = p.borrow_mut();
        let s = p
            .entry(c as *const MobileController as usize)
            .or_insert_with(|| (idle(0.), false));
        s.0.now = now;
        let out = (s.0.clone(), s.1);
        s.1 = false;
        out
    })
}

pub fn idle(now: f64) -> PhoneSensors {
    PhoneSensors {
        now,
        imu: None,
        vio: None,
        gps: None,
        heading: None,
        course: None,
        stick: None,
        motors: MotorLink::default(),
        source: DriveSource::Phone,
    }
}

pub trait Phone {
    fn push_imu(&self, s: ImuReading) -> Result<(), ControllerError>;
    fn push_vio(&self, s: VioReading) -> Result<(), ControllerError>;
    fn push_gps(&self, s: GpsFix, now: f64) -> Result<(), ControllerError>;
    fn push_heading(&self, s: HeadingFix, now: f64) -> Result<(), ControllerError>;
    fn push_course(&self, s: CourseFix, now: f64) -> Result<(), ControllerError>;
    fn set_manual_input(&self, forward: f64, turn: f64, t: f64) -> Result<(), ControllerError>;
    fn set_target(&self, t: TwistSetpoint) -> Result<(), ControllerError>;
    fn set_motor_link(&self, link: MotorLink, source: DriveSource);
    fn step(&self, now: f64) -> Result<MotorTick, ControllerError>;
    /// A local request at `t`. The phone ticks every 10 ms, so samples
    /// pushed since the last tick reach the robot (in one tick at `t`)
    /// before the request is judged.
    fn request(&self, kind: String, payload: String, t: f64) -> Result<(), ControllerError>;
}

impl Phone for MobileController {
    fn push_imu(&self, s: ImuReading) -> Result<(), ControllerError> {
        with(self, |p| p.imu = Some(s));
        Ok(())
    }
    fn push_vio(&self, s: VioReading) -> Result<(), ControllerError> {
        with(self, |p| p.vio = Some(s));
        Ok(())
    }
    fn push_gps(&self, s: GpsFix, _now: f64) -> Result<(), ControllerError> {
        with(self, |p| p.gps = Some(s));
        Ok(())
    }
    fn push_heading(&self, s: HeadingFix, _now: f64) -> Result<(), ControllerError> {
        with(self, |p| p.heading = Some(s));
        Ok(())
    }
    fn push_course(&self, s: CourseFix, _now: f64) -> Result<(), ControllerError> {
        with(self, |p| p.course = Some(s));
        Ok(())
    }
    fn set_manual_input(&self, forward: f64, turn: f64, t: f64) -> Result<(), ControllerError> {
        with(self, |p| {
            p.stick = Some(StickInput {
                forward,
                turn,
                timestamp: t,
            })
        });
        Ok(())
    }
    fn set_target(&self, t: TwistSetpoint) -> Result<(), ControllerError> {
        self.set_manual_input(t.forward / 0.5, t.yaw_rate, t.timestamp)
    }
    fn set_motor_link(&self, link: MotorLink, source: DriveSource) {
        with(self, |p| {
            p.motors = link;
            p.source = source;
        });
    }
    fn step(&self, now: f64) -> Result<MotorTick, ControllerError> {
        // After a `request` ticked at this same time, run just after it: the
        // arbiter refuses a second tick at an unchanged time.
        let now = LAST.with(|l| {
            let mut l = l.borrow_mut();
            let last = l.entry(self as *const MobileController as usize).or_insert(f64::NEG_INFINITY);
            let now = if now <= *last && now + 1e-6 > *last { *last + 1e-6 } else { now };
            *last = now;
            now
        });
        self.update(take(self, now).0)
    }
    fn request(&self, kind: String, payload: String, t: f64) -> Result<(), ControllerError> {
        let (sensors, pending) = take(self, t);
        if pending {
            LAST.with(|l| l.borrow_mut().insert(self as *const MobileController as usize, t));
            self.update(sensors)?;
        }
        self.autonomy_request(kind, payload, t)
    }
}
