//! Signed left/right effort to PWM duty and direction.
//!
//! [`MotorOutput`] efforts stay in `[-1, 1]`. This adapter is the next stage:
//! magnitude becomes PWM duty, sign becomes direction, and nothing is driven
//! unless the hardware enable switch is closed and the command is still inside
//! the motor watchdog window.
//!
//! # Fail-safe
//!
//! Zero effort is **coast**, not a brake. Duty is 0, direction is absent, and
//! [`WheelPwm::in1_in2`] is `(0, 0)`. Both inputs low is fast-decay coast on a
//! typical H-bridge. Both inputs high would brake; this adapter never produces
//! that. There is no mechanical-brake output.
//!
//! `drive_enabled` is the software copy of the driver enable / nSLEEP line.
//! It is false when the switch is open, the sample is rejected, the clock steps
//! backwards, or the watchdog expires. Write that pin from the returned
//! [`ChassisPwm`]. Deasserting enable is not itself a brake command: choose a
//! driver whose disable state coasts or disconnects, and wire the physical
//! switch so it can remove motor power or driver enable even if this process
//! is wedged.
//!
//! # Phone and link loss
//!
//! The velocity controller already emits zero effort when the phone backgrounds,
//! tracking is lost, or the velocity target goes stale. If those fresh zeros
//! are applied with a new producer timestamp, the watchdog stays kicked, both
//! channels coast, and `drive_enabled` stays true while the switch is closed.
//! The chassis is not braked.
//!
//! The watchdog is independent of that controller timeout. It measures
//! `now - commanded_at` for the last effort **accepted** by [`MotorAdapter::apply`].
//! Re-applying a stored effort with the original `commanded_at` does not extend
//! the window. When the age reaches `command_timeout` (default 200 ms), or when
//! `apply` stops and [`MotorAdapter::poll`] crosses that age, both channels coast
//! and `drive_enabled` drops. A stalled control task, a dead serial link, or a
//! bridge that keeps forwarding the last phone stamp all end there. The rover
//! can still roll. The hardware switch is the hard inhibit.
//!
//! Only call [`MotorAdapter::apply_effort_now`] for an effort computed on that
//! tick. Using it to replay the previous [`MotorOutput`] with `now` refreshed
//! defeats the watchdog.
//!
//! # Tick
//!
//! Sample the switch, then apply a command only when a new one was produced:
//!
//! ```
//! use terra_motors::{ChassisPwm, MotorAdapter, MotorAdapterConfig, MotorDirection, OutputHold};
//!
//! let mut adapter = MotorAdapter::new(MotorAdapterConfig::default()).unwrap();
//! let switch_closed = true;
//! let now = 0.0;
//! let mut pwm: ChassisPwm = adapter.set_hardware_enable(switch_closed, now);
//! pwm = adapter.apply_effort_now(now, 0.5, -0.25);
//! assert_eq!(pwm.hold, OutputHold::Live);
//! assert!(pwm.drive_enabled);
//! assert_eq!(pwm.left.duty, 0.5);
//! assert_eq!(pwm.left.direction, Some(MotorDirection::Forward));
//! assert_eq!(pwm.right.duty, 0.25);
//! assert_eq!(pwm.right.direction, Some(MotorDirection::Reverse));
//! assert_eq!(pwm.left.in1_in2(), (0.5, 0.0));
//! assert_eq!(pwm.right.in1_in2(), (0.0, 0.25));
//! let held = adapter.poll(0.2);
//! assert_eq!(held.hold, OutputHold::Watchdog);
//! assert!(!held.drive_enabled);
//! assert!(held.is_coasting());
//! ```
//!
//! Physical phone sensing and hardware actuation have not been validated on a rover.
//! Bench steps are in `crates/terra-motors/README.md`.

use terra_types::{InputError, MotorOutput};

/// Values this close to ±1 still map as full-scale duty. Anything farther is rejected.
const EFFORT_SLACK: f64 = 1e-6;

/// Direction along the rover's forward axis. Absent direction means coast, not brake.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorDirection {
    Forward,
    Reverse,
}

/// One wheel after the adapter. `duty` is a fraction of the PWM period, in `0.0..=1.0`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelPwm {
    pub duty: f64,
    /// `None` only while coasting (`duty == 0`). Never a brake request.
    pub direction: Option<MotorDirection>,
}

impl WheelPwm {
    pub const COAST: Self = Self {
        duty: 0.0,
        direction: None,
    };

    pub fn is_coast(self) -> bool {
        self.duty == 0.0 && self.direction.is_none()
    }

    /// Nearest timer count in `0..=max_count`. `max_count` is the PWM period, for example 255.
    pub fn duty_counts(self, max_count: u16) -> u16 {
        quantize_duty(self.duty, max_count)
    }

    /// Sign-magnitude pins. DIR `true` means forward. Coast returns DIR `false` and duty 0;
    /// the DIR level must not be wired to a brake mode while duty is 0.
    pub fn sign_magnitude(self) -> (bool, f64) {
        match self.direction {
            Some(MotorDirection::Forward) => (true, self.duty),
            Some(MotorDirection::Reverse) => (false, self.duty),
            None => (false, 0.0),
        }
    }

    /// IN1/IN2 duties for fast-decay coast. At most one input is nonzero.
    /// Zero effort is `(0, 0)`, never `(1, 1)` (the brake encoding on many bridges).
    pub fn in1_in2(self) -> (f64, f64) {
        match self.direction {
            Some(MotorDirection::Forward) => (self.duty, 0.0),
            Some(MotorDirection::Reverse) => (0.0, self.duty),
            None => (0.0, 0.0),
        }
    }

    /// Integer form of [`Self::in1_in2`]. The two counts are never both nonzero.
    pub fn in1_in2_counts(self, max_count: u16) -> (u16, u16) {
        let (in1, in2) = self.in1_in2();
        (quantize_duty(in1, max_count), quantize_duty(in2, max_count))
    }
}

fn quantize_duty(duty: f64, max_count: u16) -> u16 {
    if max_count == 0 {
        return 0;
    }
    let scaled = (duty.clamp(0.0, 1.0) * f64::from(max_count)).round();
    if scaled <= 0.0 {
        0
    } else if scaled >= f64::from(max_count) {
        max_count
    } else {
        scaled as u16
    }
}

/// Why the adapter is or is not passing effort through to PWM.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OutputHold {
    /// Switch closed and the latched command is inside the watchdog window.
    Live,
    /// Hardware enable switch is open. PWM is forced to coast.
    EnableOpen,
    /// Switch is closed, but no valid command has been latched since it closed or since the last disarm.
    AwaitCommand,
    /// The latched producer timestamp is older than `command_timeout`, or the sample arrived already stale.
    Watchdog,
    /// Effort was non-finite or outside `[-1, 1]`.
    InvalidEffort,
    /// Timestamp was non-finite, negative, ahead of `now`, out of order, or `now` moved backwards.
    InvalidTime,
}

impl std::fmt::Display for OutputHold {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Live => "live",
            Self::EnableOpen => "enable open",
            Self::AwaitCommand => "await command",
            Self::Watchdog => "watchdog",
            Self::InvalidEffort => "invalid effort",
            Self::InvalidTime => "invalid time",
        })
    }
}

/// Both wheels plus the driver-enable gate. This is the only struct that should be written to hardware.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChassisPwm {
    pub left: WheelPwm,
    pub right: WheelPwm,
    /// Logical driver enable. False means deassert EN / hold the driver asleep, and both wheels coast.
    pub drive_enabled: bool,
    pub hold: OutputHold,
}

impl ChassisPwm {
    fn neutral(hold: OutputHold) -> Self {
        Self {
            left: WheelPwm::COAST,
            right: WheelPwm::COAST,
            drive_enabled: false,
            hold,
        }
    }

    pub fn is_coasting(self) -> bool {
        self.left.is_coast() && self.right.is_coast()
    }
}

/// Effort sample tagged with the producer clock, not the adapter's receive time.
///
/// Keep `commanded_at` with the sample if it is queued or retransmitted.
/// Replacing it with the receive time disables the watchdog.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StampedEffort {
    /// Monotonic seconds when the producer computed `left` and `right`.
    pub commanded_at: f64,
    pub left: f64,
    pub right: f64,
}

impl StampedEffort {
    pub fn new(commanded_at: f64, left: f64, right: f64) -> Self {
        Self {
            commanded_at,
            left,
            right,
        }
    }

    pub fn from_output(commanded_at: f64, output: &MotorOutput) -> Self {
        Self::new(commanded_at, output.left, output.right)
    }
}

/// Watchdog and neutral-deadband settings. The watchdog does not read controller timeouts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorAdapterConfig {
    /// Seconds from `commanded_at` until a command is stale. Default 0.2. Maximum 1.0.
    pub command_timeout: f64,
    /// Absolute efforts at or below this value coast. Default 0, so exact zero is the only deadband hit.
    pub neutral_deadband: f64,
}

impl Default for MotorAdapterConfig {
    fn default() -> Self {
        Self {
            command_timeout: 0.2,
            neutral_deadband: 0.0,
        }
    }
}

impl MotorAdapterConfig {
    pub fn validate(self) -> Result<(), InputError> {
        validate_timeout(self.command_timeout)?;
        if !self.neutral_deadband.is_finite()
            || self.neutral_deadband < 0.0
            || self.neutral_deadband >= 1.0
        {
            return Err(InputError::InvalidConfiguration);
        }
        Ok(())
    }
}

fn validate_timeout(timeout: f64) -> Result<(), InputError> {
    if timeout.is_finite() && timeout > 0.0 && timeout <= 1.0 {
        Ok(())
    } else {
        Err(InputError::InvalidConfiguration)
    }
}

/// Map one signed effort to coast or a single-direction PWM command.
pub fn map_effort(effort: f64, deadband: f64) -> Result<WheelPwm, InputError> {
    if !effort.is_finite() || !deadband.is_finite() {
        return Err(InputError::NonFinite);
    }
    if deadband < 0.0 || deadband >= 1.0 || effort.abs() > 1.0 + EFFORT_SLACK {
        return Err(InputError::OutOfRange);
    }
    let effort = effort.clamp(-1.0, 1.0);
    if effort.abs() <= deadband {
        return Ok(WheelPwm::COAST);
    }
    let direction = if effort > 0.0 {
        MotorDirection::Forward
    } else {
        MotorDirection::Reverse
    };
    Ok(WheelPwm {
        duty: effort.abs(),
        direction: Some(direction),
    })
}

/// Result of offering a producer timestamp to the watchdog. The watchdog does not emit PWM.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WatchdogFault {
    InvalidTime,
    /// `now - commanded_at` is already at least the timeout.
    Expired,
    /// `commanded_at` is older than the timestamp already latched.
    OutOfOrder,
}

/// Freshness of the latched producer timestamp. Independent of the velocity-controller target timeout.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WatchdogState {
    Idle,
    Fresh { age: f64, commanded_at: f64 },
    Expired { commanded_at: f64 },
    InvalidTime,
}

/// Motor-command watchdog. It stores the producer timestamp of the last accepted command.
#[derive(Clone, Copy, Debug)]
pub struct MotorWatchdog {
    timeout: f64,
    last_kick: Option<f64>,
}

impl MotorWatchdog {
    pub fn new(timeout: f64) -> Result<Self, InputError> {
        validate_timeout(timeout)?;
        Ok(Self {
            timeout,
            last_kick: None,
        })
    }

    pub fn timeout(self) -> f64 {
        self.timeout
    }

    pub fn last_commanded_at(self) -> Option<f64> {
        self.last_kick
    }

    pub fn clear(&mut self) {
        self.last_kick = None;
    }

    /// Record a producer timestamp that [`Self::check_new`] already accepted.
    pub fn kick(&mut self, commanded_at: f64) {
        self.last_kick = Some(commanded_at);
    }

    /// Check a candidate command without mutating. Equal timestamps are in order; only an older one is not.
    pub fn check_new(self, now: f64, commanded_at: f64) -> Result<(), WatchdogFault> {
        if !now.is_finite()
            || now < 0.0
            || !commanded_at.is_finite()
            || commanded_at < 0.0
            || commanded_at > now
        {
            return Err(WatchdogFault::InvalidTime);
        }
        if self
            .last_kick
            .is_some_and(|previous| commanded_at < previous)
        {
            return Err(WatchdogFault::OutOfOrder);
        }
        if now - commanded_at >= self.timeout {
            return Err(WatchdogFault::Expired);
        }
        Ok(())
    }

    pub fn evaluate(self, now: f64) -> WatchdogState {
        if !now.is_finite() || now < 0.0 {
            return WatchdogState::InvalidTime;
        }
        match self.last_kick {
            None => WatchdogState::Idle,
            Some(commanded_at) if now < commanded_at => WatchdogState::InvalidTime,
            Some(commanded_at) if now - commanded_at >= self.timeout => {
                WatchdogState::Expired { commanded_at }
            }
            Some(commanded_at) => WatchdogState::Fresh {
                age: now - commanded_at,
                commanded_at,
            },
        }
    }
}

/// Software image of the hardware enable switch.
///
/// PWM is permitted only when the switch is closed **and** a command has been latched
/// since the switch last closed. Opening the switch, or the rising edge of closing it,
/// clears the latch so a stored effort cannot start the motors.
#[derive(Clone, Copy, Debug, Default)]
pub struct EnableGate {
    hardware_closed: bool,
    latched: bool,
}

impl EnableGate {
    pub fn hardware_closed(self) -> bool {
        self.hardware_closed
    }

    pub fn latched(self) -> bool {
        self.latched
    }

    /// True only when a command may be turned into PWM.
    pub fn drive_permitted(self) -> bool {
        self.hardware_closed && self.latched
    }

    pub fn set_hardware_closed(&mut self, closed: bool) {
        if !closed || !self.hardware_closed {
            self.latched = false;
        }
        self.hardware_closed = closed;
    }

    pub fn latch_command(&mut self) {
        if self.hardware_closed {
            self.latched = true;
        }
    }

    pub fn disarm(&mut self) {
        self.latched = false;
    }
}

/// Converts signed wheel effort into a chassis PWM command under the enable gate and watchdog.
#[derive(Debug)]
pub struct MotorAdapter {
    config: MotorAdapterConfig,
    watchdog: MotorWatchdog,
    gate: EnableGate,
    last_now: Option<f64>,
    left: f64,
    right: f64,
}

impl MotorAdapter {
    /// Boots with the hardware switch treated as open, so PWM is coast until enable and a command.
    pub fn new(config: MotorAdapterConfig) -> Result<Self, InputError> {
        config.validate()?;
        Ok(Self {
            watchdog: MotorWatchdog::new(config.command_timeout)?,
            config,
            gate: EnableGate::default(),
            last_now: None,
            left: 0.0,
            right: 0.0,
        })
    }

    pub fn config(&self) -> MotorAdapterConfig {
        self.config
    }

    pub fn hardware_enabled(&self) -> bool {
        self.gate.hardware_closed()
    }

    pub fn watchdog(&self) -> MotorWatchdog {
        self.watchdog
    }

    /// Sample the enable switch and return the PWM that must be written this tick.
    ///
    /// A rising or falling edge drops any latched effort. Closing the switch does not
    /// replay the previous command; the next [`Self::apply`] while it stays closed does.
    #[must_use]
    pub fn set_hardware_enable(&mut self, closed: bool, now: f64) -> ChassisPwm {
        let was_closed = self.gate.hardware_closed();
        self.gate.set_hardware_closed(closed);
        if !closed || !was_closed {
            self.watchdog.clear();
            self.left = 0.0;
            self.right = 0.0;
            self.gate.disarm();
        }
        self.poll(now)
    }

    /// Apply an effort that was just computed at `now`. Do not use this to replay a stored command.
    #[must_use]
    pub fn apply_effort_now(&mut self, now: f64, left: f64, right: f64) -> ChassisPwm {
        self.apply(now, StampedEffort::new(now, left, right))
    }

    /// Apply a [`MotorOutput`] stamped when it was produced, which may be earlier than `now`.
    #[must_use]
    pub fn apply_output(
        &mut self,
        now: f64,
        commanded_at: f64,
        output: &MotorOutput,
    ) -> ChassisPwm {
        self.apply(now, StampedEffort::from_output(commanded_at, output))
    }

    /// Apply one stamped command. The returned value is what hardware should write.
    #[must_use]
    pub fn apply(&mut self, now: f64, command: StampedEffort) -> ChassisPwm {
        if self.observe_now(now).is_err() {
            return ChassisPwm::neutral(OutputHold::InvalidTime);
        }
        if map_effort(command.left, self.config.neutral_deadband).is_err()
            || map_effort(command.right, self.config.neutral_deadband).is_err()
        {
            self.idle_clear();
            return ChassisPwm::neutral(OutputHold::InvalidEffort);
        }
        match self.watchdog.check_new(now, command.commanded_at) {
            Err(WatchdogFault::InvalidTime | WatchdogFault::OutOfOrder) => {
                self.idle_clear();
                ChassisPwm::neutral(OutputHold::InvalidTime)
            }
            // The open switch is the reason PWM is off, including when the sample is already stale.
            Err(WatchdogFault::Expired) | Ok(()) if !self.gate.hardware_closed() => {
                self.idle_clear();
                ChassisPwm::neutral(OutputHold::EnableOpen)
            }
            Err(WatchdogFault::Expired) => self.expire(command.commanded_at),
            Ok(()) => {
                self.left = command.left;
                self.right = command.right;
                self.watchdog.kick(command.commanded_at);
                self.gate.latch_command();
                self.emit_live()
            }
        }
    }

    /// Re-evaluate the latched command against `now`. Call this when no new effort arrived.
    #[must_use]
    pub fn poll(&mut self, now: f64) -> ChassisPwm {
        if self.observe_now(now).is_err() {
            return ChassisPwm::neutral(OutputHold::InvalidTime);
        }
        if !self.gate.hardware_closed() {
            self.idle_clear();
            return ChassisPwm::neutral(OutputHold::EnableOpen);
        }
        match self.watchdog.evaluate(now) {
            WatchdogState::InvalidTime => {
                self.idle_clear();
                ChassisPwm::neutral(OutputHold::InvalidTime)
            }
            WatchdogState::Expired { .. } => {
                self.gate.disarm();
                self.left = 0.0;
                self.right = 0.0;
                ChassisPwm::neutral(OutputHold::Watchdog)
            }
            WatchdogState::Idle => ChassisPwm::neutral(OutputHold::AwaitCommand),
            WatchdogState::Fresh { .. } => {
                if self.gate.drive_permitted() {
                    self.emit_live()
                } else {
                    ChassisPwm::neutral(OutputHold::AwaitCommand)
                }
            }
        }
    }

    fn observe_now(&mut self, now: f64) -> Result<(), ()> {
        if !now.is_finite() || now < 0.0 || self.last_now.is_some_and(|last| now < last) {
            self.idle_clear();
            return Err(());
        }
        self.last_now = Some(now);
        Ok(())
    }

    fn idle_clear(&mut self) {
        self.watchdog.clear();
        self.gate.disarm();
        self.left = 0.0;
        self.right = 0.0;
    }

    fn expire(&mut self, commanded_at: f64) -> ChassisPwm {
        self.watchdog.kick(commanded_at);
        self.gate.disarm();
        self.left = 0.0;
        self.right = 0.0;
        ChassisPwm::neutral(OutputHold::Watchdog)
    }

    fn emit_live(&mut self) -> ChassisPwm {
        match (
            map_effort(self.left, self.config.neutral_deadband),
            map_effort(self.right, self.config.neutral_deadband),
        ) {
            (Ok(left), Ok(right)) => ChassisPwm {
                left,
                right,
                drive_enabled: true,
                hold: OutputHold::Live,
            },
            _ => {
                self.idle_clear();
                ChassisPwm::neutral(OutputHold::InvalidEffort)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use terra_types::StopReason;

    fn adapter(timeout: f64) -> MotorAdapter {
        MotorAdapter::new(MotorAdapterConfig {
            command_timeout: timeout,
            neutral_deadband: 0.0,
        })
        .unwrap()
    }

    fn assert_neutral(pwm: ChassisPwm, hold: OutputHold) {
        assert_eq!(pwm.hold, hold, "{hold}");
        assert!(!pwm.drive_enabled);
        assert!(pwm.is_coasting());
        assert_eq!(pwm.left.duty, 0.0);
        assert_eq!(pwm.right.duty, 0.0);
        assert!(pwm.left.direction.is_none());
        assert!(pwm.right.direction.is_none());
        assert_eq!(pwm.left.in1_in2(), (0.0, 0.0));
        assert_eq!(pwm.right.in1_in2(), (0.0, 0.0));
    }

    #[test]
    fn maps_sign_to_direction_and_magnitude_to_duty() {
        let forward = map_effort(0.5, 0.0).unwrap();
        assert_eq!(forward.duty, 0.5);
        assert_eq!(forward.direction, Some(MotorDirection::Forward));
        assert_eq!(forward.sign_magnitude(), (true, 0.5));
        assert_eq!(forward.in1_in2(), (0.5, 0.0));
        assert_eq!(forward.duty_counts(255), 128);
        assert_eq!(forward.in1_in2_counts(255), (128, 0));

        let reverse = map_effort(-0.25, 0.0).unwrap();
        assert_eq!(reverse.duty, 0.25);
        assert_eq!(reverse.direction, Some(MotorDirection::Reverse));
        assert_eq!(reverse.sign_magnitude(), (false, 0.25));
        assert_eq!(reverse.in1_in2(), (0.0, 0.25));
        assert_eq!(reverse.in1_in2_counts(1000), (0, 250));

        let full = map_effort(-1.0, 0.0).unwrap();
        assert_eq!(full.duty_counts(255), 255);
        assert_eq!(map_effort(1.0 + 5e-7, 0.0).unwrap().duty, 1.0);
    }

    #[test]
    fn zero_effort_is_coast_and_never_both_inputs_high() {
        let coast = map_effort(0.0, 0.0).unwrap();
        assert!(coast.is_coast());
        assert_eq!(coast.in1_in2(), (0.0, 0.0));
        assert_eq!(coast.duty_counts(255), 0);
        for effort in [-1.0, -0.5, -0.0, 0.0, 0.25, 1.0] {
            let (in1, in2) = map_effort(effort, 0.0).unwrap().in1_in2();
            assert!(in1 == 0.0 || in2 == 0.0, "effort {effort} would brake");
            assert!(in1 >= 0.0 && in2 >= 0.0);
        }
    }

    #[test]
    fn deadband_and_rejected_efforts() {
        assert!(map_effort(0.25, 0.25).unwrap().is_coast());
        assert_eq!(
            map_effort(0.5, 0.25).unwrap().direction,
            Some(MotorDirection::Forward)
        );
        assert_eq!(map_effort(f64::NAN, 0.0), Err(InputError::NonFinite));
        assert_eq!(map_effort(f64::INFINITY, 0.0), Err(InputError::NonFinite));
        assert_eq!(map_effort(-f64::INFINITY, 0.0), Err(InputError::NonFinite));
        assert_eq!(map_effort(1.0 + 1e-3, 0.0), Err(InputError::OutOfRange));
        assert_eq!(map_effort(-1.1, 0.0), Err(InputError::OutOfRange));
        assert!(
            MotorAdapterConfig {
                command_timeout: 0.0,
                neutral_deadband: 0.0,
            }
            .validate()
            .is_err()
        );
        assert!(
            MotorAdapterConfig {
                command_timeout: 1.1,
                neutral_deadband: 0.0,
            }
            .validate()
            .is_err()
        );
        assert!(MotorAdapterConfig::default().validate().is_ok());
        assert_eq!(MotorAdapterConfig::default().command_timeout, 0.2);
    }

    #[test]
    fn watchdog_expires_on_its_own_clock_and_rejects_replays() {
        let mut watchdog = MotorWatchdog::new(0.25).unwrap();
        assert_eq!(watchdog.evaluate(0.0), WatchdogState::Idle);
        assert_eq!(
            watchdog.check_new(0.0, 0.0),
            Ok(()),
            "a command produced now is fresh"
        );
        watchdog.kick(0.0);
        assert!(matches!(
            watchdog.evaluate(0.125),
            WatchdogState::Fresh {
                commanded_at: 0.0,
                ..
            }
        ));
        assert_eq!(watchdog.check_new(0.25, 0.0), Err(WatchdogFault::Expired));
        assert!(matches!(
            watchdog.evaluate(0.25),
            WatchdogState::Expired { commanded_at: 0.0 }
        ));
        assert_eq!(
            watchdog.check_new(0.125, -0.01),
            Err(WatchdogFault::InvalidTime)
        );
        watchdog.kick(0.125);
        assert_eq!(
            watchdog.check_new(0.25, 0.0),
            Err(WatchdogFault::OutOfOrder)
        );
        assert!(MotorWatchdog::new(0.0).is_err());
        assert_ne!(
            MotorWatchdog::new(0.2).unwrap().timeout(),
            0.5,
            "motor watchdog default is not the controller target timeout"
        );
    }

    #[test]
    fn enable_gate_requires_a_command_after_the_switch_closes() {
        let mut gate = EnableGate::default();
        assert!(!gate.hardware_closed());
        assert!(!gate.drive_permitted());
        gate.latch_command();
        assert!(!gate.latched(), "an open switch cannot latch");
        gate.set_hardware_closed(true);
        assert!(gate.hardware_closed());
        assert!(!gate.drive_permitted());
        gate.latch_command();
        assert!(gate.drive_permitted());
        gate.set_hardware_closed(false);
        assert!(!gate.latched());
        gate.set_hardware_closed(true);
        assert!(!gate.drive_permitted(), "rising edge disarms");
    }

    #[test]
    fn boot_and_open_switch_block_pwm_until_a_fresh_command() {
        let mut motors = adapter(0.25);
        assert_neutral(motors.poll(0.0), OutputHold::EnableOpen);
        assert_neutral(
            motors.apply_effort_now(0.0, 1.0, -1.0),
            OutputHold::EnableOpen,
        );
        assert_neutral(
            motors.set_hardware_enable(true, 0.0),
            OutputHold::AwaitCommand,
        );
        let driving = motors.apply_effort_now(0.0, 0.5, -0.25);
        assert_eq!(driving.hold, OutputHold::Live);
        assert!(driving.drive_enabled);
        assert_eq!(driving.left.duty, 0.5);
        assert_eq!(driving.right.duty, 0.25);
        assert_eq!(driving.left.direction, Some(MotorDirection::Forward));
        assert_eq!(driving.right.direction, Some(MotorDirection::Reverse));

        assert_neutral(
            motors.set_hardware_enable(false, 0.125),
            OutputHold::EnableOpen,
        );
        assert_neutral(
            motors.set_hardware_enable(true, 0.125),
            OutputHold::AwaitCommand,
        );
        assert_neutral(motors.poll(0.125), OutputHold::AwaitCommand);
        assert_neutral(
            motors.set_hardware_enable(false, 0.25),
            OutputHold::EnableOpen,
        );
        assert_neutral(
            motors.apply(0.5, StampedEffort::new(0.0, 1.0, -1.0)),
            OutputHold::EnableOpen,
        );
    }

    #[test]
    fn zero_effort_coasts_with_enable_held_and_watchdog_then_drops_enable() {
        let mut motors = adapter(0.25);
        assert_neutral(
            motors.set_hardware_enable(true, 0.0),
            OutputHold::AwaitCommand,
        );
        let coast = motors.apply(
            0.0,
            StampedEffort::from_output(0.0, &MotorOutput::stopped(StopReason::StaleTarget)),
        );
        assert_eq!(coast.hold, OutputHold::Live);
        assert!(coast.drive_enabled, "fresh zero keeps the driver enabled");
        assert!(coast.is_coasting(), "zero effort coasts; it does not brake");

        let still = motors.apply(0.125, StampedEffort::new(0.0, 0.0, 0.0));
        assert_eq!(still.hold, OutputHold::Live);
        assert_neutral(
            motors.apply(0.25, StampedEffort::new(0.0, 1.0, 1.0)),
            OutputHold::Watchdog,
        );
        assert_neutral(motors.poll(0.25), OutputHold::Watchdog);
        let resumed = motors.apply_effort_now(0.25, 0.25, 0.0);
        assert_eq!(resumed.hold, OutputHold::Live);
        assert!(resumed.drive_enabled);
        assert_eq!(resumed.left.direction, Some(MotorDirection::Forward));
        assert!(resumed.right.is_coast());
    }

    #[test]
    fn same_stamp_does_not_extend_the_watchdog() {
        let mut motors = adapter(0.25);
        assert_neutral(
            motors.set_hardware_enable(true, 0.0),
            OutputHold::AwaitCommand,
        );
        assert_eq!(
            motors.apply(0.0, StampedEffort::new(0.0, 1.0, 1.0)).hold,
            OutputHold::Live
        );
        assert_eq!(
            motors.apply(0.2, StampedEffort::new(0.0, 1.0, 1.0)).hold,
            OutputHold::Live
        );
        assert_neutral(motors.poll(0.25), OutputHold::Watchdog);
        assert_neutral(
            motors.apply(0.26, StampedEffort::new(0.0, 1.0, 1.0)),
            OutputHold::Watchdog,
        );
    }

    #[test]
    fn invalid_effort_or_clock_drops_both_wheels() {
        let mut motors = adapter(0.25);
        assert_neutral(
            motors.set_hardware_enable(true, 0.0),
            OutputHold::AwaitCommand,
        );
        assert_eq!(
            motors.apply_effort_now(0.0, 0.5, -0.5).hold,
            OutputHold::Live
        );
        assert_neutral(
            motors.apply_effort_now(0.125, f64::NAN, 0.2),
            OutputHold::InvalidEffort,
        );
        assert_neutral(motors.poll(0.125), OutputHold::AwaitCommand);
        assert_eq!(
            motors.apply_effort_now(0.25, -0.5, 0.5).hold,
            OutputHold::Live
        );
        assert_neutral(motors.poll(0.125), OutputHold::InvalidTime);
        assert_neutral(motors.poll(0.25), OutputHold::AwaitCommand);
        assert_neutral(
            motors.apply(0.375, StampedEffort::new(0.5, 0.2, 0.2)),
            OutputHold::InvalidTime,
        );
        assert_neutral(
            motors.apply_effort_now(0.375, 1.4, 0.0),
            OutputHold::InvalidEffort,
        );
    }

    #[test]
    fn older_timestamp_disarms_instead_of_replacing_a_newer_command() {
        let mut motors = adapter(0.25);
        assert_neutral(
            motors.set_hardware_enable(true, 0.0),
            OutputHold::AwaitCommand,
        );
        assert_eq!(
            motors.apply(0.2, StampedEffort::new(0.2, 0.4, 0.4)).hold,
            OutputHold::Live
        );
        assert_neutral(
            motors.apply(0.2, StampedEffort::new(0.1, 1.0, 1.0)),
            OutputHold::InvalidTime,
        );
        assert_neutral(motors.poll(0.2), OutputHold::AwaitCommand);
    }

    #[test]
    fn deadband_coasts_without_opening_the_enable_gate() {
        let mut motors = MotorAdapter::new(MotorAdapterConfig {
            command_timeout: 0.25,
            neutral_deadband: 0.25,
        })
        .unwrap();
        assert_neutral(
            motors.set_hardware_enable(true, 0.0),
            OutputHold::AwaitCommand,
        );
        let pwm = motors.apply_effort_now(0.0, 0.25, -0.5);
        assert!(pwm.drive_enabled);
        assert!(pwm.left.is_coast());
        assert_eq!(pwm.right.duty, 0.5);
        assert_eq!(pwm.right.direction, Some(MotorDirection::Reverse));
    }

    /// Software stand-in for the wheeled-up bench procedure. It does not spin hardware.
    #[test]
    fn software_bench_sequence() {
        let mut motors = adapter(0.25);

        assert_neutral(
            motors.apply_effort_now(0.0, 1.0, -1.0),
            OutputHold::EnableOpen,
        );

        assert_neutral(
            motors.set_hardware_enable(true, 0.0),
            OutputHold::AwaitCommand,
        );

        let spin = motors.apply_effort_now(0.0, 0.5, -0.25);
        assert_eq!(spin.left.in1_in2(), (0.5, 0.0));
        assert_eq!(spin.right.in1_in2(), (0.0, 0.25));
        assert!(spin.drive_enabled);

        let zeros = motors.apply_effort_now(0.125, 0.0, 0.0);
        assert_eq!(zeros.hold, OutputHold::Live);
        assert!(zeros.drive_enabled);
        assert!(zeros.is_coasting());

        assert_neutral(motors.poll(0.375), OutputHold::Watchdog);
        assert_neutral(motors.poll(0.5), OutputHold::Watchdog);

        let again = motors.apply_effort_now(0.5, 0.25, 0.25);
        assert_eq!(again.hold, OutputHold::Live);
        assert_eq!(again.left.direction, Some(MotorDirection::Forward));
        assert_eq!(again.right.direction, Some(MotorDirection::Forward));

        assert_neutral(
            motors.set_hardware_enable(false, 0.5),
            OutputHold::EnableOpen,
        );
        assert_neutral(
            motors.apply_effort_now(0.5, 1.0, 1.0),
            OutputHold::EnableOpen,
        );
    }
}
