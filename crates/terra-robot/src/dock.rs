//! Dock outlet and gate servo: the rover profile's values with the phone's
//! local settings on top, plus the setup-screen gate test.
use super::{Runtime, RuntimeError, invalid, rover_profile};
use rover_profile::{GATE_POSITION, OUTLET_AHEAD_M, within};

/// Dock values set on the phone, kept across rover profile changes.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct DockLocal {
    outlet_ahead: Option<f64>,
    gate: Option<(u8, f64, f64)>,
}

impl Runtime {
    /// The dock's gate request this tick as a servo position, when a gate
    /// servo is set: `(actuator id, normalized position)`. With no dock
    /// running, a gate test from the setup screen holds the gate where it put it.
    pub fn gate_servo(&self, gate_open: bool) -> Option<(u8, f64)> {
        let open = gate_open || (!self.autonomy.dock_active() && self.gate_test == Some(true));
        let p = &self.dock_profile;
        p.gate_servo
            .map(|id| (id, if open { p.gate_open } else { p.gate_closed }))
    }

    /// Outlet distance ahead of the rover centre set on the phone, metres, in
    /// place of the profile's. `None` returns to the profile's value.
    ///
    /// # Errors
    /// Outside 0.05..=3 m; the current value stays.
    pub fn set_dock_outlet(&mut self, ahead: Option<f64>) -> Result<(), RuntimeError> {
        if ahead.is_some_and(|v| !within(v, &OUTLET_AHEAD_M)) {
            return Err(invalid("dock outlet must be 0.05 to 3 m ahead"));
        }
        self.dock_local.outlet_ahead = ahead;
        self.apply_dock();
        Ok(())
    }

    /// Gate servo and its open and closed positions set on the phone, in place
    /// of the profile's. `None` returns to the profile's servo.
    ///
    /// # Errors
    /// Positions outside -1..=1 or equal; the current gate stays.
    pub fn set_dock_gate(&mut self, gate: Option<(u8, f64, f64)>) -> Result<(), RuntimeError> {
        if let Some((_, open, closed)) = gate
            && !(within(open, &GATE_POSITION) && within(closed, &GATE_POSITION) && open != closed)
        {
            return Err(invalid("gate positions must differ and lie in -1..=1"));
        }
        self.dock_local.gate = gate;
        self.apply_dock();
        Ok(())
    }

    /// Hold the gate open (`Some(true)`) or closed to try its positions, or end
    /// the test (`None`). A running dock always controls the gate itself.
    ///
    /// # Errors
    /// While a dock runs, or without a gate servo.
    pub fn test_dock_gate(&mut self, open: Option<bool>) -> Result<(), RuntimeError> {
        if open.is_some() && self.autonomy.dock_active() {
            return Err(invalid("a dock is running"));
        }
        if open.is_some() && self.dock_profile.gate_servo.is_none() {
            return Err(invalid("no gate servo set"));
        }
        self.gate_test = open;
        Ok(())
    }

    pub(crate) fn apply_dock(&mut self) {
        let mut profile = self
            .rover_profile
            .clone()
            .unwrap_or_else(rover_profile::RoverProfile::terra);
        if let Some(ahead) = self.dock_local.outlet_ahead {
            profile.dock.outlet_ahead = Some(ahead);
        }
        if let Some((id, open, closed)) = self.dock_local.gate {
            profile.dock.gate_servo = Some(id);
            profile.dock.gate_open = open;
            profile.dock.gate_closed = closed;
        }
        if self.rover_profile.is_some() || self.dock_local != DockLocal::default() {
            self.autonomy.configure_dock(profile.dock_config());
        }
        self.dock_profile = profile.dock;
    }
}
