//! The controllers and path planners that can be picked by name: from
//! `terra-av.toml`, or at run time in a task from the base station.
//!
//! Each entry says what it is (`id`, `version`, `description`), which
//! parameters it takes (so the base station can draw a form), and how to
//! build it from checked parameter values. The same list is published to
//! the base station as `Capabilities`.
//!
//! To add a controller: write `planning/local/my_controller.rs` with
//! `impl LocalPlanner` and a `spec()`, then add it to [`controllers`].
use terra_comms::{GoalKind, ParamKind, ParamSpec, ParamValue, Params, StageOption, validate};

use super::global::{HybridAStar, PathPlanner, StraightLine};
use super::local::{DrivePose, Dwa, LocalPlanner, Remote, Teleop};

/// Every controller this build can run.
pub fn controllers() -> Vec<ControllerSpec> {
    vec![Teleop::spec(), DrivePose::spec(), Dwa::spec(), Remote::spec()]
}

/// Every path planner this build can run.
pub fn planners() -> Vec<PlannerSpec> {
    vec![StraightLine::spec(), HybridAStar::spec()]
}

/// The controller with this id, or why not (shown to the operator).
pub fn controller(id: &str) -> Result<ControllerSpec, String> {
    controllers()
        .into_iter()
        .find(|spec| spec.id == id)
        .ok_or_else(|| format!("unknown controller {id:?}"))
}

/// The path planner with this id, or why not (shown to the operator).
pub fn planner(id: &str) -> Result<PlannerSpec, String> {
    planners()
        .into_iter()
        .find(|spec| spec.id == id)
        .ok_or_else(|| format!("unknown planner {id:?}"))
}

/// One selectable controller (a [`LocalPlanner`]).
pub struct ControllerSpec {
    pub id: &'static str,
    /// Bumped when the controller's behaviour changes, so a logged run can
    /// be matched to the code that drove it.
    pub version: &'static str,
    pub description: &'static str,
    /// Goal kinds it drives to. Empty: it takes no goal (a stick).
    pub goals: Vec<GoalKind>,
    pub params: Vec<ParamSpec>,
    /// Build it. Only ever called with values that passed `validate`
    /// against `params`, so every parameter is present and in range.
    pub make: fn(&Params) -> Box<dyn LocalPlanner>,
}

/// One selectable path planner.
pub struct PlannerSpec {
    pub id: &'static str,
    pub version: &'static str,
    pub description: &'static str,
    pub params: Vec<ParamSpec>,
    /// Build it; same contract as [`ControllerSpec::make`].
    pub make: fn(&Params) -> Box<dyn PathPlanner>,
}

impl ControllerSpec {
    /// Check `params` (unknown name, wrong type, out of range), fill in the
    /// defaults, and build the controller. Returns the values it was built
    /// with.
    pub fn build(&self, params: &Params) -> Result<(Params, Box<dyn LocalPlanner>), String> {
        let checked = validate(&self.params, params).map_err(|e| format!("{}: {e}", self.id))?;
        let controller = (self.make)(&checked);
        Ok((checked, controller))
    }

    /// How this controller is described to the base station.
    pub fn option(&self) -> StageOption {
        StageOption {
            id: self.id.into(),
            version: self.version.into(),
            description: self.description.into(),
            params: self.params.clone(),
            goals: self.goals.clone(),
        }
    }
}

impl PlannerSpec {
    /// Like [`ControllerSpec::build`].
    pub fn build(&self, params: &Params) -> Result<(Params, Box<dyn PathPlanner>), String> {
        let checked = validate(&self.params, params).map_err(|e| format!("{}: {e}", self.id))?;
        let planner = (self.make)(&checked);
        Ok((checked, planner))
    }

    /// How this planner is described to the base station.
    pub fn option(&self) -> StageOption {
        StageOption {
            id: self.id.into(),
            version: self.version.into(),
            description: self.description.into(),
            params: self.params.clone(),
            goals: vec![GoalKind::Local],
        }
    }
}

// ------------------------------------------------- describing parameters

/// A decimal parameter, allowed in `min..=max`.
pub fn float_param(name: &str, default: f64, min: f64, max: f64, unit: &str, help: &str) -> ParamSpec {
    ParamSpec {
        name: name.into(),
        kind: ParamKind::Float,
        default: ParamValue::Float(default),
        min: Some(min),
        max: Some(max),
        unit: (!unit.is_empty()).then(|| unit.into()),
        help: help.into(),
    }
}

/// A whole-number parameter, allowed in `min..=max`.
pub fn int_param(name: &str, default: i64, min: i64, max: i64, help: &str) -> ParamSpec {
    ParamSpec {
        name: name.into(),
        kind: ParamKind::Int,
        default: ParamValue::Int(default),
        min: Some(min as f64),
        max: Some(max as f64),
        unit: None,
        help: help.into(),
    }
}

/// A boolean parameter.
pub fn bool_param(name: &str, default: bool, help: &str) -> ParamSpec {
    ParamSpec {
        name: name.into(),
        kind: ParamKind::Bool,
        default: ParamValue::Bool(default),
        min: None,
        max: None,
        unit: None,
        help: help.into(),
    }
}

// ---------------------------------------------- reading checked parameters

/// Read a decimal parameter from values that passed `validate`.
///
/// Panics if `name` is not a float in `params`. After `validate` that can
/// only mean the `make` function asks for a name its own `params` list
/// does not have: a bug that shows on the first build, at startup.
pub fn float(params: &Params, name: &str) -> f64 {
    match params.get(name) {
        Some(ParamValue::Float(value)) => *value,
        other => panic!("parameter {name:?} should be a checked float, found {other:?}"),
    }
}

/// Read a whole-number parameter; see [`float`].
pub fn int(params: &Params, name: &str) -> i64 {
    match params.get(name) {
        Some(ParamValue::Int(value)) => *value,
        other => panic!("parameter {name:?} should be a checked int, found {other:?}"),
    }
}

/// Read a boolean parameter; see [`float`].
pub fn boolean(params: &Params, name: &str) -> bool {
    match params.get(name) {
        Some(ParamValue::Bool(value)) => *value,
        other => panic!("parameter {name:?} should be a checked bool, found {other:?}"),
    }
}
