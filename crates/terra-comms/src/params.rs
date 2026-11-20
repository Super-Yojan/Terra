//! Controller parameters. The rover describes them with [`ParamSpec`] so the
//! UI can draw a form without knowing the controller; both sides check
//! values with the same [`validate`].
use std::collections::BTreeMap;
use std::fmt;

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ParamSpec {
    pub name: String,
    pub kind: ParamKind,
    pub default: ParamValue,
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub unit: Option<String>,
    pub help: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParamKind {
    Float,
    Int,
    Bool,
    /// One of these options.
    Choice(Vec<String>),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParamValue {
    Float(f64),
    Int(i64),
    Bool(bool),
    Choice(String),
}

/// Parameter values by name. A missing name means "use the default".
pub type Params = BTreeMap<String, ParamValue>;

/// Check `params` against `specs` (unknown name, wrong kind, out of range,
/// not one of the choices, not finite) and return them with defaults filled
/// in for the missing ones. An `Int` given for a `Float` parameter is
/// accepted and returned as a `Float`.
///
/// The specs themselves are checked first (duplicate names, a default that
/// doesn't fit its own spec, `min > max`), so a broken controller
/// description is reported as [`ParamError::BadSpec`] instead of producing
/// values the rover would then refuse.
pub fn validate(specs: &[ParamSpec], params: &Params) -> Result<Params, ParamError> {
    check_specs(specs)?;
    let mut out = Params::new();
    for (name, value) in params {
        let spec = specs
            .iter()
            .find(|s| &s.name == name)
            .ok_or_else(|| ParamError::Unknown { name: name.clone() })?;
        out.insert(name.clone(), check(spec, value)?);
    }
    for spec in specs {
        out.entry(spec.name.clone()).or_insert_with(|| spec.default.clone());
    }
    Ok(out)
}

/// Check a controller's parameter descriptions on their own.
pub fn check_specs(specs: &[ParamSpec]) -> Result<(), ParamError> {
    for (i, spec) in specs.iter().enumerate() {
        let bad = |reason: String| ParamError::BadSpec { name: spec.name.clone(), reason };
        if spec.name.is_empty() {
            return Err(bad("empty name".into()));
        }
        if specs[..i].iter().any(|s| s.name == spec.name) {
            return Err(bad("duplicate name".into()));
        }
        if let (Some(min), Some(max)) = (spec.min, spec.max)
            && min.partial_cmp(&max).is_none_or(|o| o.is_gt())
        {
            return Err(bad(format!("min {min} > max {max} (or NaN)")));
        }
        if matches!(&spec.kind, ParamKind::Choice(options) if options.is_empty()) {
            return Err(bad("no choices".into()));
        }
        // The default must be stored exactly as its kind (no Int-for-Float).
        let exact = matches!(
            (&spec.kind, &spec.default),
            (ParamKind::Float, ParamValue::Float(_))
                | (ParamKind::Int, ParamValue::Int(_))
                | (ParamKind::Bool, ParamValue::Bool(_))
                | (ParamKind::Choice(_), ParamValue::Choice(_))
        );
        if !exact {
            return Err(bad(format!("default is not a {}", kind_name(&spec.kind))));
        }
        check(spec, &spec.default).map_err(|e| bad(format!("default: {e}")))?;
    }
    Ok(())
}

/// Check one value, returning it as stored (an `Int` for a `Float`
/// parameter becomes a `Float`).
fn check(spec: &ParamSpec, value: &ParamValue) -> Result<ParamValue, ParamError> {
    let name = || spec.name.clone();
    let (number, stored) = match (&spec.kind, value) {
        (ParamKind::Float, ParamValue::Int(i)) => (*i as f64, ParamValue::Float(*i as f64)),
        (ParamKind::Float, ParamValue::Float(x)) => {
            if !x.is_finite() {
                return Err(ParamError::NotFinite { name: name() });
            }
            (*x, value.clone())
        }
        (ParamKind::Int, ParamValue::Int(i)) => (*i as f64, value.clone()),
        (ParamKind::Bool, ParamValue::Bool(_)) => return Ok(value.clone()),
        (ParamKind::Choice(options), ParamValue::Choice(c)) => {
            return if options.contains(c) {
                Ok(value.clone())
            } else {
                Err(ParamError::NotAChoice { name: name(), value: c.clone(), options: options.clone() })
            };
        }
        (kind, _) => return Err(ParamError::WrongKind { name: name(), expected: kind_name(kind) }),
    };
    let below = spec.min.is_some_and(|min| number < min);
    let above = spec.max.is_some_and(|max| number > max);
    if below || above {
        return Err(ParamError::OutOfRange { name: name(), value: number, min: spec.min, max: spec.max });
    }
    Ok(stored)
}

fn kind_name(kind: &ParamKind) -> &'static str {
    match kind {
        ParamKind::Float => "float",
        ParamKind::Int => "int",
        ParamKind::Bool => "bool",
        ParamKind::Choice(_) => "choice",
    }
}

/// Why a parameter was refused. Shown next to the form field [`Self::name`].
#[derive(Clone, Debug, PartialEq)]
pub enum ParamError {
    Unknown { name: String },
    WrongKind { name: String, expected: &'static str },
    OutOfRange { name: String, value: f64, min: Option<f64>, max: Option<f64> },
    NotFinite { name: String },
    NotAChoice { name: String, value: String, options: Vec<String> },
    /// The spec itself is broken (a controller bug, not operator input).
    BadSpec { name: String, reason: String },
}

impl ParamError {
    pub fn name(&self) -> &str {
        match self {
            Self::Unknown { name }
            | Self::WrongKind { name, .. }
            | Self::OutOfRange { name, .. }
            | Self::NotFinite { name }
            | Self::NotAChoice { name, .. }
            | Self::BadSpec { name, .. } => name,
        }
    }
}

impl fmt::Display for ParamError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Unknown { name } => write!(f, "{name}: unknown parameter"),
            Self::WrongKind { name, expected } => write!(f, "{name}: expected a {expected}"),
            Self::OutOfRange { name, value, min, max } => {
                write!(f, "{name}: {value} is out of range")?;
                match (min, max) {
                    (Some(lo), Some(hi)) => write!(f, " [{lo}, {hi}]"),
                    (Some(lo), None) => write!(f, " (min {lo})"),
                    (None, Some(hi)) => write!(f, " (max {hi})"),
                    (None, None) => Ok(()),
                }
            }
            Self::NotFinite { name } => write!(f, "{name}: must be a finite number"),
            Self::NotAChoice { name, value, options } => {
                write!(f, "{name}: {value:?} is not one of {}", options.join(", "))
            }
            Self::BadSpec { name, reason } => write!(f, "{name}: bad parameter spec: {reason}"),
        }
    }
}

impl std::error::Error for ParamError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn specs() -> Vec<ParamSpec> {
        let spec = |name: &str, kind, default, min, max| ParamSpec {
            name: name.into(),
            kind,
            default,
            min,
            max,
            unit: None,
            help: String::new(),
        };
        vec![
            spec("horizon", ParamKind::Float, ParamValue::Float(2.0), Some(0.1), Some(10.0)),
            spec("samples", ParamKind::Int, ParamValue::Int(20), Some(1.0), None),
            spec("reverse", ParamKind::Bool, ParamValue::Bool(false), None, None),
            spec(
                "mode",
                ParamKind::Choice(vec!["fast".into(), "safe".into()]),
                ParamValue::Choice("safe".into()),
                None,
                None,
            ),
        ]
    }

    fn params(items: &[(&str, ParamValue)]) -> Params {
        items.iter().map(|(k, v)| (k.to_string(), v.clone())).collect()
    }

    #[test]
    fn fills_defaults() {
        let out = validate(&specs(), &params(&[("horizon", ParamValue::Float(3.0))])).unwrap();
        assert_eq!(out["horizon"], ParamValue::Float(3.0));
        assert_eq!(out["samples"], ParamValue::Int(20));
        assert_eq!(out["mode"], ParamValue::Choice("safe".into()));
        assert_eq!(out.len(), 4);
    }

    #[test]
    fn rejects() {
        let cases = [
            ("nope", ParamValue::Bool(true)),
            ("horizon", ParamValue::Bool(true)),
            ("samples", ParamValue::Float(3.0)),
            ("horizon", ParamValue::Int(30)),
            ("horizon", ParamValue::Float(20.0)),
            ("horizon", ParamValue::Float(f64::NAN)),
            ("samples", ParamValue::Int(0)),
            ("mode", ParamValue::Choice("yolo".into())),
        ];
        for (name, value) in cases {
            let err = validate(&specs(), &params(&[(name, value.clone())])).unwrap_err();
            assert_eq!(err.name(), name, "{value:?}");
        }
    }

    #[test]
    fn int_for_float_becomes_float() {
        let out = validate(&specs(), &params(&[("horizon", ParamValue::Int(3))])).unwrap();
        assert_eq!(out["horizon"], ParamValue::Float(3.0));
    }

    #[test]
    fn rejects_bad_specs() {
        let mut dup = specs();
        dup.push(dup[0].clone());
        let mut bad_default = specs();
        bad_default[0].default = ParamValue::Float(99.0);
        let mut wrong_kind = specs();
        wrong_kind[0].default = ParamValue::Int(2);
        let mut bad_choice = specs();
        bad_choice[3].default = ParamValue::Choice("nope".into());
        let mut inverted = specs();
        inverted[0].min = Some(5.0);
        inverted[0].max = Some(1.0);
        for specs in [dup, bad_default, wrong_kind, bad_choice, inverted] {
            let err = validate(&specs, &Params::new()).unwrap_err();
            assert!(matches!(err, ParamError::BadSpec { .. }), "{err}");
        }
    }
}
