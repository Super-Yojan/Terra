//! Phone GPS localization report shared by both apps.
//!
//! Ports `PhoneLocalizationPolicy` from the iOS app. The app passes its latest
//! location fix and tracking facts once a second; Rust labels freshness, lists
//! the issues and builds the JSON that `MobileController::publish_localization`
//! sends. Raw coordinates are observations only: no north/local transform is
//! inferred, and freshness is a label, never a reason to drop coordinates.
use crate::ControllerError;
use serde::Serialize;
use std::collections::hash_map::RandomState;
use std::hash::{BuildHasher, Hasher};
use std::sync::{Arc, Mutex};

/// Latest platform observations. Times are seconds on one clock (iOS uses wall time).
#[derive(Clone, Debug, Default, PartialEq, uniffi::Record)]
pub struct LocalizationInput {
    /// Current time.
    pub now: f64,
    /// Fix latitude, degrees.
    pub latitude: Option<f64>,
    /// Fix longitude, degrees.
    pub longitude: Option<f64>,
    /// Fix time; `None` before the first fix.
    pub fix_time: Option<f64>,
    /// Horizontal accuracy, metres.
    pub accuracy: Option<f64>,
    /// Compass heading accuracy, degrees.
    pub heading_accuracy: Option<f64>,
    /// AR tracking is normal and the pose is fresh.
    pub tracked: bool,
    /// Location permission or service problem to show, if any.
    pub permission_reason: Option<String>,
}

/// Report published on the localization topic. Field names match the iOS JSON.
#[derive(Clone, Debug, PartialEq, Serialize, uniffi::Record)]
#[serde(rename_all = "camelCase")]
pub struct LocalizationReport {
    /// Schema version, 1.
    pub version: u32,
    /// Local AR frame identifier; changes when tracking restarts.
    #[serde(rename = "frameID")]
    pub frame_id: String,
    /// `gps` while a valid fix is fresh, otherwise `local`.
    pub mode: String,
    /// One line for the operator.
    pub reason: String,
    /// `normal` or `limited`.
    pub tracking: String,
    /// Every problem found.
    pub issues: Vec<String>,
    /// Finite reported accuracy, metres.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gps_accuracy: Option<f64>,
    /// Latitude of a valid measurement at any age.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gps_latitude: Option<f64>,
    /// Longitude of a valid measurement at any age.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gps_longitude: Option<f64>,
    /// Seconds since the fix (never negative).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gps_fix_age: Option<f64>,
    /// Original fix time of a valid measurement.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gps_observed_at: Option<f64>,
    /// The fix is within the display freshness window.
    pub gps_fresh: bool,
    /// Finite compass accuracy, degrees.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heading_accuracy: Option<f64>,
}

#[uniffi::export]
/// The JSON for `MobileController::publish_localization`.
pub fn localization_report_json(report: LocalizationReport) -> String {
    serde_json::to_string(&report).unwrap_or_default()
}

struct Localizer {
    freshness: f64,
    frame_id: String,
}

/// GPS labelling for one phone session. Calls are serialized by a mutex.
#[derive(uniffi::Object)]
pub struct PhoneLocalizer {
    state: Mutex<Localizer>,
}

#[uniffi::export]
impl PhoneLocalizer {
    #[uniffi::constructor]
    /// `display_freshness_seconds` must be finite and positive.
    pub fn new(display_freshness_seconds: f64) -> Result<Arc<Self>, ControllerError> {
        if !display_freshness_seconds.is_finite() || display_freshness_seconds <= 0.0 {
            return Err(ControllerError::InvalidInput {
                message: "GPS display freshness must be positive".into(),
            });
        }
        Ok(Arc::new(Self {
            state: Mutex::new(Localizer {
                freshness: display_freshness_seconds,
                frame_id: new_frame_id(),
            }),
        }))
    }

    /// Change the freshness window; invalid values are ignored.
    pub fn set_display_freshness(&self, seconds: f64) -> Result<(), ControllerError> {
        if seconds.is_finite() && seconds > 0.0 {
            self.lock()?.freshness = seconds;
        }
        Ok(())
    }

    /// Read `gps_display_freshness_seconds` from
    /// `MobileController::connectivity_policy_settings` JSON; other fields are ignored.
    pub fn apply_connectivity_policy(&self, json: String) -> Result<(), ControllerError> {
        let value: serde_json::Value = serde_json::from_str(&json).unwrap_or_default();
        match value["gps_display_freshness_seconds"].as_f64() {
            Some(seconds) => self.set_display_freshness(seconds),
            None => Ok(()),
        }
    }

    /// The local frame identifier.
    pub fn identifier(&self) -> Result<String, ControllerError> {
        Ok(self.lock()?.frame_id.clone())
    }

    /// Tracking restarted: a new local frame.
    pub fn reset(&self) -> Result<(), ControllerError> {
        self.lock()?.frame_id = new_frame_id();
        Ok(())
    }

    /// Label the latest observations.
    pub fn update(&self, input: LocalizationInput) -> Result<LocalizationReport, ControllerError> {
        let state = self.lock()?;
        Ok(report(&state, &input))
    }
}

impl PhoneLocalizer {
    fn lock(&self) -> Result<std::sync::MutexGuard<'_, Localizer>, ControllerError> {
        self.state.lock().map_err(|_| ControllerError::Internal)
    }
}

fn report(state: &Localizer, input: &LocalizationInput) -> LocalizationReport {
    let fix_age = input.fix_time.and_then(|t| {
        let age = input.now - t;
        (age.is_finite() && age >= -1.0).then(|| age.max(0.0))
    });
    let fresh = fix_age.is_some_and(|age| age <= state.freshness);
    let accuracy_valid = input.accuracy.is_some_and(|a| a.is_finite() && a >= 0.0);
    let coordinates_valid = input
        .latitude
        .is_some_and(|v| v.is_finite() && v.abs() <= 90.0)
        && input
            .longitude
            .is_some_and(|v| v.is_finite() && v.abs() <= 180.0);
    let has_fix = input.fix_time.is_some();
    let mut issues = Vec::new();
    if let Some(permission) = &input.permission_reason {
        issues.push(permission.clone());
    }
    if !has_fix {
        issues
            .push("Waiting for GPS fix · move to an open area with a clear view of the sky".into());
    } else if !fresh {
        issues.push(format!(
            "GPS fix stale · last known position · display freshness {:.1} s",
            state.freshness
        ));
    }
    if has_fix && !accuracy_valid {
        issues.push("GPS accuracy unavailable · waiting for a valid location measurement".into());
    }
    if has_fix && !coordinates_valid {
        issues.push(
            "GPS coordinates unavailable or outside the supported latitude/longitude range".into(),
        );
    }
    let measured = input.permission_reason.is_none()
        && fix_age.is_some()
        && accuracy_valid
        && coordinates_valid;
    let available = measured && fresh;
    let accuracy = input.accuracy.filter(|a| a.is_finite());
    let reason = match (available, accuracy, fix_age) {
        (true, Some(a), Some(age)) => {
            format!("GPS available · accuracy {a:.1} m · fix {age:.1} s old")
        }
        _ => issues.join("; "),
    };
    LocalizationReport {
        version: 1,
        frame_id: state.frame_id.clone(),
        mode: if available { "gps" } else { "local" }.into(),
        reason,
        tracking: if input.tracked { "normal" } else { "limited" }.into(),
        issues,
        gps_accuracy: accuracy,
        gps_latitude: input.latitude.filter(|_| measured),
        gps_longitude: input.longitude.filter(|_| measured),
        gps_fix_age: fix_age,
        gps_observed_at: input.fix_time.filter(|_| measured),
        gps_fresh: available,
        heading_accuracy: input.heading_accuracy.filter(|a| a.is_finite()),
    }
}

/// A random UUID-shaped identifier (uppercase, like iOS `UUID().uuidString`).
fn new_frame_id() -> String {
    let mut words = [0u64; 2];
    for (i, word) in words.iter_mut().enumerate() {
        let mut hasher = RandomState::new().build_hasher();
        hasher.write_usize(i);
        hasher.write_u128(
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos())
                .unwrap_or_default(),
        );
        *word = hasher.finish();
    }
    let hex = format!("{:016X}{:016X}", words[0], words[1]);
    format!(
        "{}-{}-4{}-{}-{}",
        &hex[0..8],
        &hex[8..12],
        &hex[13..16],
        &hex[16..20],
        &hex[20..32]
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input(now: f64) -> LocalizationInput {
        LocalizationInput {
            now,
            latitude: Some(38.0),
            longitude: Some(-77.0),
            fix_time: Some(now),
            accuracy: Some(4.0),
            ..Default::default()
        }
    }

    #[test]
    /// Ports PhoneLocalizationTests: immediate raw GPS, freshness labels, invalid fixes, restarts.
    fn raw_gps_labels() {
        let policy = PhoneLocalizer::new(3.0).unwrap();
        let fix = policy.update(input(0.0)).unwrap();
        assert_eq!(fix.mode, "gps");
        assert!(fix.issues.is_empty());
        assert_eq!(
            (fix.gps_latitude, fix.gps_longitude),
            (Some(38.0), Some(-77.0))
        );
        let coarse = LocalizationInput {
            accuracy: Some(45.0),
            ..input(1.0)
        };
        let r = policy.update(coarse).unwrap();
        assert!(r.mode == "gps" && r.reason.contains("45"));
        let stale = LocalizationInput {
            fix_time: Some(6.0),
            ..input(10.0)
        };
        let old = policy.update(stale.clone()).unwrap();
        assert!(
            old.mode == "local" && old.gps_latitude == Some(38.0) && old.reason.contains("stale")
        );
        for age in [0.0, 3.0, 8.2, 120.0] {
            let r = policy
                .update(LocalizationInput {
                    fix_time: Some(200.0 - age),
                    ..input(200.0)
                })
                .unwrap();
            assert_eq!(r.gps_latitude, Some(38.0));
            assert_eq!(r.gps_observed_at, Some(200.0 - age));
            assert_eq!(r.gps_fresh, age <= 3.0);
        }
        let relaxed = PhoneLocalizer::new(10.0).unwrap();
        assert!(relaxed.update(stale.clone()).unwrap().gps_fresh);
        relaxed
            .apply_connectivity_policy(r#"{"gps_display_freshness_seconds":3}"#.into())
            .unwrap();
        assert!(!relaxed.update(stale).unwrap().gps_fresh);
        let denied = LocalizationInput {
            permission_reason: Some("Location permission denied".into()),
            ..input(11.0)
        };
        let r = policy.update(denied).unwrap();
        assert!(r.reason.contains("permission denied") && r.gps_latitude.is_none());
        for value in [-1.0, f64::NAN, f64::INFINITY] {
            let r = policy
                .update(LocalizationInput {
                    accuracy: Some(value),
                    ..input(12.0)
                })
                .unwrap();
            assert!(r.mode == "local" && r.gps_latitude.is_none());
            assert!(localization_report_json(r).starts_with('{'));
        }
        let bad = LocalizationInput {
            latitude: Some(f64::NAN),
            ..input(13.0)
        };
        assert!(policy.update(bad).unwrap().gps_latitude.is_none());
        let missing = LocalizationInput {
            fix_time: None,
            ..input(14.0)
        };
        assert!(
            policy
                .update(missing)
                .unwrap()
                .reason
                .contains("Waiting for GPS")
        );
        let id = policy.identifier().unwrap();
        policy.reset().unwrap();
        assert_ne!(policy.identifier().unwrap(), id);
        assert_eq!(policy.update(input(15.0)).unwrap().mode, "gps");
        assert!(PhoneLocalizer::new(0.0).is_err());
    }

    #[test]
    /// The JSON keeps the iOS keys and omits missing values.
    fn report_json_keys() {
        let policy = PhoneLocalizer::new(3.0).unwrap();
        let json = localization_report_json(policy.update(input(0.0)).unwrap());
        let value: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(value["version"], 1);
        assert!(value["frameID"].as_str().unwrap().len() == 36);
        assert_eq!(value["gpsLatitude"], 38.0);
        assert_eq!(value["gpsFresh"], true);
        assert!(value.get("headingAccuracy").is_none());
    }
}
