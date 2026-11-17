//! The rover's actuator layout: wheel efforts and servo targets become one
//! actuator frame, the bytes the phone forwards to the motor board.
use std::collections::BTreeMap;
use terra_actuators::{DriveProfile, DriveProfileSet, Layout, RoutingInput, protocol};

/// The rover's synchronized drive profiles plus any servo targets set since.
pub struct ActiveLayout {
    profiles: Vec<DriveProfile>,
    servos: BTreeMap<u8, f32>,
}

fn parse<T: serde::de::DeserializeOwned>(json: &str) -> Result<T, String> {
    if json.len() > protocol::JSON_LIMIT {
        return Err("JSON document too large".into());
    }
    serde_json::from_str(json).map_err(|e| e.to_string())
}

impl ActiveLayout {
    /// Accepts drive profiles, or a full layout (a rover without paginated
    /// reads), whose port and calibration are dropped.
    pub fn parse(profiles_json: &str) -> Result<Self, String> {
        let set = match parse::<DriveProfileSet>(profiles_json) {
            Ok(set) => set,
            Err(profile_error) => parse::<Layout>(profiles_json)
                .map(DriveProfileSet::from)
                .map_err(|_| profile_error)?,
        };
        set.validate().map_err(|e| e.to_string())?;
        Ok(Self {
            profiles: set.actuators,
            servos: BTreeMap::new(),
        })
    }

    /// The layout has differential drive routes, which L2–L5 need.
    pub fn supports_vio_control(&self) -> bool {
        terra_actuators::supports_differential_drive(&self.profiles)
    }

    /// Clip a servo target to its limits; unknown or non-servo IDs are rejected.
    pub fn set_servo(&mut self, id: u8, value: f64) -> Result<(), String> {
        let profile = self
            .profiles
            .iter()
            .find(|p| p.id == id && matches!(p.route, terra_actuators::Route::Servo))
            .ok_or_else(|| format!("servo {id} is missing from the current profile"))?;
        if !value.is_finite() {
            return Err("servo target must be finite".into());
        }
        self.servos.insert(
            id,
            (value as f32).clamp(profile.limits.min, profile.limits.max),
        );
        Ok(())
    }

    /// Route wheel efforts through the layout as `[{"id","value"}]` JSON;
    /// see [`RoutingInput::wheels`].
    pub fn frame(&self, left: f64, right: f64) -> Result<String, String> {
        let input = RoutingInput::wheels(left as f32, right as f32, self.servos.clone());
        let values = terra_actuators::route_drive_profiles(&self.profiles, &input)
            .map_err(|e| e.to_string())?;
        serde_json::to_string(&values).map_err(|e| e.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use terra_actuators::ActuatorValue;

    const LAYOUT: &str = r#"{"schema_version":1,"revision":1,"actuators":[
        {"id":0,"name":"left","port":"M0","kind":"dc_motor","inverted":false,
         "limits":{"min":-1.0,"max":1.0},
         "calibration":{"type":"dc_motor","max_power_fraction":1.0},
         "route":{"type":"left_effort"},"safe":{"type":"zero"}},
        {"id":1,"name":"right","port":"M1","kind":"dc_motor","inverted":false,
         "limits":{"min":-1.0,"max":1.0},
         "calibration":{"type":"dc_motor","max_power_fraction":1.0},
         "route":{"type":"right_effort"},"safe":{"type":"zero"}}
    ]}"#;
    const PROFILES: &str = r#"{"schema_version":1,"revision":3,"actuators":[
        {"id":0,"kind":"dc_motor","limits":{"min":-1.0,"max":1.0},
         "route":{"type":"manual","forward_coefficient":1.0,"turn_coefficient":-1.0},"safe":{"type":"zero"}},
        {"id":1,"kind":"dc_motor","limits":{"min":-0.5,"max":0.5},
         "route":{"type":"right_effort"},"safe":{"type":"zero"}},
        {"id":2,"kind":"positional_servo","limits":{"min":-0.8,"max":0.8},
         "route":{"type":"servo"},"safe":{"type":"position","value":0.1}}
    ]}"#;

    #[test]
    fn active_layout_mixes_efforts_like_the_phone() {
        let mut layout = ActiveLayout::parse(PROFILES).unwrap();
        assert!(layout.supports_vio_control());
        let frame: Vec<ActuatorValue> =
            serde_json::from_str(&layout.frame(0.2, 0.6).unwrap()).unwrap();
        // Manual left mix: (l + r) / 2 - (r - l) / 2 = l.
        assert!((frame[0].value - 0.2).abs() < 1e-6);
        assert_eq!(frame[1].value, 0.5);
        assert_eq!(frame[2].value, 0.1);
        layout.set_servo(2, 2.0).unwrap();
        assert!(layout.set_servo(1, 0.0).is_err());
        let frame: Vec<ActuatorValue> =
            serde_json::from_str(&layout.frame(0.0, 0.0).unwrap()).unwrap();
        assert_eq!(frame[2].value, 0.8);
        assert!(layout.frame(1.5, 0.0).is_err());
    }

    #[test]
    /// A framed session's full layout routes wheels to the left and right motors.
    fn active_layout_accepts_a_full_layout() {
        let layout = ActiveLayout::parse(LAYOUT).unwrap();
        assert!(layout.supports_vio_control());
        let frame: Vec<ActuatorValue> =
            serde_json::from_str(&layout.frame(-0.3, 0.7).unwrap()).unwrap();
        assert!((frame[0].value + 0.3).abs() < 1e-6);
        assert!((frame[1].value - 0.7).abs() < 1e-6);
    }

    #[test]
    fn active_layout_rejects_invalid_profiles() {
        assert!(ActiveLayout::parse("{}").is_err());
        let servo_on_motor = PROFILES.replace(
            r#""route":{"type":"right_effort"}"#,
            r#""route":{"type":"servo"}"#,
        );
        assert!(ActiveLayout::parse(&servo_on_motor).is_err());
        assert!(
            !ActiveLayout::parse(
                &PROFILES.replace(r#""turn_coefficient":-1.0"#, r#""turn_coefficient":-0.5"#)
            )
            .unwrap()
            .supports_vio_control()
        );
    }
}
