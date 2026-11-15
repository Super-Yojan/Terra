//! Strict JSON and portable wire encoding at the UniFFI boundary.
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use serde::Deserialize;
use terra_actuators::protocol::{self, RelaySample};
use terra_actuators::{ActuatorValue, DriveProfile, DriveProfileSet, Layout, RoutingInput};
#[derive(Debug, thiserror::Error, uniffi::Error)]
/// Validation, JSON decoding, routing, or binary-protocol encoding failure.
pub enum MobileActuatorError {
    #[error("{message}")]
    /// Rejected input or protocol operation; `message` contains the underlying diagnostic.
    InvalidInput {
        /// Human-readable diagnostic from the rejected operation.
        message: String,
    },
}
fn invalid(error: impl std::fmt::Display) -> MobileActuatorError {
    MobileActuatorError::InvalidInput {
        message: error.to_string(),
    }
}
fn parse<T: serde::de::DeserializeOwned>(json: &str) -> Result<T, MobileActuatorError> {
    if json.len() > protocol::JSON_LIMIT {
        return Err(invalid("JSON document too large"));
    }
    serde_json::from_str(json).map_err(invalid)
}
#[uniffi::export]
/// Largest control or reply JSON document in bytes; the limit for an
/// [`ActuatorAssembler`] that reads whole replies or status documents.
pub fn actuator_json_limit() -> u32 {
    protocol::JSON_LIMIT as u32
}
#[uniffi::export]
/// Split payload bytes into protocol fragments for a maximum write size in bytes.
/// `message_id` identifies the reassembled message. The protocol JSON size cap applies.
///
/// # Errors
/// Returns `InvalidInput` if the payload or write budget cannot be fragmented
/// within protocol limits.
pub fn actuator_fragment(
    message_id: u16,
    payload: Vec<u8>,
    maximum_write_length: u32,
) -> Result<Vec<Vec<u8>>, MobileActuatorError> {
    protocol::fragment_message(
        &payload,
        message_id,
        maximum_write_length as usize,
        protocol::JSON_LIMIT,
    )
    .map_err(invalid)
}

#[uniffi::export]
/// Route a stick through a full layout.
///
/// `forward` and `turn` are normalized. Positive `turn` is a left turn, so
/// left-effort routes receive `forward - turn` and right-effort routes receive
/// `forward + turn`, each clamped to `[-1, 1]` before per-actuator limits.
/// Servos without an explicit target use their safe position.
///
/// # Errors
/// Returns `InvalidInput` for stick values outside `[-1, 1]`, oversized or
/// malformed layout JSON, or routing rejection.
pub fn actuator_route_stick(
    layout_json: String,
    forward: f32,
    turn: f32,
) -> Result<String, MobileActuatorError> {
    let layout: Layout = parse(&layout_json)?;
    let values = terra_actuators::route_commands(
        &layout,
        &RoutingInput::stick(forward, turn).map_err(invalid)?,
    )
    .map_err(invalid)?;
    serde_json::to_string(&values).map_err(invalid)
}

#[uniffi::export]
/// Route a stick through rover drive profiles (`read_drive_profile` records).
///
/// The JSON document is `{schema_version, revision, actuators:[{id, kind, limits, route, safe}]}`.
/// Mixing matches [`actuator_route_stick`]. Profiles omit calibration and port;
/// those stay on the rover.
///
/// # Errors
/// Returns `InvalidInput` for stick values outside `[-1, 1]`, oversized or
/// malformed profile JSON, or routing rejection.
pub fn actuator_route_profiles(
    profiles_json: String,
    forward: f32,
    turn: f32,
) -> Result<String, MobileActuatorError> {
    let set: DriveProfileSet = parse(&profiles_json)?;
    set.validate().map_err(invalid)?;
    let values = terra_actuators::route_drive_profiles(
        &set.actuators,
        &RoutingInput::stick(forward, turn).map_err(invalid)?,
    )
    .map_err(invalid)?;
    serde_json::to_string(&values).map_err(invalid)
}

/// The rover's synchronized drive profiles plus any servo targets set since.
pub(crate) struct ActiveLayout {
    profiles: Vec<DriveProfile>,
    servos: BTreeMap<u8, f32>,
}

impl ActiveLayout {
    /// Accepts drive profiles, or a full layout (a rover without paginated reads),
    /// whose port and calibration are dropped.
    pub(crate) fn parse(profiles_json: &str) -> Result<Self, MobileActuatorError> {
        let set = match parse::<DriveProfileSet>(profiles_json) {
            Ok(set) => set,
            Err(profile_error) => parse::<Layout>(profiles_json)
                .map(DriveProfileSet::from)
                .map_err(|_| profile_error)?,
        };
        set.validate().map_err(invalid)?;
        Ok(Self {
            profiles: set.actuators,
            servos: BTreeMap::new(),
        })
    }

    pub(crate) fn supports_vio_control(&self) -> bool {
        terra_actuators::supports_differential_drive(&self.profiles)
    }

    /// Clip a servo target to its limits; unknown or non-servo IDs are rejected.
    pub(crate) fn set_servo(&mut self, id: u8, value: f64) -> Result<(), MobileActuatorError> {
        let profile = self
            .profiles
            .iter()
            .find(|p| p.id == id && matches!(p.route, terra_actuators::Route::Servo))
            .ok_or_else(|| invalid(format!("servo {id} is missing from the current profile")))?;
        if !value.is_finite() {
            return Err(invalid("servo target must be finite"));
        }
        self.servos.insert(
            id,
            (value as f32).clamp(profile.limits.min, profile.limits.max),
        );
        Ok(())
    }

    /// Route wheel efforts the way the phone does; see [`RoutingInput::wheels`].
    pub(crate) fn frame(&self, left: f64, right: f64) -> Result<String, MobileActuatorError> {
        let input = RoutingInput::wheels(left as f32, right as f32, self.servos.clone());
        let values =
            terra_actuators::route_drive_profiles(&self.profiles, &input).map_err(invalid)?;
        serde_json::to_string(&values).map_err(invalid)
    }
}

#[derive(Deserialize)]
struct LimitCarrier {
    actuators: Vec<LimitActuator>,
}

#[derive(Deserialize)]
struct LimitActuator {
    id: u8,
    limits: terra_actuators::CommandLimits,
}

#[uniffi::export]
/// Encode streamed-relay drive bytes from routed values and a layout or profile document.
///
/// `limits_json` is any document with `actuators[].id` and `actuators[].limits.{min,max}`
/// (a full layout or a drive-profile set). `values_json` is the array returned by
/// [`actuator_route_stick`] or [`actuator_route_profiles`]. The bytes are
/// `(id: u8, effort: i8 percent)` pairs for the current rover drive characteristic.
///
/// # Errors
/// Returns `InvalidInput` when a value has no limits, cannot be quantized inside
/// those limits, or either document is oversized or malformed.
pub fn actuator_encode_relay_drive(
    limits_json: String,
    values_json: String,
) -> Result<Vec<u8>, MobileActuatorError> {
    let carrier: LimitCarrier = parse(&limits_json)?;
    let values: Vec<ActuatorValue> = parse(&values_json)?;
    let mut limits = BTreeMap::new();
    for actuator in carrier.actuators {
        if limits
            .insert(actuator.id, (actuator.limits.min, actuator.limits.max))
            .is_some()
        {
            return Err(invalid("actuator IDs must be unique"));
        }
    }
    let samples = values
        .iter()
        .map(|value| {
            let (min, max) = limits
                .get(&value.id)
                .copied()
                .ok_or_else(|| invalid("drive value is missing from the active profile"))?;
            Ok(RelaySample {
                id: value.id,
                value: value.value,
                min,
                max,
            })
        })
        .collect::<Result<Vec<_>, MobileActuatorError>>()?;
    protocol::encode_relay_drive(&samples).map_err(invalid)
}

#[uniffi::export]
/// Validate and re-encode a schema-1 control envelope.
///
/// Accepts every operation in [`protocol::ControlOperation`]: the five layout
/// operations plus the rover's paginated reads and layout edits.
///
/// # Errors
/// Returns `InvalidInput` for oversized JSON, unknown operations, or a payload
/// the shared control schema rejects.
pub fn actuator_encode_control(envelope_json: String) -> Result<Vec<u8>, MobileActuatorError> {
    if envelope_json.len() > protocol::JSON_LIMIT {
        return Err(invalid("JSON document too large"));
    }
    let envelope = protocol::decode_control(envelope_json.as_bytes()).map_err(invalid)?;
    protocol::encode_control(&envelope).map_err(invalid)
}

#[derive(uniffi::Object)]
/// Reassembles one characteristic's notification fragments.
///
/// The timeout is the protocol's 100 ms. Create one assembler per characteristic
/// and clear it on disconnect.
pub struct ActuatorAssembler {
    inner: Mutex<protocol::FragmentAssembler>,
}

#[uniffi::export]
impl ActuatorAssembler {
    #[uniffi::constructor]
    /// `limit` is the maximum logical message size in bytes, from 1 through 16384.
    ///
    /// # Errors
    /// Returns `InvalidInput` when `limit` is outside that range.
    pub fn new(limit: u32) -> Result<Arc<Self>, MobileActuatorError> {
        Ok(Arc::new(Self {
            inner: Mutex::new(protocol::FragmentAssembler::new(limit as usize).map_err(invalid)?),
        }))
    }

    /// Drop an incomplete message.
    pub fn clear(&self) {
        if let Ok(mut assembler) = self.inner.lock() {
            assembler.clear();
        }
    }

    /// Push one fragment at `now_ms` monotonic milliseconds.
    ///
    /// Returns the logical message when this fragment completes it.
    ///
    /// # Errors
    /// Returns `InvalidInput` for a malformed fragment, an expired or conflicting
    /// assembly, or a poisoned lock.
    pub fn push(
        &self,
        bytes: Vec<u8>,
        now_ms: u64,
    ) -> Result<Option<Vec<u8>>, MobileActuatorError> {
        self.inner
            .lock()
            .map_err(|_| invalid("actuator assembler lock poisoned"))?
            .push(&bytes, now_ms)
            .map_err(invalid)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn stick_routes_and_relay_quantizes() {
        let values = actuator_route_stick(LAYOUT.into(), 0.2, 0.5).unwrap();
        let bytes = actuator_encode_relay_drive(LAYOUT.into(), values).unwrap();
        assert_eq!(bytes, vec![0, (-30i8) as u8, 1, 70]);
    }

    #[test]
    fn fragment_and_control_round_trip() {
        let parts = actuator_fragment(7, b"hello".to_vec(), 7).unwrap();
        let assembler = ActuatorAssembler::new(96).unwrap();
        assert_eq!(assembler.push(parts[0].clone(), 0).unwrap(), None);
        assert_eq!(
            assembler.push(parts[1].clone(), 1).unwrap().as_deref(),
            Some(b"hello".as_slice())
        );
        let control = actuator_encode_control(
            r#"{"schema_version":1,"request_id":1000,"operation":"capabilities","payload":{}}"#
                .into(),
        )
        .unwrap();
        assert!(
            std::str::from_utf8(&control)
                .unwrap()
                .contains("capabilities")
        );
    }
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
    /// A framed session's full layout routes like the stick helper.
    fn active_layout_accepts_a_full_layout() {
        let layout = ActiveLayout::parse(LAYOUT).unwrap();
        assert!(layout.supports_vio_control());
        assert_eq!(
            layout.frame(-0.3, 0.7).unwrap(),
            actuator_route_stick(LAYOUT.into(), 0.2, 0.5).unwrap()
        );
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
