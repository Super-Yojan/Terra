//! BLE application framing. Authentication and hardware arming belong to the service.
use crate::{Actuator, ActuatorError, Layout, MAX_ACTUATORS};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
/// Maximum serialized JSON control/reply size in bytes (16 KiB).
pub const JSON_LIMIT: usize = 16 * 1024;
/// Maximum size of a paginated control request: 255 fragments at the
/// conservative 20-byte ATT write capacity.
pub const PAGINATED_LIMIT: usize = 4080;
/// Maximum length of a layout edit token.
const EDIT_TOKEN_LIMIT: usize = 128;
/// Maximum assembly age in milliseconds; age equal to this limit expires.
const ASSEMBLY_TIMEOUT_MS: u64 = 100;
fn error(message: &str) -> ActuatorError {
    ActuatorError::InvalidInput(message.into())
}
/// Bounded in-order reassembly of one logical message at a time.
///
/// Fragments have a four-byte header: little-endian `u16` message ID, zero-based
/// index, and total count. Payloads must be nonempty. Authentication belongs to the service.
pub struct FragmentAssembler {
    limit: usize,
    active: Option<Assembly>,
}
struct Assembly {
    id: u16,
    count: u8,
    next: u8,
    started: u64,
    data: Vec<u8>,
}
impl FragmentAssembler {
    /// Construct an empty assembler with a byte limit in `1..=JSON_LIMIT`.
    ///
    /// Returns [`ActuatorError::InvalidInput`] if the limit is zero or too large.
    pub fn new(limit: usize) -> Result<Self, ActuatorError> {
        if limit == 0 || limit > JSON_LIMIT {
            return Err(error("invalid logical limit"));
        }
        Ok(Self {
            limit,
            active: None,
        })
    }
    /// Discard any incomplete message without changing the size limit.
    pub fn clear(&mut self) {
        self.active = None;
    }
    /// Accept one fragment at monotonic `now_ms` milliseconds.
    ///
    /// Returns `Ok(None)` while incomplete and `Ok(Some(bytes))` on completion.
    /// A backwards clock or an age of at least 100 ms discards the old
    /// assembly before processing this fragment. Invalid headers, ordering, IDs,
    /// counts, or size return [`ActuatorError::InvalidInput`] and clear the assembly.
    pub fn push(&mut self, bytes: &[u8], now_ms: u64) -> Result<Option<Vec<u8>>, ActuatorError> {
        let result = self.push_inner(bytes, now_ms);
        if result.is_err() {
            self.clear();
        }
        result
    }
    fn push_inner(&mut self, bytes: &[u8], now: u64) -> Result<Option<Vec<u8>>, ActuatorError> {
        if self
            .active
            .as_ref()
            .is_some_and(|a| now < a.started || now - a.started >= ASSEMBLY_TIMEOUT_MS)
        {
            self.clear();
        }
        if bytes.len() < 5 {
            return Err(error("empty or short fragment"));
        }
        let id = u16::from_le_bytes([bytes[0], bytes[1]]);
        let index = bytes[2];
        let count = bytes[3];
        if count == 0 || index >= count {
            return Err(error("invalid fragment index/count"));
        }
        if self.active.is_none() {
            if index != 0 {
                return Err(error("assembly must begin at index zero"));
            }
            self.active = Some(Assembly {
                id,
                count,
                next: 0,
                started: now,
                data: Vec::new(),
            });
        }
        let a = self.active.as_mut().unwrap();
        if id != a.id || count != a.count || index != a.next {
            return Err(error("duplicate, conflicting or out-of-order fragment"));
        }
        if a.data.len() + bytes.len() - 4 > self.limit {
            return Err(error("logical message too large"));
        }
        a.data.extend_from_slice(&bytes[4..]);
        if index == count - 1 {
            return Ok(self.active.take().map(|a| a.data));
        }
        a.next += 1;
        Ok(None)
    }
}
/// One actuator command and the inclusive normalized limits used to quantize it.
///
/// `value` is the normalized command. `min` and `max` are that actuator's limits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RelaySample {
    /// Actuator ID from the active layout or drive profile.
    pub id: u8,
    /// Normalized command, finite and inside both `[-1, 1]` and `[min, max]`.
    pub value: f32,
    /// Inclusive lower limit.
    pub min: f32,
    /// Inclusive upper limit.
    pub max: f32,
}

fn normalized_percent(effort: i32) -> f32 {
    // Match the rover: quantize through f64, then the f32 the service compares.
    ((effort as f64) / 100.0) as f32
}

/// Encode streamed-relay drive bytes: repeating `(id: u8, effort: i8)` pairs.
///
/// Effort is the normalized command in percent, clamped so the f32 the rover
/// reconstructs stays inside `[min, max]`. This is the drive characteristic
/// payload for `control_mode = streamed_relay`. It is not a `TA` frame and it
/// has no fragment header. Returns [`ActuatorError::InvalidInput`] for an empty
/// or oversized record list, duplicate IDs, or a value that cannot be quantized
/// inside its limits.
pub fn encode_relay_drive(samples: &[RelaySample]) -> Result<Vec<u8>, ActuatorError> {
    if samples.is_empty() || samples.len() > MAX_ACTUATORS {
        return Err(error("expected one to sixteen actuator pairs"));
    }
    let mut ids = HashSet::new();
    let mut out = Vec::with_capacity(samples.len() * 2);
    for sample in samples {
        if !ids.insert(sample.id)
            || !sample.value.is_finite()
            || !sample.min.is_finite()
            || !sample.max.is_finite()
            || !(-1.0..=1.0).contains(&sample.value)
            || sample.value < sample.min
            || sample.value > sample.max
            || sample.min > sample.max
        {
            return Err(error("invalid actuator record"));
        }
        let mut effort = ((sample.value as f64) * 100.0).round() as i32;
        while effort > -100 && normalized_percent(effort) > sample.max {
            effort -= 1;
        }
        while effort < 100 && normalized_percent(effort) < sample.min {
            effort += 1;
        }
        let normalized = normalized_percent(effort);
        if !(-100..=100).contains(&effort) || normalized < sample.min || normalized > sample.max {
            return Err(error("actuator effort outside configured limits"));
        }
        out.push(sample.id);
        out.push(effort as i8 as u8);
    }
    Ok(out)
}

/// Split a nonempty logical message into writes with four-byte fragment headers.
///
/// `write_size` and `limit` are bytes including each write header and logical data
/// respectively. Returns [`ActuatorError::InvalidInput`] if `write_size <= 4`,
/// the logical limit is outside `1..=JSON_LIMIT`, data exceed that limit, or more
/// than 255 fragments are needed. No transport writes occur here.
///
/// ```
/// use terra_actuators::protocol::{fragment_message, FragmentAssembler};
/// let parts = fragment_message(b"hello", 7, 7, 96).unwrap();
/// let mut receiver = FragmentAssembler::new(96).unwrap();
/// assert_eq!(receiver.push(&parts[0], 0).unwrap(), None);
/// assert_eq!(receiver.push(&parts[1], 1).unwrap(), Some(b"hello".to_vec()));
/// ```
pub fn fragment_message(
    data: &[u8],
    message_id: u16,
    write_size: usize,
    limit: usize,
) -> Result<Vec<Vec<u8>>, ActuatorError> {
    if data.is_empty() || limit == 0 || limit > JSON_LIMIT || data.len() > limit || write_size <= 4
    {
        return Err(error("invalid message or negotiated write size"));
    }
    let chunk = write_size - 4;
    let count = data.len().div_ceil(chunk);
    if count > 255 {
        return Err(error("message requires more than 255 fragments"));
    }
    Ok(data
        .chunks(chunk)
        .enumerate()
        .map(|(index, part)| {
            let mut out = message_id.to_le_bytes().to_vec();
            out.extend([index as u8, count as u8]);
            out.extend(part);
            out
        })
        .collect())
}
/// Caller supplies every still outstanding ID, including transmissions awaiting completion.
/// Search the wrapping `u16` ID space from `next`, advancing it after each candidate.
///
/// Returns a currently unused ID, or [`ActuatorError::InvalidInput`] if all 65,536
/// IDs are active. Exhaustion wraps `next` back to its starting value.
pub fn allocate_message_id(next: &mut u16, active: &HashSet<u16>) -> Result<u16, ActuatorError> {
    for _ in 0..=u16::MAX {
        let id = *next;
        *next = next.wrapping_add(1);
        if !active.contains(&id) {
            return Ok(id);
        }
    }
    Err(error("all message IDs active"))
}
/// Empty JSON object for operations with no arguments; unknown fields are rejected.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EmptyPayload {}
/// Candidate layout to validate and stage before a separate commit.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StagePayload {
    /// Proposed layout; deserialization alone does not validate hardware compatibility.
    pub layout: Layout,
}
/// Identifiers binding a commit to a previously staged request.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CommitPayload {
    /// Revision of the staged candidate.
    pub staged_revision: u32,
    /// Request ID of the successful stage operation.
    pub staged_request_id: u32,
}
/// One actuator of the active layout, read page by page.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ActuatorPage {
    /// Active layout revision the reader expects; the rover rejects another.
    pub expected_revision: u32,
    pub actuator_id: u8,
}
/// Whether a layout edit starts from the active layout or from empty.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EditMode {
    Existing,
    Replace,
}
/// Open a paginated layout edit against the active revision.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BeginEditPayload {
    pub expected_revision: u32,
    pub mode: EditMode,
}
/// Add or replace one actuator in an open edit.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StageActuatorPayload {
    /// Token from `begin_layout_edit`, 1 to 128 characters.
    pub edit_token: String,
    /// Edit version the change applies to.
    pub edit_version: u32,
    pub actuator: Actuator,
}
/// Remove one actuator from an open edit.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RemoveActuatorPayload {
    pub edit_token: String,
    pub edit_version: u32,
    pub actuator_id: u8,
}
/// Refer to an open edit at a version (validate).
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EditVersionPayload {
    pub edit_token: String,
    pub edit_version: u32,
}
/// Commit an open edit over the revision it started from.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CommitEditPayload {
    pub edit_token: String,
    pub edit_version: u32,
    pub base_revision: u32,
}
/// Abandon an open edit.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DiscardEditPayload {
    pub edit_token: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(
    tag = "operation",
    content = "payload",
    rename_all = "snake_case",
    deny_unknown_fields
)]
/// JSON configuration/service operation; parsing does not execute or authorize it.
pub enum ControlOperation {
    /// Read advertised backend capabilities.
    Capabilities(EmptyPayload),
    /// Read the active layout.
    ReadLayout(EmptyPayload),
    /// Validate/stage a proposed layout.
    StageLayout(StagePayload),
    /// Commit the referenced staged candidate.
    CommitLayout(CommitPayload),
    /// Request service fault reset.
    ResetFault(EmptyPayload),
    /// Read the active layout's actuator IDs.
    LayoutIndex(EmptyPayload),
    /// Read one actuator of the active layout.
    ReadActuator(ActuatorPage),
    /// Read one actuator's drive profile.
    ReadDriveProfile(ActuatorPage),
    BeginLayoutEdit(BeginEditPayload),
    StageActuator(StageActuatorPayload),
    RemoveActuator(RemoveActuatorPayload),
    ValidateLayoutEdit(EditVersionPayload),
    CommitLayoutEdit(CommitEditPayload),
    DiscardLayoutEdit(DiscardEditPayload),
}
impl ControlOperation {
    /// Whether the rover answers this operation one page at a time; such
    /// requests are limited to [`PAGINATED_LIMIT`] bytes.
    pub fn paginated(&self) -> bool {
        !matches!(
            self,
            Self::Capabilities(_)
                | Self::ReadLayout(_)
                | Self::StageLayout(_)
                | Self::CommitLayout(_)
                | Self::ResetFault(_)
        )
    }
    fn edit_token(&self) -> Option<&str> {
        match self {
            Self::StageActuator(p) => Some(&p.edit_token),
            Self::RemoveActuator(p) => Some(&p.edit_token),
            Self::ValidateLayoutEdit(p) => Some(&p.edit_token),
            Self::CommitLayoutEdit(p) => Some(&p.edit_token),
            Self::DiscardLayoutEdit(p) => Some(&p.edit_token),
            _ => None,
        }
    }
}
/// Versioned JSON control request with a caller-supplied correlation ID.
#[derive(Clone, Debug, Serialize)]
pub struct ControlEnvelope {
    /// Control schema version; currently 1.
    pub schema_version: u32,
    /// Caller-supplied identifier echoed in the reply.
    pub request_id: u32,
    /// Tagged operation and its operation-specific payload.
    #[serde(flatten)]
    pub command: ControlOperation,
}
// Flatten and deny_unknown_fields are intentionally avoided at the parsing boundary below.
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct EnvelopeWire {
    schema_version: u32,
    request_id: u32,
    operation: String,
    payload: Box<serde_json::value::RawValue>,
}
/// Decode a strict JSON request of at most [`JSON_LIMIT`] bytes.
///
/// Returns [`ActuatorError::InvalidInput`] for malformed JSON, unknown fields or
/// operations, invalid payload shape, excessive size, or unsupported schema.
/// Layout semantic validation remains the staging service responsibility.
pub fn decode_control(data: &[u8]) -> Result<ControlEnvelope, ActuatorError> {
    if data.len() > JSON_LIMIT {
        return Err(error("JSON document too large"));
    }
    let wire: EnvelopeWire = serde_json::from_slice(data).map_err(|e| error(&e.to_string()))?;
    if wire.schema_version != 1 {
        return Err(error("unsupported control schema"));
    }
    let payload = wire.payload.get();
    macro_rules! parse {
        ($variant:ident) => {
            ControlOperation::$variant(
                serde_json::from_str(payload).map_err(|e| error(&e.to_string()))?,
            )
        };
    }
    let command = match wire.operation.as_str() {
        "capabilities" => parse!(Capabilities),
        "read_layout" => parse!(ReadLayout),
        "stage_layout" => parse!(StageLayout),
        "commit_layout" => parse!(CommitLayout),
        "reset_fault" => parse!(ResetFault),
        "layout_index" => parse!(LayoutIndex),
        "read_actuator" => parse!(ReadActuator),
        "read_drive_profile" => parse!(ReadDriveProfile),
        "begin_layout_edit" => parse!(BeginLayoutEdit),
        "stage_actuator" => parse!(StageActuator),
        "remove_actuator" => parse!(RemoveActuator),
        "validate_layout_edit" => parse!(ValidateLayoutEdit),
        "commit_layout_edit" => parse!(CommitLayoutEdit),
        "discard_layout_edit" => parse!(DiscardLayoutEdit),
        _ => return Err(error("unknown operation")),
    };
    if command
        .edit_token()
        .is_some_and(|t| t.is_empty() || t.chars().count() > EDIT_TOKEN_LIMIT)
    {
        return Err(error("invalid edit token"));
    }
    if command.paginated() && data.len() > PAGINATED_LIMIT {
        return Err(error("paginated request too large"));
    }
    Ok(ControlEnvelope {
        schema_version: wire.schema_version,
        request_id: wire.request_id,
        command,
    })
}
/// Serialize a request and validate it again through [`decode_control`].
///
/// Returns [`ActuatorError::InvalidInput`] for serialization or decoding failures,
/// including unsupported schema and documents exceeding [`JSON_LIMIT`].
pub fn encode_control(envelope: &ControlEnvelope) -> Result<Vec<u8>, ActuatorError> {
    let bytes = serde_json::to_vec(envelope).map_err(|e| error(&e.to_string()))?;
    decode_control(&bytes)?;
    Ok(bytes)
}
#[cfg(test)]
mod tests {
    use super::*;

    fn sample(id: u8, value: f32) -> RelaySample {
        RelaySample {
            id,
            value,
            min: -1.0,
            max: 1.0,
        }
    }

    #[test]
    fn relay_drive_is_id_and_signed_percent() {
        let bytes = encode_relay_drive(&[sample(2, 0.0), sample(1, -1.0), sample(3, 0.5)]).unwrap();
        assert_eq!(bytes, vec![2, 0, 1, (-100i8) as u8, 3, 50]);
    }

    #[test]
    fn relay_drive_rejects_a_value_outside_its_limit() {
        let sample = RelaySample {
            id: 0,
            value: 0.5,
            min: -0.4,
            max: 0.4,
        };
        assert!(encode_relay_drive(&[sample]).is_err());
    }

    #[test]
    fn narrow_window_that_percent_cannot_represent_is_rejected() {
        let sample = RelaySample {
            id: 0,
            value: 0.004,
            min: 0.004,
            max: 0.004,
        };
        assert!(encode_relay_drive(&[sample]).is_err());
    }

    #[test]
    fn only_paginated_requests_are_held_to_the_page_limit() {
        let token = "t".repeat(EDIT_TOKEN_LIMIT);
        let name = "n".repeat(PAGINATED_LIMIT);
        let actuator = serde_json::json!({"id": 0, "name": name, "port": "M0",
            "kind": "dc_motor", "inverted": false, "limits": {"min": -1.0, "max": 1.0},
            "calibration": {"type": "dc_motor", "max_power_fraction": 1.0},
            "route": {"type": "left_effort"}, "safe": {"type": "zero"}});
        let envelope = |operation: &str, payload: serde_json::Value| {
            serde_json::json!({"schema_version": 1, "request_id": 1,
                "operation": operation, "payload": payload})
            .to_string()
        };
        let stage = envelope(
            "stage_actuator",
            serde_json::json!({"edit_token": token, "edit_version": 1, "actuator": actuator}),
        );
        assert!(decode_control(stage.as_bytes()).is_err());
        let layout = envelope(
            "stage_layout",
            serde_json::json!({"layout": {"schema_version": 1, "revision": 1, "actuators": [actuator]}}),
        );
        assert!(decode_control(layout.as_bytes()).is_ok());
        let long_token = envelope(
            "discard_layout_edit",
            serde_json::json!({"edit_token": "t".repeat(EDIT_TOKEN_LIMIT + 1)}),
        );
        assert!(decode_control(long_token.as_bytes()).is_err());
    }
}
