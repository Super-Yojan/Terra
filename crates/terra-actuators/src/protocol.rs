//! BLE application framing. Authentication and hardware arming belong to the service.
use crate::{ActuatorError, ActuatorValue, Layout, LayoutError};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
/// Binary frame version byte; currently 1.
pub const PROTOCOL_VERSION: u8 = 1;
/// Maximum binary command size in bytes, including its 16-byte header.
pub const COMMAND_LIMIT: usize = 96;
/// Maximum serialized JSON control/reply size in bytes (16 KiB).
pub const JSON_LIMIT: usize = 16 * 1024;
/// Maximum assembly age in milliseconds; age equal to this limit expires.
pub const ASSEMBLY_TIMEOUT_MS: u64 = 100;
fn error(message: &str) -> ActuatorError {
    ActuatorError::InvalidInput(message.into())
}
/// Binary command discriminator. Only drive frames carry actuator records.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CommandKind {
    /// Request a complete set of normalized actuator commands.
    Drive = 1,
    /// Request arming; service authorization is still required.
    Arm = 2,
    /// Request removal of output authorization.
    Disarm = 3,
    /// Request the service emergency-stop policy.
    EmergencyStop = 4,
}
/// Decoded binary command with session, layout, and ordering metadata.
///
/// Encoding checks shape and normalized values. [`validate_command`] adds active
/// session/revision, sequence, coverage, and per-actuator limit checks.
#[derive(Clone, Debug, PartialEq)]
pub struct CommandFrame {
    /// Requested command action.
    pub kind: CommandKind,
    /// Service-issued session identifier.
    pub session: u32,
    /// Active layout revision expected by the sender.
    pub revision: u32,
    /// Monotonically increasing session sequence; `u32::MAX` is rejected on validation.
    pub sequence: u32,
    /// Up to 16 unique-ID records for drive; empty for all other commands.
    pub values: Vec<ActuatorValue>,
}
/// Encode a 16-byte `TA` header followed by five-byte actuator records.
///
/// Integers and `f32` values use little-endian encoding. Returns
/// [`ActuatorError::InvalidInput`] for more than 16 records, non-drive records,
/// duplicate IDs, or non-finite/out-of-range values outside `[-1, 1]`.
pub fn encode_frame(frame: &CommandFrame) -> Result<Vec<u8>, ActuatorError> {
    if frame.values.len() > 16 || (frame.kind != CommandKind::Drive && !frame.values.is_empty()) {
        return Err(error("invalid record count"));
    }
    let mut ids = HashSet::new();
    let mut out = vec![b'T', b'A', PROTOCOL_VERSION, frame.kind as u8];
    for value in [frame.session, frame.revision, frame.sequence] {
        out.extend(value.to_le_bytes());
    }
    for value in &frame.values {
        if !ids.insert(value.id) || !value.value.is_finite() || !(-1.0..=1.0).contains(&value.value)
        {
            return Err(error("invalid actuator record"));
        }
        out.push(value.id);
        out.extend(value.value.to_le_bytes());
    }
    Ok(out)
}
/// Decode and validate a binary frame without authorizing it.
///
/// Returns [`ActuatorError::InvalidInput`] for invalid size, alignment, magic,
/// version, command kind, or any record rejected by [`encode_frame`].
pub fn decode_frame(bytes: &[u8]) -> Result<CommandFrame, ActuatorError> {
    if bytes.len() < 16
        || bytes.len() > COMMAND_LIMIT
        || (bytes.len() - 16) % 5 != 0
        || &bytes[..2] != b"TA"
        || bytes[2] != PROTOCOL_VERSION
    {
        return Err(error("invalid frame header or length"));
    }
    let kind = match bytes[3] {
        1 => CommandKind::Drive,
        2 => CommandKind::Arm,
        3 => CommandKind::Disarm,
        4 => CommandKind::EmergencyStop,
        _ => return Err(error("unknown command kind")),
    };
    let word = |offset| u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap());
    let values = bytes[16..]
        .chunks_exact(5)
        .map(|record| ActuatorValue {
            id: record[0],
            value: f32::from_le_bytes(record[1..5].try_into().unwrap()),
        })
        .collect();
    let frame = CommandFrame {
        kind,
        session: word(4),
        revision: word(8),
        sequence: word(12),
        values,
    };
    encode_frame(&frame)?;
    Ok(frame)
}
/// Validate a fully decoded command without advancing sequence state on rejection.
/// The caller advances its last sequence only after safety acceptance.
/// Return success only for the supplied session/layout revision and a sequence
/// strictly above `last_sequence` and below `u32::MAX`. Drive frames must cover
/// every layout actuator exactly once and respect its inclusive limits.
///
/// All rejection paths return [`ActuatorError::InvalidInput`]. The caller must
/// separately validate the layout and authorize the requested hardware action.
pub fn validate_command(
    frame: &CommandFrame,
    session: u32,
    layout: &Layout,
    last_sequence: Option<u32>,
) -> Result<(), ActuatorError> {
    encode_frame(frame)?;
    if frame.session != session
        || frame.revision != layout.revision
        || frame.sequence == u32::MAX
        || last_sequence.is_some_and(|last| frame.sequence <= last)
    {
        return Err(error("stale session, revision or sequence"));
    }
    if frame.kind == CommandKind::Drive {
        if frame.values.len() != layout.actuators.len() {
            return Err(error("incomplete actuator coverage"));
        }
        for value in &frame.values {
            let actuator = layout
                .actuators
                .iter()
                .find(|actuator| actuator.id == value.id)
                .ok_or_else(|| error("unknown actuator"))?;
            if value.value < actuator.limits.min || value.value > actuator.limits.max {
                return Err(error("actuator value outside layout limits"));
            }
        }
    }
    Ok(())
}
/// Bounded in-order reassembly of one logical message at a time.
///
/// Fragments have a four-byte header: little-endian `u16` message ID, zero-based
/// index, and total count. Payloads must be nonempty. Default uses [`COMMAND_LIMIT`];
/// [`Self::new`] selects an explicit limit. Authentication belongs to the service.
#[derive(Default)]
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
    /// A backwards clock or age at least [`ASSEMBLY_TIMEOUT_MS`] discards the old
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
        let limit = if self.limit == 0 {
            COMMAND_LIMIT
        } else {
            self.limit
        };
        if a.data.len() + bytes.len() - 4 > limit {
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
/// Separate characteristic paths: priority binary control never waits for drive.
pub struct CommandAssemblers {
    /// Independent assembly state for ordinary drive traffic.
    pub drive: FragmentAssembler,
    /// Independent assembly state for arm, disarm, and emergency-stop traffic.
    pub priority: FragmentAssembler,
}
impl Default for CommandAssemblers {
    fn default() -> Self {
        Self {
            drive: FragmentAssembler::default(),
            priority: FragmentAssembler::default(),
        }
    }
}
impl CommandAssemblers {
    /// Assemble and decode a priority fragment at `now_ms` monotonic milliseconds.
    ///
    /// Returns `None` until complete; drive frames on this path return an input error.
    /// A decoded disarm/emergency-stop clears any partial drive message, even before
    /// session acceptance. Assembly and binary decoding errors propagate unchanged.
    pub fn push_priority(
        &mut self,
        bytes: &[u8],
        now_ms: u64,
    ) -> Result<Option<CommandFrame>, ActuatorError> {
        let Some(data) = self.priority.push(bytes, now_ms)? else {
            return Ok(None);
        };
        let frame = decode_frame(&data)?;
        if frame.kind == CommandKind::Drive {
            return Err(error("drive forbidden on priority path"));
        }
        // Clearing is fail-safe and does not imply command/session acceptance.
        if matches!(frame.kind, CommandKind::Disarm | CommandKind::EmergencyStop) {
            self.drive.clear();
        }
        Ok(Some(frame))
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
    if samples.is_empty() || samples.len() > 16 {
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
    /// Request release of the emergency-stop latch.
    ResetEmergencyStop(EmptyPayload),
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
        "reset_emergency_stop" => parse!(ResetEmergencyStop),
        _ => return Err(error("unknown operation")),
    };
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
/// Service result serialized as `ok` or `error`.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlResult {
    /// Operation succeeded.
    Ok,
    /// Operation failed; inspect reply errors and payload.
    Error,
}
/// Versioned service response correlated with a control request.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlReply {
    /// Reply schema version; currently 1.
    pub schema_version: u32,
    /// Correlation identifier copied from the request.
    pub request_id: u32,
    /// Success or failure reported by the service.
    pub result: ControlResult,
    /// Layout revision active after processing the request.
    pub active_revision: u32,
    /// Collected layout validation errors, if applicable.
    pub errors: Vec<LayoutError>,
    /// Operation-specific response data.
    pub payload: serde_json::Value,
}
/// Serialize a schema-1 reply no larger than [`JSON_LIMIT`] bytes.
///
/// Returns [`ActuatorError::InvalidInput`] for unsupported schema, serialization
/// failure, or excessive encoded size. Payload semantics are service-defined.
pub fn encode_reply(reply: &ControlReply) -> Result<Vec<u8>, ActuatorError> {
    if reply.schema_version != 1 {
        return Err(error("unsupported reply schema"));
    }
    let bytes = serde_json::to_vec(reply).map_err(|e| error(&e.to_string()))?;
    if bytes.len() > JSON_LIMIT {
        return Err(error("JSON reply too large"));
    }
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
}
