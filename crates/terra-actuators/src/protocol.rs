//! BLE application framing. Authentication and hardware arming belong to the service.
use crate::{ActuatorError, ActuatorValue, Layout, LayoutError};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
pub const PROTOCOL_VERSION: u8 = 1;
pub const COMMAND_LIMIT: usize = 96;
pub const JSON_LIMIT: usize = 16 * 1024;
pub const ASSEMBLY_TIMEOUT_MS: u64 = 100;
fn error(message: &str) -> ActuatorError { ActuatorError::InvalidInput(message.into()) }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CommandKind { Drive = 1, Arm = 2, Disarm = 3, EmergencyStop = 4 }
#[derive(Clone, Debug, PartialEq)]
pub struct CommandFrame { pub kind: CommandKind, pub session: u32, pub revision: u32, pub sequence: u32, pub values: Vec<ActuatorValue> }
pub fn encode_frame(frame: &CommandFrame) -> Result<Vec<u8>, ActuatorError> {
    if frame.values.len() > 16 || (frame.kind != CommandKind::Drive && !frame.values.is_empty()) { return Err(error("invalid record count")); }
    let mut ids = HashSet::new();
    let mut out = vec![b'T', b'A', PROTOCOL_VERSION, frame.kind as u8];
    for value in [frame.session, frame.revision, frame.sequence] { out.extend(value.to_le_bytes()); }
    for value in &frame.values {
        if !ids.insert(value.id) || !value.value.is_finite() || !(-1.0..=1.0).contains(&value.value) { return Err(error("invalid actuator record")); }
        out.push(value.id); out.extend(value.value.to_le_bytes());
    }
    Ok(out)
}
pub fn decode_frame(bytes: &[u8]) -> Result<CommandFrame, ActuatorError> {
    if bytes.len() < 16 || bytes.len() > COMMAND_LIMIT || (bytes.len()-16)%5 != 0 || &bytes[..2] != b"TA" || bytes[2] != PROTOCOL_VERSION { return Err(error("invalid frame header or length")); }
    let kind = match bytes[3] { 1 => CommandKind::Drive, 2 => CommandKind::Arm, 3 => CommandKind::Disarm, 4 => CommandKind::EmergencyStop, _ => return Err(error("unknown command kind")) };
    let word = |offset| u32::from_le_bytes(bytes[offset..offset+4].try_into().unwrap());
    let values = bytes[16..].chunks_exact(5).map(|record| ActuatorValue { id: record[0], value: f32::from_le_bytes(record[1..5].try_into().unwrap()) }).collect();
    let frame = CommandFrame { kind, session: word(4), revision: word(8), sequence: word(12), values };
    encode_frame(&frame)?; Ok(frame)
}
/// Validate a fully decoded command without advancing sequence state on rejection.
/// The caller advances its last sequence only after safety acceptance.
pub fn validate_command(frame: &CommandFrame, session: u32, layout: &Layout, last_sequence: Option<u32>) -> Result<(), ActuatorError> {
    encode_frame(frame)?;
    if frame.session != session || frame.revision != layout.revision || frame.sequence == u32::MAX || last_sequence.is_some_and(|last| frame.sequence <= last) { return Err(error("stale session, revision or sequence")); }
    if frame.kind == CommandKind::Drive {
        if frame.values.len() != layout.actuators.len() { return Err(error("incomplete actuator coverage")); }
        for value in &frame.values {
            let actuator = layout.actuators.iter().find(|actuator| actuator.id == value.id).ok_or_else(|| error("unknown actuator"))?;
            if value.value < actuator.limits.min || value.value > actuator.limits.max { return Err(error("actuator value outside layout limits")); }
        }
    }
    Ok(())
}
#[derive(Default)]
pub struct FragmentAssembler { limit: usize, active: Option<Assembly> }
struct Assembly { id: u16, count: u8, next: u8, started: u64, data: Vec<u8> }
impl FragmentAssembler {
    pub fn new(limit: usize) -> Result<Self, ActuatorError> { if limit == 0 || limit > JSON_LIMIT { return Err(error("invalid logical limit")); } Ok(Self { limit, active: None }) }
    pub fn clear(&mut self) { self.active = None; }
    pub fn push(&mut self, bytes: &[u8], now_ms: u64) -> Result<Option<Vec<u8>>, ActuatorError> {
        let result = self.push_inner(bytes, now_ms);
        if result.is_err() { self.clear(); } result
    }
    fn push_inner(&mut self, bytes: &[u8], now: u64) -> Result<Option<Vec<u8>>, ActuatorError> {
        if self.active.as_ref().is_some_and(|a| now < a.started || now-a.started >= ASSEMBLY_TIMEOUT_MS) { self.clear(); }
        if bytes.len() < 5 { return Err(error("empty or short fragment")); }
        let id = u16::from_le_bytes([bytes[0], bytes[1]]); let index = bytes[2]; let count = bytes[3];
        if count == 0 || index >= count { return Err(error("invalid fragment index/count")); }
        if self.active.is_none() {
            if index != 0 { return Err(error("assembly must begin at index zero")); }
            self.active = Some(Assembly { id, count, next: 0, started: now, data: Vec::new() });
        }
        let a = self.active.as_mut().unwrap();
        if id != a.id || count != a.count || index != a.next { return Err(error("duplicate, conflicting or out-of-order fragment")); }
        let limit = if self.limit == 0 { COMMAND_LIMIT } else { self.limit };
        if a.data.len()+bytes.len()-4 > limit { return Err(error("logical message too large")); }
        a.data.extend_from_slice(&bytes[4..]);
        if index == count-1 { return Ok(self.active.take().map(|a| a.data)); }
        a.next += 1; Ok(None)
    }
}
/// Separate characteristic paths: priority binary control never waits for drive.
pub struct CommandAssemblers { pub drive: FragmentAssembler, pub priority: FragmentAssembler }
impl Default for CommandAssemblers { fn default() -> Self { Self { drive: FragmentAssembler::default(), priority: FragmentAssembler::default() } } }
impl CommandAssemblers {
    pub fn push_priority(&mut self, bytes: &[u8], now_ms: u64) -> Result<Option<CommandFrame>, ActuatorError> {
        let Some(data) = self.priority.push(bytes, now_ms)? else { return Ok(None) };
        let frame = decode_frame(&data)?;
        if frame.kind == CommandKind::Drive { return Err(error("drive forbidden on priority path")); }
        // Clearing is fail-safe and does not imply command/session acceptance.
        if matches!(frame.kind, CommandKind::Disarm | CommandKind::EmergencyStop) { self.drive.clear(); }
        Ok(Some(frame))
    }
}
pub fn fragment_message(data: &[u8], message_id: u16, write_size: usize, limit: usize) -> Result<Vec<Vec<u8>>, ActuatorError> {
    if data.is_empty() || limit == 0 || limit > JSON_LIMIT || data.len() > limit || write_size <= 4 { return Err(error("invalid message or negotiated write size")); }
    let chunk = write_size-4; let count = data.len().div_ceil(chunk);
    if count > 255 { return Err(error("message requires more than 255 fragments")); }
    Ok(data.chunks(chunk).enumerate().map(|(index, part)| { let mut out = message_id.to_le_bytes().to_vec(); out.extend([index as u8, count as u8]); out.extend(part); out }).collect())
}
/// Caller supplies every still outstanding ID, including transmissions awaiting completion.
pub fn allocate_message_id(next: &mut u16, active: &HashSet<u16>) -> Result<u16, ActuatorError> {
    for _ in 0..=u16::MAX { let id = *next; *next = next.wrapping_add(1); if !active.contains(&id) { return Ok(id) } } Err(error("all message IDs active"))
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EmptyPayload {}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StagePayload { pub layout: Layout }
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CommitPayload { pub staged_revision: u32 }
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "operation", content = "payload", rename_all = "snake_case", deny_unknown_fields)]
pub enum ControlOperation { Capabilities(EmptyPayload), ReadLayout(EmptyPayload), StageLayout(StagePayload), CommitLayout(CommitPayload), ResetFault(EmptyPayload), ResetEmergencyStop(EmptyPayload) }
#[derive(Clone, Debug, Serialize)]
pub struct ControlEnvelope { pub schema_version: u32, pub request_id: u32, #[serde(flatten)] pub command: ControlOperation }
// Flatten and deny_unknown_fields are intentionally avoided at the parsing boundary below.
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct EnvelopeWire { schema_version: u32, request_id: u32, operation: String, payload: Box<serde_json::value::RawValue> }
pub fn decode_control(data: &[u8]) -> Result<ControlEnvelope, ActuatorError> {
    if data.len() > JSON_LIMIT { return Err(error("JSON document too large")); }
    let wire: EnvelopeWire = serde_json::from_slice(data).map_err(|e| error(&e.to_string()))?;
    if wire.schema_version != 1 { return Err(error("unsupported control schema")); }
    let payload = wire.payload.get();
    macro_rules! parse { ($variant:ident) => { ControlOperation::$variant(serde_json::from_str(payload).map_err(|e| error(&e.to_string()))?) }; }
    let command = match wire.operation.as_str() {
        "capabilities" => parse!(Capabilities), "read_layout" => parse!(ReadLayout),
        "stage_layout" => parse!(StageLayout), "commit_layout" => parse!(CommitLayout),
        "reset_fault" => parse!(ResetFault), "reset_emergency_stop" => parse!(ResetEmergencyStop),
        _ => return Err(error("unknown operation")),
    };
    Ok(ControlEnvelope { schema_version: wire.schema_version, request_id: wire.request_id, command })
}
pub fn encode_control(envelope: &ControlEnvelope) -> Result<Vec<u8>, ActuatorError> {
    let bytes = serde_json::to_vec(envelope).map_err(|e| error(&e.to_string()))?; decode_control(&bytes)?; Ok(bytes)
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlResult { Ok, Error }
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlReply { pub schema_version: u32, pub request_id: u32, pub result: ControlResult, pub active_revision: u32, pub errors: Vec<LayoutError>, pub payload: serde_json::Value }
pub fn encode_reply(reply: &ControlReply) -> Result<Vec<u8>, ActuatorError> {
    if reply.schema_version != 1 { return Err(error("unsupported reply schema")); }
    let bytes = serde_json::to_vec(reply).map_err(|e| error(&e.to_string()))?;
    if bytes.len() > JSON_LIMIT { return Err(error("JSON reply too large")); } Ok(bytes)
}
