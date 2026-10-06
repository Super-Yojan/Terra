"""Strict BLE protocol; decoding never arms hardware or refreshes a watchdog."""
from dataclasses import dataclass
from enum import IntEnum
import json
import math
import struct

PROTOCOL_VERSION = 1
COMMAND_LIMIT = 96
JSON_LIMIT = 16 * 1024
ASSEMBLY_TIMEOUT = 0.100  # monotonic seconds

class ProtocolError(ValueError):
    pass

class CommandKind(IntEnum):
    DRIVE = 1
    ARM = 2
    DISARM = 3
    EMERGENCY_STOP = 4

@dataclass(frozen=True)
class ActuatorValue:
    id: int
    value: float

@dataclass(frozen=True)
class CommandFrame:
    kind: CommandKind
    session: int
    revision: int
    sequence: int
    values: tuple[ActuatorValue, ...] = ()

def _uint(value, bits):
    if type(value) is not int or not 0 <= value < 2**bits:
        raise ProtocolError(f"expected u{bits}")
    return value

def encode_frame(frame: CommandFrame) -> bytes:
    try:
        kind = CommandKind(frame.kind)
    except (ValueError, TypeError) as exc:
        raise ProtocolError("unknown command kind") from exc
    if len(frame.values) > 16 or (kind != CommandKind.DRIVE and frame.values):
        raise ProtocolError("invalid record count")
    result = bytearray(struct.pack('<2sBBIII', b'TA', 1, kind,
        _uint(frame.session, 32), _uint(frame.revision, 32), _uint(frame.sequence, 32)))
    ids = set()
    for record in frame.values:
        _uint(record.id, 8)
        if record.id in ids or not math.isfinite(record.value) or not -1 <= record.value <= 1:
            raise ProtocolError("invalid actuator record")
        ids.add(record.id)
        result.extend(struct.pack('<Bf', record.id, record.value))
    return bytes(result)

def decode_frame(data: bytes) -> CommandFrame:
    if not 16 <= len(data) <= COMMAND_LIMIT or (len(data)-16) % 5:
        raise ProtocolError("invalid frame length")
    magic, version, kind, session, revision, sequence = struct.unpack('<2sBBIII', data[:16])
    if magic != b'TA' or version != PROTOCOL_VERSION:
        raise ProtocolError("invalid magic or version")
    try:
        frame = CommandFrame(CommandKind(kind), session, revision, sequence,
            tuple(ActuatorValue(*record) for record in struct.iter_unpack('<Bf', data[16:])))
    except ValueError as exc:
        raise ProtocolError("unknown command kind") from exc
    encode_frame(frame)
    return frame

def validate_command(frame: CommandFrame, session: int, layout: dict, last_sequence: int | None) -> None:
    """Caller advances last_sequence only after safety acceptance."""
    encode_frame(frame)
    if frame.session != session or frame.revision != layout['revision'] or frame.sequence == 0xffffffff or (last_sequence is not None and frame.sequence <= last_sequence):
        raise ProtocolError("stale session, revision or sequence")
    if frame.kind == CommandKind.DRIVE:
        actuators = {entry['id']: entry for entry in layout['actuators']}
        if {entry.id for entry in frame.values} != set(actuators):
            raise ProtocolError("incomplete or unknown actuator coverage")
        for record in frame.values:
            limits = actuators[record.id]['limits']
            if not limits['min'] <= record.value <= limits['max']:
                raise ProtocolError("actuator value outside layout limits")

class FragmentAssembler:
    def __init__(self, limit: int = COMMAND_LIMIT):
        if type(limit) is not int or not 0 < limit <= JSON_LIMIT:
            raise ProtocolError("invalid logical limit")
        self.limit = limit
        self.clear()

    def clear(self):
        self.active = None

    def push(self, data: bytes, now: float) -> bytes | None:
        try:
            return self._push(data, now)
        except (ProtocolError, ValueError, TypeError):
            self.clear()
            raise

    def _push(self, data, now):
        if not math.isfinite(now):
            raise ProtocolError("invalid monotonic time")
        if self.active is not None and (now < self.started or now >= self.started + ASSEMBLY_TIMEOUT):
            self.clear()
        if len(data) < 5:
            raise ProtocolError("empty or short fragment")
        message_id, index, count = struct.unpack('<HBB', data[:4])
        if count == 0 or index >= count:
            raise ProtocolError("invalid fragment index/count")
        if self.active is None:
            if index != 0:
                raise ProtocolError("assembly must begin at index zero")
            self.active = (message_id, count)
            self.started, self.next, self.buffer = now, 0, bytearray()
        if self.active != (message_id, count) or index != self.next:
            raise ProtocolError("duplicate, conflicting or out-of-order fragment")
        if len(self.buffer) + len(data)-4 > self.limit:
            raise ProtocolError("logical message too large")
        self.buffer.extend(data[4:])
        if index == count-1:
            result = bytes(self.buffer)
            self.clear()
            return result
        self.next += 1
        return None

class CommandAssemblers:
    def __init__(self):
        self.drive = FragmentAssembler()
        self.priority = FragmentAssembler()

    def push_priority(self, data: bytes, now: float) -> CommandFrame | None:
        assembled = self.priority.push(data, now)
        if assembled is None:
            return None
        frame = decode_frame(assembled)
        if frame.kind == CommandKind.DRIVE:
            raise ProtocolError("drive forbidden on priority path")
        if frame.kind in (CommandKind.DISARM, CommandKind.EMERGENCY_STOP):
            self.drive.clear()  # fail-safe cancellation, not session acceptance
        return frame

def fragment_message(data: bytes, message_id: int, write_size: int, limit: int = COMMAND_LIMIT) -> list[bytes]:
    _uint(message_id, 16)
    if not data or type(write_size) is not int or write_size <= 4 or type(limit) is not int or not 0 < limit <= JSON_LIMIT or len(data) > limit:
        raise ProtocolError("invalid message or negotiated write size")
    chunk = write_size - 4
    count = (len(data) + chunk - 1) // chunk
    if count > 255:
        raise ProtocolError("message requires more than 255 fragments")
    return [struct.pack('<HBB', message_id, index, count) + data[index*chunk:(index+1)*chunk] for index in range(count)]

def allocate_message_id(next_id: int, active: set[int]) -> tuple[int, int]:
    _uint(next_id, 16)
    for _ in range(65536):
        selected, next_id = next_id, (next_id + 1) & 0xffff
        if selected not in active:
            return selected, next_id
    raise ProtocolError("all message IDs active")

def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError("duplicate JSON field")
        result[key] = value
    return result

def _bad_constant(value):
    raise ProtocolError("nonfinite JSON number")

def _keys(value, expected):
    if type(value) is not dict or set(value) != set(expected):
        raise ProtocolError("missing or unknown JSON fields")

def _json(data):
    if len(data) > JSON_LIMIT:
        raise ProtocolError("JSON document too large")
    try:
        return json.loads(data.decode('utf-8'), object_pairs_hook=_object, parse_constant=_bad_constant)
    except (UnicodeError, ValueError) as exc:
        raise ProtocolError(str(exc)) from exc

def decode_control(data: bytes) -> dict:
    envelope = _json(data)
    _keys(envelope, ('schema_version', 'request_id', 'operation', 'payload'))
    if type(envelope['schema_version']) is not int or envelope['schema_version'] != 1:
        raise ProtocolError("unsupported control schema")
    _uint(envelope['request_id'], 32)
    operation, payload = envelope['operation'], envelope['payload']
    if operation in ('capabilities', 'read_layout', 'reset_fault', 'reset_emergency_stop'):
        _keys(payload, ())
    elif operation == 'commit_layout':
        _keys(payload, ('staged_revision',))
        _uint(payload['staged_revision'], 32)
    elif operation == 'stage_layout':
        _keys(payload, ('layout',))
        _validate_layout_shape(payload['layout'])
    else:
        raise ProtocolError("unknown operation")
    return envelope

def _validate_layout_shape(layout):
    """Wire shape only; task 3 must validate resources and calibration safety."""
    _keys(layout, ('schema_version', 'revision', 'actuators'))
    _uint(layout['schema_version'], 8); _uint(layout['revision'], 32)
    if type(layout['actuators']) is not list:
        raise ProtocolError("actuators must be array")
    calibration_fields = {
        'dc_motor': ('max_power_fraction',),
        'bidirectional_esc': ('reverse_us', 'neutral_us', 'forward_us', 'arming_duration_ms'),
        'unidirectional_esc': ('stop_us', 'full_power_us', 'arming_duration_ms'),
        'positional_servo': ('min_us', 'center_us', 'max_us')}
    for entry in layout['actuators']:
        _keys(entry, ('id', 'name', 'port', 'kind', 'inverted', 'limits', 'calibration', 'route', 'safe'))
        _uint(entry['id'], 8)
        if type(entry['name']) is not str or type(entry['port']) is not str or type(entry['inverted']) is not bool or entry['kind'] not in calibration_fields:
            raise ProtocolError("invalid actuator fields")
        _keys(entry['limits'], ('min', 'max'))
        for value in entry['limits'].values(): _number(value)
        calibration = entry['calibration']; kind = calibration.get('type') if isinstance(calibration, dict) else None
        if kind not in calibration_fields: raise ProtocolError("unknown calibration")
        _keys(calibration, ('type', *calibration_fields[kind]))
        for key in calibration_fields[kind]:
            _number(calibration[key]) if key == 'max_power_fraction' else _uint(calibration[key], 32)
        route = entry['route']; route_type = route.get('type') if isinstance(route, dict) else None
        if route_type not in ('left_effort', 'right_effort', 'manual', 'servo'): raise ProtocolError("unknown route")
        _keys(route, ('type', 'forward_coefficient', 'turn_coefficient') if route_type == 'manual' else ('type',))
        if route_type == 'manual': _number(route['forward_coefficient']); _number(route['turn_coefficient'])
        safe = entry['safe']; safe_type = safe.get('type') if isinstance(safe, dict) else None
        if safe_type not in ('zero', 'position', 'disabled'): raise ProtocolError("unknown safe output")
        _keys(safe, ('type', 'value') if safe_type == 'position' else ('type',))
        if safe_type == 'position': _number(safe['value'])

def _number(value):
    if type(value) not in (int, float) or not math.isfinite(value): raise ProtocolError("expected finite number")

def encode_control(envelope: dict) -> bytes:
    try: data = json.dumps(envelope, separators=(',', ':'), allow_nan=False).encode('utf-8')
    except (ValueError, TypeError) as exc: raise ProtocolError(str(exc)) from exc
    decode_control(data)
    return data

def encode_reply(reply: dict) -> bytes:
    _keys(reply, ('schema_version', 'request_id', 'result', 'active_revision', 'errors', 'payload'))
    if type(reply['schema_version']) is not int or reply['schema_version'] != 1 or reply['result'] not in ('ok', 'error'): raise ProtocolError("invalid reply schema/result")
    _uint(reply['request_id'], 32); _uint(reply['active_revision'], 32)
    if type(reply['errors']) is not list: raise ProtocolError("errors must be array")
    for error in reply['errors']:
        _keys(error, ('actuator_id', 'code', 'message'))
        if error['actuator_id'] is not None: _uint(error['actuator_id'], 8)
        if type(error['code']) is not str or type(error['message']) is not str: raise ProtocolError("invalid error fields")
    try: data = json.dumps(reply, separators=(',', ':'), allow_nan=False).encode('utf-8')
    except (ValueError, TypeError) as exc: raise ProtocolError(str(exc)) from exc
    if len(data) > JSON_LIMIT: raise ProtocolError("JSON reply too large")
    return data
