"""Monotonic, explicit-arm safety controller. Transport supplies authenticated frames."""
from copy import deepcopy
from dataclasses import dataclass
import math
import struct
from .layout import validate_layout, validate_structure
from .protocol import CommandFrame, CommandKind, ProtocolError, validate_command

WATCHDOG_SECONDS = 0.200

@dataclass(frozen=True)
class OutputCommand:
    id: int
    value: float | None  # None means disable servo pulses; propulsion always receives zero.

@dataclass(frozen=True)
class OutputBatch:
    commands: tuple[OutputCommand, ...]
    armed: bool
    arming: bool
    reason: str | None
    fault: str | None
    emergency_stop: bool

class SafetyController:
    def __init__(self, capabilities: dict | None = None):
        self.capabilities = deepcopy(capabilities)
        self.layout = None
        self.session = None
        self.subscribed = False
        self.hardware_gate = False
        self.last_sequence = None
        self.last_time = None
        self.fault = None
        self.emergency_stop = False
        self._disarm('boot')

    def _disarm(self, reason):
        self.armed = False
        self.arming = False
        self.reason = reason
        self.mailbox = None
        self.safe_received = None
        self.arm_deadline = None

    @staticmethod
    def _finite_time(value):
        try:
            return type(value) in (int, float) and math.isfinite(value)
        except (OverflowError, ValueError, TypeError):
            return False

    def _clock(self, now):
        if not self._finite_time(now):
            self.fault = 'invalid_clock'
            self._disarm(self.fault)
            return False
        if self.last_time is not None and now < self.last_time:
            self.fault = 'clock_regression'
            self._disarm(self.fault)
            return False
        self.last_time = now
        return True

    def _expire(self, now):
        stamp = self.mailbox[1] if self.mailbox else self.safe_received
        if stamp is not None and now >= stamp + WATCHDOG_SECONDS:
            self._disarm('watchdog')

    def configure(self, layout: dict, now: float):
        if not self._clock(now): raise ValueError('invalid clock')
        if self.armed or self.arming: raise ValueError('configuration requires disarmed state')
        errors = validate_layout(layout, self.capabilities) if self.capabilities is not None else validate_structure(layout)
        if errors: raise ValueError(errors)
        self.layout = deepcopy(layout)
        self._disarm('configuration_changed')

    def connect(self, session: int, now: float):
        if type(session) is not int or not 0 <= session < 2**32: raise ValueError('invalid session')
        self._clock(now)
        self.session = session
        self.subscribed = False
        self.hardware_gate = False
        self.last_sequence = None
        self._disarm('connected')

    def set_status_subscribed(self, subscribed: bool, now: float):
        if not self._clock(now): return
        self.subscribed = bool(subscribed)
        if not self.subscribed: self._disarm('status_unsubscribed')

    def accept(self, frame: CommandFrame, received_at: float, now: float) -> bool:
        if not self._clock(now): return False
        self._expire(now)
        if not self._finite_time(received_at) or received_at > now or now >= received_at + WATCHDOG_SECONDS:
            return False
        if self.session is None or self.layout is None: return False
        try:
            validate_command(frame, self.session, self.layout, self.last_sequence)
        except (ProtocolError, ValueError, TypeError, OverflowError):
            return False
        if frame.kind == CommandKind.EMERGENCY_STOP:
            self.emergency_stop = True
            self._disarm('emergency_stop')
        elif frame.kind == CommandKind.DISARM:
            self._disarm('requested')
        elif frame.kind == CommandKind.ARM:
            if self.armed or self.arming or not self.subscribed or not self.hardware_gate or self.fault or self.emergency_stop or self.safe_received is None:
                return False
            durations = [a['calibration'].get('arming_duration_ms', 0) for a in self.layout['actuators']]
            self.arming = True
            self.reason = 'arming'
            self.arm_deadline = now + max(durations, default=0) / 1000
        else:
            if self.fault or self.emergency_stop: return False
            if self.armed:
                self.mailbox = (frame.values, received_at)
            else:
                if not self._safe_frame(frame): return False
                self.safe_received = received_at
        self.last_sequence = frame.sequence
        return True

    def _safe_frame(self, frame):
        entries = {a['id']: a for a in self.layout['actuators']}
        for record in frame.values:
            safe = entries[record.id]['safe']
            target = safe.get('value', 0)
            if safe['type'] == 'disabled':
                limits = entries[record.id]['limits']
                target = max(limits['min'], min(limits['max'], 0))
            if record.value != struct.unpack('<f', struct.pack('<f', target))[0]: return False
        return True

    def tick(self, now: float, hardware_gate: bool) -> OutputBatch:
        valid_clock = self._clock(now)
        self.hardware_gate = bool(hardware_gate)
        if not self.hardware_gate: self._disarm('hardware_gate_open')
        if valid_clock:
            self._expire(now)
            if self.arming and now >= self.arm_deadline:
                self.arming = False
                self.armed = True
                self.reason = 'awaiting_command'
                self.mailbox = None  # only a frame accepted after completion may actuate
        commands = self._safe_outputs()
        if self.armed and self.mailbox:
            commands = tuple(OutputCommand(v.id, v.value) for v in self.mailbox[0])
        return OutputBatch(commands, self.armed, self.arming, self.reason, self.fault, self.emergency_stop)

    def _safe_outputs(self):
        return tuple(OutputCommand(a['id'], None if a['safe']['type'] == 'disabled' else a['safe'].get('value', 0.0)) for a in (self.layout or {'actuators': []})['actuators'])

    def disconnect(self, now: float):
        self._clock(now)
        self.session = None
        self.last_sequence = None
        self.subscribed = False
        self.hardware_gate = False
        self._disarm('disconnected')

    def latch_fault(self, reason: str, now: float):
        self._clock(now)
        self.fault = str(reason)
        self._disarm(self.fault)

    def reset_fault(self, now: float):
        if not self._clock(now): return
        self.fault = None
        self._disarm('fault_reset')

    def reset_emergency_stop(self, now: float):
        if not self._clock(now): return
        self.emergency_stop = False
        self._disarm('emergency_stop_reset')

    def status(self, now: float) -> dict:
        if self._clock(now): self._expire(now)
        return dict(schema_version=1, session=self.session, active_revision=self.layout['revision'] if self.layout else None,
                    armed=self.armed, arming=self.arming, hardware_gate=self.hardware_gate,
                    status_subscribed=self.subscribed, fault=self.fault, emergency_stop=self.emergency_stop,
                    stop_reason=self.reason, last_sequence=self.last_sequence)
