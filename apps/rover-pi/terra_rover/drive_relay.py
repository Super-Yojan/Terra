"""Hold the latest actuator efforts for a short lease; then send neutral."""
from copy import deepcopy
from dataclasses import dataclass
import logging
import time

from .layout import normalize_layout, validate_layout, validate_structure
from .observability import record_event
from .protocol import ProtocolError, decode_drive, encode_drive, validate_drive

# How long one drive command keeps the outputs moving. The phone streams at
# 20 Hz (every 50 ms) and the output loop also runs every 50 ms, so a lease of
# a single loop turned every late Bluetooth packet into a stop and made the
# rover jitter. 250 ms matches the Zenoh cmd_vel lease. An explicit neutral
# command, disconnect, fault or configuration change still stops at once.
DRIVE_COMMAND_TIMEOUT_SECONDS = 0.25


@dataclass(frozen=True)
class OutputCommand:
    id: int
    value: float | None


@dataclass(frozen=True)
class OutputBatch:
    commands: tuple[OutputCommand, ...]


class DriveRelay:
    def __init__(self, capabilities=None):
        self.capabilities = deepcopy(capabilities)
        self.layout = None
        self.connected = False
        self.output_active = False
        self.subscribed = False
        self.fault = None
        self.values = None
        self.received_at = None
        self.reason = 'boot'

    def configure(self, layout):
        candidate = normalize_layout(layout)
        errors = validate_layout(candidate, self.capabilities) if self.capabilities is not None else validate_structure(candidate)
        if errors: raise ValueError(errors)
        self.stop('configuration_changed')
        self.layout = candidate

    def connect(self):
        self.stop('connected')
        self.connected = True
        self.subscribed = False

    def stop(self, reason='requested'):
        if self.values is not None or self.output_active or self.reason != reason:
            record_event('relay', 'motion.stop', {'reason': reason,
                         'previous_active': self.output_active})
        self.values = None
        self.received_at = None
        self.output_active = False
        self.reason = reason

    def set_status_subscribed(self, subscribed):
        self.subscribed = bool(subscribed)
        if not subscribed: self.stop('status_unsubscribed')

    def accept(self, values, now=None):
        accepted = False
        if self.connected and self.layout is not None and self.fault is None:
            try:
                values = decode_drive(encode_drive(values))
                validate_drive(values, self.layout)
            except (ProtocolError, ValueError, TypeError, AttributeError, OverflowError):
                pass
            else:
                self.values = values
                self.received_at = time.monotonic() if now is None else now
                self.reason = 'drive'
                accepted = True
        if not accepted: self.stop('rejected_command')
        record_event('bluetooth', 'command.accepted' if accepted else 'command.rejected',
                     {'actuators': [dict(id=v.id, effort=v.value) for v in values] if accepted else [],
                      'stop_reason': self.reason, 'fault': self.fault})
        return accepted

    def output(self, now=None):
        entries = (self.layout or {'actuators': []})['actuators']
        commands = {a['id']: OutputCommand(a['id'], None if a['safe']['type'] == 'disabled' else a['safe'].get('value', 0.0))
                    for a in entries}
        now = time.monotonic() if now is None else now
        values = self.values
        if values is not None and now - self.received_at > DRIVE_COMMAND_TIMEOUT_SECONDS:
            values = self.values = self.received_at = None
        if values is not None and self.connected and self.fault is None:
            commands.update((v.id, OutputCommand(v.id, v.normalized)) for v in values)
            self.output_active = any(c.value not in (None, 0) for c in commands.values())
        else:
            if self.reason == 'drive': self.stop('no_command')
            self.output_active = False
        return OutputBatch(tuple(commands.values()))

    def disconnect(self):
        self.stop('disconnected')
        self.connected = False
        self.output_active = False
        self.subscribed = False

    def latch_fault(self, reason):
        if self.fault != str(reason):
            logging.getLogger(__name__).error('Rover fault: %s', reason)
        self.fault = str(reason)
        self.stop(self.fault)

    def reset_fault(self):
        self.fault = None
        self.stop('fault_reset')

    def status(self):
        return dict(schema_version=1, connected=self.connected,
                    active_revision=self.layout['revision'] if self.layout else None,
                    control_mode='streamed_relay', output_active=self.output_active,
                    status_subscribed=self.subscribed, fault=self.fault,
                    stop_reason=self.reason)
