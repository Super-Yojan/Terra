"""Output adapters. Importing this module never imports the hardware library."""
from copy import deepcopy
from importlib import import_module, metadata
import math
import traceback
from pathlib import Path
from typing import Protocol

from .layout import normalize_layout, validate_layout
from .safety import OutputBatch

KINDS = ['dc_motor', 'bidirectional_esc', 'unidirectional_esc', 'positional_servo']
MOTOR_PINS = {'M0': ['P11', 'P10'], 'M1': ['P9', 'P8'],
              'M2': ['P6', 'P7'], 'M3': ['P4', 'P5']}

class BackendError(RuntimeError):
    def __init__(self, port, operation, message):
        self.port, self.operation = port, operation
        super().__init__(f'{operation} on {port or "backend"}: {message}')

class BackendValidationError(BackendError):
    """Rejected before any hardware resource or output is changed."""

def _forget_exception_owners(exc):
    """Release vendor frames/partial constructor owners before resource transfer."""
    seen = set()
    pending = [exc]
    while pending:
        current = pending.pop()
        if id(current) in seen: continue
        seen.add(id(current))
        pending.extend(child for child in (current.__cause__, current.__context__) if child is not None)
        if current.__traceback__ is not None:
            traceback.clear_frames(current.__traceback__)
        current.__traceback__ = None
        current.__cause__ = None
        current.__context__ = None

class Backend(Protocol):
    def capabilities(self) -> dict: ...
    def configure(self, layout: dict) -> None: ...
    def apply(self, batch: OutputBatch) -> None: ...
    def read_gate(self) -> bool: ...
    def require_gate_open(self) -> None: ...
    def close(self) -> None: ...

def file_gate_reader(path):
    """Only an exact ASCII 1 (with surrounding whitespace) denotes closed gate."""
    gate_path = Path(path)
    def read():
        token = gate_path.read_text(encoding='ascii').strip()
        if token not in ('0', '1'): raise ValueError('gate file must contain 0 or 1')
        return token == '1'
    return read

def _capabilities(pwm_ports, library, version, occupied_resources):
    ports = {}
    for name, resources in MOTOR_PINS.items():
        ports[name] = dict(kinds=['dc_motor'], resources=resources[:],
                           timer=f'pwm-timer-{int(resources[0][1:]) // 4}', frequency_hz=100)
    for name in pwm_ports:
        if name not in {f'P{i}' for i in range(12)}:
            raise BackendError(name, 'capabilities', 'expected canonical P0 through P11')
        ports[name] = dict(kinds=KINDS[1:], resources=[name],
                           timer=f'pwm-timer-{int(name[1:]) // 4}', frequency_hz=50)
    return dict(board='Fusion HAT', library=library, library_version=version,
                supported_kinds=KINDS[:], ports=ports, occupied_resources=list(occupied_resources))

def _safe(a):
    if a['safe']['type'] == 'disabled': return None
    if a['safe']['type'] == 'position': return a['safe']['value']
    return 0.0

def _mapped(a, value):
    kind, c = a['kind'], a['calibration']
    if value is None:
        if kind != 'positional_servo': raise ValueError('only servo can disable pulses')
        return None
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('output must be finite')
    if not a['limits']['min'] <= value <= a['limits']['max']:
        raise ValueError('output outside configured limits')
    if a['inverted']:
        # For unidirectional ESC inversion changes direction neither electrically
        # nor numerically: its zero must always remain stop.
        if kind == 'unidirectional_esc':
            raise ValueError('unidirectional ESC cannot be inverted')
        value = -value
    if kind == 'dc_motor': return value * c['max_power_fraction'] * 100
    if kind == 'unidirectional_esc': return round(c['stop_us'] + value * (c['full_power_us'] - c['stop_us']))
    low, center, high = ((c['reverse_us'], c['neutral_us'], c['forward_us'])
                         if kind == 'bidirectional_esc' else (c['min_us'], c['center_us'], c['max_us']))
    return round(center + value * (high - center if value >= 0 else center - low))

class _Adapter:
    def __init__(self, caps, gate_reader):
        self._caps, self._gate_reader = caps, gate_reader
        self._layout = None
        self._outputs = {}
        self._closed = False

    def capabilities(self): return deepcopy(self._caps)

    def read_gate(self):
        if self._closed or self._gate_reader is None: return False
        try: return self._gate_reader() is True
        except Exception: return False

    def require_gate_open(self):
        if self._closed or self._gate_reader is None:
            raise BackendError(None, 'gate', 'gate state unavailable')
        try:
            state = self._gate_reader()
        except Exception as exc:
            raise BackendError(None, 'gate', 'gate state unreadable') from exc
        if state is not False:
            raise BackendError(None, 'gate', 'gate must be confirmed open')

    def configure(self, layout):
        if self._closed: raise BackendError(None, 'configure', 'backend closed')
        errors = validate_layout(layout, self._caps)
        if errors: raise BackendValidationError(None, 'configure', str(errors))
        candidate = normalize_layout(layout)
        self._before_configure()
        self._prepare()
        previous = self._layout
        if previous is not None:
            self._release_for_configure()
        self._layout = candidate
        failure = None
        failed_port = None
        try:
            for a in candidate['actuators']:
                self._open(a)
                self._write(a, _mapped(a, _safe(a)))
        except Exception as exc:
            failure, failed_port = str(exc), a['port']
            _forget_exception_owners(exc)
        # Exit the handler before retiring resources or opening replacements.
        # Only diagnostic strings survive; vendor constructor/write frames must
        # not retain a PWM whose destructor can later disable its replacement.
        if failure is not None:
            failures = self._safe_all()
            if previous is not None:
                try:
                    self._release_for_configure()
                    self._layout = previous
                    for old in previous['actuators']:
                        self._open(old)
                        self._write(old, _mapped(old, _safe(old)))
                except Exception as rollback:
                    failures.append(f'rollback: {rollback}')
                    _forget_exception_owners(rollback)
                    failures.extend(self._safe_all())
            raise BackendError(failed_port, 'configure', f'{failure}; safe failures: {failures}') from None

    def _before_configure(self): pass

    def _release_for_configure(self):
        failures = self._safe_all()
        if failures: raise BackendError(None, 'configure', str(failures))
        self._outputs.clear()

    def _prepare(self): pass
    def _open(self, a): pass

    def _safe_all(self):
        failures = []
        for a in (self._layout or {}).get('actuators', []):
            if isinstance(self, FusionHatBackend) and a['port'] not in self._outputs:
                continue
            try: self._write(a, _mapped(a, _safe(a)))
            except Exception as exc: failures.append(f'{a["port"]}: {exc}')
            # Motor.power can fail between its two physical leg writes.
            # Attempt both legs separately even after a successful zero write.
            if a['kind'] == 'dc_motor' and a['port'] in self._outputs:
                output = self._outputs[a['port']]
                for leg in ('pwm_a', 'pwm_b'):
                    try: getattr(output, leg).pulse_width_percent(0)
                    except Exception as exc: failures.append(f'{a["port"]}/{leg}: {exc}')
        return failures

    def apply(self, batch: OutputBatch):
        if self._closed or self._layout is None:
            raise BackendError(None, 'apply', 'backend not configured or closed')
        port = None
        try:
            values = {c.id: c.value for c in batch.commands}
            entries = self._layout['actuators']
            if len(values) != len(batch.commands) or set(values) != {a['id'] for a in entries}:
                raise ValueError('batch must contain each configured ID exactly once')
            active = batch.armed and not batch.arming and not batch.fault and not batch.emergency_stop
            if active and not self.read_gate():
                raise ValueError('physical gate is open or unreadable')
            mapped = [(a, _mapped(a, values[a['id']] if active else _safe(a))) for a in entries]
            for a, value in mapped:
                port = a['port']
                self._write(a, value)
        except Exception as exc:
            failure = str(exc)
            _forget_exception_owners(exc)
        else:
            return
        failures = self._safe_all()
        raise BackendError(port, 'apply', f'{failure}; safe failures: {failures}') from None

    def close(self):
        failures = self._safe_all()
        # Keep ESC pulses enabled at safe stop/neutral. Hardware shutdown/power
        # isolation is separate; PWM.close would disable these safety pulses.
        self._closed = True
        if failures: raise BackendError(None, 'close', '; '.join(failures))

class MockBackend(_Adapter):
    def __init__(self, pwm_ports=(), occupied_resources=()):
        self.gate = False
        self.applied = {}
        self.history = []
        self.fail_ports = set()
        super().__init__(_capabilities(pwm_ports, 'mock', '1', occupied_resources), lambda: self.gate)

    def _write(self, a, value):
        if a['port'] in self.fail_ports: raise IOError('injected output failure')
        self.applied[a['id']] = value
        self.history.append((a['id'], a['port'], value))

class FusionHatBackend(_Adapter):
    def __init__(self, pwm_ports=(), gate_reader=None, *, supported_library_versions=('1.14.0',),
                 motor_factory=None, pwm_factory=None, library_version=None, occupied_resources=()):
        self._versions = tuple(supported_library_versions)
        self._motor_factory, self._pwm_factory = motor_factory, pwm_factory
        self._version = library_version
        super().__init__(_capabilities(pwm_ports, 'fusion_hat', library_version or 'unloaded', occupied_resources), gate_reader)

    def _before_configure(self):
        try:
            self.require_gate_open()
        except BackendError as exc:
            raise BackendValidationError(None, 'gate', str(exc)) from exc

    def _release_for_configure(self):
        self.require_gate_open()
        failures = self._safe_all()
        for port, output in self._outputs.items():
            legs = (output.pwm_a, output.pwm_b) if port in MOTOR_PINS else (output,)
            for leg in legs:
                try: leg.close()
                except Exception as exc: failures.append(f'{port}: {exc}')
        if failures: raise BackendError(None, 'configure', str(failures))
        self._outputs.clear()

    def _prepare(self):
        try:
            version = self._version or metadata.version('fusion_hat')
            if not self._versions or version not in self._versions:
                raise ValueError(f'library version {version} is not deployment-approved')
            motor = self._motor_factory or import_module('fusion_hat.motor').Motor
            pwm = self._pwm_factory or import_module('fusion_hat.pwm').PWM
            if getattr(motor, 'MOTOR_PINS', None) != MOTOR_PINS or getattr(motor, 'DEFAULT_FREQ', None) != 100:
                raise ValueError('unsupported motor resource/frequency map')
            if getattr(pwm, 'CHANNEL_NUM', None) != 12:
                raise ValueError('unsupported PWM channel map')
            for cls, methods in ((motor, ('power',)), (pwm, ('freq', 'pulse_width', 'pulse_width_percent', 'enable', 'close'))):
                if not all(callable(getattr(cls, name, None)) for name in methods):
                    raise ValueError('unsupported output API')
            self._motor_factory, self._pwm_factory = motor, pwm
            self._caps['library_version'] = version
        except Exception as exc:
            raise BackendError(None, 'library', str(exc)) from exc

    def _open(self, a):
        if a['kind'] == 'dc_motor':
            output = self._motor_factory(a['port'], freq=100, min=0, max=100, is_reversed=False)
        else:
            output = self._pwm_factory(int(a['port'][1:]), freq=50)
        self._outputs[a['port']] = output

    def _write(self, a, value):
        output = self._outputs[a['port']]
        if a['kind'] == 'dc_motor': output.power(value)
        elif value is None: output.enable(False)
        else:
            output.pulse_width(value)
            output.enable(True)
