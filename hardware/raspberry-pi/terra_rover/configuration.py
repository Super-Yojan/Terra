"""Disarmed configuration transactions; callers serialize this with the output worker."""
from copy import deepcopy
import json
import os
from pathlib import Path
import tempfile

from .backend import BackendValidationError
from .layout import normalize_layout, validate_layout
from .protocol import JSON_LIMIT, ProtocolError, _json, encode_control

CACHE_LIMIT = 256


def _error(code, message):
    return dict(actuator_id=None, code=code, message=str(message)[:512])


class ConfigurationStore:
    """No constructor I/O. Call load(now) before exposing the BLE service.

    A single service lock must cover every store, safety, and backend operation.
    Only authenticated owner connections may invoke begin_connection/handle.
    """
    def __init__(self, path, safety, backend):
        self.path = Path(path)
        self.safety = safety
        self.backend = backend
        self._active = None
        self._revision = 0
        self._stage = None
        self._connected = False
        self._cache = {}
        self.errors = []
        self._loaded = False

    @property
    def active_revision(self):
        return self._revision

    def read(self):
        return deepcopy(self._active)

    def _validate(self, layout):
        candidate = normalize_layout(layout)
        errors = validate_layout(candidate, self.backend.capabilities())
        if errors:
            return None, errors
        # Bounds apply to the persisted layout as well as the enclosing request.
        if len(self._serialize(candidate)) > JSON_LIMIT:
            raise ProtocolError('layout exceeds JSON size limit')
        return candidate, []

    @staticmethod
    def _serialize(layout):
        return json.dumps(layout, ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode('utf-8')

    def _read_file(self):
        with self.path.open('rb') as stream:
            data = stream.read(JSON_LIMIT + 1)
        candidate, errors = self._validate(_json(data))
        if errors:
            raise ValueError(errors)
        return candidate

    def _gate_errors(self, now):
        status = self.safety.status(now)
        if status['armed'] or status['arming']:
            return [_error('configuration_armed', 'configuration requires disarmed state')]
        if self.backend.capabilities().get('library') != 'mock':
            try: self.backend.require_gate_open()
            except Exception as exc: return [_error('hardware_gate', exc)]
        if status['fault'] in ('invalid_clock', 'clock_regression'):
            return [_error('clock', status['fault'])]
        return []

    def load(self, now):
        """Load once at startup. Corruption faults without configuring hardware."""
        if self._loaded:
            raise RuntimeError('configuration store already loaded')
        self._loaded = True
        try:
            candidate = self._read_file()
        except FileNotFoundError:
            return None
        except Exception as exc:
            self.errors = [_error('persisted_layout', exc)]
            self.safety.latch_fault('invalid_persisted_layout', now)
            return None
        self._revision = candidate['revision']
        errors = self._gate_errors(now)
        if errors:
            self.errors = errors
            self.safety.latch_fault('boot_configuration_failed', now)
            return None
        try:
            self.backend.configure(candidate)
            self.safety.configure(candidate, now)
        except Exception as exc:
            self.errors = [_error('backend_configuration', exc)]
            self.safety.latch_fault('boot_configuration_failed', now)
            return None
        self._active = candidate
        self._revision = candidate['revision']
        self.errors = []
        return self.read()

    def begin_connection(self, session, now):
        self.safety.connect(session, now)
        self._connected = True
        self._stage = None
        self._cache.clear()

    def end_connection(self, now):
        self._connected = False
        self._stage = None
        self._cache.clear()
        self.safety.disconnect(now)

    def _reply(self, request_id, errors=(), payload=None):
        return dict(schema_version=1, request_id=request_id,
                    result='error' if errors else 'ok', active_revision=self.active_revision,
                    errors=deepcopy(list(errors)), payload=deepcopy(payload))

    def stage(self, request_id, layout, expected_revision=None, now=None):
        """Direct convenience wrapper; layout.revision is the expected base revision."""
        if expected_revision is not None and layout.get('revision') != expected_revision:
            raise ProtocolError('expected revision disagrees with layout.revision')
        if now is None:
            raise ValueError('monotonic now is required')
        return self.handle(dict(schema_version=1, request_id=request_id, operation='stage_layout',
                                payload=dict(layout=layout)), now)

    def commit(self, request_id, now, *, staged_request_id, staged_revision):
        return self.handle(dict(schema_version=1, request_id=request_id, operation='commit_layout',
                                payload=dict(staged_request_id=staged_request_id,
                                             staged_revision=staged_revision)), now)

    def handle_bytes(self, data, now):
        from .protocol import decode_control
        return self.handle(decode_control(data), now)

    def handle(self, envelope, now):
        # Serialize then decode through strict protocol boundary, including direct callers.
        from .protocol import decode_control
        envelope = decode_control(encode_control(envelope))
        request_id = envelope['request_id']
        fingerprint = encode_control(envelope)
        if not self._connected:
            return self._reply(request_id, [_error('connection', 'authenticated connection required')])
        if request_id in self._cache:
            original, reply = self._cache[request_id]
            if original != fingerprint:
                return self._reply(request_id, [_error('request_id_reused', 'request ID already used for a different request')])
            return deepcopy(reply)
        if len(self._cache) >= CACHE_LIMIT:
            return self._reply(request_id, [_error('request_cache_full', 'reconnect with a new session before further requests')])
        if not self._loaded:
            reply = self._reply(request_id, [_error('not_loaded', 'load configuration before accepting requests')])
        else:
            try:
                reply = self._dispatch(envelope, now)
            except (ProtocolError, ValueError, TypeError, OverflowError) as exc:
                reply = self._reply(request_id, [_error('configuration', exc)])
        self._cache[request_id] = (fingerprint, deepcopy(reply))
        return reply

    def _dispatch(self, envelope, now):
        request_id, operation, payload = envelope['request_id'], envelope['operation'], envelope['payload']
        if operation == 'capabilities':
            return self._reply(request_id, payload=self.backend.capabilities())
        if operation == 'read_layout':
            return self._reply(request_id, self.errors, self.read())
        errors = self._gate_errors(now)
        if errors:
            return self._reply(request_id, errors)
        if operation == 'reset_fault':
            # An unavailable/uncertain hardware layout cannot become armable by reset.
            if self.errors:
                return self._reply(request_id, self.errors)
            self.safety.reset_fault(now)
            return self._reply(request_id)
        if operation == 'reset_emergency_stop':
            self.safety.reset_emergency_stop(now)
            return self._reply(request_id)
        if operation == 'stage_layout':
            candidate, errors = self._validate(payload['layout'])
            if errors:
                return self._reply(request_id, errors)
            if candidate['revision'] != self.active_revision:
                return self._reply(request_id, [_error('stale_revision', 'layout.revision must equal active revision')])
            if candidate['revision'] == 0xffffffff:
                return self._reply(request_id, [_error('revision_exhausted', 'layout revision cannot increment')])
            self._stage = (request_id, candidate)
            return self._reply(request_id, payload=dict(staged_request_id=request_id, staged_revision=candidate['revision']))
        if self._stage is None or payload['staged_request_id'] != self._stage[0] or payload['staged_revision'] != self._stage[1]['revision']:
            return self._reply(request_id, [_error('staged_layout', 'commit must reference the exact pending stage')])
        candidate, errors = self._validate(self._stage[1])
        if errors:
            return self._reply(request_id, errors)
        if candidate['revision'] != self.active_revision:
            return self._reply(request_id, [_error('stale_revision', 'active layout changed after staging')])
        candidate['revision'] += 1
        self._stage = None  # consumed even on failure; retry requires a new stage
        return self._commit_candidate(request_id, candidate, now)

    def _persist(self, candidate):
        # Parent must be provisioned by deployment; never silently create a new path.
        descriptor, temporary = tempfile.mkstemp(prefix='.' + self.path.name + '.', dir=self.path.parent)
        try:
            with os.fdopen(descriptor, 'wb') as stream:
                stream.write(self._serialize(candidate))
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            directory = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass

    def _commit_candidate(self, request_id, candidate, now):
        previous = self.read()
        try:
            self.backend.configure(candidate)
        except BackendValidationError as exc:
            # Preflight has not touched resources: keep the valid layout and fault state.
            return self._reply(request_id, [_error('backend_validation', exc)])
        except Exception as exc:
            # Adapter owns resource rollback. Do not open replacement resources here.
            self.errors = [_error('backend_configuration', exc)]
            # Adapter failure can include failed rollback; no matching layout is assured.
            self._active = None
            self.safety.layout = None
            self.safety.latch_fault('backend_configuration_failed', now)
            return self._reply(request_id, self.errors)
        try:
            self._persist(candidate)
        except Exception as exc:
            errors = [_error('persistence', exc)]
            try:
                try:
                    durable = self._read_file()
                except FileNotFoundError:
                    if previous is not None:
                        raise ValueError('previous persisted layout disappeared')
                    durable = None
                if durable != previous and durable != candidate:
                    raise ValueError('persisted file is neither previous nor candidate layout')
                # configure owns safe retirement and rollback; never construct outputs here.
                reconciled = durable if durable is not None else dict(schema_version=1, revision=0, actuators=[])
                self.backend.configure(reconciled)
                self.safety.configure(reconciled, now)
                self._active = durable
                if durable is not None:
                    self._revision = durable['revision']
                if durable is None:
                    self.safety.layout = None
            except Exception as rollback:
                # Retire this attempted revision when file/backend agreement is unknown.
                self._revision = max(self._revision, candidate['revision'])
                errors.append(_error('reconciliation', rollback))
                self._active = None
                self.safety.layout = None
                self.safety.latch_fault('configuration_reconciliation_failed', now)
            else:
                # Directory fsync failure can leave the new file visible but durability unknown.
                self.safety.latch_fault('configuration_persistence_failed', now)
            self.errors = errors
            return self._reply(request_id, errors, self.read())
        try:
            self.safety.configure(candidate, now)
        except Exception as exc:
            self._revision = candidate['revision']
            self._active = None
            self.safety.layout = None
            self.errors = [_error('reconciliation', exc)]
            self.safety.latch_fault('configuration_reconciliation_failed', now)
            return self._reply(request_id, self.errors)
        self._active = candidate
        self._revision = candidate['revision']
        self.errors = []
        return self._reply(request_id, payload=self.read())

    def status(self, now):
        result = self.safety.status(now)
        result['active_revision'] = self.active_revision
        result['layout_available'] = self._active is not None and self.safety.layout is not None
        try:
            self.backend.require_gate_open()
            result['hardware_gate_open_confirmed'] = True
        except Exception:
            result['hardware_gate_open_confirmed'] = False
        result.update(service_state='fault' if result['fault'] else 'armed' if result['armed'] else 'arming' if result['arming'] else 'disarmed',
                      configuration_errors=deepcopy(self.errors), battery=None, battery_reason='unsupported')
        return result
