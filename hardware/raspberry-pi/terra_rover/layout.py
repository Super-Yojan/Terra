"""Layout validation matching the portable Rust layout schema."""
from .protocol import ProtocolError, _validate_layout_shape


def validate_structure(layout: dict) -> list[dict]:
    errors = []
    def add(identifier, code, message):
        errors.append(dict(actuator_id=identifier, code=code, message=message))
    try:
        _validate_layout_shape(layout)
    except ProtocolError as exc:
        add(None, 'schema_version', str(exc))
        return errors
    if layout['schema_version'] != 1:
        add(None, 'schema_version', 'schema version must be 1')
    if len(layout['actuators']) > 16:
        add(None, 'actuator_count', 'at most 16 actuators are supported')
    ids = set()
    for a in layout['actuators']:
        identifier, kind = a['id'], a['kind']
        if identifier in ids: add(identifier, 'duplicate_id', 'actuator IDs must be unique')
        ids.add(identifier)
        if not a['port'].strip(): add(identifier, 'port', 'port must not be empty')
        lo, hi = a['limits']['min'], a['limits']['max']
        if not (0 if kind == 'unidirectional_esc' else -1) <= lo <= hi <= 1:
            add(identifier, 'command_limits', 'limits outside normalized range or unordered')
        c = a['calibration']
        valid = c['type'] == kind
        if valid:
            if kind == 'dc_motor': valid = 0 < c['max_power_fraction'] <= 1
            elif kind == 'bidirectional_esc': valid = 0 < c['reverse_us'] < c['neutral_us'] < c['forward_us'] < 20000
            elif kind == 'unidirectional_esc': valid = 0 < c['stop_us'] < c['full_power_us'] < 20000
            else: valid = 0 < c['min_us'] < c['center_us'] < c['max_us'] < 20000
        if not valid: add(identifier, 'calibration', 'calibration must match kind and have valid power or pulses')
        servo = kind == 'positional_servo'
        if servo != (a['route']['type'] == 'servo'): add(identifier, 'route_kind', 'route must match actuator kind')
        if a['route']['type'] == 'manual' and not all(-1 <= a['route'][key] <= 1 for key in ('forward_coefficient', 'turn_coefficient')):
            add(identifier, 'route_coefficients', 'manual coefficients must lie in [-1, 1]')
        safe = a['safe']
        valid = (safe['type'] == 'zero' and not servo and lo <= 0 <= hi or
                 safe['type'] == 'disabled' and servo or
                 safe['type'] == 'position' and servo and lo <= safe['value'] <= hi and -1 <= safe['value'] <= 1)
        if not valid: add(identifier, 'safe_output', 'safe policy must match kind and limits')
    return errors


def validate_layout(layout: dict, capabilities: dict) -> list[dict]:
    errors = validate_structure(layout)
    try:
        _validate_layout_shape(layout)
    except ProtocolError:
        return errors
    def add(a, code, message):
        errors.append(dict(actuator_id=a['id'], code=code, message=message))
    resources = set(capabilities['occupied_resources'])
    ports, timers = set(), {}
    for a in layout['actuators']:
        if a['port'] in ports: add(a, 'resource_conflict', 'port is already assigned')
        ports.add(a['port'])
        port = capabilities['ports'].get(a['port'])
        if port is None:
            add(a, 'unsupported_port', 'port is not advertised by backend')
            continue
        if a['kind'] not in capabilities['supported_kinds'] or a['kind'] not in port['kinds']:
            add(a, 'unsupported_kind', 'output kind unsupported on this port')
        if not port['resources']: add(a, 'resource_map', 'physical resource metadata required')
        own = set()
        for resource in port['resources']:
            if not resource or resource in own:
                add(a, 'resource_map', 'resource identifiers must be nonempty and unique')
                continue
            own.add(resource)
            if resource in resources: add(a, 'resource_conflict', 'physical resource occupied or shared')
            resources.add(resource)
        frequency = port['frequency_hz']
        if type(frequency) is not int or frequency <= 0 or (a['kind'] != 'dc_motor' and frequency != 50):
            add(a, 'frequency', 'ESC and servo channels require 50 Hz; motor frequency must be nonzero')
        timer = port['timer']
        if timer is not None:
            if timer in timers and timers[timer] != frequency: add(a, 'timer_conflict', 'shared timer frequencies must match')
            timers[timer] = frequency
    return errors
