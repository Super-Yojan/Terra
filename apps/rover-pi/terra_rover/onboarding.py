"""Physical enrollment controls and durable, exclusive device identity."""
import json
import os
from pathlib import Path
import re
import secrets
import stat
import string
import tempfile

class HoldDetector:
    def __init__(self):
        self.raw = None
        self.changed = 0
        self.stable = None
        self.released = False
        self.started = None

    def update(self, pressed, now):
        if pressed != self.raw:
            self.raw, self.changed = pressed, now
        if now - self.changed < .1:
            return False
        if pressed != self.stable:
            self.stable = pressed
            if not pressed:
                self.released, self.started = True, None
            elif self.released:
                self.started = now
        if pressed and self.started is not None and now - self.started >= 3:
            self.released, self.started = False, None
            return True
        return False

class HatIndicators:
    def __init__(self, button_path, led_path):
        self.button_path, self.led_path = button_path, led_path

    def pressed(self):
        value = self.button_path.read_text().strip()
        if value not in ('0', '1'):
            raise ValueError('Fusion HAT button must report 0 or 1')
        return value == '1'

    def set_led(self, enabled):
        self.led_path.write_text('1' if enabled else '0')


def validate_owner(identity):
    if (not isinstance(identity, dict)
            or not isinstance(identity.get('address'), str)
            or not re.fullmatch(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}', identity['address'])
            or identity.get('address_type') not in ('public', 'random')):
        raise ValueError('invalid owner identity; repair provisioning explicitly')
    return dict(address=identity['address'].upper(), address_type=identity['address_type'])


def load_owner(path):
    try:
        with path.open() as stream:
            mode = os.fstat(stream.fileno()).st_mode
            if not stat.S_ISREG(mode) or stat.S_IMODE(mode) != 0o600:
                raise ValueError('owner file must be a private regular file (0600)')
            return validate_owner(json.load(stream))
    except FileNotFoundError:
        # Broken symlinks are provisioning errors, not fresh devices.
        if path.is_symlink():
            raise ValueError('owner file is a broken symlink')
        return None


def _exclusive_json(path, value):
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(dir=path.parent, prefix='.identity-')
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream)
            stream.flush()
            os.fsync(stream.fileno())
        # Hard link is atomic and refuses an existing target, unlike replace().
        os.link(temporary, path)
        try:
            directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        except OSError:
            # A failed commit must not masquerade as successful provisioning.
            path.unlink()
            raise
    finally:
        os.unlink(temporary)


def save_owner(path, identity):
    _exclusive_json(path, validate_owner(identity))


def load_device_name(path):
    try:
        value = json.loads(path.read_text())
    except FileNotFoundError:
        if path.is_symlink():
            raise ValueError('device name is a broken symlink')
        name = 'terra-' + ''.join(secrets.choice(string.ascii_uppercase + string.digits) for _ in range(6))
        try:
            _exclusive_json(path, {'name': name})
        except FileExistsError:
            return load_device_name(path)
        return name
    if not isinstance(value, dict) or not re.fullmatch(r'terra-[A-Z0-9]{6}', value.get('name', '')):
        raise ValueError('invalid persisted rover name')
    return value['name']


def pairing_name(name):
    """Distinguish enrollment before an encrypted connection is attempted."""
    return name if name.endswith("-pair") else name + "-pair"
