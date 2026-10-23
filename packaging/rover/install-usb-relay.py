#!/usr/bin/env python3
"""Install the Mac-side relay after provisioning the Pi observer and SSH login."""
import argparse
import json
import os
from pathlib import Path
import plistlib
import shutil
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='pi@raspberrypi.local')
    parser.add_argument('--endpoint', required=True)
    parser.add_argument('--token-file', required=True, type=Path)
    parser.add_argument('--control-path', help='Optional existing authenticated SSH control socket')
    args = parser.parse_args()
    if sys.platform != 'darwin':
        parser.error('this installer requires macOS')
    from urllib.parse import urlparse
    parsed = urlparse(args.endpoint)
    if parsed.scheme != 'https' or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment:
        parser.error('a plain HTTPS collector endpoint is required')
    if args.host.startswith('-') or args.host.count('@') != 1:
        parser.error('host must be user@hostname')
    os.umask(0o077)
    root = Path.home() / 'Library/Application Support/Terra/usb-log-relay'
    root.mkdir(parents=True, exist_ok=True)
    root.chmod(0o700)
    package = root / 'terra_rover'
    package.mkdir(exist_ok=True)
    (package / '__init__.py').touch()
    source = Path(__file__).resolve().parents[2] / 'hardware/raspberry-pi/terra_rover'
    for name in ('observability.py', 'usb_queue.py'):
        shutil.copyfile(source / name, package / name)
    key = root / 'id_ed25519'
    if not key.exists():
        subprocess.run(['ssh-keygen', '-q', '-t', 'ed25519', '-N', '', '-C', 'terra-usb-log-relay', '-f', str(key)], check=True)
    host = args.host.split('@')[1]
    # Only use a host key from an already verified SSH login; never trust a scan.
    verified = subprocess.run(['ssh-keygen', '-F', host, '-f', str(Path.home() / '.ssh/known_hosts')], capture_output=True, text=True, check=True)
    known = root / 'known_hosts'
    known.write_text(verified.stdout)
    known.chmod(0o600)
    token = root / 'upload.token'
    shutil.copyfile(args.token_file, token)
    token.chmod(0o600)
    config = root / 'config.json'
    config.write_text(json.dumps({'host': args.host, 'key': str(key), 'known_hosts': str(known),
                                 'endpoint': args.endpoint, 'token_file': str(token), 'status_file': str(root / 'status.json')}))
    config.chmod(0o600)
    ssh = ['ssh'] + (['-S', args.control_path] if args.control_path else []) + [args.host]
    public = key.with_suffix('.pub').read_text().strip()
    line = 'restrict,command="sudo -n -u terra-observability /usr/bin/python3 /opt/terra-observability-pi/terra_rover/usb_queue.py --serve" ' + public
    # Authorized key is idempotent and cannot provide a shell or forwarding.
    remote = 'umask 077; mkdir -p ~/.ssh; touch ~/.ssh/authorized_keys; IFS= read -r line; grep -qxF -- "$line" ~/.ssh/authorized_keys || printf "%s\\n" "$line" >> ~/.ssh/authorized_keys'
    subprocess.run(ssh + [remote], input=line + '\n', text=True, check=True)
    agent = Path.home() / 'Library/LaunchAgents/com.terra.usb-log-relay.plist'
    agent.parent.mkdir(exist_ok=True)
    agent.write_bytes(plistlib.dumps({'Label': 'com.terra.usb-log-relay',
        'ProgramArguments': [sys.executable, '-m', 'terra_rover.usb_queue', '--config', str(config)],
        'EnvironmentVariables': {'PYTHONPATH': str(root)}, 'RunAtLoad': True, 'KeepAlive': True, 'ThrottleInterval': 10,
        'StandardOutPath': str(root / 'relay.log'), 'StandardErrorPath': str(root / 'relay-error.log')}))
    domain = f'gui/{os.getuid()}'
    subprocess.run(['launchctl', 'bootout', domain, str(agent)], capture_output=True)
    subprocess.run(['launchctl', 'bootstrap', domain, str(agent)], check=True)
    print(f'USB log relay installed. Connection status: {root / "status.json"}')


if __name__ == '__main__':
    main()
