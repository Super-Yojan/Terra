"""Restricted USB queue access and Mac relay; never sends rover commands."""
import argparse
from contextlib import closing
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import time

DATABASE = Path('/var/lib/terra-observability-pi/events.db')
MAX_WIRE_BYTES = 1024 * 1024


def access_queue(path, request):
    action = request.get('action')
    allowed = {'action', 'limit'} if action == 'batch' else {'action', 'ids'}
    if set(request) - allowed:
        raise ValueError('unknown request fields')
    if action == 'batch':
        limit = request.get('limit', 64)
        if type(limit) is not int or not 1 <= limit <= 64:
            raise ValueError('invalid batch size')
        with closing(sqlite3.connect(f'file:{path}?mode=ro', uri=True, timeout=5)) as db:
            rows = db.execute('SELECT payload FROM events WHERE ack=0 ORDER BY stored_ns LIMIT ?', (limit,)).fetchall()
        events = []
        for row in rows:
            event = json.loads(row[0])
            if len(json.dumps({'events': events + [event]}).encode()) > 900 * 1024:
                break
            events.append(event)
        return {'events': events}
    if action == 'ack':
        ids = request.get('ids')
        if not isinstance(ids, list) or len(ids) > 64:
            raise ValueError('invalid acknowledgments')
        for identity in ids:
            if (not isinstance(identity, dict) or set(identity) != {'producer_id', 'boot_id', 'sequence'}
                    or not isinstance(identity['producer_id'], str) or not isinstance(identity['boot_id'], str)
                    or type(identity['sequence']) is not int or identity['sequence'] < 0):
                raise ValueError('invalid identity')
        with closing(sqlite3.connect(f'file:{path}?mode=rw', uri=True, timeout=5)) as db, db:
            db.executemany('UPDATE events SET ack=1 WHERE id=?', [(json.dumps(i, sort_keys=True),) for i in ids])
        return {'ok': True}
    raise ValueError('unsupported action')


class SshJournal:
    def __init__(self, host, key, known_hosts):
        if host.startswith('-'):
            raise ValueError('invalid host')
        self.command = ['ssh', '-T', '-i', str(key), '-o', 'IdentitiesOnly=yes',
                        '-o', 'BatchMode=yes', '-o', 'ControlMaster=no', '-o', 'ControlPath=none',
                        '-o', 'StrictHostKeyChecking=yes',
                        '-o', f'UserKnownHostsFile="{known_hosts}"', '-o', 'ConnectTimeout=8',
                        '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2', host]

    def request(self, request):
        result = subprocess.run(self.command, input=json.dumps(request), text=True,
                                capture_output=True, timeout=25, check=True)
        if len(result.stdout.encode()) > MAX_WIRE_BYTES:
            raise ValueError('oversized queue response')
        return json.loads(result.stdout)

    def pending(self, limit=64):
        return self.request({'action': 'batch', 'limit': min(limit, 64)})['events']

    def acknowledge(self, ids):
        if self.request({'action': 'ack', 'ids': ids}) != {'ok': True}:
            raise ValueError('queue acknowledgment failed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serve', action='store_true')
    parser.add_argument('--config', type=Path)
    args = parser.parse_args()
    if args.serve:
        raw = sys.stdin.buffer.read(MAX_WIRE_BYTES + 1)
        if len(raw) > MAX_WIRE_BYTES:
            raise ValueError('oversized request')
        print(json.dumps(access_queue(DATABASE, json.loads(raw))))
        return
    if not args.config:
        parser.error('--config is required for the Mac relay')
    from .observability import ObservationUploader
    config = json.loads(args.config.read_text())
    # Reuse the HTTPS uploader and its strict cloud acknowledgment validation.
    uploader = object.__new__(ObservationUploader)
    uploader.journal = SshJournal(config['host'], config['key'], config['known_hosts'])
    uploader.endpoint = config['endpoint'].rstrip('/')
    from urllib.parse import urlparse
    parsed = urlparse(uploader.endpoint)
    if parsed.scheme != 'https' or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment:
        raise ValueError('HTTPS endpoint required')
    uploader.token = Path(config['token_file']).read_text().strip()
    status = Path(config['status_file'])
    delay = 1
    while True:
        try:
            count = uploader.once()
            state = {'connected': True, 'uploaded': count, 'checked_at_ns': time.time_ns()}
            delay = 1
        except Exception as error:
            # Only the exception type is persisted; no credentials or remote output.
            state = {'connected': False, 'error': type(error).__name__, 'checked_at_ns': time.time_ns()}
            delay = min(30, delay * 2)
        temporary = status.with_suffix('.tmp')
        temporary.write_text(json.dumps(state))
        os.replace(temporary, status)
        time.sleep(delay)


if __name__ == '__main__':
    main()
