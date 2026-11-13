"""Independent bounded operational journal. No actuator or BLE dependencies."""
import json
import hashlib
import itertools
import os
from pathlib import Path
import queue
import sqlite3
import threading
import time
import uuid

# Mirrors terra-observability's contract.rs; tests/fixtures/observability/contract.json pins both.
SCHEMA_VERSION = 1
MAX_EVENT_BYTES = 128 * 1024
MAX_UPLOAD_BATCH_BYTES = 900 * 1024
UPLOAD_BATCH_EVENTS = 64
PRODUCER_ROVER = '_producer'
GAP_KIND = 'observability.gap'
# The collector rejects a whole batch if any attribute key contains one of these,
# so an unredacted event would block every later upload.
SECRET_KEY_FRAGMENTS = ('password', 'authorization', 'credential', 'private_key',
                        'secret', 'access_token', 'refresh_token')
SEVERITIES = ('debug', 'info', 'warning', 'error')

_journal = None

def is_secret_key(key):
    lowered = key.lower()
    return any(fragment in lowered for fragment in SECRET_KEY_FRAGMENTS)

def redact(value):
    """Copy of `value` without credential-named keys at any depth."""
    if isinstance(value, dict):
        return {key: redact(item) for key, item in value.items() if not is_secret_key(key)}
    if isinstance(value, list):
        return [redact(item) for item in value]
    return value

def _valid_input(event):
    """Same admission rule as EventInput::validate; the collector rejects the batch otherwise."""
    def text(value):
        return isinstance(value, str) and 0 < len(value.encode()) <= 256
    session = event.get('session_id')
    return (text(event.get('rover_id')) and text(event.get('component')) and text(event.get('kind'))
            and (session is None or (isinstance(session, str) and len(session.encode()) <= 256))
            and event.get('severity') in SEVERITIES and isinstance(event.get('attributes'), dict))

def set_journal(journal):
    global _journal
    _journal = journal

def record_event(component, kind, attributes):
    try:
        if _journal is not None:
            _journal.record({'component': component, 'kind': kind, 'attributes': attributes})
    except Exception:
        # Observation failure never changes the safety controller's result.
        pass

class ObservationJournal:
    def __init__(self, path, producer_id, rover_id, capacity=4096, max_bytes=512*1024*1024, emit_journal=False, retention_seconds=7*86400):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.producer, self.rover, self.boot = producer_id, str(rover_id), str(uuid.uuid4())
        self.sequence = itertools.count()
        self.emit_journal = emit_journal
        self.dropped = 0
        self.failures = 0
        self.retention_seconds = retention_seconds
        if max_bytes < 1024 or retention_seconds <= 0: raise ValueError("invalid journal limits")
        self.max_bytes = max_bytes
        self.queue = queue.Queue(maxsize=capacity)
        self._db().close()
        self.worker = threading.Thread(target=self._work, daemon=True, name='observation-journal')
        self.worker.start()

    def _db(self):
        db = sqlite3.connect(self.path, timeout=2)
        db.execute('PRAGMA journal_mode=WAL')
        db.execute('PRAGMA synchronous=FULL')
        db.execute('CREATE TABLE IF NOT EXISTS events (id TEXT PRIMARY KEY, payload TEXT, bytes INTEGER, stored_ns INTEGER, ack INTEGER DEFAULT 0)')
        return db

    def _envelope(self, event):
        sequence = next(self.sequence)
        source_boot = hashlib.sha256(event["source_cursor"].encode()).hexdigest() if event.get("source_cursor") else self.boot
        if event.get("source_cursor"): sequence = 0
        return {'schema_version': SCHEMA_VERSION, 'id': {'producer_id': self.producer, 'boot_id': source_boot, 'sequence': sequence},
                'input': {'rover_id': self.rover, 'session_id': event.get('session_id'),
                          'component': event['component'], 'kind': event['kind'], 'severity': event.get('severity', 'info'),
                          'wall_time_ns': event.get('wall_time_ns', time.time_ns()),
                          'monotonic_ns': event.get('monotonic_ns', time.monotonic_ns()),
                          'cause_id': event.get('cause_id'), 'attributes': redact(event.get('attributes', {}))}}

    def record(self, event):
        try:
            envelope = self._envelope(event)
            if not _valid_input(envelope['input']):
                self.dropped += 1
                return False
            payload = json.dumps(envelope, separators=(',', ':'), allow_nan=False)
            if len(payload.encode()) > MAX_EVENT_BYTES:
                self.dropped += 1
                return False
            self.queue.put_nowait(payload)
            return True
        except (queue.Full, ValueError, TypeError, KeyError):
            self.dropped += 1
            return False

    def record_envelope(self, envelope):
        try:
            if envelope['schema_version'] != SCHEMA_VERSION or envelope['id']['producer_id'] != self.producer: return False
            envelope = dict(envelope, input=dict(envelope['input'], attributes=redact(envelope['input'].get('attributes'))))
            if not _valid_input(envelope['input']): return False
            payload=json.dumps(envelope,separators=(',', ':'),allow_nan=False)
            if len(payload.encode()) > MAX_EVENT_BYTES: return False
            self.queue.put_nowait(payload)
            return True
        except (queue.Full,ValueError,TypeError,KeyError):
            self.dropped += 1
            return False

    def _persist(self, db, payload):
        event = json.loads(payload)
        identity = json.dumps(event['id'], sort_keys=True)
        now = time.time_ns()
        with db:
            db.execute('INSERT OR IGNORE INTO events(id,payload,bytes,stored_ns) VALUES(?,?,?,?)',
                       (identity, payload, len(payload.encode()), now))
            rows = db.execute('SELECT id,bytes,ack,stored_ns FROM events ORDER BY ack DESC,stored_ns').fetchall()
            total = sum(r[1] for r in rows)
            lost = 0
            for identity, size, ack, stamp in rows:
                if total <= self.max_bytes and stamp >= now-self.retention_seconds*10**9:
                    continue
                db.execute('DELETE FROM events WHERE id=?', (identity,))
                total -= size
                if not ack: lost += 1
            if lost:
                self.dropped += lost

    def _work(self):
        db = self._db()
        reported = 0
        while True:
            item = self.queue.get()
            try:
                if item is None:
                    break
                if self.dropped > reported:
                    count = self.dropped-reported
                    gap = self._envelope({'component':'observability','kind':GAP_KIND,'attributes':{'dropped':count}})
                    self._persist(db, json.dumps(gap))
                    reported += count
                if isinstance(item, threading.Event):
                    item.set()
                else:
                    self._persist(db, item)
                    if self.emit_journal: print(item, flush=True)
            except (OSError, sqlite3.Error):
                self.failures += 1
                self.dropped += 1
                if isinstance(item, threading.Event): item.set()
            finally:
                self.queue.task_done()
        db.close()

    def pending(self, limit=UPLOAD_BATCH_EVENTS):
        with self._db() as db:
            events = [json.loads(row[0]) for row in db.execute('SELECT payload FROM events WHERE ack=0 ORDER BY stored_ns LIMIT ?', (min(limit, 200),))]
        # Events stored before redaction existed would otherwise be rejected on every retry.
        for event in events:
            event['input']['attributes'] = redact(event['input'].get('attributes'))
        return events

    def acknowledge(self, ids):
        with self._db() as db:
            db.executemany('UPDATE events SET ack=1 WHERE id=?', [(json.dumps(i, sort_keys=True),) for i in ids])

    def flush(self, timeout=5):
        barrier = threading.Event()
        self.queue.put(barrier, timeout=timeout)
        if not barrier.wait(timeout): raise TimeoutError('observation flush timeout')

    def close(self):
        self.queue.put(None, timeout=5)
        self.worker.join(timeout=5)


def configure_from_environment():
    """Optional startup initialization, off the output loop. No credentials are logged."""
    path = os.getenv('TERRA_OBSERVABILITY_DB')
    if not path: return
    try:
        journal = ObservationJournal(path, os.environ['TERRA_OBSERVABILITY_PRODUCER'], os.environ['TERRA_OBSERVABILITY_ROVER'], emit_journal=True, max_bytes=int(os.getenv('TERRA_OBSERVABILITY_EVENT_BYTES',512*1024*1024)), retention_seconds=int(os.getenv('TERRA_OBSERVABILITY_RETENTION_SECONDS',7*86400)))
        set_journal(journal)
        record_event('pi', 'lifecycle.start', {})
        endpoint = os.getenv('TERRA_OBSERVABILITY_ENDPOINT')
        token_file = os.getenv('TERRA_OBSERVABILITY_UPLOAD_TOKEN_FILE')
        if endpoint and token_file:
            journal.uploader = ObservationUploader(journal, endpoint, Path(token_file).read_text().strip())
    except (OSError, sqlite3.Error, KeyError, ValueError):
        pass

class ObservationUploader:
    """Background retry worker. Failed responses never acknowledge local records."""
    def __init__(self, journal, endpoint, token):
        from urllib.parse import urlparse
        parsed = urlparse(endpoint)
        if not (parsed.scheme == 'https' or (parsed.scheme == 'http' and parsed.hostname in ('127.0.0.1', 'localhost', '::1'))):
            raise ValueError('HTTPS endpoint required')
        if parsed.username or parsed.password or not token:
            raise ValueError('invalid upload settings')
        self.journal, self.endpoint, self.token = journal, endpoint.rstrip('/'), token
        self.stop = threading.Event()
        self.failures = 0
        self.worker = threading.Thread(target=self._work, daemon=True, name='observation-upload')
        self.worker.start()

    def once(self):
        import urllib.request
        events = self.journal.pending(UPLOAD_BATCH_EVENTS)
        if not events: return 0
        payload = json.dumps({'events': events}).encode()
        while len(payload) > MAX_UPLOAD_BATCH_BYTES:
            events.pop()
            payload = json.dumps({'events': events}).encode()
        request = urllib.request.Request(self.endpoint+'/v1/events', data=payload,
                    headers={'Authorization': 'Bearer '+self.token, 'Content-Type': 'application/json'})
        # Disable redirects so ingestion credentials cannot be forwarded to another host.
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self, req, fp, code, msg, headers, newurl): return None
        with urllib.request.build_opener(NoRedirect).open(request, timeout=15) as response:
            data = json.loads(response.read(1024*1024))
        ids = data['acknowledged']
        expected = [e['id'] for e in events]
        if not isinstance(ids, list) or any(identity not in expected for identity in ids):
            raise ValueError('invalid acknowledgment')
        self.journal.acknowledge(ids)
        return len(ids)

    def _work(self):
        import random
        delay = 1
        while not self.stop.is_set():
            try: self.once(); delay = 1
            except Exception: self.failures += 1; delay = min(60, delay*2)
            self.stop.wait(delay+random.random()/2)
