import json
import pathlib
import tempfile
import unittest
from terra_rover.observability import ObservationJournal
from terra_rover.usb_queue import access_queue, SshJournal


class UsbQueueTests(unittest.TestCase):
    def test_batch_preserves_identity_and_only_acknowledges_named_events(self):
        with tempfile.TemporaryDirectory() as folder:
            path = pathlib.Path(folder) / 'events.db'
            journal = ObservationJournal(path, 'pi-rover0', '0')
            try:
                for kind in ('first', 'second'):
                    journal.record({'component': 'pi', 'kind': kind, 'attributes': {}})
                journal.flush()
                original = journal.pending()
                batch = access_queue(path, {'action': 'batch', 'limit': 1})
                self.assertEqual(batch['events'], original[:1])
                access_queue(path, {'action': 'ack', 'ids': [original[0]['id']]})
                self.assertEqual(journal.pending(), original[1:])
            finally:
                journal.close()

    def test_requests_cannot_choose_database_or_run_commands(self):
        with tempfile.TemporaryDirectory() as folder:
            path = pathlib.Path(folder) / 'events.db'
            journal = ObservationJournal(path, 'pi-rover0', '0')
            try:
                for request in ({'action': 'shell'}, {'action': 'batch', 'limit': 10000},
                                {'action': 'ack', 'ids': ['bad']},
                                {'action': 'batch', 'path': '/etc/shadow'}):
                    with self.assertRaises(ValueError):
                        access_queue(path, request)
            finally:
                journal.close()

    def test_transport_failure_does_not_acknowledge(self):
        import subprocess
        from unittest.mock import patch
        remote = SshJournal('pi@raspberrypi.local', '/tmp/key', '/tmp/known_hosts')
        batch = json.dumps({'events': [{'id': 'e1', 'kind': 'k'}]})
        with patch('subprocess.run', return_value=subprocess.CompletedProcess('ssh', 0, batch, '')) as run, \
                patch('urllib.request.OpenerDirector.open', side_effect=OSError('offline')):
            uploader = _uploader(remote)
            with self.assertRaises(OSError):
                uploader.once()
        actions = [json.loads(call.kwargs['input'])['action'] for call in run.call_args_list]
        self.assertEqual(actions, ['batch'])
        with patch('subprocess.run', side_effect=subprocess.TimeoutExpired('ssh', 20)):
            with self.assertRaises(subprocess.TimeoutExpired):
                remote.pending()


def _uploader(journal):
    """An uploader whose background worker has already stopped, so the test drives `once`."""
    from terra_rover.observability import ObservationUploader

    class Gate:
        open = False
        def pending(self, limit):
            return journal.pending(limit) if self.open else []
        def acknowledge(self, ids):
            journal.acknowledge(ids)

    gate = Gate()
    uploader = ObservationUploader(gate, 'https://collector.example', 'test-only')
    uploader.stop.set()
    uploader.worker.join(timeout=5)
    gate.open = True
    return uploader


class RelayDeliveryTests(unittest.TestCase):
    def test_cloud_failure_keeps_backlog_then_reconnect_acknowledges(self):
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as folder:
            path = pathlib.Path(folder) / 'events.db'
            journal = ObservationJournal(path, 'pi-rover0', '0')
            try:
                journal.record({'component': 'pi', 'kind': 'test.offline', 'attributes': {}})
                journal.flush()
                original = journal.pending()
                class Remote:
                    def pending(self, limit):
                        return access_queue(path, {'action': 'batch', 'limit': limit})['events']
                    def acknowledge(self, ids):
                        access_queue(path, {'action': 'ack', 'ids': ids})
                uploader = _uploader(Remote())
                with patch('urllib.request.OpenerDirector.open', side_effect=OSError('offline')):
                    with self.assertRaises(OSError): uploader.once()
                self.assertEqual(journal.pending(), original)
                class Response:
                    def __enter__(self): return self
                    def __exit__(self, *args): pass
                    def read(self, limit): return json.dumps({'acknowledged': [original[0]['id']]}).encode()
                with patch('urllib.request.OpenerDirector.open', return_value=Response()):
                    self.assertEqual(uploader.once(), 1)
                self.assertEqual(journal.pending(), [])
            finally:
                journal.close()
