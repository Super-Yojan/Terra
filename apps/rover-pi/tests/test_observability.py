"""Pi observation journal: persistence, journald projection and the shared event contract."""
import json
import pathlib
import tempfile
import unittest

from terra_rover import observability
from terra_rover.log_forwarder import project_journal
from terra_rover.observability import ObservationJournal

FIXTURE = pathlib.Path(__file__).resolve().parents[3] / 'tests/fixtures/observability/contract.json'


class JournalTests(unittest.TestCase):
    def test_pi_backlog_survives_without_phone(self):
        with tempfile.TemporaryDirectory() as d:
            path = pathlib.Path(d) / 'events.db'
            j = ObservationJournal(path, 'pi', '4')
            self.assertTrue(j.record({'component': 'pi', 'kind': 'lifecycle.boot', 'attributes': {}}))
            j.flush(); j.close()
            j = ObservationJournal(path, 'pi', '4')
            self.assertEqual(len(j.pending()), 1)
            j.close()

    def test_credentials_never_reach_the_upload_batch(self):
        # The collector rejects a whole batch over one secret key; the Pi used to retry it forever.
        with tempfile.TemporaryDirectory() as d:
            j = ObservationJournal(pathlib.Path(d) / 'events.db', 'pi', '4')
            try:
                self.assertTrue(j.record({'component': 'pi', 'kind': 'network.join',
                                          'attributes': {'ssid': 'lab', 'wifi_Password': 'x',
                                                         'nested': [{'access_token': 'y', 'ok': 1}]}}))
                j.flush()
                attributes = j.pending()[0]['input']['attributes']
                self.assertEqual(attributes, {'ssid': 'lab', 'nested': [{'ok': 1}]})
            finally:
                j.close()

    def test_event_the_collector_would_reject_is_dropped_not_queued(self):
        with tempfile.TemporaryDirectory() as d:
            j = ObservationJournal(pathlib.Path(d) / 'events.db', 'pi', '4')
            try:
                self.assertFalse(j.record({'component': 'pi', 'kind': 'x', 'severity': 'fatal', 'attributes': {}}))
                self.assertFalse(j.record({'component': '', 'kind': 'x', 'attributes': {}}))
                j.flush()
                self.assertEqual([e['input']['kind'] for e in j.pending()], [observability.GAP_KIND])
            finally:
                j.close()


@unittest.skipUnless(FIXTURE.exists(), 'shared fixture is only present in the repository checkout')
class ContractTests(unittest.TestCase):
    def test_constants_match_the_shared_contract(self):
        fixture = json.loads(FIXTURE.read_text())
        self.assertEqual(observability.SCHEMA_VERSION, fixture['schema_version'])
        self.assertEqual(observability.MAX_EVENT_BYTES, fixture['max_event_bytes'])
        self.assertEqual(observability.MAX_UPLOAD_BATCH_BYTES, fixture['max_upload_batch_bytes'])
        self.assertEqual(observability.PRODUCER_ROVER, fixture['producer_rover'])
        self.assertEqual(observability.GAP_KIND, fixture['gap_kind'])
        self.assertEqual(list(observability.SECRET_KEY_FRAGMENTS), fixture['secret_key_fragments'])
        self.assertEqual(observability.UPLOAD_BATCH_EVENTS, fixture['upload_batch_events'])
        self.assertEqual(observability.MAX_FIELD_BYTES, fixture['max_field_bytes'])
        self.assertEqual(list(observability.SEVERITIES), fixture['severities'])
        self.assertEqual(list(observability.SENSITIVE_JOURNAL_WORDS), fixture['sensitive_journal_words'])
        self.assertEqual(observability.SENSITIVE_JOURNAL_PLACEHOLDER, fixture['sensitive_journal_placeholder'])

    def test_sensitive_journal_messages_match_the_shared_cases(self):
        for case in json.loads(FIXTURE.read_text())['journal_messages']:
            self.assertEqual(observability.sensitive_journal_message(case['message']), case['sensitive'], case['message'])


class JournalProjectionTests(unittest.TestCase):
    def test_cursor_identity_is_stable_and_units_are_allowlisted(self):
        value = {'_SYSTEMD_UNIT': 'bluetooth.service', '__CURSOR': 'stable-cursor', '__REALTIME_TIMESTAMP': '123',
                 '__MONOTONIC_TIMESTAMP': '50', 'MESSAGE': 'disconnected'}
        a = project_journal(value); b = project_journal(value)
        self.assertEqual(a['source_cursor'], b['source_cursor']); self.assertEqual(a['wall_time_ns'], 123000)
        value['_SYSTEMD_UNIT'] = 'unrelated.service'
        self.assertIsNone(project_journal(value))

    def test_kernel_hat_failure_is_recorded(self):
        entry = {'_TRANSPORT': 'kernel', '__CURSOR': 'kernel-cursor', '__REALTIME_TIMESTAMP': '123',
                 '__MONOTONIC_TIMESTAMP': '50', 'MESSAGE': 'fusion_hat 1-0017: probe failed with error -5'}
        event = project_journal(entry)
        self.assertIsNotNone(event); self.assertEqual(event['component'], 'kernel')
        self.assertIn('probe failed', event['attributes']['message'])

    def test_systemd_unit_failure_is_not_hidden_by_init_scope(self):
        entry = {'_SYSTEMD_UNIT': 'init.scope', 'UNIT': 'terra-rover.service', '__CURSOR': 'unit-cursor',
                 'MESSAGE': 'Main process exited, status=1/FAILURE'}
        self.assertIsNotNone(project_journal(entry))

    def test_hotspot_disconnect_and_secret_redaction(self):
        entry = {'_SYSTEMD_UNIT': 'hostapd.service', '__CURSOR': 'ap-cursor', 'MESSAGE': 'wlan0: AP-STA-DISCONNECTED'}
        self.assertIn('DISCONNECTED', project_journal(entry)['attributes']['message'])
        entry['MESSAGE'] = 'WPA: 4-Way Handshake failed - pre-shared key may be incorrect (PSK mismatch)'
        self.assertIn('PSK mismatch', project_journal(entry)['attributes']['message'])
        for secret in ('psk=private-fixture-value', 'wpa_psk=private-fixture-value'):
            entry['MESSAGE'] = secret
            self.assertEqual(project_journal(entry)['attributes']['message'], '[sensitive journal message omitted]')


if __name__ == '__main__':
    unittest.main()
