import json
import unittest
from terra_rover.ble import status_read_document


class StatusReadTests(unittest.TestCase):
    def test_admission_status_fits_att_read_and_preserves_authorization_fields(self):
        payload = dict(schema_version=1, type='status', connected=True,
                       active_revision=4294967295, control_mode='streamed_relay', output_active=False,
                       layout_available=False, fault=None, configuration_allowed=True,
                       configuration_errors=[dict(message='x' * 2000)],
                       battery=None, battery_reason='unsupported', service_state='stopped')
        self.assertGreater(len(json.dumps(payload).encode()), 512)
        raw = status_read_document(payload)
        self.assertLessEqual(len(raw), 512)
        decoded = json.loads(raw)
        self.assertEqual(decoded['active_revision'], 4294967295)
        self.assertTrue(decoded['configuration_allowed'])
        self.assertFalse(decoded['layout_available'])
        self.assertFalse(decoded['output_active'])

    def test_large_fault_remains_a_fault_without_truncating_json(self):
        payload = dict(schema_version=1, type='status', active_revision=1, output_active=False,
                       layout_available=True, fault='⚠' * 1000, configuration_allowed=False)
        raw = status_read_document(payload)
        self.assertLessEqual(len(raw), 512)
        self.assertIsInstance(json.loads(raw)['fault'], str)
        self.assertFalse(json.loads(raw)['configuration_allowed'])

    def test_periodic_status_retains_stop_reason_without_bulk_diagnostics(self):
        payload = dict(schema_version=1, type='status', active_revision=2, output_active=False,
                       stop_reason='no_command', configuration_errors=['x' * 2000], battery_reason='unsupported')
        decoded = json.loads(status_read_document(payload))
        self.assertEqual(decoded['stop_reason'], 'no_command')
        self.assertNotIn('configuration_errors', decoded)

    def test_long_backend_fault_in_stop_reason_does_not_kill_status_stream(self):
        detail = 'backend: apply on M0: permission denied; safe failures: ' + 'M0/P11 unavailable; ' * 40
        payload = dict(schema_version=1, type='status', connected=True,
                       active_revision=4294967295, control_mode='streamed_relay', output_active=False,
                       layout_available=True, fault=detail, stop_reason=detail, configuration_allowed=True)
        raw = status_read_document(payload)
        self.assertLessEqual(len(raw), 512)
        decoded = json.loads(raw)
        self.assertTrue(decoded['fault'].startswith('backend: apply on M0:'))
        self.assertTrue(decoded['stop_reason'])
        self.assertEqual(decoded['active_revision'], 4294967295)
        self.assertFalse(decoded['output_active'])

    def test_fault_prefix_is_preserved_when_other_fields_fill_packet(self):
        detail = 'backend: apply on M0: permission denied; safe failures: ' + 'x' * 2000
        decoded = json.loads(status_read_document(dict(schema_version=1, type='status',
            active_revision=1, output_active=False, fault=detail, stop_reason='configuration_changed',
            layout_available=True, configuration_allowed=False)))
        self.assertTrue(decoded['fault'].startswith('backend: apply on M0: permission denied'))
