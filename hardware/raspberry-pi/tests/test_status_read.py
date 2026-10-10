import json
import unittest
from terra_rover.ble import status_read_document


class StatusReadTests(unittest.TestCase):
    def test_admission_status_fits_att_read_and_preserves_authorization_fields(self):
        payload = dict(schema_version=1, type='status', session=4294967295,
                       active_revision=4294967295, armed=False, arming=False,
                       layout_available=False, fault=None, emergency_stop=False,
                       gate_mode='external_power_cutoff', bench_enabled=False,
                       hardware_gate_open_confirmed=False, configuration_allowed=True,
                       configuration_errors=[dict(message='x' * 2000)],
                       battery=None, battery_reason='unsupported', service_state='disarmed')
        self.assertGreater(len(json.dumps(payload).encode()), 512)
        raw = status_read_document(payload)
        self.assertLessEqual(len(raw), 512)
        decoded = json.loads(raw)
        self.assertEqual(decoded['session'], 4294967295)
        self.assertTrue(decoded['configuration_allowed'])
        self.assertFalse(decoded['layout_available'])
        self.assertFalse(decoded['armed'])
        self.assertFalse(decoded['hardware_gate_open_confirmed'])

    def test_large_fault_remains_a_fault_without_truncating_json(self):
        payload = dict(schema_version=1, type='status', session=1, active_revision=1,
                       armed=False, arming=False, layout_available=True,
                       fault='⚠' * 1000, emergency_stop=False,
                       configuration_allowed=False, gate_mode='external_power_cutoff')
        raw = status_read_document(payload)
        self.assertLessEqual(len(raw), 512)
        self.assertIsInstance(json.loads(raw)['fault'], str)
        self.assertFalse(json.loads(raw)['configuration_allowed'])

    def test_periodic_status_retains_watchdog_and_sequence_without_bulk_diagnostics(self):
        payload = dict(schema_version=1, type='status', session=1, active_revision=2,
                       armed=False, arming=False, last_sequence=42, stop_reason='watchdog',
                       configuration_errors=['x' * 2000], battery_reason='unsupported')
        decoded = json.loads(status_read_document(payload))
        self.assertEqual(decoded['last_sequence'], 42)
        self.assertEqual(decoded['stop_reason'], 'watchdog')
        self.assertNotIn('configuration_errors', decoded)
