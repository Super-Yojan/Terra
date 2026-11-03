"""Fault details survive transport compaction without flooding the journal."""
import unittest
from terra_rover.drive_relay import DriveRelay

class FaultDiagnosticsTests(unittest.TestCase):
    def test_full_fault_logged_once_per_transition_and_again_after_reset(self):
        relay = DriveRelay()
        detail = 'backend: apply on M0: ' + 'failure details ' * 100
        with self.assertLogs('terra_rover.drive_relay', level='ERROR') as logs:
            relay.latch_fault(detail)
            relay.latch_fault(detail)
            relay.reset_fault()
            relay.latch_fault(detail)
        self.assertEqual(len(logs.output), 2)
        self.assertIn(detail, logs.output[0])
        self.assertFalse(relay.output_active)
        self.assertEqual(relay.fault, detail)
