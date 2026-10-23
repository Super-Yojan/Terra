"""Fault details survive transport compaction without flooding the journal."""
import unittest
from terra_rover.safety import SafetyController

class FaultDiagnosticsTests(unittest.TestCase):
    def test_full_fault_logged_once_per_transition_and_again_after_reset(self):
        safety = SafetyController()
        detail = 'backend: apply on M0: ' + 'failure details ' * 100
        with self.assertLogs('terra_rover.safety', level='ERROR') as logs:
            safety.latch_fault(detail, 1)
            safety.latch_fault(detail, 2)
            safety.reset_fault(3)
            safety.latch_fault(detail, 4)
        self.assertEqual(len(logs.output), 2)
        self.assertIn(detail, logs.output[0])
        self.assertFalse(safety.armed)
        self.assertEqual(safety.fault, detail)
