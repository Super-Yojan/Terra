"""Battery cutoff is external: no fictitious Pi gate is required or reported."""
import json
from pathlib import Path
import tempfile
import unittest

from terra_rover.backend import FusionHatBackend, MockBackend, BackendError
from terra_rover.configuration import ConfigurationStore
from terra_rover.safety import SafetyController
from terra_rover.protocol import CommandFrame, CommandKind, ActuatorValue


class BatterySwitchTests(unittest.TestCase):
    def test_backend_requires_no_gate_but_closed_backend_stays_unavailable(self):
        backend = FusionHatBackend(('P0', 'P1'))
        backend.external_power_cutoff = True
        backend.require_gate_open()
        self.assertTrue(backend.read_gate())
        backend._closed = True
        self.assertFalse(backend.read_gate())
        with self.assertRaises(BackendError):
            backend.require_gate_open()

    def test_physical_gate_backend_still_rejects_missing_signal(self):
        backend = FusionHatBackend(('P0', 'P1'))
        self.assertFalse(backend.read_gate())
        with self.assertRaises(BackendError):
            backend.require_gate_open()

    def test_status_allows_disarmed_configuration_without_claiming_gate_open(self):
        with tempfile.TemporaryDirectory() as temporary:
            backend = MockBackend(('P0', 'P1'))
            backend.external_power_cutoff = True
            backend._caps['library'] = 'fusion_hat'
            safety = SafetyController(backend.capabilities())
            store = ConfigurationStore(Path(temporary) / 'layout.json', safety, backend)
            store.load(0)
            status = store.status(0)
            self.assertEqual(status['gate_mode'], 'external_power_cutoff')
            self.assertFalse(status['hardware_gate_open_confirmed'])
            self.assertTrue(status['configuration_allowed'])
            safety.arming = True
            self.assertFalse(store.status(0)['configuration_allowed'])
            safety.arming = False
            safety.armed = True
            self.assertFalse(store.status(0)['configuration_allowed'])

    def test_two_esc_layout_can_be_staged_and_committed_without_arming(self):
        with tempfile.TemporaryDirectory() as temporary:
            backend = MockBackend(('P0', 'P1'))
            backend.external_power_cutoff = True
            backend._caps['library'] = 'fusion_hat'
            safety = SafetyController(backend.capabilities())
            store = ConfigurationStore(Path(temporary) / 'layout.json', safety, backend)
            store.load(0)
            store.begin_connection(123, 0)
            layout = dict(schema_version=1, revision=0, actuators=[dict(
                id=index, name=name, port=port, kind='bidirectional_esc', inverted=False,
                limits=dict(min=-1, max=1),
                calibration=dict(type='bidirectional_esc', reverse_us=1000, neutral_us=1500,
                                 forward_us=2000, arming_duration_ms=2000),
                route=dict(type=role), safe=dict(type='zero'))
                for index, name, port, role in [(0, 'Left ESC', 'P0', 'left_effort'),
                                               (1, 'Right ESC', 'P1', 'right_effort')]])
            staged = store.handle(dict(schema_version=1, request_id=1, operation='stage_layout',
                                       payload=dict(layout=layout)), .1)
            self.assertEqual(staged['result'], 'ok', staged)
            committed = store.handle(dict(schema_version=1, request_id=2, operation='commit_layout',
                                          payload=dict(staged_revision=0, staged_request_id=1)), .1)
            self.assertEqual(committed['result'], 'ok', committed)
            self.assertEqual(json.loads(store.path.read_text())['revision'], 1)
            self.assertFalse(store.status(.1)['armed'])
            safety.set_status_subscribed(True, .1)
            safety.tick(.1, backend.read_gate())
            self.assertFalse(safety.accept(CommandFrame(CommandKind.ARM, 123, 1, 1), .11, .11))
            safe = (ActuatorValue(0, 0), ActuatorValue(1, 0))
            self.assertTrue(safety.accept(CommandFrame(CommandKind.DRIVE, 123, 1, 1, safe), .12, .12))
            self.assertTrue(safety.accept(CommandFrame(CommandKind.ARM, 123, 1, 2), .13, .13))
            self.assertFalse(store.status(.13)['configuration_allowed'])
            for sequence in range(3, 25):
                now = .14 + (sequence - 3) * .1
                self.assertTrue(safety.accept(CommandFrame(CommandKind.DRIVE, 123, 1, sequence, safe), now, now))
                safety.tick(now, backend.read_gate())
            self.assertTrue(store.status(2.24)['armed'])
            self.assertFalse(store.status(2.24)['configuration_allowed'])
            batch = safety.tick(2.45, backend.read_gate())
            self.assertFalse(batch.armed)
            self.assertEqual(batch.reason, 'watchdog')
            self.assertTrue(all(command.value == 0 for command in batch.commands))


if __name__ == '__main__':
    unittest.main()
