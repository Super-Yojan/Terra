import json
from pathlib import Path
import tempfile
import unittest
from terra_rover.backend import BenchGate, MockBackend
from terra_rover.configuration import ConfigurationStore
from terra_rover.safety import SafetyController
from terra_rover.protocol import decode_control, ProtocolError

class BenchModeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.backend = MockBackend()
        self.backend.bench_gate = BenchGate()
        self.backend._gate_reader = self.backend.bench_gate
        self.safety = SafetyController(self.backend.capabilities())
        path = Path(self.temp.name) / 'layout.json'
        layout = json.loads((Path(__file__).resolve().parents[3] / 'crates/terra-actuators/presets/terra-mini.json').read_text())
        path.write_text(json.dumps(layout))
        self.store = ConfigurationStore(path, self.safety, self.backend)
        self.store.load(0)
        self.store.begin_connection(123, 0)
        self.request_id = 0
    def enable(self, enabled):
        self.request_id += 1
        return self.store.handle(dict(schema_version=1, request_id=self.request_id, operation='set_bench_enabled', payload=dict(enabled=enabled)), .1)
    def test_explicit_enable_never_arms_and_resets_on_session_change(self):
        self.assertFalse(self.backend.bench_gate())
        self.assertEqual(self.enable(True)['result'], 'ok')
        self.assertTrue(self.backend.bench_gate())
        self.assertFalse(self.store.status(.1)['armed'])
        self.assertEqual(self.store.status(.1)['gate_mode'], 'bench')
        self.store.end_connection(.2)
        self.assertFalse(self.backend.bench_gate())
        self.store.begin_connection(456, .3)
        self.assertFalse(self.backend.bench_gate())
    def test_disable_disarms(self):
        self.enable(True)
        self.assertEqual(self.enable(False)['result'], 'ok')
        self.assertFalse(self.backend.bench_gate())
        self.assertFalse(self.store.status(.1)['armed'])
    def test_fault_blocks_enable_but_not_disable(self):
        self.safety.latch_fault('test', 0)
        self.assertEqual(self.enable(True)['result'], 'error')
        self.assertFalse(self.backend.bench_gate())
        self.assertEqual(self.enable(False)['result'], 'ok')
    def test_unconnected_owner_cannot_enable(self):
        self.store.end_connection(0)
        self.assertEqual(self.enable(True)['result'], 'error')
        self.assertFalse(self.backend.bench_gate())
    def test_physical_mode_cannot_be_overridden(self):
        del self.backend.bench_gate
        self.assertEqual(self.enable(True)['result'], 'error')
    def test_protocol_rejects_non_boolean_enable(self):
        for value in (1, 'true', None):
            with self.assertRaises(ProtocolError):
                decode_control(json.dumps(dict(schema_version=1, request_id=1, operation='set_bench_enabled',payload=dict(enabled=value))).encode())
