"""The rover decoder agrees with the control contract fixture that terra-actuators also reads."""
import json
import pathlib
import unittest

from terra_rover.protocol import DRIVE_LIMIT, JSON_LIMIT, ProtocolError, decode_control

FIXTURE = pathlib.Path(__file__).resolve().parents[3] / 'tests/fixtures/actuators/control_operations.json'


@unittest.skipUnless(FIXTURE.exists(), 'shared fixture is only present in the repository checkout')
class ControlContractTests(unittest.TestCase):
    def setUp(self):
        self.fixture = json.loads(FIXTURE.read_text())

    def test_limits_match_the_shared_contract(self):
        self.assertEqual(JSON_LIMIT, self.fixture['json_limit'])
        self.assertEqual(DRIVE_LIMIT, 2 * self.fixture['max_actuators'])

    def test_operations_match_the_shared_contract(self):
        for case in self.fixture['envelopes']:
            envelope = dict(schema_version=1, request_id=1, operation=case['operation'], payload=case['payload'])
            try:
                decode_control(json.dumps(envelope).encode())
                accepted = True
            except ProtocolError:
                accepted = False
            self.assertEqual(accepted, case['pi'], case['operation'])


if __name__ == '__main__':
    unittest.main()
