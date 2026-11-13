"""The rover decoder and layout validator agree with the contract fixtures that terra-actuators also reads."""
import json
import pathlib
import unittest

from terra_rover import ble
from terra_rover.layout import validate_layout
from terra_rover.protocol import DRIVE_LIMIT, JSON_LIMIT, MAX_ACTUATORS, PAGINATED_LIMIT, ProtocolError, decode_control

FIXTURE = pathlib.Path(__file__).resolve().parents[3] / 'tests/fixtures/actuators/control_operations.json'
LAYOUT_FIXTURE = pathlib.Path(__file__).resolve().parents[3] / 'tests/fixtures/actuators/layout_validation.json'


@unittest.skipUnless(FIXTURE.exists(), 'shared fixture is only present in the repository checkout')
class ControlContractTests(unittest.TestCase):
    def setUp(self):
        self.fixture = json.loads(FIXTURE.read_text())

    def test_limits_match_the_shared_contract(self):
        self.assertEqual(JSON_LIMIT, self.fixture['json_limit'])
        self.assertEqual(PAGINATED_LIMIT, self.fixture['paginated_limit'])
        self.assertEqual(MAX_ACTUATORS, self.fixture['max_actuators'])
        self.assertEqual(DRIVE_LIMIT, 2 * self.fixture['max_actuators'])

    def test_gatt_uuids_match_the_shared_contract(self):
        gatt = self.fixture['gatt']
        self.assertEqual(ble.SERVICE_UUID, gatt['service'])
        self.assertEqual(ble.DRIVE_UUID, gatt['drive'])
        self.assertEqual(ble.STATUS_UUID, gatt['status'])
        self.assertEqual(ble.CONTROL_UUID, gatt['control'])
        self.assertEqual(ble.REPLY_UUID, gatt['reply'])

    def test_operations_match_the_shared_contract(self):
        for case in self.fixture['envelopes']:
            envelope = dict(schema_version=1, request_id=1, operation=case['operation'], payload=case['payload'])
            try:
                decode_control(json.dumps(envelope).encode())
                accepted = True
            except ProtocolError:
                accepted = False
            self.assertEqual(accepted, case['pi'], case['operation'])

    def test_layout_validation_matches_the_shared_cases(self):
        fixture = json.loads(LAYOUT_FIXTURE.read_text())
        for case in fixture['cases']:
            codes = sorted(error['code'] for error in validate_layout(case['layout'], fixture['capabilities']))
            self.assertEqual(codes, sorted(case['errors']), case['name'])


if __name__ == '__main__':
    unittest.main()
