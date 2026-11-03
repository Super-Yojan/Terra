"""A drive command holds for a short lease so late Bluetooth packets do not stop the rover."""
import unittest

from terra_rover.drive_relay import DRIVE_COMMAND_TIMEOUT_SECONDS, DriveRelay
from terra_rover.protocol import ActuatorValue

def layout():
    return dict(schema_version=1, revision=1, actuators=[dict(
        id=index, name=name, port=port, kind='dc_motor', inverted=False, limits=dict(min=-1, max=1),
        calibration=dict(type='dc_motor', max_power_fraction=1.0), route=dict(type=name), safe=dict(type='zero'))
        for index, (name, port) in enumerate([('left_effort', 'M0'), ('right_effort', 'M1')])])

def efforts(batch): return [command.value for command in batch.commands]

class DriveTimeoutTests(unittest.TestCase):
    def setUp(self):
        self.relay = DriveRelay()
        self.relay.configure(layout())
        self.relay.connect()

    def test_command_holds_across_output_loops_until_timeout(self):
        self.assertTrue(self.relay.accept((ActuatorValue(0, 50), ActuatorValue(1, 50)), now=10.0))
        for now in (10.0, 10.05, 10.1, 10.0 + DRIVE_COMMAND_TIMEOUT_SECONDS):
            self.assertEqual(efforts(self.relay.output(now)), [.5, .5])
        self.assertEqual(efforts(self.relay.output(10.0 + DRIVE_COMMAND_TIMEOUT_SECONDS + .01)), [0.0, 0.0])
        self.assertEqual(self.relay.reason, 'no_command')

    def test_neutral_command_and_disconnect_stop_immediately(self):
        self.relay.accept((ActuatorValue(0, 50), ActuatorValue(1, 50)), now=1.0)
        self.relay.accept((ActuatorValue(0, 0), ActuatorValue(1, 0)), now=1.02)
        self.assertEqual(efforts(self.relay.output(1.03)), [0.0, 0.0])
        self.relay.accept((ActuatorValue(0, 50), ActuatorValue(1, 50)), now=2.0)
        self.relay.disconnect()
        self.assertEqual(efforts(self.relay.output(2.01)), [0.0, 0.0])

if __name__ == '__main__': unittest.main()
