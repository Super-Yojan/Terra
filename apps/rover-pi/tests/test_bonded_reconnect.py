"""A bonded owner's reconnect must keep status/reply notifications flowing.

BlueZ keeps the CCC state of a bonded device across disconnects and answers an
identical CCC write on reconnect without calling StartNotify. The rover must
not mute its own notifications when the owner disconnects.
"""
import unittest
from types import SimpleNamespace
from dbus_next import Variant
from terra_rover.ble import BlePeripheral


class BondedReconnectTests(unittest.TestCase):
    def setUp(self):
        self.rover = BlePeripheral()
        self.path = '/org/bluez/hci0/dev_owner'
        self.rover.owner = {'address': 'AA:BB:CC:DD:EE:FF', 'address_type': 'public'}
        self.rover.devices[self.path] = dict(Address=self.rover.owner['address'],
            AddressType='public', Connected=True, Bonded=True, Paired=True)
        self.sent = {'status': [], 'reply': []}
        self.rover.chars = {role: SimpleNamespace(notifying=True, publish=self.sent[role].append)
                            for role in ('status', 'reply')}
        self.options = {'device': Variant('o', self.path), 'mtu': Variant('q', 185)}

    def test_notifications_survive_owner_reconnect(self):
        self.rover.authorize(self.options)
        self.rover.disconnect(self.path, 0)
        # Reconnect: BlueZ does not call StartNotify again for the bonded owner.
        self.rover.authorize(self.options)
        self.assertTrue(self.rover.chars['status'].notifying)
        self.rover._emit('status', {'type': 'status', 'connected': True}, self.rover.generation)
        self.rover._emit('reply', {'request_id': 1}, self.rover.generation)
        self.assertTrue(self.sent['status'])
        self.assertTrue(self.sent['reply'])


if __name__ == '__main__':
    unittest.main()
