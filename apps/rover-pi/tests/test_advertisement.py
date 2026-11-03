import unittest
from types import SimpleNamespace
from dbus_next.service import ServiceInterface
from bless.backends.bluezdbus.dbus.advertisement import BlueZLEAdvertisement, Type
from terra_rover.ble import SERVICE_UUID

class AdvertisementTests(unittest.TestCase):
    def test_bless_local_name_preserves_bluez_interface_and_properties(self):
        app = SimpleNamespace(app_name='terra-F5NTN5', base_path='/org/bluez/terraF5NTN5')
        advertisement = BlueZLEAdvertisement(Type.PERIPHERAL, 1, app)
        advertisement.ServiceUUIDs = [SERVICE_UUID]
        self.assertEqual(advertisement.name, 'org.bluez.LEAdvertisement1')
        properties = {p.name:p.prop_getter(advertisement) for p in ServiceInterface._get_properties(advertisement)}
        self.assertEqual(properties['LocalName'], 'terra-F5NTN5')
        self.assertEqual(properties['ServiceUUIDs'], [SERVICE_UUID])
