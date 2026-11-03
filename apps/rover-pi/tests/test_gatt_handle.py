import unittest
from types import SimpleNamespace
from dbus_next import PropertyAccess
from dbus_next.service import ServiceInterface
from bless.backends.bluezdbus.dbus.service import BlueZGattService
from terra_rover.ble import SERVICE_UUID
from terra_rover.bless_transport import StableGattService


class StableGattHandleTests(unittest.IsolatedAsyncioTestCase):
    async def test_requested_handle_is_exposed_to_bluez_and_object_manager(self):
        app = SimpleNamespace(base_path='/org/bluez/terra', bus=None, destination='org.bluez')
        original = BlueZGattService(SERVICE_UUID, True, 1, app)
        service = StableGattService(original, 0x1b)
        properties = {p.name: p for p in ServiceInterface._get_properties(service)}
        self.assertEqual(properties['Handle'].access, PropertyAccess.READWRITE)
        self.assertEqual(properties['Handle'].prop_getter(service), 0x1b)
        description = await service.get_obj()
        self.assertEqual(description['Handle'].signature, 'q')
        self.assertEqual(description['Handle'].value, 0x1b)
        properties['Handle'].prop_setter(service, 0x20)
        self.assertEqual((await service.get_obj())['Handle'].value, 0x20)
        self.assertEqual(description['UUID'].value, SERVICE_UUID)

    async def test_out_of_range_handles_are_rejected(self):
        app = SimpleNamespace(base_path='/org/bluez/terra', bus=None, destination='org.bluez')
        original = BlueZGattService(SERVICE_UUID, True, 1, app)
        for handle in (-1, 65536):
            with self.assertRaises(ValueError):
                StableGattService(original, handle)
