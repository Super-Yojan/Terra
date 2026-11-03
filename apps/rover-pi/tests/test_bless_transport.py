import unittest
from types import SimpleNamespace
from dbus_next import DBusError, Variant
from dbus_next.service import ServiceInterface
from bless.backends.bluezdbus.dbus.characteristic import BlueZGattCharacteristic, Flags
from terra_rover.bless_transport import PeerCharacteristic

class BlessTransportTests(unittest.IsolatedAsyncioTestCase):
    def make(self, flags, read=None, write=None, notify=None):
        app=SimpleNamespace(subscribed_characteristics=[],Read=None,Write=None)
        service=SimpleNamespace(path='/service',app=app)
        original=BlueZGattCharacteristic('7e5a0012-4c2b-4f91-9e3a-1d8c6b2a0f10',flags,1,service)
        return PeerCharacteristic(original, read, write, notify)
    async def test_encrypted_read_preserves_peer_and_slices_long_read(self):
        seen=[]
        async def read(options): seen.append(options['device'].value); return b'abcdef'
        char=self.make([Flags.ENCRYPT_READ],read=read)
        options={'device':Variant('o','/phone'),'offset':Variant('q',2)}
        self.assertEqual(await PeerCharacteristic.ReadValue.__wrapped__(char,options),b'cdef')
        self.assertEqual(seen,['/phone'])
        self.assertEqual(char.name,'org.bluez.GattCharacteristic1')
        self.assertIn('encrypt-read',char.Flags)
    async def test_missing_peer_fails_closed(self):
        char=self.make([Flags.ENCRYPT_READ],read=lambda options:b'private')
        with self.assertRaises(DBusError): await PeerCharacteristic.ReadValue.__wrapped__(char,{})
    async def test_write_preserves_peer_and_rejects_prepared_writes(self):
        seen=[]
        char=self.make([Flags.ENCRYPT_WRITE],write=lambda value,options:seen.append((value,options['device'].value)))
        options={'device':Variant('o','/phone')}
        await PeerCharacteristic.WriteValue.__wrapped__(char,b'drive',options)
        self.assertEqual(seen,[(b'drive','/phone')])
        with self.assertRaises(DBusError): await PeerCharacteristic.WriteValue.__wrapped__(char,b'drive',{**options,'offset':Variant('q',1)})
    async def test_subscription_requires_owner_before_notifying(self):
        def notify(enabled): raise DBusError('org.bluez.Error.NotAuthorized','not owner')
        char=self.make([Flags.ENCRYPT_READ,Flags.NOTIFY],notify=notify)
        self.assertFalse(char.Notifying)
        with self.assertRaises(DBusError): PeerCharacteristic.StartNotify.__wrapped__(char)
        self.assertFalse(char.Notifying)
    async def test_plaintext_characteristic_is_rejected(self):
        with self.assertRaises(ValueError): self.make([Flags.READ],read=lambda options:b'private')

    async def test_prepared_write_never_executes_command_callback(self):
        writes=[]; invalid=[]
        char=self.make([Flags.ENCRYPT_WRITE],write=lambda value,options:writes.append(value))
        char.invalid_write=lambda options: invalid.append(options['device'].value)
        with self.assertRaises(DBusError):
            await PeerCharacteristic.WriteValue.__wrapped__(char,b'command',
                {'device':Variant('o','/phone'),'prepare-authorize':Variant('b',True)})
        self.assertEqual(writes,[])
        self.assertEqual(invalid,['/phone'])

    async def test_subscription_state_clears_on_disconnect(self):
        char=self.make([Flags.ENCRYPT_READ,Flags.NOTIFY],read=lambda options:b'status',notify=lambda enabled:None)
        PeerCharacteristic.StartNotify.__wrapped__(char)
        self.assertTrue(char.Notifying)
        self.assertEqual(char._service.app.subscribed_characteristics,[char.UUID])
        char.notifying=False
        self.assertFalse(char.Notifying)
        self.assertEqual(char._service.app.subscribed_characteristics,[])
