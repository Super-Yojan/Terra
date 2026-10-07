"""Exercise exported enrollment and teardown against a simulated BlueZ bus."""
import asyncio
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from dbus_next import Variant
from terra_rover import pairing
from terra_rover.bless_transport import PeerCharacteristic
from bless.backends.bluezdbus import server as bless_server
from terra_rover.onboarding import HatIndicators, load_owner

class BlueZ:
    def __init__(self, failure=None, disconnect=False):
        self.path='/org/bluez/hci0'
        self.exports={}; self.calls=[]; self.failure=failure; self.disconnect_candidate=disconnect
        self.device='/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF'
        self.finished=False
    async def connect(self): return self
    async def introspect(self,*args): return None
    def get_proxy_object(self,*args): return self
    def get_interface(self,*args): return self
    def export(self,path,obj): self.exports[path]=obj
    def unexport(self,path,obj=None): self.exports.pop(path,None)
    async def call_get(self,*args): return Variant('s','rover2')
    def add_message_handler(self,handler): self.handler=handler
    def on_interfaces_removed(self,handler): pass
    async def call_add_match(self,*args): pass
    async def call_get_managed_objects(self):
        result={'/org/bluez/hci0':{k:{} for k in ('org.bluez.Adapter1','org.bluez.GattManager1','org.bluez.LEAdvertisingManager1')}}
        if self.finished:
            result[self.device]={'org.bluez.Device1':dict(Address=Variant('s','AA:BB:CC:DD:EE:FF'),AddressType=Variant('s','random'),**{k:Variant('b',True) for k in ('Paired','Bonded','Connected')})}
        return result
    async def call_register_advertisement(self,*args):
        self.calls.append('advertise')
        self.finished=True
        agent=self.exports[pairing.ROOT+'/agent']
        pairing.ButtonPairingAgent.RequestAuthorization.__wrapped__(agent,self.device)
        if self.disconnect_candidate:
            agent.session.connection_changed(self.device,False)
        else:
            status=next(obj for obj in self.exports.values() if isinstance(obj,PeerCharacteristic))
            response=await PeerCharacteristic.ReadValue.__wrapped__(status,{'device':Variant('o',self.device)})
            self.owner_present_at_response=load_owner(self.owner) is not None
            self.assert_response=response
    async def call_set(self,interface,key,value):
        self.calls.append((key,value.value))
        if self.failure==key and value.value is False: raise RuntimeError('injected cleanup failure')
    async def call_unregister_advertisement(self,*args): self.calls.append('unadvertise')
    async def call_unregister_application(self,*args): self.calls.append('unregister-gatt')
    async def call_unregister_agent(self,*args): self.calls.append('unregister-agent')
    async def call_remove_device(self,*args): self.calls.append('remove-device')
    async def call_register_agent(self,*args): pass
    async def call_request_default_agent(self,*args): pass
    async def call_register_application(self,*args): pass
    def disconnect(self): self.calls.append('disconnect-bus')

class PairingIntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def exercise(self, failure=None, disconnect=False):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); b=root/'button'; l=root/'led'; b.write_text('0'); l.touch()
            args=SimpleNamespace(owner=root/'owner', adapter=None,setup_seconds=1)
            bus=BlueZ(failure,disconnect); bus.owner=args.owner
            with patch.object(bless_server,'MessageBus',return_value=bus):
                if failure:
                    with self.assertRaises(pairing.PairingCleanupError): await pairing.pair_owner(args,HatIndicators(b,l),'terra-ABC123')
                else:
                    result=await pairing.pair_owner(args,HatIndicators(b,l),'terra-ABC123')
                    self.assertEqual(result,not disconnect)
            self.assertIn(('Pairable',False),bus.calls)
            self.assertIn(('Discoverable',False),bus.calls)
            self.assertIn('unregister-agent',bus.calls)
            self.assertIn('unregister-gatt',bus.calls)
            self.assertEqual(l.read_text(),'0')
            if disconnect:
                self.assertIsNone(load_owner(args.owner)); self.assertIn('remove-device',bus.calls)
            else:
                self.assertTrue(bus.owner_present_at_response)
                self.assertNotIn('remove-device',bus.calls)
    async def test_owner_committed_before_success_response_and_cleanup(self): await self.exercise()
    async def test_cleanup_failure_blocks_success_but_attempts_all_operations(self): await self.exercise('Pairable')
    async def test_candidate_disconnect_cancels_and_removes_new_bond(self): await self.exercise(disconnect=True)

    async def test_led_failure_cannot_mask_cleanup_failure(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); b=root/'button'; l=root/'led'; b.write_text('0'); l.touch()
            args=SimpleNamespace(owner=root/'owner',adapter=None,setup_seconds=1)
            bus=BlueZ('Pairable'); bus.owner=args.owner
            class FaultyHat(HatIndicators):
                def set_led(self,value):
                    if 'disconnect-bus' in bus.calls: raise OSError('LED vanished')
                    super().set_led(value)
            with patch.object(bless_server,'MessageBus',return_value=bus):
                with self.assertRaises(pairing.PairingCleanupError):
                    await pairing.pair_owner(args,FaultyHat(b,l),'terra-ABC123')

    async def test_bless_advertisement_has_name_and_uuid_during_registration(self):
        from dbus_next.service import ServiceInterface
        class InspectBlueZ(BlueZ):
            async def call_register_advertisement(self,path,options):
                advertisement=self.exports[path]
                properties={p.name:p.prop_getter(advertisement) for p in ServiceInterface._get_properties(advertisement)}
                if advertisement.name != 'org.bluez.LEAdvertisement1': raise AssertionError('incorrect interface')
                if properties['LocalName'] != 'terra-ABC123': raise AssertionError('missing local name')
                if properties['ServiceUUIDs'] != [pairing.SERVICE_UUID]: raise AssertionError('missing discovery UUID')
                return await super().call_register_advertisement(path,options)
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); b=root/'button'; l=root/'led'; b.write_text('0'); l.touch()
            args=SimpleNamespace(owner=root/'owner',adapter=None,setup_seconds=1)
            bus=InspectBlueZ(); bus.owner=args.owner
            with patch.object(bless_server,'MessageBus',return_value=bus):
                self.assertTrue(await pairing.pair_owner(args,HatIndicators(b,l),'terra-ABC123'))

    async def test_start_failure_disconnects_bus_and_closes_pairing(self):
        class FailingBlueZ(BlueZ):
            async def call_register_advertisement(self,*args): raise RuntimeError('radio failed')
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); b=root/'button'; l=root/'led'; b.write_text('0'); l.touch()
            args=SimpleNamespace(owner=root/'owner',adapter=None,setup_seconds=1)
            bus=FailingBlueZ(); bus.owner=args.owner
            with patch.object(bless_server,'MessageBus',return_value=bus):
                with self.assertRaisesRegex(RuntimeError,'radio failed'):
                    await pairing.pair_owner(args,HatIndicators(b,l),'terra-ABC123')
            self.assertIn('unadvertise',bus.calls)
            self.assertIn('unregister-gatt',bus.calls)
            self.assertIn('disconnect-bus',bus.calls)
            self.assertIn(('Pairable',False),bus.calls)
            self.assertIsNone(load_owner(args.owner))
