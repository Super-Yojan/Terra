"""Standalone Linux peripheral entry point."""
import argparse
import asyncio
import json
import os
from pathlib import Path
import tempfile
import time
import sys

from dbus_next import BusType, DBusError, Variant, PropertyAccess
from dbus_next.aio import MessageBus
from dbus_next.service import ServiceInterface, method, dbus_property
from .backend import MockBackend, FusionHatBackend, file_gate_reader
from .ble import (BlePeripheral, ROOT, SERVICE, SERVICE_UUID, STATUS_UUID,
                  GattService, ObjectManager, Advertisement)

class SetupAgent(ServiceInterface):
    def __init__(self, expected, deadline):
        super().__init__('org.bluez.Agent1'); self.expected = expected; self.deadline = deadline
        self.confirmed = set()
    def check(self, device):
        if time.monotonic() >= self.deadline or not device.endswith('/dev_'+self.expected.replace(':', '_')):
            raise DBusError('org.bluez.Error.Rejected', 'outside expected owner setup')
    @method()
    async def RequestConfirmation(self, device: 'o', passkey: 'u'):
        self.check(device)
        print(f'Confirm {self.expected} displays {passkey:06d} (type yes): ', end='', flush=True)
        loop = asyncio.get_running_loop()
        response = loop.create_future()
        def ready():
            if not response.done(): response.set_result(sys.stdin.readline())
        loop.add_reader(sys.stdin.fileno(), ready)
        try:
            answer = await asyncio.wait_for(response, max(0, self.deadline-time.monotonic()))
        except asyncio.TimeoutError as exc:
            raise DBusError('org.bluez.Error.Rejected', 'setup expired') from exc
        finally: loop.remove_reader(sys.stdin.fileno())
        self.check(device)
        if answer.strip() != 'yes': raise DBusError('org.bluez.Error.Rejected', 'local confirmation refused')
        self.confirmed.add(device)
    @method()
    def RequestAuthorization(self, device: 'o'):
        raise DBusError('org.bluez.Error.Rejected', 'numeric confirmation required')
    @method()
    def AuthorizeService(self, device: 'o', uuid: 's'):
        self.check(device)
        if device not in self.confirmed: raise DBusError('org.bluez.Error.Rejected', 'confirm pairing locally')
    @method()
    def Cancel(self): self.confirmed.clear()
    @method()
    def Release(self): self.confirmed.clear()

class SetupStatus(ServiceInterface):
    """Encrypted read initiates pairing; no output backend or write API exists."""
    def __init__(self, agent, gate_file):
        super().__init__('org.bluez.GattCharacteristic1')
        self.agent, self.gate_file = agent, gate_file
    @dbus_property(access=PropertyAccess.READ)
    def UUID(self) -> 's': return STATUS_UUID
    @dbus_property(access=PropertyAccess.READ)
    def Service(self) -> 'o': return SERVICE
    @dbus_property(access=PropertyAccess.READ)
    def Flags(self) -> 'as': return ['read', 'encrypt-read']
    @method()
    def ReadValue(self, options: 'a{sv}') -> 'ay':
        device = options.get('device')
        if device is None: raise DBusError('org.bluez.Error.NotAuthorized', 'device required')
        self.agent.check(device.value)
        if device.value not in self.agent.confirmed:
            raise DBusError('org.bluez.Error.NotAuthorized', 'local numeric confirmation required')
        if self.gate_file.read_text().strip() != '0':
            raise DBusError('org.bluez.Error.NotAuthorized', 'physical gate must be open')
        data = b'{"schema_version":1,"type":"setup","armed":false,"pairing_confirmed":true}'
        offset = options.get('offset', Variant('q', 0)).value
        if offset > len(data): raise DBusError('org.bluez.Error.InvalidOffset', 'offset')
        return data[offset:]

async def setup(args):
    if args.owner.exists(): raise RuntimeError('owner already provisioned; remove owner explicitly to replace')
    if not args.expected_peer: raise RuntimeError('--expected-peer Bluetooth address is required')
    if args.gate_file is None or file_gate_reader(args.gate_file)():
        raise RuntimeError('setup requires a readable physical gate file showing 0')
    if args.gate_file.read_text().strip() != '0': raise RuntimeError('gate must explicitly show 0')
    bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
    manager = bus.get_proxy_object('org.bluez', '/', await bus.introspect('org.bluez', '/')).get_interface('org.freedesktop.DBus.ObjectManager')
    objects = await manager.call_get_managed_objects()
    adapter = args.adapter or next(p for p,i in objects.items() if 'org.bluez.Adapter1' in i)
    proxy = bus.get_proxy_object('org.bluez', adapter, await bus.introspect('org.bluez', adapter))
    properties = proxy.get_interface('org.freedesktop.DBus.Properties')
    agents = bus.get_proxy_object('org.bluez', '/org/bluez', await bus.introspect('org.bluez', '/org/bluez')).get_interface('org.bluez.AgentManager1')
    deadline = time.monotonic()+args.setup_seconds
    agent = SetupAgent(args.expected_peer.upper(), deadline)
    path = ROOT+'/agent'; bus.export(path, agent)
    registered = gatt_registered = advertised = False
    gatt = proxy.get_interface('org.bluez.GattManager1')
    advertising = proxy.get_interface('org.bluez.LEAdvertisingManager1')
    status_path = SERVICE+'/status'
    bus.export(SERVICE, GattService())
    bus.export(status_path, SetupStatus(agent, args.gate_file))
    tree = {SERVICE: {'org.bluez.GattService1': {'UUID': Variant('s', SERVICE_UUID), 'Primary': Variant('b', True)}},
            status_path: {'org.bluez.GattCharacteristic1': {'UUID': Variant('s', STATUS_UUID), 'Service': Variant('o', SERVICE), 'Flags': Variant('as', ['read', 'encrypt-read'])}}}
    bus.export(ROOT, ObjectManager(tree))
    bus.export(ROOT+'/advertisement', Advertisement(args.name))
    try:
        await agents.call_register_agent(path, 'DisplayYesNo'); registered = True
        await agents.call_request_default_agent(path)
        await properties.call_set('org.bluez.Adapter1', 'Powered', Variant('b', True))
        await properties.call_set('org.bluez.Adapter1', 'PairableTimeout', Variant('u', args.setup_seconds))
        await properties.call_set('org.bluez.Adapter1', 'DiscoverableTimeout', Variant('u', args.setup_seconds))
        await properties.call_set('org.bluez.Adapter1', 'Pairable', Variant('b', True))
        await properties.call_set('org.bluez.Adapter1', 'Discoverable', Variant('b', True))
        await gatt.call_register_application(ROOT, {}); gatt_registered = True
        await advertising.call_register_advertisement(ROOT+'/advertisement', {}); advertised = True
        print('Pair the expected phone now; compare the numeric code locally.', flush=True)
        while time.monotonic() < deadline:
            if args.gate_file.read_text().strip() != '0': raise RuntimeError('gate changed during setup')
            objects = await manager.call_get_managed_objects()
            for device in agent.confirmed:
                fields = objects.get(device, {}).get('org.bluez.Device1', {})
                if fields.get('Bonded', Variant('b', False)).value and fields.get('Paired', Variant('b', False)).value:
                    identity = dict(address=fields['Address'].value, address_type=fields['AddressType'].value)
                    fd, temporary = tempfile.mkstemp(dir=args.owner.parent, prefix='.owner-')
                    try:
                        with os.fdopen(fd, 'w') as stream:
                            json.dump(identity, stream); stream.flush(); os.fsync(stream.fileno())
                        os.replace(temporary, args.owner)
                        directory = os.open(args.owner.parent, os.O_RDONLY | os.O_DIRECTORY)
                        try: os.fsync(directory)
                        finally: os.close(directory)
                    finally:
                        if os.path.exists(temporary): os.unlink(temporary)
                    print('Owner saved. Restart the normal service.', flush=True); return
            await asyncio.sleep(.25)
        raise RuntimeError('owner setup expired')
    finally:
        # Attempt every cleanup even if BlueZ lost its bus/adapter mid-setup.
        for operation in (
            lambda: properties.call_set('org.bluez.Adapter1', 'Pairable', Variant('b', False)),
            lambda: properties.call_set('org.bluez.Adapter1', 'Discoverable', Variant('b', False)),
            lambda: advertising.call_unregister_advertisement(ROOT+'/advertisement') if advertised else asyncio.sleep(0),
            lambda: gatt.call_unregister_application(ROOT) if gatt_registered else asyncio.sleep(0),
            lambda: agents.call_unregister_agent(path) if registered else asyncio.sleep(0)):
            try: await operation()
            except Exception: pass
        bus.disconnect()

def main():
    parser = argparse.ArgumentParser(description='Terra owner-only actuator peripheral')
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--mock-gate-closed', action='store_true', help='explicit simulated motion interlock')
    parser.add_argument('--name', default='Terra Rover')
    parser.add_argument('--config', type=Path, default=Path('/var/lib/terra-rover/layout.json'))
    parser.add_argument('--owner', type=Path, default=Path('/var/lib/terra-rover/owner.json'))
    parser.add_argument('--pwm-ports', default='', help='comma-separated physically exposed P0-P11 ports')
    parser.add_argument('--gate-file', type=Path)
    parser.add_argument('--adapter')
    parser.add_argument('--setup-owner', action='store_true')
    parser.add_argument('--expected-peer')
    parser.add_argument('--setup-seconds', type=int, default=60)
    args = parser.parse_args()
    if not 1 <= args.setup_seconds <= 60: parser.error('setup seconds must be 1 through 60')
    if args.setup_owner: asyncio.run(setup(args)); return
    ports = tuple(p.strip() for p in args.pwm_ports.split(',') if p.strip())
    if args.mock:
        backend = MockBackend(ports); backend.gate = args.mock_gate_closed
    else:
        if args.gate_file is None: parser.error('real hardware requires --gate-file')
        backend = FusionHatBackend(ports, file_gate_reader(args.gate_file))
    asyncio.run(BlePeripheral(args.name, args.adapter).run(backend, args.config, args.owner))

if __name__ == '__main__': main()
