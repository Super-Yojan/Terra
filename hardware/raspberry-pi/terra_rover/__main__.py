"""Standalone Linux peripheral entry point."""
import argparse
import asyncio
import json
import os
from pathlib import Path
import tempfile
import time
import sys
from importlib import metadata

from dbus_next import BusType, DBusError, Variant, PropertyAccess
from dbus_next.aio import MessageBus
from dbus_next.service import ServiceInterface, method, dbus_property
from .backend import MockBackend, FusionHatBackend, file_gate_reader
from .ble import (BlePeripheral, ROOT, SERVICE, SERVICE_UUID, STATUS_UUID,
                  )
from .bless_transport import BlessTransport

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
    transport = BlessTransport(args.name, SERVICE_UUID, args.adapter)
    await transport.prepare()
    bus = transport.bus
    registered = False
    properties = agents = None
    path = ROOT + "/agent"
    try:
        manager = bus.get_proxy_object('org.bluez', '/', await bus.introspect('org.bluez', '/')).get_interface('org.freedesktop.DBus.ObjectManager')
        objects = await manager.call_get_managed_objects()
        adapter = transport.adapter.path
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
        status = SetupStatus(agent, args.gate_file)
        def read(options):
            SetupStatus.ReadValue.__wrapped__(status, options)
            return b'{"schema_version":1,"type":"setup","armed":false,"pairing_confirmed":true}'
        await transport.add(STATUS_UUID, read=read)
        await agents.call_register_agent(path, 'DisplayYesNo'); registered = True
        await agents.call_request_default_agent(path)
        await properties.call_set('org.bluez.Adapter1', 'Powered', Variant('b', True))
        await properties.call_set('org.bluez.Adapter1', 'PairableTimeout', Variant('u', args.setup_seconds))
        await properties.call_set('org.bluez.Adapter1', 'DiscoverableTimeout', Variant('u', args.setup_seconds))
        await properties.call_set('org.bluez.Adapter1', 'Pairable', Variant('b', True))
        await properties.call_set('org.bluez.Adapter1', 'Discoverable', Variant('b', True))
        await transport.start()
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
            lambda: properties.call_set('org.bluez.Adapter1', 'Pairable', Variant('b', False)) if properties else asyncio.sleep(0),
            lambda: properties.call_set('org.bluez.Adapter1', 'Discoverable', Variant('b', False)) if properties else asyncio.sleep(0),
            lambda: agents.call_unregister_agent(path) if registered else asyncio.sleep(0)):
            try: await operation()
            except Exception: pass
        await transport.close()

def main():
    parser = argparse.ArgumentParser(description='Terra owner-only actuator peripheral')
    parser.add_argument('--check-bundle', action='store_true', help='verify bundled driver and D-Bus libraries without accessing hardware')
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--mock-gate-closed', action='store_true', help='explicit simulated motion interlock')
    parser.add_argument('--name', default='Terra Rover')
    parser.add_argument('--config', type=Path, default=Path('/var/lib/terra-rover/layout.json'))
    parser.add_argument('--owner', type=Path, default=Path('/var/lib/terra-rover/owner.json'))
    parser.add_argument('--pwm-ports', default='', help='comma-separated physically exposed P0-P11 ports')
    parser.add_argument('--gate-file', type=Path)
    parser.add_argument('--adapter')
    parser.add_argument('--button-pairing', action='store_true', help='Fusion HAT button first-owner enrollment; then normal operation')
    parser.add_argument('--button-file', type=Path, default=Path('/sys/class/fusion_hat/fusion_hat/button'))
    parser.add_argument('--led-file', type=Path, default=Path('/sys/class/fusion_hat/fusion_hat/led'))
    parser.add_argument('--device-name-file', type=Path, default=Path('/var/lib/terra-rover/device-name.json'))
    parser.add_argument('--setup-owner', action='store_true')
    parser.add_argument('--expected-peer')
    parser.add_argument('--setup-seconds', type=int, default=60)
    args = parser.parse_args()
    if args.check_bundle:
        # Only imports and API checks: no PWM objects, radio connection or outputs.
        backend = FusionHatBackend()
        backend._prepare()
        if metadata.version('dbus-next') != '0.2.3':
            raise RuntimeError('unsupported dbus-next version')
        for package, version in [('bless', '0.3.0'), ('bleak', '1.1.1')]:
            if metadata.version(package) != version: raise RuntimeError('unsupported ' + package + ' version')
        print(json.dumps({'bless': metadata.version('bless'), 'bleak': metadata.version('bleak'), 'fusion_hat': backend.capabilities()['library_version'],
                          'dbus-next': metadata.version('dbus-next')}))
        return
    if not 1 <= args.setup_seconds <= 60: parser.error('setup seconds must be 1 through 60')
    if args.setup_owner and args.button_pairing: parser.error('choose one enrollment mode')
    if args.setup_owner: asyncio.run(setup(args)); return
    if args.button_pairing:
        from .lifecycle import run_customer_service
        try:
            asyncio.run(run_customer_service(args, run_normal))
        except asyncio.CancelledError:
            return
    else:
        from .onboarding import load_owner
        if load_owner(args.owner) is None:
            parser.error('no owner saved; use --button-pairing for customer enrollment or --setup-owner for terminal setup')
        asyncio.run(run_normal(args, args.name))

async def run_normal(args, name):
    ports = tuple(p.strip() for p in args.pwm_ports.split(',') if p.strip())
    if args.mock:
        backend = MockBackend(ports); backend.gate = args.mock_gate_closed
    else:
        if args.gate_file is None: raise RuntimeError('real hardware requires --gate-file')
        backend = FusionHatBackend(ports, file_gate_reader(args.gate_file))
    await BlePeripheral(name, args.adapter).run(backend, args.config, args.owner)

if __name__ == '__main__': main()
