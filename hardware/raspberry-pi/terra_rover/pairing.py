"""Time-limited, physically authorized first-owner enrollment over BlueZ."""
import asyncio
import time
from dbus_next import BusType, DBusError, Variant, PropertyAccess
from dbus_next.aio import MessageBus
from dbus_next.service import ServiceInterface, method, dbus_property
from .ble import ROOT, SERVICE, SERVICE_UUID, STATUS_UUID
from .bless_transport import BlessTransport
from .onboarding import load_owner, save_owner

class PairingCleanupError(Exception):
    """Teardown could not confirm the radio is closed; stop the service."""

class PairingSession:
    def __init__(self, deadline, preexisting_bonds, clock=time.monotonic):
        self.deadline, self.preexisting_bonds, self.clock = deadline, preexisting_bonds, clock
        self.candidate = None
        self.transport = None
        self.read_seen = False
        self.active = True

    def check(self, device):
        if not self.active or self.clock() >= self.deadline or device != self.candidate:
            raise DBusError('org.bluez.Error.Rejected', 'outside physical pairing window')

    def authorize(self, device):
        if (not self.active or self.clock() >= self.deadline or device in self.preexisting_bonds
                or self.transport not in (None, device) or self.candidate not in (None, device)):
            raise DBusError('org.bluez.Error.Rejected', 'pairing window unavailable')
        self.candidate = self.transport = device

    def read(self, device):
        self.check(device)
        self.read_seen = True

    def ready(self, fields):
        return (self.active and self.clock() < self.deadline and self.candidate is not None
                and self.read_seen and all(fields.get(k, Variant('b', False)).value
                                           for k in ('Paired', 'Bonded', 'Connected')))

    def connection_changed(self, device, connected):
        if not connected:
            if device in (self.candidate, self.transport):
                self.cancel()
            return False
        if (not self.active or self.clock() >= self.deadline
                or device in self.preexisting_bonds or self.transport not in (None, device)):
            return False
        self.transport = device
        return True

    def cancel(self):
        self.active = False
        self.read_seen = False

class ButtonPairingAgent(ServiceInterface):
    def __init__(self, session):
        super().__init__('org.bluez.Agent1')
        self.session = session

    @method()
    def RequestAuthorization(self, device: 'o'):
        self.session.authorize(device)

    @method()
    def RequestConfirmation(self, device: 'o', passkey: 'u'):
        # NoInputNoOutput uses Just Works; never accept an unexpected fallback.
        raise DBusError('org.bluez.Error.Rejected', 'unexpected confirmation method')

    @method()
    def AuthorizeService(self, device: 'o', uuid: 's'):
        self.session.check(device)
        if uuid.lower() != SERVICE_UUID.lower():
            raise DBusError('org.bluez.Error.Rejected', 'only Terra enrollment permitted')

    @method()
    def Cancel(self):
        self.session.cancel()

    @method()
    def Release(self):
        self.session.cancel()

class ButtonSetupStatus(ServiceInterface):
    def __init__(self, session, commit):
        super().__init__('org.bluez.GattCharacteristic1')
        self.session, self.commit = session, commit

    @dbus_property(access=PropertyAccess.READ)
    def UUID(self) -> 's': return STATUS_UUID
    @dbus_property(access=PropertyAccess.READ)
    def Service(self) -> 'o': return SERVICE
    @dbus_property(access=PropertyAccess.READ)
    def Flags(self) -> 'as': return ['read', 'encrypt-read']

    @method()
    async def ReadValue(self, options: 'a{sv}') -> 'ay':
        device = options.get('device')
        if device is None:
            raise DBusError('org.bluez.Error.NotAuthorized', 'device required')
        self.session.check(device.value)
        data = b'{"schema_version":1,"type":"setup","armed":false,"pairing_confirmed":true}'
        offset = options.get('offset', Variant('q', 0)).value
        if offset > len(data):
            raise DBusError('org.bluez.Error.InvalidOffset', 'offset')
        self.session.read(device.value)
        await self.commit(device.value)
        return data[offset:]

async def pair_owner(args, indicators, name):
    if load_owner(args.owner) is not None:
        raise RuntimeError('owner already provisioned')
    transport = BlessTransport(name, SERVICE_UUID, args.adapter)
    await transport.prepare()
    bus = transport.bus
    session = None
    properties = agents = gatt = advertising = adapter_interface = None
    registered = gatt_registered = advertised = saved = False
    path, advertisement = ROOT + '/agent', ROOT + '/advertisement'
    try:
        manager = bus.get_proxy_object('org.bluez', '/', await bus.introspect('org.bluez', '/')).get_interface('org.freedesktop.DBus.ObjectManager')
        objects = await manager.call_get_managed_objects()
        adapter = transport.adapter.path
        proxy = bus.get_proxy_object('org.bluez', adapter, await bus.introspect('org.bluez', adapter))
        properties = proxy.get_interface('org.freedesktop.DBus.Properties')
        adapter_interface = proxy.get_interface('org.bluez.Adapter1')
        gatt = proxy.get_interface('org.bluez.GattManager1')
        advertising = proxy.get_interface('org.bluez.LEAdvertisingManager1')
        agents = bus.get_proxy_object('org.bluez', '/org/bluez', await bus.introspect('org.bluez', '/org/bluez')).get_interface('org.bluez.AgentManager1')
        preexisting = {p for p,i in objects.items() if any(i.get('org.bluez.Device1', {}).get(k, Variant('b',False)).value for k in ('Paired','Bonded'))}
        session = PairingSession(time.monotonic() + args.setup_seconds, preexisting)
        def changed(message):
            if (message.interface == 'org.freedesktop.DBus.Properties'
                    and message.member == 'PropertiesChanged'
                    and message.body[0] == 'org.bluez.Device1'):
                updates = message.body[1]
                if 'Connected' in updates and not saved:
                    session.connection_changed(message.path, updates['Connected'].value)
        bus.add_message_handler(changed)
        dbus = bus.get_proxy_object('org.freedesktop.DBus', '/org/freedesktop/DBus', await bus.introspect('org.freedesktop.DBus', '/org/freedesktop/DBus')).get_interface('org.freedesktop.DBus')
        await dbus.call_add_match("type='signal',sender='org.bluez',interface='org.freedesktop.DBus.Properties',member='PropertiesChanged',path_namespace='" + adapter + "'")
        manager.on_interfaces_removed(lambda device, interfaces: session.connection_changed(device,False) if 'org.bluez.Device1' in interfaces and not saved else None)
        bus.export(path, ButtonPairingAgent(session))
        status_path = SERVICE + '/status'
        async def commit(device):
            nonlocal saved
            if saved:
                session.check(device)
                return
            current = await manager.call_get_managed_objects()
            fields = current.get(device, {}).get('org.bluez.Device1', {})
            indicators.pressed()
            if not session.ready(fields):
                raise DBusError('org.bluez.Error.NotAuthorized', 'connected encrypted bond required')
            save_owner(args.owner, {'address':fields['Address'].value, 'address_type':fields['AddressType'].value})
            saved = True
        status = ButtonSetupStatus(session, commit)
        async def read(options):
            # Bless handles offset slicing once, after the enrollment transaction.
            await ButtonSetupStatus.ReadValue.__wrapped__(status, options)
            return b'{"schema_version":1,"type":"setup","armed":false,"pairing_confirmed":true}'
        await transport.add(STATUS_UUID, read=read)
        await agents.call_register_agent(path, 'NoInputNoOutput'); registered = True
        await agents.call_request_default_agent(path)
        for key, value in [('Powered', Variant('b',True)), ('PairableTimeout',Variant('u',args.setup_seconds)),
                           ('DiscoverableTimeout',Variant('u',args.setup_seconds)), ('Pairable',Variant('b',True)), ('Discoverable',Variant('b',True))]:
            await properties.call_set('org.bluez.Adapter1', key, value)
        await transport.start()
        print(f'Pairing window open: {name}', flush=True)
        while session.active and time.monotonic() < session.deadline:
            indicators.pressed()  # Invalid/unavailable hardware cancels enrollment.
            current = await manager.call_get_managed_objects()
            for device, interfaces in current.items():
                fields = interfaces.get('org.bluez.Device1', {})
                if fields.get('Connected',Variant('b',False)).value and not session.connection_changed(device,True):
                    peer = bus.get_proxy_object('org.bluez',device,await bus.introspect('org.bluez',device)).get_interface('org.bluez.Device1')
                    await peer.call_disconnect()
            now = time.monotonic()
            indicators.set_led(int(now * (8 if session.candidate else 2)) % 2 == 0)
            if saved:
                # Let the successful encrypted response reach the phone before teardown.
                await asyncio.sleep(.3)
                print('Owner saved; switching to normal rover service.', flush=True)
                return True
            await asyncio.sleep(.05)
        return False
    finally:
        if session: session.cancel()
        operations = []
        if properties:
            operations.extend(lambda key=key: properties.call_set('org.bluez.Adapter1',key,Variant('b',False)) for key in ('Pairable','Discoverable'))
        if registered: operations.append(lambda: agents.call_unregister_agent(path))
        if session and session.candidate and not saved and session.candidate not in session.preexisting_bonds and adapter_interface:
            operations.append(lambda: adapter_interface.call_remove_device(session.candidate))
        failures = []
        for operation in operations:
            try: await asyncio.wait_for(operation(), 2)
            except Exception as exc: failures.append(str(exc))
        try: await transport.close()
        except Exception as exc: failures.append(str(exc))
        try: indicators.set_led(False)
        except OSError as exc: failures.append(str(exc))
        if failures:
            raise PairingCleanupError('Bluetooth cleanup failed: ' + '; '.join(failures))
