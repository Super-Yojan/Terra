"""Bless owns GATT and advertising; this adapter retains BlueZ peer options.

Bless 0.3.0 drops request options in its public callbacks. Keep this narrow,
version-pinned adapter so encrypted requests can still enforce owner identity.
"""
import asyncio
import inspect
import os
from dbus_next import DBusError, Variant, PropertyAccess
from dbus_next.service import method, dbus_property
from bless.backends.bluezdbus.server import BlessServerBlueZDBus
from bless.backends.bluezdbus.dbus.characteristic import BlueZGattCharacteristic, Flags
from bless.backends.attribute import GATTAttributePermissions as Permissions
from bless.backends.characteristic import GATTCharacteristicProperties as Properties
from bless.backends.bluezdbus.dbus.service import BlueZGattService


class StableGattService(BlueZGattService):
    """Optional commissioned handle prevents relocation across service restarts."""
    def __init__(self, original, handle):
        if type(handle) is not int or not 0 <= handle <= 65535:
            raise ValueError('GATT service handle must be an unsigned 16-bit integer')
        super().__init__(original.UUID, original.Primary,
                         int(original.path.rsplit('service', 1)[1], 16), original.app)
        self._requested_handle = handle

    @dbus_property(access=PropertyAccess.READWRITE)
    def Handle(self) -> 'q':
        return self._requested_handle

    @Handle.setter
    def Handle(self, value: 'q'):
        self._requested_handle = value

    async def get_obj(self):
        result = await super().get_obj()
        result['Handle'] = Variant('q', self._requested_handle)
        return result

class PeerCharacteristic(BlueZGattCharacteristic):
    def __init__(self, original, read=None, write=None, notify=None, invalid_write=None):
        if any(flag in original.Flags for flag in ('read', 'write', 'write-without-response')):
            raise ValueError('Terra characteristics require encryption')
        super().__init__(original.UUID, [Flags(flag) for flag in original.Flags],
                         int(original.path.rsplit('char', 1)[1]), original._service)
        self.read_callback, self.write_callback, self.notify_callback = read, write, notify
        self.invalid_write = invalid_write
        self._notifying = False

    @staticmethod
    def require_peer(options):
        if 'device' not in options:
            raise DBusError('org.bluez.Error.NotAuthorized', 'device required')

    @method()
    async def ReadValue(self, options: 'a{sv}') -> 'ay':
        self.require_peer(options)
        if self.read_callback is None:
            raise DBusError('org.bluez.Error.NotPermitted', 'not readable')
        data = self.read_callback(options)
        if inspect.isawaitable(data): data = await data
        offset = options.get('offset', Variant('q', 0)).value
        if offset > len(data): raise DBusError('org.bluez.Error.InvalidOffset', 'offset')
        return bytes(data[offset:])

    @method()
    async def WriteValue(self, value: 'ay', options: 'a{sv}'):
        self.require_peer(options)
        if self.write_callback is None:
            raise DBusError('org.bluez.Error.NotPermitted', 'not writable')
        # The application callback also validates fragments and disarms malformed drive.
        if options.get('offset', Variant('q', 0)).value or options.get('prepare-authorize', Variant('b', False)).value:
            # Call the authorization/safety callback before rejecting a prepared write.
            if self.invalid_write is not None:
                result = self.invalid_write(options)
                if inspect.isawaitable(result): await result
            raise DBusError('org.bluez.Error.NotSupported', 'use application fragments')
        result = self.write_callback(bytes(value), options)
        if inspect.isawaitable(result): await result

    @method()
    def StartNotify(self):
        if 'notify' not in self.Flags or self.notify_callback is None:
            raise DBusError('org.bluez.Error.NotSupported', 'not notifiable')
        self.notify_callback(True)
        self.notifying = True

    @method()
    def StopNotify(self):
        if self.notify_callback is not None: self.notify_callback(False)
        self.notifying = False

    @property
    def notifying(self): return self._notifying

    @notifying.setter
    def notifying(self, enabled):
        self._notifying = enabled
        subscribed = self._service.app.subscribed_characteristics
        if enabled and self.UUID not in subscribed: subscribed.append(self.UUID)
        if not enabled and self.UUID in subscribed: subscribed.remove(self.UUID)
        self.emit_properties_changed({'Notifying': enabled})

class BlessTransport:
    def __init__(self, name, service_uuid, adapter=None):
        self.server = BlessServerBlueZDBus(name, adapter=adapter.rsplit('/', 1)[-1] if adapter else '')
        self.service_uuid = service_uuid
        self.started = False
        self.alias = None

    async def prepare(self):
        try:
            await self.server.setup_task
            self.bus = self.server.bus
            self.adapter = self.server.adapter
            await self.server.add_new_service(self.service_uuid)
            configured_handle = os.environ.get('TERRA_GATT_SERVICE_HANDLE')
            if configured_handle is not None:
                service = self.server.services[self.service_uuid.lower()]
                original = service.gatt
                stable = StableGattService(original, int(configured_handle, 0))
                self.bus.unexport(original.path, original)
                self.server.app.services[self.server.app.services.index(original)] = stable
                service.gatt = service.obj = stable
                self.bus.export(stable.path, stable)
        except BaseException:
            if hasattr(self.server, "bus"): self.server.bus.disconnect()
            raise
        return self

    async def add(self, uuid, read=None, write=None, notify=None, invalid_write=None):
        properties = Properties(0)
        permissions = Permissions(0)
        if read:
            properties |= Properties.read
            permissions |= Permissions.readable | Permissions.read_encryption_required
        if write:
            properties |= Properties.write
            permissions |= Permissions.writeable | Permissions.write_encryption_required
        if notify: properties |= Properties.notify
        await self.server.add_new_characteristic(self.service_uuid, uuid, properties, bytearray(), permissions)
        char = self.server.get_characteristic(uuid)
        original = char.gatt
        peer = PeerCharacteristic(original, read, write, notify, invalid_write)
        self.bus.unexport(original.path, original)
        original._service.characteristics[original._service.characteristics.index(original)] = peer
        char.gatt = char.obj = peer
        self.bus.export(peer.path, peer)
        def publish(data):
            if peer.notifying:
                char.value = bytearray(data)
                self.server.update_value(self.service_uuid, uuid)
        peer.publish = publish
        return peer

    async def start(self):
        properties = self.adapter.get_interface('org.freedesktop.DBus.Properties')
        self.alias = (await properties.call_get('org.bluez.Adapter1', 'Alias')).value
        # Mark before awaiting so partial starts are torn down too.
        self.started = True
        await self.server.start()

    async def close(self):
        failures = []
        server = self.server
        try:
            if self.started:
                operations = []
                if server.app.advertisements:
                    operations.append(lambda: server.app.stop_advertising(server.adapter))
                operations.append(lambda: server.app.unregister(server.adapter))
                for operation in operations:
                    try: await asyncio.wait_for(operation(), 2)
                    except Exception as exc:
                        if not isinstance(exc, DBusError) or exc.type != 'org.bluez.Error.DoesNotExist':
                            failures.append(str(exc))
                if self.alias is not None:
                    try: await asyncio.wait_for(server.app.set_name(server.adapter, self.alias), 2)
                    except Exception as exc: failures.append(str(exc))
        finally:
            if hasattr(server, 'bus'): server.bus.disconnect()
        if failures: raise RuntimeError('Bless cleanup failed: ' + '; '.join(failures))
