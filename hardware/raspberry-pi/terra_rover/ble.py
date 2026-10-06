"""BlueZ owner-only GATT transport; all actuator work belongs to one worker."""
import asyncio
from collections import deque
import json
import os
from pathlib import Path
import secrets
import signal
import threading
import time

from dbus_next import BusType, DBusError, Variant, PropertyAccess
from dbus_next.aio import MessageBus
from dbus_next.service import ServiceInterface, method, dbus_property

from .configuration import ConfigurationStore
from .safety import SafetyController
from .protocol import (FragmentAssembler, CommandKind, ProtocolError, decode_frame,
                       decode_control, encode_reply, fragment_message, JSON_LIMIT)

SERVICE_UUID = '7e5a0010-4c2b-4f91-9e3a-1d8c6b2a0f10'
DRIVE_UUID = '7e5a0011-4c2b-4f91-9e3a-1d8c6b2a0f10'
CONTROL_UUID = '7e5a0013-4c2b-4f91-9e3a-1d8c6b2a0f10'
STATUS_UUID = '7e5a0012-4c2b-4f91-9e3a-1d8c6b2a0f10'
REPLY_UUID = '7e5a0014-4c2b-4f91-9e3a-1d8c6b2a0f10'
ROOT = '/com/terra/rover'
SERVICE = ROOT + '/service0'

def denied(message):
    return DBusError('org.bluez.Error.NotAuthorized', message)

class GattService(ServiceInterface):
    def __init__(self): super().__init__('org.bluez.GattService1')
    @dbus_property(access=PropertyAccess.READ)
    def UUID(self) -> 's': return SERVICE_UUID
    @dbus_property(access=PropertyAccess.READ)
    def Primary(self) -> 'b': return True

class Characteristic(ServiceInterface):
    def __init__(self, peripheral, name, uuid, flags):
        super().__init__('org.bluez.GattCharacteristic1')
        self.peripheral, self.name, self.uuid, self.flags = peripheral, name, uuid, flags
        self.value, self.notifying = b'', False
    @dbus_property(access=PropertyAccess.READ)
    def UUID(self) -> 's': return self.uuid
    @dbus_property(access=PropertyAccess.READ)
    def Service(self) -> 'o': return SERVICE
    @dbus_property(access=PropertyAccess.READ)
    def Flags(self) -> 'as': return self.flags
    @dbus_property(access=PropertyAccess.READ)
    def Value(self) -> 'ay': return self.value
    @dbus_property(access=PropertyAccess.READ)
    def Notifying(self) -> 'b': return self.notifying
    @method()
    def ReadValue(self, options: 'a{sv}') -> 'ay':
        self.peripheral.authorize(options)
        if self.name not in ('status', 'reply'): raise DBusError('org.bluez.Error.NotPermitted', 'not readable')
        data = self.peripheral.status_bytes if self.name == 'status' else self.peripheral.reply_bytes
        offset = options.get('offset', Variant('q', 0)).value
        if offset > len(data): raise DBusError('org.bluez.Error.InvalidOffset', 'offset')
        return data[offset:]
    @method()
    def WriteValue(self, value: 'ay', options: 'a{sv}'):
        peer = self.peripheral.authorize(options)
        if self.name in ('status', 'reply'): raise DBusError('org.bluez.Error.NotPermitted', 'not writable')
        if options.get('offset', Variant('q', 0)).value or options.get('prepare-authorize', Variant('b', False)).value:
            raise DBusError('org.bluez.Error.NotSupported', 'use application fragments')
        try: self.peripheral.handle_write(peer, self.name, bytes(value), time.monotonic())
        except ProtocolError as exc: raise DBusError('org.bluez.Error.InvalidValueLength', str(exc)) from exc
    @method()
    def StartNotify(self):
        # BlueZ supplies no peer argument. Never infer admission from this call.
        if 'notify' not in self.flags: raise DBusError('org.bluez.Error.NotSupported', 'not notifiable')
        self.peripheral.require_single_owner()
        self.notifying = True
        self.emit_properties_changed({'Notifying': True})
        if self.name == 'status': self.peripheral.enqueue('subscribe', True)
    @method()
    def StopNotify(self):
        self.notifying = False
        self.emit_properties_changed({'Notifying': False})
        if self.name == 'status': self.peripheral.enqueue('subscribe', False)
    def publish(self, data):
        if self.notifying:
            self.value = data
            self.emit_properties_changed({'Value': data})

class ObjectManager(ServiceInterface):
    def __init__(self, objects):
        super().__init__('org.freedesktop.DBus.ObjectManager'); self.objects = objects
    @method()
    def GetManagedObjects(self) -> 'a{oa{sa{sv}}}':
        return self.objects

class Advertisement(ServiceInterface):
    def __init__(self, name):
        super().__init__('org.bluez.LEAdvertisement1'); self.name = name
    @dbus_property(access=PropertyAccess.READ)
    def Type(self) -> 's': return 'peripheral'
    @dbus_property(access=PropertyAccess.READ)
    def ServiceUUIDs(self) -> 'as': return [SERVICE_UUID]
    @dbus_property(access=PropertyAccess.READ)
    def LocalName(self) -> 's': return self.name
    @method()
    def Release(self): pass

class BlePeripheral:
    def __init__(self, name='Terra Rover', adapter=None):
        self.name, self.adapter = name, adapter
        self.peer = None
        self.generation = 0
        self.devices = {}
        self.owner = None
        self.lock = threading.Lock()
        self.events = deque(maxlen=32)
        self.drive = None
        self.priority = None
        self.configuring = False
        self.assemblers = {name: FragmentAssembler(limit) for name, limit in
                           [('drive', 96), ('priority', 96), ('json', JSON_LIMIT)]}
        self.routes = {}
        self.stop = threading.Event()
        self.status_bytes = b'{}'
        self.reply_bytes = b'{}'
        self.notification_size = 20
        self.next_message = 0
    def clear(self):
        for assembler in self.assemblers.values(): assembler.clear()
        self.routes.clear()
    def require_single_owner(self):
        connected = [path for path, p in self.devices.items() if p.get('Connected')]
        if self.peer is None or connected != [self.peer]: raise denied('read owner status before subscribing; only owner may be connected')
        self._owner_device(self.peer)
    def _owner_device(self, peer):
        p = self.devices.get(peer, {})
        if not p.get('Connected') or not p.get('Bonded') or not p.get('Paired'):
            raise denied('connected bonded owner required')
        if (p.get('Address'), p.get('AddressType')) != (self.owner['address'], self.owner['address_type']):
            raise denied('owner identity mismatch')
    def authorize(self, options):
        device = options.get('device')
        if device is None: raise denied('device option required')
        peer = device.value
        mtu = options.get('mtu')
        if mtu is not None: self.notification_size = max(20, min(512, mtu.value - 3))
        self._owner_device(peer)
        if self.peer is not None and self.peer != peer: raise denied('another owner connection is admitted')
        if any(path != peer and p.get('Connected') for path, p in self.devices.items()):
            raise denied('disconnect other centrals before admission')
        if self.peer is None:
            self.peer = peer; self.generation += 1; self.clear()
            self.enqueue('connect', secrets.randbelow(0xfffffffe) + 1)
        return peer
    def enqueue(self, kind, value):
        with self.lock:
            if len(self.events) == self.events.maxlen: raise DBusError('org.bluez.Error.InProgress', 'worker queue full')
            self.events.append((self.generation, kind, value, time.monotonic()))
    def disconnect(self, peer, now):
        if peer != self.peer: return
        self.peer = None; self.generation += 1; self.clear()
        for char in self.chars.values(): char.notifying = False
        with self.lock:
            self.drive = self.priority = None
            self.events.clear()
            self.events.append((self.generation, 'disconnect', None, now))
    def handle_write(self, peer, characteristic, value, now):
        if peer != self.peer: raise denied('owner is not admitted')
        self._owner_device(peer)
        if len(value) < 5: raise ProtocolError('fragment too short')
        message = int.from_bytes(value[:2], 'little')
        if characteristic == 'drive': route = 'drive'
        elif characteristic == 'control':
            if value[2] == 0:
                route = 'priority' if value[4:6] == b'TA' else 'json'
                self.routes[message] = (route, now)
                # Bound route table even for intentionally abandoned first fragments.
                self.routes = {k: v for k, v in self.routes.items() if now-v[1] < .1}
                if len(self.routes) > 3: self.routes.clear(); raise ProtocolError('too many active messages')
            else:
                entry = self.routes.get(message)
                if entry is None or now-entry[1] >= .1: raise ProtocolError('unknown or expired control message')
                route = entry[0]
        else: raise ProtocolError('unknown characteristic')
        try: data = self.assemblers[route].push(value, now)
        except ProtocolError:
            self.routes.pop(message, None); raise
        if data is None: return
        self.routes.pop(message, None)
        if route == 'json': self.enqueue('json', decode_control(data)); return
        frame = decode_frame(data)
        if (route == 'drive') != (frame.kind == CommandKind.DRIVE): raise ProtocolError('wrong characteristic for command')
        with self.lock:
            if self.configuring: raise DBusError('org.bluez.Error.InProgress', 'disarmed configuration in progress')
            item = (self.generation, frame, now)
            if frame.kind in (CommandKind.DISARM, CommandKind.EMERGENCY_STOP):
                self.drive = None; self.assemblers['drive'].clear()
                if self.priority is None or self.priority[1].kind != CommandKind.EMERGENCY_STOP:
                    self.priority = item
            elif route == 'drive': self.drive = item
            elif self.priority is None: self.priority = item
            else: raise DBusError('org.bluez.Error.InProgress', 'priority command pending')
    def _emit(self, characteristic, payload, generation):
        if generation != self.generation or self.peer is None: return
        try: self.require_single_owner()
        except DBusError: return
        raw = json.dumps(payload, separators=(',', ':'), allow_nan=False).encode()
        if characteristic == 'reply': self.reply_bytes = raw
        if len(raw) > JSON_LIMIT: return
        self.next_message = (self.next_message + 1) & 0xffff
        # Conservative ATT capacity 20; documents longer than 4080 cannot fit
        # in 255 fragments. Use 128 only after negotiated MTU observation.
        size = self.notification_size
        try: pieces = fragment_message(raw, self.next_message, size, JSON_LIMIT)
        except ProtocolError: return
        for piece in pieces: self.chars[characteristic].publish(piece)
    def _worker(self):
        while not self.stop.is_set():
            started = time.monotonic()
            with self.lock:
                events = list(self.events); self.events.clear()
                priority, drive = self.priority, self.drive
                self.priority = self.drive = None
            # Priority is processed before disk/backend configuration.
            if priority is not None and priority[0] == self.generation:
                generation, frame, received = priority
                accepted = self.safety.accept(frame, received, time.monotonic())
                self.loop.call_soon_threadsafe(self._emit, 'status', dict(schema_version=1, type='command_acceptance', session=frame.session, sequence=frame.sequence, accepted=accepted), generation)
                priority = None
            for generation, kind, value, received in events:
                if generation != self.generation: continue
                if kind == 'connect': self.store.begin_connection(value, started)
                elif kind == 'disconnect': self.store.end_connection(started)
                elif kind == 'subscribe': self.safety.set_status_subscribed(value, started)
                elif kind == 'json':
                    with self.lock: self.configuring = True
                    try:
                        if not self.safety.status(time.monotonic())['armed'] and not self.safety.status(time.monotonic())['arming'] and self.safety.layout is not None:
                            self.backend.apply(self.safety.tick(time.monotonic(), self.backend.read_gate()))
                        reply = self.store.handle(value, time.monotonic())
                        self.loop.call_soon_threadsafe(self._emit, 'reply', reply, generation)
                    except Exception as exc: self.safety.latch_fault('control_worker: '+str(exc), time.monotonic())
                    finally:
                        with self.lock: self.configuring = False
            for item in (priority, drive):
                if item is None or item[0] != self.generation: continue
                generation, frame, received = item
                accepted = self.safety.accept(frame, received, time.monotonic())
                acknowledgement = dict(schema_version=1, type='command_acceptance', session=frame.session,
                                       sequence=frame.sequence, accepted=accepted)
                self.loop.call_soon_threadsafe(self._emit, 'status', acknowledgement, generation)
            now = time.monotonic()
            try:
                batch = self.safety.tick(now, self.backend.read_gate())
                if self.safety.layout is not None: self.backend.apply(batch)
            except Exception as exc:
                self.safety.latch_fault('backend: '+str(exc), time.monotonic())
                try: self.backend.apply(self.safety.tick(time.monotonic(), False))
                except Exception: pass
            if now-self.last_status >= .1:
                status = self.store.status(now)
                status.update(type='status', generation=self.generation)
                self.status_bytes = json.dumps(status, separators=(',', ':')).encode()
                self.loop.call_soon_threadsafe(self._emit, 'status', status, self.generation)
                self.last_status = now
            self.stop.wait(max(0, .010-(time.monotonic()-started)))
        self.store.end_connection(time.monotonic())
        try:
            if self.safety.layout is not None: self.backend.apply(self.safety.tick(time.monotonic(), False))
        finally: self.backend.close()
    async def run(self, backend, store_path: Path, owner_store: Path):
        self.owner = json.loads(Path(owner_store).read_text())
        if os.stat(owner_store).st_mode & 0o077: raise RuntimeError('owner file must have mode 0600')
        self.backend = backend; self.safety = SafetyController(backend.capabilities())
        self.store = ConfigurationStore(store_path, self.safety, backend)
        self.store.load(time.monotonic())
        self.loop = asyncio.get_running_loop(); self.last_status = 0
        self.notification_size = 20
        self.bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
        manager = self.bus.get_proxy_object('org.bluez', '/', await self.bus.introspect('org.bluez', '/')).get_interface('org.freedesktop.DBus.ObjectManager')
        objects = await manager.call_get_managed_objects()
        self.devices = {p: {k: v.value for k, v in interfaces['org.bluez.Device1'].items()} for p, interfaces in objects.items() if 'org.bluez.Device1' in interfaces}
        adapters = [p for p, i in objects.items() if 'org.bluez.GattManager1' in i and 'org.bluez.LEAdvertisingManager1' in i]
        adapter = self.adapter or next(iter(adapters), None)
        if adapter not in adapters: raise RuntimeError('no BlueZ GATT/advertising adapter')
        proxy = self.bus.get_proxy_object('org.bluez', adapter, await self.bus.introspect('org.bluez', adapter))
        props = proxy.get_interface('org.freedesktop.DBus.Properties')
        await props.call_set('org.bluez.Adapter1', 'Pairable', Variant('b', False))
        await props.call_set('org.bluez.Adapter1', 'Powered', Variant('b', True))
        self.chars = {name: Characteristic(self, name, uuid, flags) for name, uuid, flags in [
            ('drive', DRIVE_UUID, ['write', 'encrypt-write']),
            ('control', CONTROL_UUID, ['write', 'encrypt-write']),
            ('reply', REPLY_UUID, ['read', 'encrypt-read', 'notify']),
            ('status', STATUS_UUID, ['read', 'encrypt-read', 'notify'])]}
        tree = {SERVICE: {'org.bluez.GattService1': {'UUID': Variant('s', SERVICE_UUID), 'Primary': Variant('b', True)}}}
        self.bus.export(SERVICE, GattService())
        for name, char in self.chars.items():
            path = SERVICE+'/'+name; self.bus.export(path, char)
            tree[path] = {'org.bluez.GattCharacteristic1': {'UUID': Variant('s', char.uuid), 'Service': Variant('o', SERVICE), 'Flags': Variant('as', char.flags)}}
        self.bus.export(ROOT, ObjectManager(tree))
        self.bus.export(ROOT+'/advertisement', Advertisement(self.name))
        def changed(message):
            if message.interface != 'org.freedesktop.DBus.Properties' or message.member != 'PropertiesChanged': return
            interface, updates, invalidated = message.body
            if interface != 'org.bluez.Device1': return
            device = self.devices.setdefault(message.path, {})
            device.update({k: v.value for k, v in updates.items()})
            for key in invalidated: device.pop(key, None)
            if not device.get('Connected') or not device.get('Bonded'): self.disconnect(message.path, time.monotonic())
            if message.path != self.peer and device.get('Connected') and self.peer is not None:
                self.disconnect(self.peer, time.monotonic())
        self.bus.add_message_handler(changed)
        # ObjectManager changes cover newly discovered paired devices.
        manager.on_interfaces_added(lambda p, i: self.devices.update({p: {k:v.value for k,v in i['org.bluez.Device1'].items()}}) if 'org.bluez.Device1' in i else None)
        manager.on_interfaces_removed(lambda p, i: (self.disconnect(p, time.monotonic()), self.devices.pop(p, None)) if 'org.bluez.Device1' in i else None)
        # Install bus match for Device1 property signals on the whole adapter tree.
        dbus = self.bus.get_proxy_object('org.freedesktop.DBus', '/org/freedesktop/DBus', await self.bus.introspect('org.freedesktop.DBus', '/org/freedesktop/DBus')).get_interface('org.freedesktop.DBus')
        await dbus.call_add_match("type='signal',sender='org.bluez',interface='org.freedesktop.DBus.Properties',member='PropertiesChanged',path_namespace='"+adapter+"'")
        gatt = proxy.get_interface('org.bluez.GattManager1'); advertising = proxy.get_interface('org.bluez.LEAdvertisingManager1')
        registered = advertised = False
        worker = None
        done = asyncio.Event()
        for sig in (signal.SIGINT, signal.SIGTERM): self.loop.add_signal_handler(sig, done.set)
        try:
            await gatt.call_register_application(ROOT, {}); registered = True
            worker = threading.Thread(target=self._worker, name='terra-output', daemon=False); worker.start()
            await advertising.call_register_advertisement(ROOT+'/advertisement', {}); advertised = True
            disconnect_task = asyncio.create_task(self.bus.wait_for_disconnect())
            done_task = asyncio.create_task(done.wait())
            await asyncio.wait([disconnect_task, done_task], return_when=asyncio.FIRST_COMPLETED)
            for task in (disconnect_task, done_task): task.cancel()
        finally:
            self.stop.set()
            if worker is not None: await asyncio.to_thread(worker.join)
            else: backend.close()
            if advertised:
                try: await advertising.call_unregister_advertisement(ROOT+'/advertisement')
                except Exception: pass
            if registered:
                try: await gatt.call_unregister_application(ROOT)
                except Exception: pass
            self.bus.disconnect()
