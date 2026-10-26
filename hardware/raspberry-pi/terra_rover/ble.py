"""BlueZ owner-only GATT transport; all actuator work belongs to one worker."""
import asyncio
from collections import deque
import json
import logging
import os
from pathlib import Path
import signal
import threading
import time

from dbus_next import BusType, DBusError, Variant, PropertyAccess
from .bless_transport import BlessTransport


from .configuration import ConfigurationStore
from .drive_relay import DriveRelay
from .protocol import (FragmentAssembler, ProtocolError, decode_drive,
                       decode_control, fragment_message, JSON_LIMIT)

SERVICE_UUID = '7e5a0010-4c2b-4f91-9e3a-1d8c6b2a0f10'
DRIVE_UUID = '7e5a0011-4c2b-4f91-9e3a-1d8c6b2a0f10'
CONTROL_UUID = '7e5a0013-4c2b-4f91-9e3a-1d8c6b2a0f10'
STATUS_UUID = '7e5a0012-4c2b-4f91-9e3a-1d8c6b2a0f10'
REPLY_UUID = '7e5a0014-4c2b-4f91-9e3a-1d8c6b2a0f10'
ROOT = '/com/terra/rover'
SERVICE = ROOT + '/service0'
OUTPUT_LOOP_SECONDS = .050

def denied(message):
    return DBusError('org.bluez.Error.NotAuthorized', message)

def status_read_document(payload):
    # Keep admission and event-driven status compact. Bulk configuration
    # diagnostics are returned by control replies.
    keys = ('schema_version', 'type', 'connected', 'active_revision', 'control_mode', 'output_active',
            'layout_available', 'fault',
            'configuration_allowed', 'stop_reason')
    compact = {key: payload[key] for key in keys if key in payload}
    raw = json.dumps(compact, separators=(',', ':'), allow_nan=False).encode()
    if len(raw) > 512 and compact.get('fault') and compact.get('stop_reason') == compact['fault']:
        compact['stop_reason'] = 'fault'
        raw = json.dumps(compact, separators=(',', ':'), allow_nan=False).encode()
    # Both fields can contain the same arbitrarily long backend exception.
    # Keep useful prefixes, sizing the encoded JSON (including Unicode escapes),
    # rather than replacing the fault or letting the output worker crash.
    while len(raw) > 512:
        candidates = [key for key in ('fault', 'stop_reason')
                      if isinstance(compact.get(key), str) and len(compact[key]) > 4]
        if not candidates:
            break
        key = max(candidates, key=lambda key: len(json.dumps(compact[key])))
        compact[key] = compact[key][:max(1, len(compact[key]) * 3 // 4 - 3)] + '...'
        raw = json.dumps(compact, separators=(',', ':'), allow_nan=False).encode()
    if len(raw) > 512:
        raise ProtocolError('admission status exceeds ATT read limit')
    return raw


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
        self.invalid_drive = None
        self.configuring = False
        self.assemblers = {'json': FragmentAssembler(JSON_LIMIT, timeout=2.0)}
        self.stop = threading.Event()
        self.worker_error = None
        self.status_bytes = b'{}'
        self.reply_bytes = b'{}'
        self.notification_size = 20
        self.next_message = 0
        self.unsubscribed_since = {}
        self.last_connection_recovery = None
    def _stalled_owner(self, path):
        device = self.devices.get(path, {})
        status = self.chars.get('status')
        return (self.owner is not None and device.get('Connected')
                and device.get('Bonded') and device.get('Paired')
                and (device.get('Address'), device.get('AddressType')) ==
                    (self.owner['address'], self.owner['address_type'])
                and not (self.peer == path and status is not None and status.notifying))

    async def recover_stalled_connection(self, now):
        # A connected BlueZ device may outlive the phone's GATT session (for
        # example through a classic audio connection). Phone-side cancellation
        # alone cannot clear that state. Give admission/subscription a full
        # minute, then disconnect only the bonded owner, never remove its bond.
        candidates = {path for path in self.devices if self._stalled_owner(path)}
        self.unsubscribed_since = {
            path: self.unsubscribed_since.get(path, now) for path in candidates
        }
        if self.last_connection_recovery is not None and now - self.last_connection_recovery < 120:
            return
        for path, started in self.unsubscribed_since.items():
            if now - started < 60:
                continue
            self.last_connection_recovery = now
            try:
                description = await asyncio.wait_for(self.bus.introspect('org.bluez', path), 5)
                # Introspection yields: a healthy subscription may have arrived.
                if not self._stalled_owner(path) or self.unsubscribed_since.get(path) != started:
                    return
                device = self.bus.get_proxy_object('org.bluez', path, description).get_interface('org.bluez.Device1')
                self.disconnect(path, now)
                logging.getLogger(__name__).warning(
                    'Recovering stalled owner connection after %.1fs without Terra status subscription',
                    now - started)
                await asyncio.wait_for(device.call_disconnect(), 5)
            except Exception as exc:
                logging.getLogger(__name__).warning('Owner connection recovery failed: %s', exc)
            return

    async def _recover_owner_connections(self):
        while not self.stop.is_set():
            await self.recover_stalled_connection(time.monotonic())
            await asyncio.sleep(1)

    def clear(self):
        for assembler in self.assemblers.values(): assembler.clear()
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
            self.enqueue('connect', None)
        return peer
    def enqueue(self, kind, value):
        with self.lock:
            if len(self.events) == self.events.maxlen: raise DBusError('org.bluez.Error.InProgress', 'worker queue full')
            self.events.append((self.generation, kind, value, time.monotonic()))
    def disconnect(self, peer, now):
        self.unsubscribed_since.pop(peer, None)
        if peer != self.peer: return
        self.peer = None; self.generation += 1; self.clear()
        for char in self.chars.values(): char.notifying = False
        with self.lock:
            self.drive = None
            self.invalid_drive = None
            self.events.clear()
            self.events.append((self.generation, 'disconnect', None, now))
    def malformed_drive(self, peer):
        if peer != self.peer: raise denied('owner is not admitted')
        self._owner_device(peer)
        with self.lock:
            self.invalid_drive = self.generation
            self.drive = None

    def handle_write(self, peer, characteristic, value, now):
        if peer != self.peer: raise denied('owner is not admitted')
        self._owner_device(peer)
        try: return self._handle_write(characteristic, value, now)
        except ProtocolError:
            if characteristic == 'drive': self.malformed_drive(peer)
            raise

    def _handle_write(self, characteristic, value, now):
        if characteristic == 'control':
            data = self.assemblers['json'].push(value, now)
            if data is not None: self.enqueue('json', decode_control(data))
            return
        if characteristic != 'drive': raise ProtocolError('unknown characteristic')
        values = decode_drive(value)
        with self.lock:
            if self.configuring: raise DBusError('org.bluez.Error.InProgress', 'configuration in progress')
            pending = {v.id: v for v in self.drive[1]} if self.drive is not None and self.drive[0] == self.generation else {}
            pending.update((v.id, v) for v in values)
            if len(pending) > 16: raise ProtocolError('too many actuator IDs in output loop')
            self.drive = (self.generation, tuple(pending.values()))

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
    async def join_worker(self, worker):
        await asyncio.to_thread(worker.join)
        if self.worker_error is not None:
            raise RuntimeError('output worker failed: ' + str(self.worker_error)) from self.worker_error

    async def monitor_worker(self, worker):
        await self.join_worker(worker)
        if not self.stop.is_set():
            raise RuntimeError('output worker stopped unexpectedly')

    def _worker(self):
        try:
            self._output_loop()
        except BaseException as exc:
            self.worker_error = exc
            logging.getLogger(__name__).exception('Output worker failed; stopping Bluetooth service')
        finally:
            try:
                self.store.end_connection()
                if self.relay.layout is not None:
                    self.backend.apply(self.relay.output())
            except BaseException as exc:
                self.worker_error = self.worker_error or exc
            finally:
                try: self.backend.close()
                except BaseException as exc: self.worker_error = self.worker_error or exc

    def _apply_output(self):
        try:
            self.backend.apply(self.relay.output())
        except Exception as exc:
            self.relay.latch_fault('backend: '+str(exc))
            try: self.backend.apply(self.relay.output())
            except Exception: logging.getLogger(__name__).exception('Unable to send neutral output')

    def _output_loop(self):
        while not self.stop.is_set():
            started = time.monotonic()
            with self.lock:
                events = list(self.events); self.events.clear()
                drive, self.drive = self.drive, None
                invalid_drive, self.invalid_drive = self.invalid_drive, None
            for generation, kind, value, received in events:
                if generation != self.generation: continue
                if kind == 'connect': self.store.begin_connection()
                elif kind == 'disconnect': self.store.end_connection()
                elif kind == 'subscribe': self.relay.set_status_subscribed(value)
            if invalid_drive == self.generation:
                self.relay.stop('malformed_drive')
                drive = None
            for generation, kind, value, received in events:
                if generation != self.generation or kind != 'json': continue
                with self.lock:
                    self.configuring = True
                    self.drive = None
                # Configuration never carries an old effort into replacement outputs.
                drive = None
                self.relay.stop('configuration')
                if self.relay.layout is not None: self._apply_output()
                try:
                    operation_started = time.monotonic()
                    reply = self.store.handle(value)
                    logging.getLogger(__name__).debug('configuration operation=%s request_id=%s queue_seconds=%.3f elapsed_seconds=%.3f result=%s', value['operation'], value['request_id'], operation_started-received, time.monotonic()-operation_started, reply['result'])
                    self.loop.call_soon_threadsafe(self._emit, 'reply', reply, generation)
                except Exception as exc: self.relay.latch_fault('control_worker: '+str(exc))
                finally:
                    with self.lock: self.configuring = False
            if drive is not None and drive[0] == self.generation:
                self.relay.accept(drive[1])
            # output() holds the latest efforts for DRIVE_COMMAND_TIMEOUT_SECONDS, then returns neutral.
            if self.relay.layout is not None: self._apply_output()
            status = self.store.status()
            status.update(type='status', generation=self.generation)
            raw = status_read_document(status)
            if raw != self.status_bytes:
                self.status_bytes = raw
                self.loop.call_soon_threadsafe(self._emit, 'status', json.loads(raw), self.generation)
            self.stop.wait(max(0., OUTPUT_LOOP_SECONDS - (time.monotonic() - started)))

    async def run(self, backend, store_path: Path, owner_store: Path, *, manage_signals=True):
        self.owner = json.loads(Path(owner_store).read_text())
        if os.stat(owner_store).st_mode & 0o077: raise RuntimeError('owner file must have mode 0600')
        self.backend = backend; self.relay = DriveRelay(backend.capabilities())
        self.store = ConfigurationStore(store_path, self.relay, backend)
        self.store.load()
        self.loop = asyncio.get_running_loop()
        self.notification_size = 20
        transport = BlessTransport(self.name, SERVICE_UUID, self.adapter)
        worker = None
        disconnect_task = done_task = recovery_task = worker_task = None
        try:
            await transport.prepare()
            self.bus = transport.bus
            manager = self.bus.get_proxy_object('org.bluez', '/', await self.bus.introspect('org.bluez', '/')).get_interface('org.freedesktop.DBus.ObjectManager')
            objects = await manager.call_get_managed_objects()
            self.devices = {p: {k: v.value for k, v in interfaces['org.bluez.Device1'].items()} for p, interfaces in objects.items() if 'org.bluez.Device1' in interfaces}
            adapters = [p for p, i in objects.items() if 'org.bluez.GattManager1' in i and 'org.bluez.LEAdvertisingManager1' in i]
            adapter = transport.adapter.path
            if adapter not in adapters: raise RuntimeError('no BlueZ GATT/advertising adapter')
            proxy = self.bus.get_proxy_object('org.bluez', adapter, await self.bus.introspect('org.bluez', adapter))
            props = proxy.get_interface('org.freedesktop.DBus.Properties')
            await props.call_set('org.bluez.Adapter1', 'Pairable', Variant('b', False))
            await props.call_set('org.bluez.Adapter1', 'Powered', Variant('b', True))
            self.chars = {}
            for role, uuid in [('drive', DRIVE_UUID), ('control', CONTROL_UUID),
                               ('reply', REPLY_UUID), ('status', STATUS_UUID)]:
                def read(options, role=role):
                    self.authorize(options)
                    return self.status_bytes if role == 'status' else self.reply_bytes
                def write(value, options, role=role):
                    peer = self.authorize(options)
                    if options.get('offset', Variant('q', 0)).value or options.get('prepare-authorize', Variant('b', False)).value:
                        if role == 'drive': self.malformed_drive(peer)
                        raise DBusError('org.bluez.Error.NotSupported', 'use application fragments')
                    try: self.handle_write(peer, role, value, time.monotonic())
                    except ProtocolError as exc:
                        raise DBusError('org.bluez.Error.InvalidValueLength', str(exc)) from exc
                def invalid_write(options, role=role):
                    peer = self.authorize(options)
                    if role == 'drive': self.malformed_drive(peer)
                def notify(enabled, role=role):
                    if enabled: self.require_single_owner()
                    if role == 'status': self.enqueue('subscribe', enabled)
                readable = role in ('status', 'reply')
                self.chars[role] = await transport.add(uuid, read=read if readable else None,
                    write=None if readable else write, notify=notify if readable else None,
                    invalid_write=invalid_write if not readable else None)
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
            def added(path, interfaces):
                if 'org.bluez.Device1' not in interfaces: return
                self.devices[path] = {k: v.value for k, v in interfaces['org.bluez.Device1'].items()}
                if self.peer is not None and path != self.peer and self.devices[path].get('Connected'):
                    self.disconnect(self.peer, time.monotonic())
            manager.on_interfaces_added(added)
            manager.on_interfaces_removed(lambda p, i: (self.disconnect(p, time.monotonic()), self.devices.pop(p, None)) if 'org.bluez.Device1' in i else None)
            # Install bus match for Device1 property signals on the whole adapter tree.
            dbus = self.bus.get_proxy_object('org.freedesktop.DBus', '/org/freedesktop/DBus', await self.bus.introspect('org.freedesktop.DBus', '/org/freedesktop/DBus')).get_interface('org.freedesktop.DBus')
            await dbus.call_add_match("type='signal',sender='org.bluez',interface='org.freedesktop.DBus.Properties',member='PropertiesChanged',path_namespace='"+adapter+"'")
            done = asyncio.Event()
            if manage_signals:
                for sig in (signal.SIGINT, signal.SIGTERM): self.loop.add_signal_handler(sig, done.set)
            worker = threading.Thread(target=self._worker, name='terra-output', daemon=False); worker.start()
            worker_task = asyncio.create_task(self.monitor_worker(worker))
            await transport.start()
            recovery_task = asyncio.create_task(self._recover_owner_connections())
            disconnect_task = asyncio.create_task(self.bus.wait_for_disconnect())
            done_task = asyncio.create_task(done.wait())
            completed, _ = await asyncio.wait([disconnect_task, done_task, worker_task], return_when=asyncio.FIRST_COMPLETED)
            if worker_task in completed:
                worker_task.result()
            for task in (disconnect_task, done_task): task.cancel()
        finally:
            for task in (disconnect_task, done_task, recovery_task, worker_task):
                if task is not None: task.cancel()
            await asyncio.gather(*(task for task in (disconnect_task, done_task, recovery_task, worker_task) if task is not None), return_exceptions=True)
            self.stop.set()
            try:
                if worker is not None: await self.join_worker(worker)
                else: backend.close()
            finally:
                try: await transport.close()
                finally:
                    if manage_signals:
                        for sig in (signal.SIGINT, signal.SIGTERM): self.loop.remove_signal_handler(sig)
