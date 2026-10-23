"""Stalled owner recovery must never interrupt a subscribed Terra session."""
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock
from terra_rover.ble import BlePeripheral


class ConnectionRecoveryTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.rover = BlePeripheral()
        self.path = '/org/bluez/hci0/dev_owner'
        self.rover.owner = {'address': 'AA:BB:CC:DD:EE:FF', 'address_type': 'public'}
        self.rover.devices[self.path] = dict(Address=self.rover.owner['address'],
            AddressType='public', Connected=True, Bonded=True, Paired=True)
        self.rover.chars = {'status': SimpleNamespace(notifying=False)}
        self.device = SimpleNamespace(call_disconnect=AsyncMock())
        self.rover.bus = SimpleNamespace(introspect=AsyncMock(return_value=None),
            get_proxy_object=lambda *args: SimpleNamespace(get_interface=lambda name: self.device))

    async def tick(self, *times):
        for now in times:
            await self.rover.recover_stalled_connection(now)

    async def test_disconnects_only_after_full_grace_period(self):
        await self.tick(0, 15, 30, 59)
        self.device.call_disconnect.assert_not_awaited()
        await self.tick(60)
        self.device.call_disconnect.assert_awaited_once()
        self.assertEqual(self.rover.owner['address'], 'AA:BB:CC:DD:EE:FF')
        self.assertTrue(self.rover.devices[self.path]['Bonded'])

    async def test_healthy_subscription_is_never_reset_even_when_idle(self):
        self.rover.peer = self.path
        self.rover.chars['status'].notifying = True
        await self.tick(0, 60, 600)
        self.device.call_disconnect.assert_not_awaited()

    async def test_new_subscription_cancels_pending_recovery(self):
        await self.tick(0, 59)
        self.rover.peer = self.path
        self.rover.chars['status'].notifying = True
        await self.tick(60)
        self.rover.chars['status'].notifying = False
        await self.tick(61, 120)
        self.device.call_disconnect.assert_not_awaited()
        await self.tick(121)
        self.device.call_disconnect.assert_awaited_once()

    async def test_unrelated_or_disconnected_device_is_not_reset(self):
        self.rover.devices[self.path]['Address'] = 'other'
        await self.tick(0, 90)
        self.rover.devices[self.path]['Address'] = self.rover.owner['address']
        self.rover.devices[self.path]['Connected'] = False
        await self.tick(100, 200)
        self.device.call_disconnect.assert_not_awaited()

    async def test_failed_disconnect_is_rate_limited(self):
        self.device.call_disconnect.side_effect = RuntimeError('busy')
        with self.assertLogs('terra_rover.ble', level='WARNING'):
            await self.tick(0, 60, 61, 120, 179)
        self.assertEqual(self.device.call_disconnect.await_count, 1)
        with self.assertLogs('terra_rover.ble', level='WARNING'):
            await self.tick(180)
        self.assertEqual(self.device.call_disconnect.await_count, 2)

    async def test_recovery_clears_old_commands_and_invalidates_generation(self):
        self.rover.peer = self.path
        self.rover.drive = object()
        generation = self.rover.generation
        await self.tick(0, 60)
        self.assertIsNone(self.rover.peer)
        self.assertIsNone(self.rover.drive)
        self.assertGreater(self.rover.generation, generation)
        self.assertEqual(self.rover.events[-1][1], 'disconnect')

    async def test_subscription_during_introspection_prevents_disconnect(self):
        async def subscribe(*args):
            self.rover.peer = self.path
            self.rover.chars['status'].notifying = True
        self.rover.bus.introspect.side_effect = subscribe
        await self.tick(0, 60)
        self.device.call_disconnect.assert_not_awaited()

    async def test_physical_disconnect_restarts_grace_period(self):
        await self.tick(0, 59)
        self.rover.devices[self.path]['Connected'] = False
        await self.tick(60)
        self.rover.devices[self.path]['Connected'] = True
        await self.tick(61, 120)
        self.device.call_disconnect.assert_not_awaited()
        await self.tick(121)
        self.device.call_disconnect.assert_awaited_once()

    async def test_disconnect_and_reconnect_between_watchdog_ticks_gets_full_grace(self):
        await self.tick(0, 59)
        self.rover.disconnect(self.path, 59.5)
        await self.tick(60, 119)
        self.device.call_disconnect.assert_not_awaited()
        await self.tick(120)
        self.device.call_disconnect.assert_awaited_once()
