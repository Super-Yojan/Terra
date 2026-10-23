import asyncio
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from terra_rover.onboarding import save_owner
from terra_rover import lifecycle

class LifecycleTests(unittest.IsolatedAsyncioTestCase):
    async def test_owned_boot_skips_pairing_and_runs_normal_service(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); b=root/'button'; l=root/'led'; b.write_text('0'); l.touch()
            args=SimpleNamespace(owner=root/'owner', device_name_file=root/'name',button_file=b,led_file=l)
            save_owner(args.owner, {'address':'AA:BB:CC:DD:EE:FF','address_type':'public'})
            calls=[]
            async def normal(args,name): calls.append(name)
            async def pair(*args): self.fail('owned devices must not pair')
            await lifecycle.run_customer_service(args,normal,pair)
            self.assertRegex(calls[0],r'^terra-')
            self.assertEqual(l.read_text(),'0')
    async def test_unowned_hold_transitions_without_constructing_motor_backend(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); b=root/'button'; l=root/'led'; b.write_text('0'); l.touch()
            args=SimpleNamespace(owner=root/'owner', device_name_file=root/'name',button_file=b,led_file=l)
            calls=[]
            async def normal(args,name): calls.append('normal')
            async def pair(args,hat,name):
                self.assertEqual(calls,[])
                save_owner(args.owner,{'address':'AA:BB:CC:DD:EE:FF','address_type':'random'})
                calls.append('pair'); return True
            with patch.object(lifecycle.HoldDetector,'update',return_value=True), patch.object(lifecycle,'flash_success',new=lambda hat: asyncio.sleep(0)):
                await lifecycle.run_customer_service(args,normal,pair)
            self.assertEqual(calls,['pair','normal'])
    async def test_invalid_owner_fails_closed(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); p=root/'owner'; p.write_text('{}'); p.chmod(0o600)
            args=SimpleNamespace(owner=p, device_name_file=root/'name',button_file=root/'button',led_file=root/'led')
            async def forbidden(*args): self.fail('must not start')
            with self.assertRaises(ValueError): await lifecycle.run_customer_service(args,forbidden,forbidden)

    async def test_owned_hold_stops_outputs_before_clearing_and_repairing(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d); b = root/'button'; l = root/'led'; b.write_text('0'); l.touch()
            args = SimpleNamespace(owner=root/'owner', device_name_file=root/'name', button_file=b, led_file=l)
            save_owner(args.owner, {'address':'AA:BB:CC:DD:EE:FF', 'address_type':'public'})
            calls = []; running = asyncio.Event()
            async def normal(args, name):
                if 'pair' in calls: calls.append('new-normal'); return
                calls.append('normal'); running.set()
                try: await asyncio.Event().wait()
                finally: calls.append('outputs-closed')
            async def remove_bond(args, owner):
                self.assertEqual(calls, ['normal', 'outputs-closed'])
                self.assertTrue(args.owner.exists())
                calls.append('bond-removed')
            async def pair(args, hat, name):
                self.assertFalse(args.owner.exists())
                calls.append('pair')
                save_owner(args.owner, {'address':'11:22:33:44:55:66', 'address_type':'random'})
                return True
            with patch.object(lifecycle.HoldDetector, 'update', side_effect=lambda *args: running.is_set()), patch.object(lifecycle, 'remove_owner_bond', remove_bond, create=True), patch.object(lifecycle, 'flash_success', new=lambda hat: asyncio.sleep(0)):
                await asyncio.wait_for(lifecycle.run_customer_service(args, normal, pair), 1)
            self.assertEqual(calls, ['normal', 'outputs-closed', 'bond-removed', 'pair', 'new-normal'])

    async def test_failed_bond_removal_preserves_owner_and_never_pairs(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d); b = root/'button'; l = root/'led'; b.write_text('0'); l.touch()
            args = SimpleNamespace(owner=root/'owner', device_name_file=root/'name', button_file=b, led_file=l)
            identity = {'address':'AA:BB:CC:DD:EE:FF', 'address_type':'public'}
            save_owner(args.owner, identity)
            async def normal(*args): await asyncio.Event().wait()
            async def pair(*args): self.fail('must not pair after failed reset')
            async def remove_bond(*args): raise RuntimeError('bond cleanup failed')
            with patch.object(lifecycle.HoldDetector, 'update', return_value=True), patch.object(lifecycle, 'remove_owner_bond', remove_bond, create=True):
                with self.assertRaisesRegex(RuntimeError, 'bond cleanup failed'):
                    await asyncio.wait_for(lifecycle.run_customer_service(args, normal, pair), 1)
            from terra_rover.onboarding import load_owner
            self.assertEqual(load_owner(args.owner), identity)

    async def test_reset_pairing_timeout_leaves_unowned_without_reopening(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d); b = root/'button'; l = root/'led'; b.write_text('1'); l.touch()
            args = SimpleNamespace(owner=root/'owner', device_name_file=root/'name', button_file=b, led_file=l)
            save_owner(args.owner, {'address':'AA:BB:CC:DD:EE:FF', 'address_type':'public'})
            calls = []; expired = asyncio.Event()
            async def normal(*args):
                try: await asyncio.Event().wait()
                finally: calls.append('stopped')
            async def remove_bond(*args): calls.append('removed')
            async def pair(*args): calls.append('pair'); expired.set(); return False
            with patch.object(lifecycle.HoldDetector, 'update', side_effect=lambda *args: not expired.is_set()), patch.object(lifecycle, 'remove_owner_bond', remove_bond):
                service = asyncio.create_task(lifecycle.run_customer_service(args, normal, pair))
                try:
                    await asyncio.wait_for(expired.wait(), 1)
                    await asyncio.sleep(.12)
                    self.assertFalse(args.owner.exists())
                    self.assertEqual(calls, ['stopped', 'removed', 'pair'])
                finally:
                    service.cancel()
                    with self.assertRaises(asyncio.CancelledError): await service

    async def test_output_shutdown_failure_prevents_reset(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d); b = root/'button'; l = root/'led'; b.write_text('0'); l.touch()
            args = SimpleNamespace(owner=root/'owner', device_name_file=root/'name', button_file=b, led_file=l)
            save_owner(args.owner, {'address':'AA:BB:CC:DD:EE:FF', 'address_type':'public'})
            async def normal(*args):
                try: await asyncio.Event().wait()
                finally: raise RuntimeError('outputs did not close')
            async def forbidden(*args): self.fail('must not reset or pair')
            with patch.object(lifecycle.HoldDetector, 'update', return_value=True), patch.object(lifecycle, 'remove_owner_bond', forbidden):
                with self.assertRaisesRegex(RuntimeError, 'outputs did not close'):
                    await asyncio.wait_for(lifecycle.run_customer_service(args, normal, forbidden), 1)
            self.assertTrue(args.owner.exists())
