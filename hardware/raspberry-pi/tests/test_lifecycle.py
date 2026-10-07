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
