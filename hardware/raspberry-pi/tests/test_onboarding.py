import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from terra_rover import onboarding as o

class OnboardingTests(unittest.TestCase):
    def test_boot_hold_requires_release_and_hold_fires_once(self):
        h = o.HoldDetector()
        self.assertFalse(h.update(True, 0))
        self.assertFalse(h.update(True, 10))
        h.update(False, 11); h.update(False, 11.2)
        h.update(True, 12); h.update(True, 12.2)
        self.assertFalse(h.update(True, 14.9))
        self.assertTrue(h.update(True, 15.3))
        self.assertFalse(h.update(True, 20))
    def test_short_press_does_not_trigger(self):
        h = o.HoldDetector()
        for p,t in [(False,0),(False,.2),(True,1),(True,1.2),(False,2),(False,2.2)]:
            self.assertFalse(h.update(p,t))
    def test_owner_is_private_validated_and_never_overwritten(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'owner.json'; identity={'address':'AA:BB:CC:DD:EE:FF','address_type':'random'}
            self.assertIsNone(o.load_owner(p))
            o.save_owner(p,identity)
            self.assertEqual(o.load_owner(p),identity)
            with self.assertRaises(FileExistsError): o.save_owner(p,identity)
            p.chmod(0o644)
            with self.assertRaises(ValueError): o.load_owner(p)
    def test_corrupt_owner_never_becomes_unowned(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'owner.json'; p.write_text('{}'); p.chmod(0o600)
            with self.assertRaises(ValueError): o.load_owner(p)
    def test_name_persists(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'name.json'; name=o.load_device_name(p)
            self.assertRegex(name,r'^terra-[A-Z0-9]{6}$')
            self.assertEqual(o.load_device_name(p),name)
    def test_indicators_reject_invalid_button(self):
        with tempfile.TemporaryDirectory() as d:
            b=Path(d)/'button'; l=Path(d)/'led'; b.write_text('1\n'); l.touch()
            hat=o.HatIndicators(b,l)
            self.assertTrue(hat.pressed()); hat.set_led(True); self.assertEqual(l.read_text(),'1')
            b.write_text('unexpected')
            with self.assertRaises(ValueError): hat.pressed()

    def test_directory_sync_failure_rolls_back_owner(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'owner'; original=os.fsync; count=[0]
            def failing(fd):
                count[0]+=1
                if count[0]==2: raise OSError('directory sync failed')
                original(fd)
            with patch('terra_rover.onboarding.os.fsync',side_effect=failing):
                with self.assertRaises(OSError): o.save_owner(p,{'address':'AA:BB:CC:DD:EE:FF','address_type':'random'})
            self.assertFalse(p.exists())

class PairingNameTests(unittest.TestCase):
    def test_pairing_name(self):
        from terra_rover.onboarding import pairing_name
        self.assertEqual(pairing_name("terra-ABC123"), "terra-ABC123-pair")
        self.assertEqual(pairing_name("terra-ABC123-pair"), "terra-ABC123-pair")
