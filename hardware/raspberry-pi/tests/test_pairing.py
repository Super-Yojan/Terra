import unittest
from dbus_next import DBusError, Variant
from terra_rover.pairing import PairingSession, ButtonSetupStatus

class PairingTests(unittest.IsolatedAsyncioTestCase):
    def test_only_one_candidate_can_enter_window(self):
        s=PairingSession(60,set(),clock=lambda: 1)
        s.authorize('/phone1')
        with self.assertRaises(DBusError): s.authorize('/phone2')
    def test_expiry_revokes_candidate(self):
        now=[1]; s=PairingSession(60,set(),clock=lambda:now[0]); s.authorize('/phone')
        now[0]=60
        with self.assertRaises(DBusError): s.authorize('/phone')
    def test_existing_bond_cannot_claim_new_owner(self):
        s=PairingSession(60,{'/old'},clock=lambda:1)
        with self.assertRaises(DBusError): s.authorize('/old')
    def test_owner_requires_encrypted_read_and_connected_bond(self):
        s=PairingSession(60,set(),clock=lambda:1); s.authorize('/phone')
        fields={k:Variant('b',True) for k in ('Paired','Bonded','Connected')}
        self.assertFalse(s.ready(fields))
        s.read('/phone'); self.assertTrue(s.ready(fields))
        fields['Connected']=Variant('b',False); self.assertFalse(s.ready(fields))
        s.cancel(); self.assertFalse(s.ready(fields))
    async def test_status_rejects_unapproved_device_and_is_disarmed(self):
        s=PairingSession(60,set(),clock=lambda:1)
        committed=[]
        async def commit(device): committed.append(device)
        status=ButtonSetupStatus(s,commit)
        # dbus-next wraps exported methods; exercise the underlying function.
        read=ButtonSetupStatus.ReadValue.__wrapped__
        with self.assertRaises(DBusError): await read(status,{'device':Variant('o','/phone')})
        s.authorize('/phone')
        self.assertIn(b'"armed":false',await read(status,{'device':Variant('o','/phone')}))
        self.assertTrue(s.read_seen)
        self.assertEqual(committed,['/phone'])

    def test_candidate_disconnect_revokes_window(self):
        s=PairingSession(60,set(),clock=lambda:1); s.authorize('/phone')
        s.connection_changed('/phone',False)
        with self.assertRaises(DBusError): s.authorize('/phone')

    def test_first_connection_waits_for_authorization_but_excludes_competitors(self):
        s=PairingSession(60,set(),clock=lambda:1)
        self.assertTrue(s.connection_changed('/phone',True))
        self.assertFalse(s.connection_changed('/other',True))
        with self.assertRaises(DBusError): s.authorize('/other')
        s.authorize('/phone')
