import asyncio
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch
from terra_rover import __main__ as cli

class TerminalPairingTests(unittest.TestCase):
    def test_terminal_enrollment_uses_pairing_advertisement(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            gate = root / 'gate'; gate.write_text('0')
            args = SimpleNamespace(owner=root/'owner', expected_peer='AA:BB:CC:DD:EE:FF',
                                   gate_file=gate, name='terra-ABC123', adapter=None)
            class Prepared(Exception): pass
            class Transport:
                def __init__(self, name, service, adapter):
                    self.name = name
                    assert name == 'terra-ABC123-pair'
                async def prepare(self): raise Prepared()
            with patch.object(cli, 'BlessTransport', Transport):
                with self.assertRaises(Prepared): asyncio.run(cli.setup(args))
