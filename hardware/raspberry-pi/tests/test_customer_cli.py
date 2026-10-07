import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

class CustomerCliTests(unittest.TestCase):
    def test_unowned_service_waits_without_gate_and_stops_cleanly(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); button=root/'button'; led=root/'led'; button.write_text('0'); led.touch()
            rover=Path(__file__).resolve().parents[1]/'rover.py'
            process=subprocess.Popen([sys.executable,str(rover),'--button-pairing','--owner',str(root/'owner'), '--device-name-file',str(root/'name'),'--button-file',str(button),'--led-file',str(led)], stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            try:
                deadline=time.monotonic()+3
                while not (root/'name').exists() and process.poll() is None and time.monotonic()<deadline: time.sleep(.02)
                self.assertTrue((root/'name').exists())
                self.assertIsNone(process.poll())
                time.sleep(.05)
                process.send_signal(signal.SIGTERM)
                out,err=process.communicate(timeout=5)
                self.assertEqual(process.returncode,0,err.decode())
                self.assertFalse((root/'owner').exists())
                self.assertEqual(led.read_text(),'0')
            finally:
                if process.poll() is None: process.kill(); process.communicate()
