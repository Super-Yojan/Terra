"""Output thread failures must reach the ownership-reset lifecycle."""
import asyncio
import threading
from types import SimpleNamespace
import unittest
from terra_rover.ble import BlePeripheral

class OutputShutdownTests(unittest.IsolatedAsyncioTestCase):
    async def test_backend_close_failure_propagates_after_thread_join(self):
        peripheral = BlePeripheral()
        peripheral.stop.set()
        closed = []
        def close():
            closed.append(True)
            raise OSError('hardware close failed')
        peripheral.backend = SimpleNamespace(close=close)
        peripheral.store = SimpleNamespace(end_connection=lambda now: None)
        peripheral.safety = SimpleNamespace(layout=None)
        worker = threading.Thread(target=peripheral._worker)
        worker.start()
        with self.assertRaisesRegex(RuntimeError, 'output worker failed'):
            await peripheral.join_worker(worker)
        self.assertFalse(worker.is_alive())
        self.assertTrue(closed)

    async def test_safe_output_failure_still_closes_backend_and_blocks_reset(self):
        peripheral = BlePeripheral()
        peripheral.stop.set()
        closed = []
        def apply(batch): raise OSError('safe output failed')
        peripheral.backend = SimpleNamespace(apply=apply, close=lambda: closed.append(True))
        peripheral.store = SimpleNamespace(end_connection=lambda now: None)
        peripheral.safety = SimpleNamespace(layout=object(), tick=lambda now, gate: object())
        worker = threading.Thread(target=peripheral._worker)
        worker.start()
        with self.assertRaisesRegex(RuntimeError, 'output worker failed'):
            await peripheral.join_worker(worker)
        self.assertEqual(closed, [True])

    async def test_worker_crash_is_logged_and_monitor_reports_failure(self):
        peripheral = BlePeripheral()
        peripheral.backend = SimpleNamespace(close=lambda: None)
        peripheral.store = SimpleNamespace(end_connection=lambda now: None)
        peripheral.safety = SimpleNamespace(layout=None)
        def fail(): raise ValueError('status serialization failed')
        peripheral._output_loop = fail
        worker = threading.Thread(target=peripheral._worker)
        with self.assertLogs('terra_rover.ble', level='ERROR') as captured:
            worker.start()
            await asyncio.to_thread(worker.join)
        self.assertIn('status serialization failed', '\n'.join(captured.output))
        with self.assertRaisesRegex(RuntimeError, 'output worker failed'):
            await peripheral.monitor_worker(worker)

    async def test_unexpected_clean_worker_exit_is_not_a_healthy_service(self):
        peripheral = BlePeripheral()
        worker = threading.Thread(target=lambda: None)
        worker.start()
        with self.assertRaisesRegex(RuntimeError, 'output worker stopped unexpectedly'):
            await peripheral.monitor_worker(worker)
