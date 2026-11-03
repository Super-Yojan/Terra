"""Customer service: wait for physical enrollment, then serve the saved owner."""
import asyncio
import os
import signal
import time
from .onboarding import HatIndicators, HoldDetector, load_device_name, load_owner
from .pairing import pair_owner, remove_owner_bond

async def wait_for_hold(hat, detector):
    while True:
        try:
            if detector.update(hat.pressed(), time.monotonic()):
                return
        except (OSError, ValueError) as exc:
            print(f'USR unavailable: {exc}', flush=True)
            detector = HoldDetector()
        await asyncio.sleep(.05)

async def serve_until_hold(args, name, normal, hat, detector):
    service = asyncio.create_task(normal(args, name))
    button = asyncio.create_task(wait_for_hold(hat, detector))
    try:
        done, _ = await asyncio.wait((service, button), return_when=asyncio.FIRST_COMPLETED)
        if service in done:
            await service
            return False
        await button
        return True
    finally:
        # Await output-worker shutdown and GATT teardown before ownership changes.
        for task in (service, button):
            if not task.done(): task.cancel()
        results = await asyncio.gather(service, button, return_exceptions=True)
        for result in results:
            if isinstance(result, BaseException) and not isinstance(result, asyncio.CancelledError):
                raise result

async def flash_success(hat):
    for _ in range(3):
        hat.set_led(True); await asyncio.sleep(.15)
        hat.set_led(False); await asyncio.sleep(.15)

async def run_customer_service(args, normal, pair=pair_owner):
    # Validate before interpreting a missing file as permission to enroll.
    owner = load_owner(args.owner)
    name = load_device_name(args.device_name_file)
    hat = HatIndicators(args.button_file, args.led_file)
    detector = HoldDetector()
    loop = asyncio.get_running_loop()
    task = asyncio.current_task()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, task.cancel)
    try:
        while True:
            if owner is not None:
                hat.set_led(True)
                if not await serve_until_hold(args, name, normal, hat, detector):
                    return
                # A cleanup failure leaves the owner intact and exits fail-closed.
                await asyncio.wait_for(remove_owner_bond(args, owner), 15)
                args.owner.unlink()
                directory = os.open(args.owner.parent, os.O_RDONLY | os.O_DIRECTORY)
                try: os.fsync(directory)
                finally: os.close(directory)
                owner = None
                print('Ownership cleared by USR; opening pairing window.', flush=True)
                enroll = True
            else:
                enroll = False
            try:
                hat.set_led(False)
                pressed = hat.pressed()
                if enroll or detector.update(pressed, time.monotonic()):
                    if await asyncio.wait_for(pair(args, hat, name), getattr(args, 'setup_seconds', 60) + 15):
                        owner = load_owner(args.owner)
                        if owner is None:
                            raise RuntimeError('pairing completed without durable owner')
                        await flash_success(hat)
                    detector = HoldDetector()
                await asyncio.sleep(.05)
            except (OSError, ValueError, RuntimeError) as exc:
                # Reset hold state after any fault; no stale hold may authorize retry.
                print(f'Enrollment unavailable: {exc}', flush=True)
                detector = HoldDetector()
                try:
                    for _ in range(2):
                        hat.set_led(True); await asyncio.sleep(.1)
                        hat.set_led(False); await asyncio.sleep(.1)
                except OSError:
                    pass
                await asyncio.sleep(2)
                owner = load_owner(args.owner)
    finally:
        try: hat.set_led(False)
        except OSError: pass
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.remove_signal_handler(sig)
