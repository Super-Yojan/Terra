"""Customer service: wait for physical enrollment, then serve the saved owner."""
import asyncio
import signal
import time
from .onboarding import HatIndicators, HoldDetector, load_device_name, load_owner
from .pairing import pair_owner

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
        while owner is None:
            try:
                hat.set_led(False)
                pressed = hat.pressed()
                if detector.update(pressed, time.monotonic()):
                    if await asyncio.wait_for(pair(args, hat, name), getattr(args, 'setup_seconds', 60) + 15):
                        owner = load_owner(args.owner)
                        if owner is None:
                            raise RuntimeError('pairing completed without durable owner')
                        await flash_success(hat)
                        break
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
        hat.set_led(True)
        await normal(args, name)
    finally:
        try: hat.set_led(False)
        except OSError: pass
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.remove_signal_handler(sig)
