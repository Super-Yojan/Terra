"""Fusion HAT channels are kernel-global: a disabled DC motor leg must not latch a fault."""
import errno
import gc
import unittest

from terra_rover.backend import FusionHatBackend, MOTOR_PINS
from terra_rover.drive_relay import OutputBatch, OutputCommand

ENABLED = {}

class FakePWM:
    """Mirrors fusion_hat.pwm.PWM against the driver: duty writes to a disabled channel fail with EINVAL."""
    CHANNEL_NUM = 12
    def __init__(self, channel, freq=50, **kwargs):
        self.channel = int(str(channel).lstrip('P'))
        self.duty = 0
        self.enable()
    def freq(self, value=None): pass
    def enable(self, enable=True): ENABLED[self.channel] = enable
    def pulse_width(self, value=None): self.pulse_width_percent(value)
    def pulse_width_percent(self, value=None):
        if not ENABLED[self.channel]: raise OSError(errno.EINVAL, 'Invalid argument')
        self.duty = value
    def close(self): self.enable(False)
    def __del__(self): self.close()

class FakeMotor:
    MOTOR_PINS = MOTOR_PINS
    DEFAULT_FREQ = 100
    def __init__(self, motor, **kwargs):
        self.pwm_a, self.pwm_b = (FakePWM(pin) for pin in MOTOR_PINS[motor])
    def power(self, value):
        power = int(abs(value))
        self.pwm_a.pulse_width_percent(power if value > 0 else 0)
        self.pwm_b.pulse_width_percent(0 if value > 0 else power)

def four_motor_layout():
    return dict(schema_version=1, revision=1, actuators=[dict(
        id=index, name=port, port=port, kind='dc_motor', inverted=False, limits=dict(min=-1, max=1),
        calibration=dict(type='dc_motor', max_power_fraction=1.0),
        route=dict(type='left_effort' if index % 2 == 0 else 'right_effort'), safe=dict(type='zero'))
        for index, port in enumerate(MOTOR_PINS)])

def batch(value):
    return OutputBatch(tuple(OutputCommand(index, value) for index in range(len(MOTOR_PINS))))

class FusionHatChannelTests(unittest.TestCase):
    def backend(self):
        backend = FusionHatBackend(motor_factory=FakeMotor, pwm_factory=FakePWM, library_version='1.14.0')
        backend.configure(four_motor_layout())
        return backend

    def setUp(self): ENABLED.clear()

    def test_every_channel_is_available_without_port_configuration(self):
        ports = FusionHatBackend().capabilities()['ports']
        self.assertEqual(set(ports), set(MOTOR_PINS) | {f'P{i}' for i in range(12)})

    def test_motor_leg_disabled_elsewhere_is_reenabled_instead_of_faulting(self):
        backend = self.backend()
        for channel in ENABLED: ENABLED[channel] = False
        backend.apply(batch(.5))
        self.assertTrue(all(ENABLED.values()))
        self.assertEqual(backend._outputs['M0'].pwm_a.duty, 50)

    def test_dropped_backend_does_not_disable_replacement_channels(self):
        old = self.backend()
        new = self.backend()
        del old; gc.collect()
        new.apply(batch(-.25))
        self.assertEqual(new._outputs['M3'].pwm_b.duty, 25)
        self.assertTrue(all(ENABLED.values()))

if __name__ == '__main__': unittest.main()
