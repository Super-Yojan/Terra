import unittest
from terra_rover.safety import SafetyController
from terra_rover.protocol import CommandFrame, CommandKind, ActuatorValue

class MotionPauseTests(unittest.TestCase):
    def armed(self):
        c = SafetyController()
        c.layout = {'revision': 1, 'actuators': [{'id': 0, 'kind': 'motor', 'safe': {'type': 'zero'}, 'limits': {'min': -1, 'max': 1}, 'calibration': {}}]}
        c.session = 7
        c.subscribed = c.hardware_gate = c.armed = True
        c.arming = False
        c.armed_at = 0.
        c.mailbox = ((ActuatorValue(0, .5),), .1)
        c.last_time = .1
        c.last_sequence = 1
        return c

    def frame(self, kind, sequence, value=0.):
        return CommandFrame(kind, 7, 1, sequence, (ActuatorValue(0, value),) if kind == CommandKind.DRIVE else ())

    def test_expiry_stops_without_disarming_and_requires_explicit_recovery(self):
        c = self.armed()
        b = c.tick(.31, True)
        self.assertTrue(b.armed)
        self.assertEqual(b.reason, 'watchdog_paused')
        self.assertTrue(all(v.value == 0 for v in b.commands))
        # Neither a renewed nonzero stream nor neutral alone can restart motion.
        c.accept(self.frame(CommandKind.DRIVE, 2, .5), .32, .32)
        self.assertTrue(all(v.value == 0 for v in c.tick(.33, True).commands))
        self.assertFalse(c.accept(self.frame(CommandKind.ARM, 3), .34, .34))
        self.assertTrue(c.accept(self.frame(CommandKind.DRIVE, 4), .35, .35))
        self.assertTrue(c.motion_paused)
        self.assertTrue(c.accept(self.frame(CommandKind.ARM, 5), .36, .36))
        self.assertFalse(c.motion_paused)
        self.assertTrue(all(v.value == 0 for v in c.tick(.37, True).commands))
        self.assertTrue(c.accept(self.frame(CommandKind.DRIVE, 6, .5), .38, .38))
        self.assertEqual(c.tick(.39, True).commands[0].value, .5)

    def test_disconnect_fault_and_emergency_stop_still_disarm(self):
        for action in ('disconnect', 'fault', 'emergency'):
            c = self.armed()
            c.tick(.31, True)
            if action == 'disconnect': c.disconnect(.32)
            elif action == 'fault': c.latch_fault('test', .32)
            else: c.accept(self.frame(CommandKind.EMERGENCY_STOP, 2), .32, .32)
            self.assertFalse(c.armed)

    def test_expired_safe_frame_cannot_resume(self):
        c = self.armed()
        c.tick(.31, True)
        c.accept(self.frame(CommandKind.DRIVE, 2), .32, .32)
        self.assertFalse(c.accept(self.frame(CommandKind.ARM, 3), .53, .53))
        self.assertTrue(c.motion_paused)

    def test_first_packet_after_gap_cannot_turn_pause_into_protocol_disarm(self):
        c = self.armed()
        self.assertTrue(c.accept(self.frame(CommandKind.DRIVE, 2, .5), .31, .31))
        self.assertTrue(c.armed and c.motion_paused)
        self.assertTrue(all(v.value == 0 for v in c.tick(.32, True).commands))
        self.assertIsNone(c.safe_received)
