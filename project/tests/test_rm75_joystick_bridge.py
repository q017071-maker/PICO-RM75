import importlib.util
import json
import math
from pathlib import Path
import unittest

MODULE = Path(__file__).resolve().parents[1] / 'Assets/Scripts/rm75_joystick_bridge.py'
spec = importlib.util.spec_from_file_location('rm75_joystick_bridge', MODULE)
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Assets" / "Scripts"))
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def stick(seq=0, session='stick-1', valid=True, lx=0, ly=0, rx=0, ry=0):
    return json.dumps({'sequence': seq, 'session': session, 'valid': valid,
                       'left_x': lx, 'left_y': ly, 'right_x': rx,
                       'right_y': ry}).encode()


def base(seq=0, session='base-1', grip=0, valid=True):
    return json.dumps({
        'session': session, 'sequence': seq, 'sent_time': 1.0 + seq * .033,
        'x': 1.0, 'y': 2.0, 'z': 3.0,
        'qx': 0., 'qy': 0., 'qz': 0., 'qw': 1.,
        'trigger': 0., 'grip': grip, 'valid': valid,
        'device_valid': True, 'position_valid': True,
        'rotation_valid': True, 'grip_valid': True,
        'tracked_valid': True, 'tracked': True
    }).encode()


def unpack(blob):
    return json.loads(blob)


class JoystickBridgeTests(unittest.TestCase):
    def new_ready(self, lx=0, ly=0, rx=0, ry=0):
        a = module.JoystickAdapter()
        a.receive_stick(stick(lx=lx, ly=ly, rx=rx, ry=ry), 1.0)
        self.assertTrue(unpack(a.process_base(base(grip=0), 1.01))['valid'])
        self.assertTrue(unpack(a.process_base(base(seq=1, grip=1), 1.02))['valid'])
        return a

    def test_stick_axes_move_through_original_z_minus_x_y_mapping(self):
        a = self.new_ready(lx=0, ly=1, ry=1)
        data = unpack(a.process_base(base(seq=2, grip=1), 1.07))
        # 0.05m/s * 0.05s along robot X, Y, Z; synthetic PICO coords.
        self.assertAlmostEqual(data['x'], 0.0)
        self.assertAlmostEqual(data['y'], +0.0125 / math.sqrt(2))
        self.assertAlmostEqual(data['z'], +0.0125 / math.sqrt(2))
        self.assertEqual(data['grip'], 1)
        self.assertTrue(data['tracked_valid'])
        self.assertTrue(data['device_valid'])

    def test_positive_roll_maps_to_positive_pico_z_quaternion(self):
        a = self.new_ready(rx=1.0)
        d = unpack(a.process_base(base(seq=2, grip=1), 1.07))
        expected_angle = module.MAX_ROLL_SPEED * module.MAX_INTEGRATION_DT
        self.assertAlmostEqual(d['qz'], math.sin(expected_angle/2))
        self.assertAlmostEqual(d['qw'], math.cos(expected_angle/2))
        self.assertAlmostEqual(d['x'], 0)
        self.assertAlmostEqual(d['y'], 0)
        self.assertAlmostEqual(d['z'], 0)

    def test_deadzone_and_inactivity(self):
        a = self.new_ready(lx=.01, ly=.08, rx=.03, ry=.02)
        d = unpack(a.process_base(base(seq=2, grip=1), 1.07))
        self.assertEqual([d['x'], d['y'], d['z'], d['qz']], [0,0,0,0])
        d = unpack(a.process_base(base(seq=3, grip=1), 1.12))
        self.assertTrue(d['valid'])

    def test_stick_loss_forces_disarm_and_release_before_rearm(self):
        a = self.new_ready(ly=1)
        d = unpack(a.process_base(base(seq=2, grip=1), 1.22))
        self.assertFalse(d['valid'])
        a.receive_stick(stick(seq=1, ly=1), 1.23)
        d = unpack(a.process_base(base(seq=3, grip=1), 1.24))
        self.assertFalse(d['valid'])
        d = unpack(a.process_base(base(seq=4, grip=0), 1.25))
        self.assertTrue(d['valid'])
        d = unpack(a.process_base(base(seq=5, grip=1), 1.26))
        self.assertTrue(d['valid'])
        self.assertEqual(d['z'], 0)

    def test_releasing_grip_resets_virtual_reference(self):
        a = self.new_ready(ly=1)
        a.process_base(base(seq=2, grip=1), 1.07)
        released = unpack(a.process_base(base(seq=3, grip=0), 1.08))
        self.assertEqual(released['z'], 0)
        self.assertTrue(released['valid'])

    def test_original_invalid_tracking_never_upgraded(self):
        a = self.new_ready(ly=1)
        d = unpack(a.process_base(base(seq=2, grip=1, valid=False), 1.06))
        self.assertFalse(d['valid'])
        # No automatic re-engage while still holding Grip.
        d = unpack(a.process_base(base(seq=3, grip=1, valid=True), 1.07))
        self.assertFalse(d['valid'])

    def test_replay_sequence_ignored(self):
        a = module.JoystickAdapter()
        self.assertTrue(a.receive_stick(stick(seq=8), 1.0))
        self.assertFalse(a.receive_stick(stick(seq=7), 1.1))
        self.assertEqual(a.stick.received_at, 1.0)

    def test_invalid_inputs_rejected(self):
        a = module.JoystickAdapter()
        for value in (1.1, float('nan'), True, '1', None):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    a.receive_stick(stick(lx=value), 1.0)

    def test_base_session_replacement_requires_grip_release(self):
        a = self.new_ready(ly=1)
        a.process_base(base(seq=2, grip=1), 1.07)
        d = unpack(a.process_base(base(seq=0, session='base-2', grip=1), 1.08))
        self.assertFalse(d['valid'])
        d = unpack(a.process_base(base(seq=1, session='base-2', grip=0), 1.09))
        self.assertTrue(d['valid'])
        self.assertEqual(d['z'], 0.0)

    def test_diagonal_is_not_faster_than_full_stick(self):
        x, y = module.radial_deadzone(1, 1)
        self.assertLessEqual(math.hypot(x,y), 1 + 1e-9)
        self.assertAlmostEqual(x, y)

    def test_stale_repeated_stick_sequence_does_not_refresh_watchdog(self):
        a = self.new_ready(ly=1)
        self.assertFalse(a.receive_stick(stick(seq=0, ly=1), 1.19))
        d = unpack(a.process_base(base(seq=2, grip=1), 1.22))
        self.assertFalse(d['valid'])

    def test_source_restart_requires_release(self):
        a = self.new_ready(ly=1)
        a.process_base(base(seq=2, grip=1), 1.07)
        a.receive_stick(stick(seq=0, session='stick-new', ly=1), 1.08)
        d = unpack(a.process_base(base(seq=3, grip=1), 1.09))
        self.assertFalse(d['valid'])
        d = unpack(a.process_base(base(seq=4, grip=0), 1.10))
        self.assertTrue(d['valid'])


if __name__ == '__main__':
    unittest.main()
