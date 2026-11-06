"""Fault and kinematic checks for the addon, independent of robot SDK."""
import json
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_rm75_joystick_bridge import base, stick, module as bridge, unpack


class SafetyTests(unittest.TestCase):
    def ready(self, **axes):
        a = bridge.JoystickAdapter()
        a.receive_stick(stick(**axes), 1.)
        a.process_base(base(), 1.01)
        a.process_base(base(seq=1, grip=1), 1.02)
        return a

    def test_each_axis_both_signs(self):
        for axis, field, scale in [('ly','z',1), ('lx','x',-1), ('ry','y',1), ('rx','qz',1)]:
            for sign in [-1, 1]:
                with self.subTest(axis=axis, sign=sign):
                    a = self.ready(**{axis: sign})
                    d = unpack(a.process_base(base(seq=2, grip=1), 1.07))
                    expected = (math.sin(sign * .05 * .05 / 2) if field == 'qz'
                                else sign * scale * .05 * .05 / .2)
                    self.assertAlmostEqual(d[field], expected)

    def test_combined_translation_speed_limit(self):
        a = self.ready(lx=1, ly=1, ry=1)
        d = unpack(a.process_base(base(seq=2, grip=1), 1.07))
        distance = math.hypot(d['x'], d['y'], d['z']) * .2
        self.assertAlmostEqual(distance, .05 * .05)

    def test_stick_release_zeroes_velocity(self):
        a = self.ready(ly=1)
        first = unpack(a.process_base(base(seq=2, grip=1), 1.07))
        a.receive_stick(stick(seq=1), 1.08)
        second = unpack(a.process_base(base(seq=3, grip=1), 1.10))
        self.assertEqual(first['z'], second['z'])

    def test_real_pose_motion_is_not_added(self):
        a = self.ready()
        d = json.loads(base(seq=2, grip=1))
        d.update(x=99, y=-44, z=23, qx=.5, qy=.5, qz=.5, qw=.5)
        result = unpack(a.process_base(json.dumps(d).encode(), 1.07))
        self.assertEqual([result[k] for k in ('x','y','z','qx','qy','qz','qw')], [0,0,0,0,0,0,1])

    def test_malformed_stick_latches_release(self):
        a = self.ready(ly=1)
        with self.assertRaises(ValueError): a.receive_stick(b'{}', 1.04)
        a.receive_stick(stick(seq=1, ly=1), 1.05)
        self.assertFalse(unpack(a.process_base(base(seq=2, grip=1), 1.06))['valid'])

    def test_unread_base_fields_not_sanitized_by_virtual_pose(self):
        for field, value in [('x',float('nan')), ('grip',2), ('sequence',-1), ('session',''), ('valid',1)]:
            a = self.ready()
            data = json.loads(base(seq=2, grip=1)); data[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                a.process_base(json.dumps(data).encode(), 1.06)

    def test_base_timeout_requires_release_even_with_fresh_stick(self):
        a = self.ready(ly=1)
        a.receive_stick(stick(seq=1, ly=1), 1.4)
        self.assertFalse(unpack(a.process_base(base(seq=2, grip=1), 1.41))['valid'])

    def test_stick_reconnection_without_base_packet_cannot_hide_timeout(self):
        a = self.ready(ly=1)
        a.receive_stick(stick(seq=1, ly=1), 1.25)
        self.assertFalse(unpack(a.process_base(base(seq=2, grip=1), 1.26))['valid'])

    def test_source_switch_and_retirement(self):
        a = self.ready()
        a.receive_stick(stick(seq=1), 1.04, ('pico', 10))
        self.assertFalse(unpack(a.process_base(base(seq=2, grip=1), 1.05))['valid'])
        self.assertFalse(a.receive_stick(stick(seq=2), 1.06))
        a.process_base(base(seq=3), 1.07)
        self.assertFalse(unpack(a.process_base(base(seq=4, grip=1), 1.08, ('pico', 11)))['valid'])

    def test_sent_time_replay_does_not_refresh_stick(self):
        a = bridge.JoystickAdapter()
        def packet(seq, sent):
            d = json.loads(stick(seq=seq)); d['sent_time'] = sent
            return json.dumps(d).encode()
        a.receive_stick(packet(0, 1.), 1.)
        self.assertFalse(a.receive_stick(packet(1, 1.), 1.1))
        self.assertEqual(a.stick.received_at, 1.)

    def test_diagnostic_flags_preserved_without_fabrication(self):
        a = self.ready()
        d = json.loads(base(seq=2, grip=1)); d.pop('tracked_valid'); d['tracked'] = False
        r = unpack(a.process_base(json.dumps(d).encode(), 1.07))
        self.assertNotIn('tracked_valid', r)
        self.assertFalse(r['tracked'])
        self.assertNotIn('tracking_verified', r)

    def test_deadzone_boundary(self):
        self.assertEqual(bridge.radial_deadzone(.15, 0), (0,0))
        self.assertGreater(bridge.radial_deadzone(.151, 0)[0], 0)


class PreviewModeTests(unittest.TestCase):
    def test_live_rejected_before_original_entry(self):
        import rm75_joystick_preview as preview
        from unittest.mock import patch
        with patch.object(preview.preview, 'main') as entry:
            with self.assertRaises(ValueError): preview.main(['--live'])
            entry.assert_not_called()

    def test_direct_pose_port_rejected(self):
        import rm75_joystick_preview as preview
        with self.assertRaises(ValueError): preview.main([])

    def test_four_axis_simulation_local(self):
        import importlib.util
        import contextlib
        import io
        path=Path(__file__).resolve().parents[1]/'scripts/rm75_joystick_offline.py'
        spec=importlib.util.spec_from_file_location('offline_joystick_test',path)
        sim=importlib.util.module_from_spec(spec); spec.loader.exec_module(sim)
        with contextlib.redirect_stdout(io.StringIO()):
            sim.simulate(path.parents[1]/'Assets/Scripts/rm75_preview_core.py')

if __name__ == '__main__': unittest.main()
