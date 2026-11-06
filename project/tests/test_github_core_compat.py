"""Integration checks against the real rm75_preview_core.py when installed in a checkout.
No SDK, network, or robot access. Skipped when running from standalone ZIP.
"""
import importlib.util
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Assets" / "Scripts"))
import os
import unittest

ROOT = Path(__file__).resolve().parents[1]
CORE_PATH = Path(os.environ.get('RM75_COMPAT_CORE', ROOT / 'Assets' / 'Scripts' / 'rm75_preview_core.py'))
BRIDGE_PATH = ROOT / 'Assets' / 'Scripts' / 'rm75_joystick_bridge.py'


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


bridge = load_module('rm75_joystick_bridge_integration', BRIDGE_PATH)
core = load_module('rm75_preview_core_integration', CORE_PATH) if CORE_PATH.is_file() else None


def base(seq, grip):
    return json.dumps(dict(session='real-pico-01', sequence=seq, sent_time=seq * .033 + 1,
        x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1,
        grip=grip, trigger=0, valid=True,
        device_valid=True, position_valid=True, rotation_valid=True,
        grip_valid=True, tracked_valid=True, tracked=True)).encode()


def stick(seq, ly=0, valid=True):
    return json.dumps(dict(session='stick-one', sequence=seq, valid=valid,
        left_x=0, left_y=ly, right_x=0, right_y=0)).encode()


@unittest.skipUnless(core, 'The original GitHub rm75_preview_core.py is not inside the standalone addon')
class RealCoreCompatibilityTests(unittest.TestCase):
    def test_new_packet_passes_actual_github_parser(self):
        a = bridge.JoystickAdapter()
        a.receive_stick(stick(0, ly=1), 1.0)
        forwarded = a.process_base(base(0, 0), 1.01)
        packet = core.ControllerPacket.parse(forwarded)
        self.assertEqual(packet.session, 'real-pico-01')
        if hasattr(packet, 'tracking_verified'):
            self.assertTrue(packet.tracking_verified)
        else:
            self.assertTrue(json.loads(forwarded)['tracked_valid'])
        self.assertTrue(packet.valid)
        self.assertEqual(packet.position, (0, 0, 0))

    def test_real_preview_engine_gets_incremental_forward_target(self):
        a = bridge.JoystickAdapter()
        config = core.PreviewConfig(scale=.2, axes=('z','-x','y'), radius=.05, speed=.02,
                                    timeout=.25, jump_limit=.1)
        engine = core.PreviewEngine(config, lambda: [0, 0, 0, 0, 0, 0])
        peer = ('127.0.0.1', 55000)
        a.receive_stick(stick(0, ly=1), 1.0)
        # First packet identifies a session, second releases, third activates, fourth moves.
        for seq, grip, now in [(0,0,1.01),(1,0,1.04),(2,1,1.07),(3,1,1.10)]:
            pkt = core.ControllerPacket.parse(a.process_base(base(seq,grip),now))
            target = engine.accept(pkt,now,peer)
        self.assertIsNotNone(target)
        self.assertEqual(engine.status, 'ACTIVE')
        self.assertGreater(target[0], 0.)
        self.assertAlmostEqual(target[1], 0.)
        self.assertAlmostEqual(target[2], 0.)

    def test_loss_disarms_actual_preview_engine(self):
        a = bridge.JoystickAdapter()
        engine = core.PreviewEngine(core.PreviewConfig(),lambda:[0,0,0,0,0,0])
        peer = ('127.0.0.1', 55000)
        a.receive_stick(stick(0), 1.0)
        for seq, grip, now in [(0,0,1.01),(1,0,1.04),(2,1,1.07)]:
            packet = core.ControllerPacket.parse(a.process_base(base(seq,grip),now))
            engine.accept(packet,now,peer)
        self.assertEqual(engine.status,'ACTIVE')
        packet = core.ControllerPacket.parse(a.process_base(base(3,1),1.25))
        self.assertFalse(packet.valid)
        engine.accept(packet,1.25,peer)
        self.assertEqual(engine.status,'WAIT_RELEASE')


if __name__ == '__main__':
    unittest.main()
