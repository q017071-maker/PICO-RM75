import json
from pathlib import Path
import socket
import subprocess
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / 'Assets' / 'Scripts'
sys.path.insert(0, str(SCRIPTS))

try:
    import xrobotoolkit_udp_bridge as bridge
except ModuleNotFoundError:
    bridge = None

from rm75_preview_core import ControllerPacket


class FakeSdk:
    def __init__(self):
        self.timestamp_ns = 100
        self.pose = [0.1, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0]
        self.grip = 0.75
        self.trigger = 0.25

    def get_time_stamp_ns(self):
        return self.timestamp_ns

    def get_right_controller_pose(self):
        return list(self.pose)

    def get_right_grip(self):
        return self.grip

    def get_right_trigger(self):
        return self.trigger


class FailingSdk(FakeSdk):
    def get_right_controller_pose(self):
        raise RuntimeError('tracking read failed')


class FakeUdpSocket:
    def __init__(self):
        self.sent = []

    def sendto(self, payload, destination):
        self.sent.append((payload, destination))
        return len(payload)


class BridgePacketTests(unittest.TestCase):
    def test_zero_identity_tracking_pose_is_invalid_even_with_live_buttons(self):
        encoder = bridge.BridgeEncoder(
            stale_timeout=0.25,
            session_factory=lambda: 'session-a',
        )

        payload = encoder.encode(
            timestamp_ns=100,
            pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            grip=1.0,
            trigger=0.5,
            now=1.0,
        )

        decoded = json.loads(payload)
        self.assertFalse(decoded['valid'])
        self.assertEqual(decoded['grip'], 0.0)
        self.assertEqual(decoded['trigger'], 0.0)

    def test_fresh_right_controller_sample_matches_existing_protocol(self):
        if bridge is None:
            self.fail('xrobotoolkit_udp_bridge is not implemented')

        encoder = bridge.BridgeEncoder(
            stale_timeout=0.25,
            session_factory=lambda: 'session-a',
        )
        payload = encoder.encode(
            timestamp_ns=100,
            pose=[0.1, -0.2, 0.3, 0.0, 0.5, 0.0, 0.8660254],
            grip=0.75,
            trigger=0.25,
            now=1.5,
        )

        decoded = json.loads(payload)
        self.assertEqual(decoded, {
            'session': 'session-a',
            'sequence': 0,
            'sent_time': 1.5,
            'x': 0.1,
            'y': -0.2,
            'z': 0.3,
            'qx': 0.0,
            'qy': 0.5,
            'qz': 0.0,
            'qw': 0.8660254,
            'trigger': 0.25,
            'grip': 0.75,
            'valid': True,
        })
        parsed = ControllerPacket.parse(payload)
        self.assertEqual(parsed.position, (0.1, -0.2, 0.3))
        self.assertEqual(parsed.grip, 0.75)
        self.assertTrue(parsed.valid)

    def test_repeated_device_timestamp_becomes_invalid_at_timeout(self):
        encoder = bridge.BridgeEncoder(
            stale_timeout=0.25,
            session_factory=lambda: 'session-a',
        )
        sample = dict(
            timestamp_ns=100,
            pose=[0.1, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
            grip=1.0,
            trigger=0.0,
        )

        first = json.loads(encoder.encode(now=1.0, **sample))
        before_timeout = json.loads(encoder.encode(now=1.249, **sample))
        at_timeout = json.loads(encoder.encode(now=1.25, **sample))

        self.assertTrue(first['valid'])
        self.assertTrue(before_timeout['valid'])
        self.assertFalse(at_timeout['valid'])
        self.assertEqual(at_timeout['sequence'], 2)
        self.assertEqual(at_timeout['sent_time'], 1.25)

    def test_malformed_sdk_sample_becomes_parseable_invalid_heartbeat(self):
        cases = [
            dict(pose=[0.0] * 6, grip=0.0, trigger=0.0),
            dict(pose=[0.0, 0.0, float('nan'), 0.0, 0.0, 0.0, 1.0],
                 grip=0.0, trigger=0.0),
            dict(pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                 grip=-0.1, trigger=0.0),
            dict(pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                 grip=0.0, trigger=1.1),
        ]
        for changes in cases:
            with self.subTest(changes=changes):
                encoder = bridge.BridgeEncoder(
                    stale_timeout=0.25,
                    session_factory=lambda: 'session-a',
                )
                try:
                    payload = encoder.encode(timestamp_ns=100, now=1.0, **changes)
                    decoded = json.loads(payload)
                    parsed = ControllerPacket.parse(payload)
                except Exception as exc:
                    self.fail(f'malformed SDK sample raised {exc!r}')
                self.assertFalse(decoded['valid'])
                self.assertFalse(parsed.valid)
                self.assertEqual(decoded['grip'], 0.0)
                self.assertEqual(decoded['trigger'], 0.0)
                self.assertEqual(
                    [decoded[key] for key in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                )

    def test_device_timestamp_reset_starts_new_control_session(self):
        sessions = iter(('session-a', 'session-b'))
        encoder = bridge.BridgeEncoder(
            stale_timeout=0.25,
            session_factory=lambda: next(sessions),
        )
        sample = dict(
            pose=[0.1, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
            grip=0.0,
            trigger=0.0,
        )

        before_reset = json.loads(encoder.encode(
            timestamp_ns=200, now=1.0, **sample))
        after_reset = json.loads(encoder.encode(
            timestamp_ns=100, now=2.0, **sample))

        self.assertEqual((before_reset['session'], before_reset['sequence']),
                         ('session-a', 0))
        self.assertEqual((after_reset['session'], after_reset['sequence']),
                         ('session-b', 0))
        self.assertTrue(after_reset['valid'])


class BridgeUdpTests(unittest.TestCase):
    def test_send_once_delivers_sdk_sample_to_udp_boundary(self):
        send_once = getattr(bridge, 'send_once', None)
        self.assertIsNotNone(send_once, 'send_once is not implemented')
        encoder = bridge.BridgeEncoder(
            stale_timeout=0.25,
            session_factory=lambda: 'session-a',
        )
        udp = FakeUdpSocket()
        destination = ('127.0.0.1', 5005)
        send_once(encoder, FakeSdk(), udp, destination, now=1.5)
        payload, actual_destination = udp.sent[0]

        decoded = json.loads(payload)
        self.assertEqual(actual_destination, destination)
        self.assertEqual(decoded['sent_time'], 1.5)
        self.assertEqual(decoded['x'], 0.1)
        self.assertEqual(decoded['grip'], 0.75)
        self.assertEqual(decoded['trigger'], 0.25)
        self.assertTrue(decoded['valid'])

    def test_sdk_read_failure_sends_invalid_heartbeat(self):
        encoder = bridge.BridgeEncoder(
            stale_timeout=0.25,
            session_factory=lambda: 'session-a',
        )
        udp = FakeUdpSocket()
        try:
            bridge.send_once(encoder, FailingSdk(), udp,
                             ('127.0.0.1', 5005), now=1.5)
        except Exception as exc:
            self.fail(f'SDK read failure escaped bridge: {exc!r}')
        payload, unused_destination = udp.sent[0]

        decoded = json.loads(payload)
        self.assertFalse(decoded['valid'])
        self.assertEqual(decoded['sequence'], 0)
        self.assertEqual(decoded['grip'], 0.0)
        self.assertEqual(decoded['trigger'], 0.0)
        self.assertEqual(decoded['qw'], 1.0)


class BridgeCliTests(unittest.TestCase):
    def test_help_does_not_require_installed_sdk(self):
        script = SCRIPTS / 'xrobotoolkit_udp_bridge.py'
        result = subprocess.run(
            [sys.executable, '-S', str(script), '--help'],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--host', result.stdout)
        self.assertIn('--port', result.stdout)
        self.assertIn('--rate', result.stdout)
        self.assertIn('--stale-timeout', result.stdout)


if __name__ == '__main__':
    unittest.main()
