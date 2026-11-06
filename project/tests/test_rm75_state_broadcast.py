import json
import math
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'Assets' / 'Scripts'))

from rm75_state_broadcast import StateBroadcaster, encode_state_packet


JOINTS = [0.416, 9.003, 0.346, -34.931, -1.889, -56.030, -0.179]
STATE = {'joint': JOINTS, 'pose': [-.21, 0, .70, 0, -1.4, 0]}


class FakeSocket:
    def __init__(self, failing_hosts=()):
        self.failing_hosts = set(failing_hosts)
        self.attempts = []
        self.closed = False

    def sendto(self, payload, destination):
        self.attempts.append((payload, destination))
        if destination[0] in self.failing_hosts:
            raise OSError(f'cannot reach {destination[0]}')
        return len(payload)

    def close(self):
        self.closed = True


class StatePacketTests(unittest.TestCase):
    def test_packet_has_exact_version_one_contract(self):
        packet = json.loads(encode_state_packet(JOINTS, 7, 12.5))
        self.assertEqual(set(packet), {
            'type', 'version', 'sequence', 'sent_time', 'connected', 'joint_deg'
        })
        self.assertEqual(packet, {
            'type': 'rm75_state',
            'version': 1,
            'sequence': 7,
            'sent_time': 12.5,
            'connected': True,
            'joint_deg': JOINTS,
        })

    def test_packet_rejects_invalid_joint_sequence_and_time_values(self):
        invalid_joints = [JOINTS[:6], JOINTS + [0], [True, *JOINTS[1:]],
                          [math.nan, *JOINTS[1:]], [math.inf, *JOINTS[1:]]]
        for joints in invalid_joints:
            with self.subTest(joints=joints), self.assertRaises(ValueError):
                encode_state_packet(joints, 0, 0.0)
        for sequence in (-1, True, 1.5):
            with self.subTest(sequence=sequence), self.assertRaises(ValueError):
                encode_state_packet(JOINTS, sequence, 0.0)
        for sent_time in (-0.1, True, math.nan, math.inf):
            with self.subTest(sent_time=sent_time), self.assertRaises(ValueError):
                encode_state_packet(JOINTS, 0, sent_time)


class StateBroadcasterTests(unittest.TestCase):
    def test_publish_increments_sequence_and_targets_localhost_and_pico(self):
        now = [3.0]
        udp = FakeSocket()
        broadcaster = StateBroadcaster(udp, lambda: now[0], port=5006,
                                       warn=lambda unused: None)
        broadcaster.publish(STATE, '192.168.30.88')
        now[0] = 3.2
        broadcaster.publish(STATE, '192.168.30.88')

        destinations = [destination for unused, destination in udp.attempts]
        self.assertEqual(destinations, [
            ('127.0.0.1', 5006), ('192.168.30.88', 5006),
            ('127.0.0.1', 5006), ('192.168.30.88', 5006),
        ])
        packets = [json.loads(payload) for payload, unused in udp.attempts]
        self.assertEqual([packet['sequence'] for packet in packets], [0, 0, 1, 1])
        self.assertEqual([packet['sent_time'] for packet in packets],
                         [3.0, 3.0, 3.2, 3.2])

    def test_loopback_pico_destination_is_not_duplicated(self):
        udp = FakeSocket()
        broadcaster = StateBroadcaster(udp, lambda: 0.0, warn=lambda unused: None)
        broadcaster.publish(STATE, '127.0.0.1')
        self.assertEqual([destination for unused, destination in udp.attempts],
                         [('127.0.0.1', 5006)])

    def test_send_failure_is_nonfatal_attempts_all_targets_and_throttles_warning(self):
        now = [0.0]
        warnings = []
        udp = FakeSocket(failing_hosts={'127.0.0.1'})
        broadcaster = StateBroadcaster(udp, lambda: now[0], warning_interval=1.0,
                                       warn=warnings.append)
        broadcaster.publish(STATE, '192.168.30.88')
        now[0] = 0.5
        broadcaster.publish(STATE, '192.168.30.88')
        now[0] = 1.0
        broadcaster.publish(STATE, '192.168.30.88')

        self.assertEqual(len(udp.attempts), 6)
        self.assertEqual(len(warnings), 2)
        self.assertIn('127.0.0.1:5006', warnings[0])
        self.assertIn('cannot reach', broadcaster.last_warning)

    def test_constructor_rejects_invalid_configuration_and_close_owns_socket(self):
        for port in (0, 65536, True):
            with self.subTest(port=port), self.assertRaises(ValueError):
                StateBroadcaster(FakeSocket(), lambda: 0.0, port=port)
        for warning_interval in (0, -1, math.nan, True):
            with self.subTest(interval=warning_interval), self.assertRaises(ValueError):
                StateBroadcaster(FakeSocket(), lambda: 0.0,
                                 warning_interval=warning_interval)
        udp = FakeSocket()
        StateBroadcaster(udp, lambda: 0.0).close()
        self.assertTrue(udp.closed)


if __name__ == '__main__':
    unittest.main()
