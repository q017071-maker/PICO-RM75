"""Real UDP socket smoke test, with NO robot SDK and NO device control."""
import json
import errno
import os
import select
import socket
import subprocess
import sys
import time
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / 'Assets/Scripts/rm75_joystick_bridge.py'


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def frame(session, sequence, grip):
    return json.dumps({
        'session': session, 'sequence': sequence, 'sent_time': 1.0 + sequence * .033,
        'x':0.0,'y':0.0,'z':0.0,'qx':0.0,'qy':0.0,'qz':0.0,'qw':1.0,
        'trigger':0.0,'grip':grip,'valid':True,
        'device_valid':True,'position_valid':True,'rotation_valid':True,
        'grip_valid':True,'tracked_valid':True,'tracked':True
    }).encode()


class UDPRelaySmokeTests(unittest.TestCase):
    def test_socket_relay(self):
        try:
            ports = set()
            while len(ports) < 4:
                ports.add(free_port())
            base_port, stick_port, robot_port, state_port = ports
        except PermissionError as exc:
            if exc.errno not in (errno.EPERM, errno.EACCES) or os.environ.get('RM75_REQUIRE_UDP') == '1':
                raise
            self.skipTest('Sandbox denies UDP sockets; rerun with RM75_REQUIRE_UDP=1 in host terminal')
        self.assertEqual(len({base_port,stick_port,robot_port}), 3)
        command = [sys.executable, str(BRIDGE), '--bind', '127.0.0.1',
                   '--base-ip', '127.0.0.1', '--stick-ip', '127.0.0.1',
                   '--base-port', str(base_port), '--stick-port', str(stick_port),
                   '--robot-port', str(robot_port), '--state-port', str(state_port),
                   '--state-destination-ip', '127.0.0.2']
        process = subprocess.Popen([command[0], '-u', *command[1:]],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            self.assertTrue(select.select([process.stdout], [], [], 5)[0], 'Bridge startup timed out')
            self.assertIn('JOYSTICK BRIDGE', process.stdout.readline())
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, \
                    socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as pico, \
                    socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as unity, \
                    socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as feedback:
                receiver.bind(('127.0.0.1', robot_port))
                receiver.settimeout(1.5)
                feedback.bind(('127.0.0.2', state_port))
                feedback.settimeout(1.5)
                unity.sendto(json.dumps({'session':'unity-1', 'sequence':0,
                                         'valid':True,'left_x':0,'left_y':1,
                                         'right_x':0,'right_y':1}).encode(),
                             ('127.0.0.1', stick_port))
                time.sleep(.04)
                for seq, grip in [(0,0),(1,1),(2,1)]:
                    if seq == 2:
                        time.sleep(.04)
                    pico.sendto(frame('real-pico', seq, grip), ('127.0.0.1', base_port))
                    result = json.loads(receiver.recvfrom(4096)[0])
                    self.assertEqual(result['sequence'], seq)
                    self.assertTrue(result['valid'])
                    self.assertTrue(result['tracked_valid'])
                    if seq == 2:
                        self.assertGreater(result['z'], 0.0)
                        self.assertGreater(result['y'], 0.0)
                        self.assertEqual(result['qx'], 0.0)
                measured = b'{"type":"rm75_measured_joints","joint":[1,2,3,4,5,6,7]}'
                receiver.sendto(measured, ('127.0.0.1', state_port))
                self.assertEqual(feedback.recvfrom(4096)[0], measured)
                # Companion loss invalidates even though the base stream resumes.
                time.sleep(.22)
                pico.sendto(frame('real-pico',3,1), ('127.0.0.1',base_port))
                self.assertFalse(json.loads(receiver.recvfrom(4096)[0])['valid'])
            self.assertIsNone(process.poll(), 'Relay should still be running')
        finally:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
            if process.stdout is not None:
                process.stdout.close()
            if process.stderr is not None:
                process.stderr.close()


if __name__ == '__main__':
    unittest.main()
