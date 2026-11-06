"""Bridge XRoboToolkit controller samples into the existing RM75 UDP protocol."""
import argparse
import json
import math
import socket
import sys
import time
import uuid


SAFE_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]


def _finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _valid_sample(timestamp_ns, pose, grip, trigger):
    return (
        type(timestamp_ns) is int and timestamp_ns > 0 and
        isinstance(pose, (list, tuple)) and len(pose) == 7 and
        all(_finite_number(value) for value in pose) and
        math.hypot(*pose[3:]) > 1e-6 and
        (math.hypot(*pose[:3]) > 1e-6 or math.hypot(*pose[3:6]) > 1e-6) and
        _finite_number(grip) and 0.0 <= grip <= 1.0 and
        _finite_number(trigger) and 0.0 <= trigger <= 1.0
    )


class BridgeEncoder:
    def __init__(self, stale_timeout, session_factory):
        self.stale_timeout = stale_timeout
        self._session_factory = session_factory
        self.session = session_factory()
        self.sequence = 0
        self._last_timestamp_ns = None
        self._last_advance_time = None

    def encode(self, *, timestamp_ns, pose, grip, trigger, now):
        sample_valid = _valid_sample(timestamp_ns, pose, grip, trigger)
        if (sample_valid and self._last_timestamp_ns is not None and
                timestamp_ns < self._last_timestamp_ns):
            self.session = self._session_factory()
            self.sequence = 0
        if sample_valid and timestamp_ns != self._last_timestamp_ns:
            self._last_timestamp_ns = timestamp_ns
            self._last_advance_time = now
        valid = (sample_valid and self._last_advance_time is not None and
                 now - self._last_advance_time < self.stale_timeout)
        if not sample_valid:
            pose, grip, trigger = SAFE_POSE, 0.0, 0.0
        payload = {
            'session': self.session,
            'sequence': self.sequence,
            'sent_time': now,
            'x': pose[0],
            'y': pose[1],
            'z': pose[2],
            'qx': pose[3],
            'qy': pose[4],
            'qz': pose[5],
            'qw': pose[6],
            'trigger': trigger,
            'grip': grip,
            'valid': valid,
        }
        self.sequence += 1
        return json.dumps(payload, separators=(',', ':'),
                          allow_nan=False).encode('utf-8')


def send_once(encoder, sdk, udp, destination, *, now):
    try:
        timestamp_ns = sdk.get_time_stamp_ns()
        pose = sdk.get_right_controller_pose()
        grip = sdk.get_right_grip()
        trigger = sdk.get_right_trigger()
    except Exception:
        timestamp_ns, pose, grip, trigger = 0, SAFE_POSE, 0.0, 0.0
    payload = encoder.encode(
        timestamp_ns=timestamp_ns,
        pose=pose,
        grip=grip,
        trigger=trigger,
        now=now,
    )
    udp.sendto(payload, destination)
    return payload


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=('Read the XRoboToolkit right controller and send the existing '
                     'RM75 preview UDP protocol. This program never connects to the robot.'))
    parser.add_argument('--host', default='127.0.0.1',
                        help='rm75_target_preview.py host')
    parser.add_argument('--port', type=int, default=5005,
                        help='rm75_target_preview.py UDP port')
    parser.add_argument('--rate', type=float, default=50.0,
                        help='output packets per second')
    parser.add_argument('--stale-timeout', type=float, default=0.25,
                        help='seconds before an unchanged XR timestamp becomes invalid')
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error('--rate must be a finite positive number')
    if not math.isfinite(args.stale_timeout) or args.stale_timeout <= 0:
        parser.error('--stale-timeout must be a finite positive number')

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError:
        print('ERROR: xrobotoolkit-sdk is not installed in this Python environment',
              file=sys.stderr)
        return 1

    destination = (args.host, args.port)
    encoder = BridgeEncoder(args.stale_timeout,
                            session_factory=lambda: uuid.uuid4().hex)
    period = 1.0 / args.rate
    initialized = False
    last_valid = None
    next_report = 0.0
    try:
        xrt.init()
        initialized = True
        print(f'XRoboToolkit bridge → UDP {args.host}:{args.port} at {args.rate:g} Hz',
              flush=True)
        print('RIGHT controller only | Grip=teleop enable | robot connection: none',
              flush=True)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            next_tick = time.monotonic()
            while True:
                now = time.monotonic()
                payload = send_once(encoder, xrt, udp, destination, now=now)
                packet = json.loads(payload)
                if packet['valid'] != last_valid:
                    state = 'VALID' if packet['valid'] else 'INVALID'
                    print(f'{state}: XR controller stream', flush=True)
                    last_valid = packet['valid']
                if now >= next_report:
                    print('RIGHT xyz=(' + ', '.join(
                        f"{packet[key]:+.3f}" for key in ('x', 'y', 'z')) +
                        f") grip={packet['grip']:.2f} trigger={packet['trigger']:.2f}",
                        flush=True)
                    next_report = now + 1.0
                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()
    except KeyboardInterrupt:
        print('XRoboToolkit bridge stopped.', flush=True)
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr, flush=True)
        return 1
    finally:
        if initialized:
            xrt.close()


if __name__ == '__main__':
    sys.exit(main())
