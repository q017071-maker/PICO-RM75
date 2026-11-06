"""Opt-in, robot-SDK-free dual-thumbstick relay for RM75-PICO.

Legacy PICO pose stream -> UDP :5005 -> this bridge -> UDP :5008 ->
independent read-only rm75_joystick_preview.py on loopback :5008.

A companion Unity script sends two thumbsticks -> UDP :5007.  The relay
synthesizes a *virtual PICO controller pose*; the existing validated
PreviewEngine and LiveMotionController keep all their original arming,
workspace, orientation, IK, timeouts and safety protection.

This file NEVER connects to or commands the robot.  Mode switching is done by
stopping the robot controller and restarting it with/without this relay.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
import selectors
import socket
import time
from typing import Any

from rm75_preview_core import ControllerPacket
from rm75_joystick_settings import TRANSLATION_SPEED_M_S

# Mapping remains axes=(z,-x,y), scale=.2. Only the joystick translation
# profile changes; the original pose-follow live configuration is untouched.
AXES_SCALE = 0.2
MAX_TRANSLATION_SPEED = TRANSLATION_SPEED_M_S
MAX_ROLL_SPEED = 0.050         # rad/s; existing orientation velocity limit
STICK_DEADZONE = 0.15
STICK_TIMEOUT = 0.20           # seconds; fail closed if companion stops
MAX_INTEGRATION_DT = 0.05
DEFAULT_STATE_PORT = 5006
MAX_UDP_BYTES = 4096


class SenderRate:
    """Rolling sample rate from one sender's clock; not one-way latency."""
    def __init__(self):
        self._source = None
        self._sequence = -1
        self._times = deque(maxlen=51)

    def observe(self, source, sequence, sent_time):
        if source != self._source:
            self._source, self._sequence = source, -1
            self._times.clear()
        if sent_time is None or sequence <= self._sequence:
            return
        if self._times and sent_time <= self._times[-1]:
            return
        self._sequence = sequence
        self._times.append(sent_time)

    def label(self):
        if len(self._times) < 3:
            return '?'
        return f'{(len(self._times)-1)/(self._times[-1]-self._times[0]):.1f}'


def _finite(value: Any) -> bool:
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _axis_pair(left: Any, right: Any) -> tuple[float, float]:
    if not _finite(left) or not _finite(right) or not (-1 <= left <= 1 and -1 <= right <= 1):
        raise ValueError('Thumbsticks must be finite numbers in [-1,1]')
    return float(left), float(right)


def radial_deadzone(x: float, y: float, deadzone: float = STICK_DEADZONE) -> tuple[float, float]:
    magnitude = math.hypot(x, y)
    if magnitude <= deadzone:
        return 0.0, 0.0
    factor = (min(1.0, magnitude) - deadzone) / (1 - deadzone) / magnitude
    return x * factor, y * factor


@dataclass(frozen=True)
class StickSample:
    lx: float
    ly: float
    rx: float
    ry: float
    sequence: int
    session: str
    valid: bool
    received_at: float
    sent_time: float | None = None

    @classmethod
    def parse(cls, payload: bytes, received_at: float) -> 'StickSample':
        try:
            packet = json.loads(payload)
        except (UnicodeError, ValueError) as exc:
            raise ValueError('Invalid thumbstick JSON') from exc
        if not isinstance(packet, dict):
            raise ValueError('Thumbstick packet must be an object')
        if (type(packet.get('sequence')) is not int or packet['sequence'] < 0 or
                not isinstance(packet.get('session'), str) or not packet['session'].strip() or
                type(packet.get('valid')) is not bool):
            raise ValueError('Stick session/sequence/valid missing or invalid')
        sent = packet.get('sent_time')
        if sent is not None and (not _finite(sent) or sent < 0):
            raise ValueError('Invalid stick sent_time')
        lx, ly = _axis_pair(packet.get('left_x'), packet.get('left_y'))
        rx, ry = _axis_pair(packet.get('right_x'), packet.get('right_y'))
        return cls(lx, ly, rx, ry, packet['sequence'], packet['session'],
                   packet['valid'], received_at, sent)


class PicoDiscovery:
    """Pin one IP after a fresh, verified released-Grip base/stick pair.

    Discovery only selects an input source. It never asserts tracking validity
    or bypasses the adapter/preview release-to-rearm state machines. A different
    IP requires restarting the bridge. This is source selection, not network
    authentication.
    """

    def __init__(self):
        self.selected_ip = None
        self._sticks = {}

    def observe(self, kind, data, peer, now):
        if self.selected_ip is not None:
            return self.selected_ip if peer[0] == self.selected_ip else None
        self._sticks = {ip: value for ip, value in self._sticks.items()
                        if 0 <= now - value[0] < STICK_TIMEOUT}
        try:
            if kind == 'stick':
                sample = StickSample.parse(data, now)
                if sample.valid:
                    if len(self._sticks) >= 8 and peer[0] not in self._sticks:
                        return None
                    self._sticks[peer[0]] = (now, data, peer)
                else:
                    self._sticks.pop(peer[0], None)
            elif kind == 'base':
                packet = ControllerPacket.parse(data)
                if packet.tracking_verified and packet.grip <= .3 and peer[0] in self._sticks:
                    self.selected_ip = peer[0]
                    return self.selected_ip
        except ValueError:
            # An invalid companion must not leave an earlier discovery sample.
            if kind == 'stick':
                self._sticks.pop(peer[0], None)
        return None

    def selected_stick(self):
        return self._sticks[self.selected_ip]


class JoystickAdapter:
    """Converts real PICO tracking packets into virtual motion packets.

    Never creates valid tracking metadata.  All existing diagnostics, Grip,
    session, sequence and timestamps remain from the ORIGINAL stream.
    """

    def __init__(self) -> None:
        self.stick: StickSample | None = None
        self._stick_session: str | None = None
        self._stick_seq = -1
        self._base_session: str | None = None
        self._base_seq = -1
        self._base_sent_time = -1.0
        self._last_at: float | None = None
        self._need_release = True
        self._virtual_pico_position = [0.0, 0.0, 0.0]
        self._roll = 0.0
        self._was_gripped = False
        self._retired_sticks = set()
        self._retired_bases = set()
        self._stick_source = None
        self._base_source = None
        self._stick_sent = None
        self._base_received = None

    def invalidate(self):
        self._need_release = True
        self.stick = None
        self._reset_virtual()

    def receive_stick(self, packet: bytes, now: float, peer=None) -> bool:
        try:
            sample = StickSample.parse(packet, now)
        except ValueError:
            self.invalidate()
            raise
        source = (peer, sample.session)
        if source in self._retired_sticks:
            return False
        if source != self._stick_source:
            if self._stick_source is not None:
                self._retired_sticks.add(self._stick_source)
            self.invalidate()
            self._stick_source = source
            self._stick_session = sample.session
            self._stick_seq = -1
            self._stick_sent = None
        if sample.sequence <= self._stick_seq:
            return False
        if self._stick_sent is not None:
            if sample.sent_time is None or sample.sent_time <= self._stick_sent:
                return False
            if sample.sent_time - self._stick_sent >= STICK_TIMEOUT:
                self.invalidate()
        if self.stick is not None and now - self.stick.received_at >= STICK_TIMEOUT:
            self.invalidate()
        self._stick_seq = sample.sequence
        self._stick_sent = sample.sent_time
        self.stick = sample
        if not sample.valid:
            self._need_release = True
            self._reset_virtual()
        return True

    def _reset_virtual(self) -> None:
        self._virtual_pico_position = [0.0, 0.0, 0.0]
        self._roll = 0.0
        self._last_at = None
        self._was_gripped = False

    def process_base(self, data: bytes, now: float, peer=None) -> bytes:
        try:
            return self._process_base(data, now, peer)
        except ValueError:
            self.invalidate()
            raise

    def _process_base(self, data: bytes, now: float, peer=None) -> bytes:
        try:
            packet = json.loads(data)
        except (UnicodeError, ValueError) as exc:
            raise ValueError('Invalid base controller JSON') from exc
        if not isinstance(packet, dict):
            raise ValueError('Base packet must be JSON object')
        if (not isinstance(packet.get('session'), str) or
                type(packet.get('sequence')) is not int or
                not _finite(packet.get('grip')) or
                not _finite(packet.get('sent_time'))):
            raise ValueError('Invalid base session/sequence/sent_time/grip')
        original = ControllerPacket.parse(data)  # Validate before replacing pose fields.
        source = (peer, packet['session'])
        if source in self._retired_bases:
            raise ValueError('Retired base source')
        # Sender switches and grip release must not keep old integrated offsets.
        if source != self._base_source:
            if self._base_source is not None:
                self._retired_bases.add(self._base_source)
            self._base_source = source
            self._base_session = packet['session']
            self._base_seq = -1
            self._base_sent_time = -1.0
            self._need_release = True
            self._reset_virtual()
        if packet['sequence'] <= self._base_seq or packet['sent_time'] <= self._base_sent_time:
            raise ValueError('Base replay/nonmonotonic metadata rejected')
        if (self._base_received is not None and now - self._base_received >= STICK_TIMEOUT):
            self._need_release = True
            self._reset_virtual()
        if self._base_sent_time >= 0 and packet['sent_time'] - self._base_sent_time >= .25:
            self._need_release = True
            self._reset_virtual()
        self._base_received = now
        self._base_seq = packet['sequence']
        self._base_sent_time = packet['sent_time']
        if packet['grip'] <= 0.3:
            self._need_release = False
            self._reset_virtual()
        fresh = (self.stick is not None and self.stick.valid and
                 0 <= now - self.stick.received_at < STICK_TIMEOUT)
        if not fresh:
            # Invalid tracking triggers the ORIGINAL PreviewEngine disarm.
            # Never fabricate tracking verification or keep running on stale axes.
            packet['valid'] = False
            self._need_release = True
            self._reset_virtual()
        elif self._need_release:
            packet['valid'] = False
        elif not original.tracking_verified:
            packet['valid'] = False
            self._need_release = True
            self._reset_virtual()
        else:
            if self._last_at is not None:
                dt = now - self._last_at
                if dt < 0 or dt >= STICK_TIMEOUT:
                    self._need_release = True
                    packet['valid'] = False
                    self._reset_virtual()
                elif self._was_gripped and packet['grip'] >= 0.7:
                    dt = min(dt, MAX_INTEGRATION_DT)
                    lx, ly = radial_deadzone(self.stick.lx, self.stick.ly)
                    rx, ry = radial_deadzone(self.stick.rx, self.stick.ry)
                    # In robot-base axes: left up => +X, left right => +Y,
                    # right up => +Z.  Invert z,-x,y and scale=.2 to synthesize
                    # a PICO pose that the EXISTING PreviewEngine will map.
                    norm = max(1., math.sqrt(ly * ly + lx * lx + ry * ry))
                    dx = (ly / norm * MAX_TRANSLATION_SPEED * dt)
                    dy = (lx / norm * MAX_TRANSLATION_SPEED * dt)
                    dz = (ry / norm * MAX_TRANSLATION_SPEED * dt)
                    self._virtual_pico_position[0] -= dy / AXES_SCALE
                    self._virtual_pico_position[1] += dz / AXES_SCALE
                    self._virtual_pico_position[2] += dx / AXES_SCALE
                    self._roll += rx * MAX_ROLL_SPEED * dt
            if packet.get('valid') is True:
                self._last_at = now
                self._was_gripped = packet['grip'] >= 0.7
        # The PICO source is left-handed, and the existing orientation mapper
        # conjugates this quaternion.  The axes basis has det=-1; a
        # positive PICO-Z quaternion here yields +robot-base-X after both
        # conjugation and the reflected z,-x,y basis transform.
        half = self._roll / 2
        packet.update({
            'x': self._virtual_pico_position[0],
            'y': self._virtual_pico_position[1],
            'z': self._virtual_pico_position[2],
            'qx': 0.0, 'qy': 0.0, 'qz': math.sin(half), 'qw': math.cos(half),
        })
        return json.dumps(packet, separators=(',', ':'), allow_nan=False).encode('utf-8')


def _bind_udp(host: str, port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((host, port))
        sock.setblocking(False)
        return sock
    except BaseException:
        sock.close()
        raise


def serve(args: argparse.Namespace) -> None:
    if bool(args.base_ip) != bool(args.stick_ip):
        raise ValueError('Set both --base-ip and --stick-ip, or omit both for discovery')
    discovery = PicoDiscovery() if not args.base_ip else None
    if (args.base_port == args.stick_port or args.base_port == args.robot_port or
            args.stick_port == args.robot_port or
            args.state_port in (args.base_port, args.stick_port, args.robot_port)):
        raise ValueError('Base, stick and forwarded robot UDP ports must be distinct')
    if any(not 1 <= port <= 65535 for port in
           (args.base_port, args.stick_port, args.robot_port, args.state_port)):
        raise ValueError('Invalid UDP port')
    adapter = JoystickAdapter()
    base_rate, stick_rate = SenderRate(), SenderRate()
    sel = selectors.DefaultSelector()
    base = stick = outgoing = state = None
    try:
        base = _bind_udp(args.bind, args.base_port)
        stick = _bind_udp(args.bind, args.stick_port)
        outgoing = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sel.register(base, selectors.EVENT_READ, 'base')
        sel.register(stick, selectors.EVENT_READ, 'stick')
        if args.state_relay:
            state = _bind_udp('127.0.0.1', args.state_port)
            sel.register(state, selectors.EVENT_READ, 'state')
        print('JOYSTICK BRIDGE (NO ROBOT SDK) | PICO port', args.base_port,
              '| stick port', args.stick_port, '| forwarded robot port', args.robot_port,
              '| state relay:', args.state_relay, flush=True)
        print('Offline/read-only validation only. Release Grip to arm.', flush=True)
        if discovery is not None:
            print('自动识别 PICO：保持同一 Wi-Fi，戴上头显进入应用，松开右手 Grip。', flush=True)
        last_controller_ip = None
        last_warning = -float('inf')
        last_input_report = -float('inf')
        last_wait_report = time.monotonic() if discovery is not None else 0.0
        while True:
            events = sel.select(timeout=0.05)
            if discovery is not None and discovery.selected_ip is None:
                now = time.monotonic()
                if now - last_wait_report >= 5:
                    print('等待 PICO 的手柄与摇杆数据；确认应用在前台并松开 Grip。'
                          '若电脑 Wi-Fi 地址改变，插着 USB 时运行 bash 启动.sh configure-pico。', flush=True)
                    last_wait_report = now
            # When both sockets are ready, apply current stick input before
            # integrating the matching base sample. Avoid a one-sample delay.
            events = sorted(events, key=lambda event: event[0].data != 'stick')
            for key, _ in events:
                source = key.data
                sock = key.fileobj
                data, peer = sock.recvfrom(MAX_UDP_BYTES)
                now = time.monotonic()
                if discovery is not None and source != 'state':
                    was_selected = discovery.selected_ip is not None
                    if discovery.observe(source, data, peer, now) is None:
                        continue
                    if not was_selected:
                        at, payload, stick_peer = discovery.selected_stick()
                        adapter.receive_stick(payload, at, stick_peer)
                        print(f'已识别并锁定 PICO {discovery.selected_ip}，无需手动填写 IP。', flush=True)
                if source == 'stick':
                    if args.stick_ip and peer[0] != args.stick_ip:
                        continue
                    try:
                        if adapter.receive_stick(data, now, peer):
                            p = adapter.stick
                            stick_rate.observe((peer, p.session), p.sequence, p.sent_time)
                    except ValueError as exc:
                        if now - last_warning >= 1:
                            print('REJECT STICK:', exc, flush=True)
                            last_warning = now
                elif source == 'base':
                    if args.base_ip and peer[0] != args.base_ip:
                        continue
                    last_controller_ip = peer[0]
                    try:
                        virtual = adapter.process_base(data, now, peer)
                    except ValueError as exc:
                        # Dropping all traffic causes the existing packet watchdog
                        # to disarm.  Do not forward malformed base data.
                        if now - last_warning >= 1:
                            print('REJECT BASE:', exc, flush=True)
                            last_warning = now
                        continue
                    outgoing.sendto(virtual, ('127.0.0.1', args.robot_port))
                    output = json.loads(virtual)
                    base_rate.observe((peer, output['session']), output['sequence'], output['sent_time'])
                    if now - last_input_report >= 1:
                        axes = None if adapter.stick is None else (
                            adapter.stick.lx, adapter.stick.ly, adapter.stick.rx, adapter.stick.ry)
                        print(f'INPUT {peer[0]} valid={output["valid"]} '
                              f'grip={output["grip"]:.2f} axes(lx,ly,rx,ry)={axes} '
                              f'base_hz={base_rate.label()} stick_hz={stick_rate.label()}', flush=True)
                        last_input_report = now
                else:
                    # Existing robot sender sees 127.0.0.1 and sends its state
                    # back to this local bridge. Preserve original PICO feedback.
                    target_ip = args.state_destination_ip or last_controller_ip
                    if target_ip and target_ip not in ('127.0.0.1', 'localhost') and peer[0] == '127.0.0.1':
                        outgoing.sendto(data, (target_ip, args.state_port))
    finally:
        sel.close()
        for sock in (base, stick, outgoing, state):
            if sock is not None:
                sock.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--base-port', type=int, default=5005)
    parser.add_argument('--stick-port', type=int, default=5007)
    parser.add_argument('--robot-port', type=int, default=5008)
    parser.add_argument('--base-ip', help='Optional exact controller source IP allowlist')
    parser.add_argument('--stick-ip', help='Optional exact joystick source IP allowlist')
    parser.add_argument('--state-port', type=int, default=DEFAULT_STATE_PORT)
    parser.add_argument('--state-destination-ip',
                        help='Actual PICO/Unity state receiver IP (if distinct from base sender)')
    parser.add_argument('--no-state-relay', dest='state_relay', action='store_false')
    args = parser.parse_args(argv)
    try:
        serve(args)
    except KeyboardInterrupt:
        print('Joystick relay stopped. Robot-side watchdog will disarm.', flush=True)
    except (OSError, ValueError) as exc:
        print('ERROR:', exc, flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
