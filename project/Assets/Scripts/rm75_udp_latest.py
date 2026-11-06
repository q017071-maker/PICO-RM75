"""Bounded UDP coalescing for joystick input; never discard a stop/fault edge.

Only consecutive verified held-Grip motion samples from one source may be
replaced by a newer sample. Release, invalid tracking/JSON, metadata changes,
replays, sender gaps and pose jumps are returned immediately to the unchanged
safety loop. An overflowing queue terminates that loop, invoking its stop.
"""
import math
from rm75_preview_core import ControllerPacket


def _parse(data):
    try:
        return ControllerPacket.parse(data)
    except ValueError:
        return None


def _active(packet):
    return (packet is not None and packet.valid and packet.tracking_verified
            and packet.grip >= .7)


def _continuous(left, right):
    if not (_active(left) and _active(right)):
        return False
    if (left.session != right.session or right.sequence <= left.sequence or
            not 0 < right.sent_time - left.sent_time < .25 or
            math.dist(left.position, right.position) > .1):
        return False
    dot = abs(sum(a*b for a,b in zip(left.rotation, right.rotation)))
    return 2*math.acos(min(1.,dot)) <= math.radians(3)


class LatestInputSocket:
    def __init__(self, udp, *, max_drain=64):
        if type(max_drain) is not int or max_drain < 1:
            raise ValueError('max_drain must be positive')
        self._udp = udp
        self._max_drain = max_drain
        self._active_input = False
        self._timeout = None
        self._previous = None
        self.received = 0
        self.coalesced = 0

    def __getattr__(self, name):
        return getattr(self._udp, name)

    def bind(self, address):
        result = self._udp.bind(address)
        self._active_input = address == ('127.0.0.1', 5008)
        return result

    def settimeout(self, value):
        self._udp.settimeout(value)
        self._timeout = value

    def _deliver(self, item, packet):
        self._previous = (item[1], packet) if _active(packet) else None
        return item

    def recvfrom(self, size):
        item = self._udp.recvfrom(size)
        if not self._active_input:
            return item
        self.received += 1
        current = _parse(item[0])
        if not _active(current):
            return self._deliver(item, current)
        if self._previous is not None:
            peer, previous = self._previous
            if peer != item[1] or not _continuous(previous, current):
                return self._deliver(item, current)
        self._udp.setblocking(False)
        try:
            for _ in range(self._max_drain):
                try:
                    candidate = self._udp.recvfrom(size)
                except BlockingIOError:
                    return self._deliver(item, current)
                self.received += 1
                packet = _parse(candidate[0])
                self.coalesced += 1
                if item[1] != candidate[1] or not _continuous(current, packet):
                    # Return the barrier itself; no later packet can hide it.
                    return self._deliver(candidate, packet)
                item, current = candidate, packet
            raise RuntimeError('UDP input backlog exceeded; stopping instead of executing queued motion')
        finally:
            self._udp.settimeout(self._timeout)

    def close(self):
        if self.coalesced:
            print(f'UDP INPUT | received={self.received} coalesced={self.coalesced}', flush=True)
        return self._udp.close()


class SocketModuleProxy:
    """Replace one entry module's socket factory without altering Python socket."""
    def __init__(self, module):
        self._module = module

    def __getattr__(self, name):
        return getattr(self._module, name)

    def socket(self, *args, **kwargs):
        return LatestInputSocket(self._module.socket(*args, **kwargs))
