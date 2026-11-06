"""Queue latency handling must preserve stop, tracking and source boundaries."""
import collections
import json
import math
import socket
from types import SimpleNamespace
from unittest.mock import patch
import pytest
from test_rm75_joystick_bridge import base
from rm75_udp_latest import LatestInputSocket, SocketModuleProxy

PEER=('127.0.0.1',40000)
EMPTY=object()


def frame(seq,grip=1,**changes):
    data=json.loads(base(seq=seq,grip=grip));data.update(changes)
    return json.dumps(data).encode(),PEER


class RawSocket:
    def __init__(self,rows=(),clock=None):
        self.rows=collections.deque(rows);self.blocking=True;self.timeout=None;self.closed=False;self.clock=clock
    def bind(self,address):self.address=address
    def settimeout(self,value):self.timeout=value;self.blocking=value!=0
    def setblocking(self,value):self.blocking=value
    def recvfrom(self,size):
        if not self.rows or self.rows[0] is EMPTY:
            if self.rows:self.rows.popleft()
            if not self.blocking:raise BlockingIOError
            raise KeyboardInterrupt
        item=self.rows.popleft()
        if self.clock is not None:
            self.clock[0]=json.loads(item[0])['sent_time']
        return item
    def close(self):self.closed=True


def reader(rows,**kwargs):
    raw=RawSocket(rows);wrapped=LatestInputSocket(raw,**kwargs)
    wrapped.bind(('127.0.0.1',5008));wrapped.settimeout(.05)
    return wrapped,raw


def test_motion_prefix_is_replaced_with_latest_sample():
    udp,raw=reader([frame(i) for i in range(6)])
    assert json.loads(udp.recvfrom(4096)[0])['sequence']==5
    assert udp.coalesced==5 and udp.received==6
    assert raw.timeout==.05


@pytest.mark.parametrize('barrier',[
    frame(2,grip=0),frame(2,grip=.5),frame(2,valid=False),
    frame(2,tracked=False),(b'{}',PEER),frame(2,session='new-session'),
    (frame(2)[0],('127.0.0.1',40001)),frame(0),frame(2,sent_time=1.5),
    frame(2,z=3.2),frame(2,qz=math.sin(.08/2),qw=math.cos(.08/2)),
])
def test_release_fault_source_replay_and_jump_not_hidden_by_later_motion(barrier):
    udp,raw=reader([frame(0),frame(1),barrier,frame(3)])
    assert udp.recvfrom(4096)==barrier
    assert len(raw.rows)==1  # Do not read past the boundary.
    assert raw.timeout==.05


def test_first_queued_jump_relative_to_last_delivered_not_hidden():
    udp,raw=reader([frame(0),EMPTY,frame(1,z=3.2),frame(2)])
    assert udp.recvfrom(4096)==frame(0)
    assert udp.recvfrom(4096)==frame(1,z=3.2)
    assert len(raw.rows)==1


def test_queue_overflow_fails_closed_and_restores_timeout():
    udp,raw=reader([frame(i) for i in range(10)],max_drain=3)
    with pytest.raises(RuntimeError,match='backlog exceeded'):udp.recvfrom(4096)
    assert raw.timeout==.05


def test_other_sockets_are_not_coalesced_and_socket_module_not_mutated():
    raw=RawSocket([frame(0),frame(1)]);module=SimpleNamespace(socket=lambda *a:raw)
    proxy=SocketModuleProxy(module);udp=proxy.socket()
    udp.bind(('127.0.0.1',5006))
    assert udp.recvfrom(4096)==frame(0)
    assert len(raw.rows)==1 and proxy is not module


def test_burst_release_stops_fake_live_arm_before_rearming():
    from rm75_live_control import LiveMotionController
    from rm75_log_replay import offline_orientation_mapper
    from test_rm75_live_integration import ControllerArm,ControllerKinematics
    import rm75_joystick_live as live
    import rm75_target_preview as stable
    events=[];clock=[0.];arm=ControllerArm(events)
    rows=[frame(0,0),frame(1,0),frame(2),EMPTY,frame(3),frame(4),frame(5,0),frame(6),EMPTY,frame(7),EMPTY,frame(8,0)]
    raw=RawSocket(rows,clock);udp=LatestInputSocket(raw)
    kin=ControllerKinematics(clock);kin.uses_remote_ik=True
    motion=LiveMotionController(arm,lambda:arm.state[1],kin,lambda:clock[0])
    def run(argv):
        args=stable.build_parser().parse_args(argv)
        return stable.run(args,lambda:arm,lambda *a:udp,lambda:clock[0],
            input_func=lambda prompt:live.CONFIRMATION,stdin_isatty=lambda:True,
            live_factory=lambda *a:motion,orientation_factory=offline_orientation_mapper)
    with patch.object(stable,'main',side_effect=run):assert live.main([])==0
    assert events.count('slow-stop')==2
    assert events.index('slow-stop')<events.index('robot-delete')
    assert len(arm.moves)==3  # Initial, reanchored, next motion; queued old moves dropped.
    assert raw.closed and udp.coalesced==2
