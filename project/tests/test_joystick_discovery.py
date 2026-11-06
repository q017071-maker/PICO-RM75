"""Automatic source selection never enables movement or fabricates tracking."""
import json
from types import SimpleNamespace
from unittest import mock
import pytest
from test_rm75_joystick_bridge import base, stick, module as bridge

PICO=('10.1.0.2',1000)
STICK=('10.1.0.2',1001)


def test_selects_fresh_pair_only_when_grip_released_and_pins_ip():
    d=bridge.PicoDiscovery()
    assert d.observe('stick',stick(),STICK,1) is None
    assert d.observe('base',base(grip=1),PICO,1.01) is None
    assert d.observe('base',base(seq=1),PICO,1.04)==PICO[0]
    assert d.selected_stick()==(1,stick(),STICK)
    assert d.observe('stick',stick(),('other-pico',999),1.05) is None
    assert d.selected_ip==PICO[0]


def test_base_alone_or_mixed_devices_never_selected():
    d=bridge.PicoDiscovery()
    assert d.observe('base',base(),PICO,1) is None
    d.observe('stick',stick(),('10.1.0.3',1001),1.01)
    assert d.observe('base',base(seq=1),PICO,1.04) is None
    assert d.selected_ip is None


@pytest.mark.parametrize('field',['valid','tracked','tracked_valid','device_valid',
                                  'position_valid','rotation_valid','grip_valid'])
def test_unverified_base_never_selected(field):
    d=bridge.PicoDiscovery()
    d.observe('stick',stick(),STICK,1)
    payload=json.loads(base());payload.pop(field)
    assert d.observe('base',json.dumps(payload).encode(),PICO,1.01) is None
    assert d.selected_ip is None


def test_stale_stick_and_invalid_companion_never_selected():
    for bad in [stick(valid=False),b'{}',b'bad json']:
        d=bridge.PicoDiscovery()
        d.observe('stick',stick(),STICK,1)
        d.observe('stick',bad,STICK,1.01)
        assert d.observe('base',base(),PICO,1.02) is None
    d=bridge.PicoDiscovery()
    d.observe('stick',stick(),STICK,1)
    assert d.observe('base',base(),PICO,1.21) is None


def test_candidate_memory_bounded():
    d=bridge.PicoDiscovery()
    for n in range(100): d.observe('stick',stick(),(f'10.1.0.{n}',1001),1)
    assert len(d._sticks)==8


def test_auto_routing_and_feedback_without_manual_ip():
    sent=[]
    class Sock:
        def __init__(self,frames=()):self.frames=iter(frames)
        def recvfrom(self,size):return next(self.frames)
        def sendto(self,data,peer):sent.append((data,peer))
        def close(self):pass
    b=Sock([(base(),PICO),(base(seq=1),PICO),(base(seq=2,grip=1),PICO),
            (base(seq=3,grip=1),PICO),(base(),('other-pico',1000))])
    s=Sock([(stick(ly=1),STICK)])
    state=Sock([(b'actual joints',('127.0.0.1',1010))])
    schedule=iter([('base',b),('stick',s),('base',b),('base',b),('base',b),('base',b),('state',state)])
    clock=[1.]
    class Selector:
        def register(self,*a):pass
        def close(self):pass
        def select(self,timeout):
            try:kind,sock=next(schedule)
            except StopIteration:raise KeyboardInterrupt
            clock[0]+=.01
            return [(SimpleNamespace(data=kind,fileobj=sock),1)]
    args=SimpleNamespace(base_ip=None,stick_ip=None,bind='0.0.0.0',base_port=5005,
                         stick_port=5007,robot_port=5008,state_port=5006,
                         state_relay=True,state_destination_ip=None)
    with mock.patch.object(bridge,'_bind_udp',side_effect=[b,s,state]), \
         mock.patch.object(bridge.socket,'socket',return_value=Sock()), \
         mock.patch.object(bridge.selectors,'DefaultSelector',return_value=Selector()), \
         mock.patch.object(bridge.time,'monotonic',side_effect=lambda:clock[0]), \
         mock.patch('builtins.print'):
        with pytest.raises(KeyboardInterrupt):bridge.serve(args)
    frames=[json.loads(data) for data,peer in sent if peer==('127.0.0.1',5008)]
    assert len(frames)==3
    assert all(p['session']=='base-1' and p['tracked_valid'] for p in frames)
    assert frames[-1]['z']>0
    assert sent[-1]==(b'actual joints',(PICO[0],5006))
