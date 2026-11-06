"""Deterministic simulated UDP routing, including status return, without sockets."""
import json
from types import SimpleNamespace
from unittest import TestCase, mock
from test_rm75_joystick_bridge import module as bridge, base, stick


class TransportTests(TestCase):
    def test_routing_and_allowlists(self):
        sent=[]
        class Sock:
            def __init__(self, frames=()): self.frames=iter(frames)
            def recvfrom(self,size): return next(self.frames)
            def sendto(self,data,address): sent.append((data,address))
            def close(self): pass
        pico=('10.0.0.2',40000)
        joystick=('10.0.0.2',40001)
        b=Sock([(base(),('untrusted',1)),(base(),pico),(base(seq=1,grip=1),pico),(base(seq=2,grip=1),pico)])
        s=Sock([(stick(ly=1),joystick)])
        state=Sock([(b'measured-state',('127.0.0.1',30000))])
        class Selector:
            def register(self,*args): pass
            def close(self): pass
            def select(self,timeout):
                source,sock=next(events)
                return [(SimpleNamespace(data=source,fileobj=sock),1)]
        def event_stream():
            yield 'base',b
            yield 'stick',s
            for _ in range(3): yield 'base',b
            yield 'state',state
            raise KeyboardInterrupt
        events=event_stream()
        args=SimpleNamespace(base_ip=pico[0],stick_ip=joystick[0],bind='0.0.0.0',base_port=5005,stick_port=5007,robot_port=5008,state_port=5006,state_relay=True,state_destination_ip=pico[0])
        with mock.patch.object(bridge,'_bind_udp',side_effect=[b,s,state]), mock.patch.object(bridge.socket,'socket',return_value=Sock()), mock.patch.object(bridge.selectors,'DefaultSelector',return_value=Selector()), mock.patch.object(bridge.time,'monotonic',side_effect=[1,1.01,1.02,1.03,1.06,1.07]), mock.patch('builtins.print'):
            with self.assertRaises(KeyboardInterrupt): bridge.serve(args)
        self.assertEqual(len(sent),4)
        self.assertEqual(sent[-1],(b'measured-state',(pico[0],5006)))
        frames=[json.loads(data) for data,address in sent[:-1]]
        self.assertEqual([p['sequence'] for p in frames],[0,1,2])
        self.assertGreater(frames[-1]['z'],0)
        self.assertTrue(all(addr==('127.0.0.1',5008) for _,addr in sent[:-1]))
