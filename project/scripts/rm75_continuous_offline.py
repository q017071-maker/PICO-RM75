"""Synthetic continuous target trace; no SDK, sockets, robot or collision model."""
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'Assets/Scripts'))
from rm75_continuous_core import ContinuousEngine
from rm75_preview_core import ControllerPacket,PreviewConfig
from rm75_joystick_bridge import JoystickAdapter
from rm75_log_replay import offline_orientation_mapper


def main():
    clock=[1.]
    measured={'state':{'joint':[0.]*7,'pose':[0.]*6},'at':1.}
    engine=ContinuousEngine(PreviewConfig(radius=.005,speed=.05,require_verified_tracking=True,
        orientation_radius=.17453292519943295,orientation_jump_limit=.05235987755982989,orientation_speed=.05),
        lambda:list(measured['state']['pose']),measured=lambda:measured,clock=lambda:clock[0],
        map_orientation=offline_orientation_mapper(('z','-x','y')))
    adapter=JoystickAdapter()
    print(json.dumps(dict(type='continuous_target_simulation',synthetic=True,
        target_math_only=True,robot_connected=False,physical_reach_not_validated=True)))
    for seq in range(605):
        now=clock[0]=1+seq*.02;measured['at']=now
        grip=1 if 2<=seq<604 else 0
        stick=dict(session='sim-stick',sequence=seq,sent_time=now,valid=True,
                   left_x=0,left_y=1 if 3<=seq<604 else 0,right_x=0,right_y=0)
        raw=dict(session='sim-base',sequence=seq,sent_time=now,valid=True,
            x=0,y=0,z=0,qx=0,qy=0,qz=0,qw=1,trigger=0,grip=grip,
            device_valid=True,position_valid=True,rotation_valid=True,
            grip_valid=True,tracked_valid=True,tracked=True)
        adapter.receive_stick(json.dumps(stick).encode(),now)
        packet=ControllerPacket.parse(adapter.process_base(json.dumps(raw).encode(),now))
        target=engine.accept(packet,now,('127.0.0.1',5008))
        if target is not None:
            # Explicit synthetic first-order response, not robot dynamics.
            measured['state']['pose']=[a+.5*(b-a) for a,b in zip(measured['state']['pose'],target)]
        print(json.dumps(dict(time=now,status=engine.status,target=target,
                              simulated_pose=measured['state']['pose'])))
    assert measured['state']['pose'][0]>.59 and engine.status=='READY'


if __name__=='__main__':main()
