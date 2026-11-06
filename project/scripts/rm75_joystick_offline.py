"""Four-axis addon simulation; no robot SDK, sockets, or device input."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'Assets' / 'Scripts'))
from rm75_joystick_bridge import JoystickAdapter
from rm75_joystick_preview import joystick_engine_class
from rm75_joystick_settings import POSITION_RADIUS_M, TRANSLATION_SPEED_M_S


def simulate(core_path, joystick_mode=True):
    spec = importlib.util.spec_from_file_location('joystick_offline_core', core_path)
    core = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = core
    spec.loader.exec_module(core)
    # Analytic RPY <-> quaternion conversions, SDK-free.
    def euler(rpy):
        r,p,y = [v/2 for v in rpy]
        cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
        return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]
    def angles(q):
        w,x,y,z=q
        return [math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)), math.asin(max(-1,min(1,2*(w*y-z*x)))), math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]
    pose=[.1,.2,.3,.3,-.4,.2]
    for axis in ['left_y','left_x','right_y','right_x']:
        for sign in [-1,1]:
            a=JoystickAdapter()
            mapper=lambda ref,cur,rpy: core.map_relative_orientation(ref,cur,rpy,('z','-x','y'),euler,angles)
            engine=(joystick_engine_class(core.PreviewEngine) if joystick_mode else core.PreviewEngine)(core.PreviewConfig(radius=POSITION_RADIUS_M if joystick_mode else .03, speed=TRANSLATION_SPEED_M_S if joystick_mode else .01), lambda: pose.copy(), map_orientation=mapper)
            rows=[]
            virtual=[]
            for seq,grip in enumerate([0,0,1,1,1,1,0]):
                now=1+seq*.03
                sticks=dict(session='sticks',sequence=seq,sent_time=now,valid=True,left_x=0,left_y=0,right_x=0,right_y=0)
                sticks[axis]=sign if seq<4 else 0
                a.receive_stick(json.dumps(sticks).encode(),now)
                base=dict(session='base',sequence=seq,sent_time=now,valid=True,x=9,y=8,z=7,qx=0,qy=0,qz=0,qw=1,trigger=0,grip=grip,
                          device_valid=True,position_valid=True,rotation_valid=True,grip_valid=True,tracked_valid=True,tracked=True)
                output=a.process_base(json.dumps(base).encode(),now)
                virtual.append(json.loads(output))
                packet=core.ControllerPacket.parse(output)
                target=engine.accept(packet,now,('offline',0))
                rows.append(target)
                print(json.dumps(dict(axis=axis,sign=sign,time=now,grip=grip,status=engine.status,target=target)))
            moved=rows[3]
            assert rows[2]==pose
            for held in virtual[4:6]:
                assert all(abs(held[k]-virtual[3][k])<1e-10 for k in ('x','y','z','qx','qy','qz','qw')), 'Released stick must stop integration'
            if axis!='right_x':
                idx={'left_y':0,'left_x':1,'right_y':2}[axis]
                filtered=hasattr(engine, '_filtered_position') and not joystick_mode
                fraction=.25 if filtered else 1.
                assert abs(moved[idx]-pose[idx]-sign*TRANSLATION_SPEED_M_S*.03*fraction)<1e-10
                assert all(abs(x-y)<1e-10 for x,y in zip(moved[3:],pose[3:]))
                if filtered:
                    assert abs(rows[5][idx]-pose[idx]-sign*TRANSLATION_SPEED_M_S*.03*(1-.75**3))<1e-10
                else:
                    assert all(abs(x-y)<1e-10 for x,y in zip(rows[5],moved))
            else:
                # Verify base-X rotation by comparing matrices at a nonzero RPY.
                actual=core._quaternion_matrix_wxyz(tuple(euler(moved[3:])))
                ref=core._quaternion_matrix_wxyz(tuple(euler(pose[3:])))
                t=sign*.05*.03
                rx=[[1,0,0],[0,math.cos(t),-math.sin(t)],[0,math.sin(t),math.cos(t)]]
                expected=core._matrix_multiply(rx,ref)
                assert max(abs(actual[i][j]-expected[i][j]) for i in range(3) for j in range(3))<1e-10
            assert engine.status=='READY'
            engine.tick(2.)
            assert engine.status=='WAIT_RELEASE'

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--core',type=Path,default=ROOT/'Assets/Scripts/rm75_preview_core.py')
    parser.add_argument('--original-engine', action='store_true', help='Reproduce pose-follow filter tail for comparison')
    args=parser.parse_args()
    simulate(args.core, not args.original_engine)
