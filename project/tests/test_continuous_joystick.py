"""Continuous translation uses fake feedback/arm only, never a robot SDK handle."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'Assets/Scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
import copy
import json
import math
from types import SimpleNamespace
from unittest.mock import patch
import pytest
from rm75_continuous_core import ContinuousEngine,ContinuousMotionController,read_joint_bounds,LEAD_M
from rm75_preview_core import PreviewConfig,ControllerPacket
from rm75_live_control import LiveControlFault,MotionOutcome
from rm75_log_replay import offline_orientation_mapper,_euler_to_quaternion,_quaternion_to_euler
from test_rm75_joystick_bridge import base,stick,module as bridge


def engine_setup(axis='ly',sign=1,follow=True):
    adapter=bridge.JoystickAdapter();clock=[1.];measurement={'state':{'joint':[0.]*7,'pose':[0.]*6},'at':1.}
    engine=ContinuousEngine(PreviewConfig(radius=.005,speed=.05,require_verified_tracking=True,
        orientation_radius=math.radians(10),orientation_jump_limit=math.radians(3),orientation_speed=.05),
        lambda:list(measurement['state']['pose']),measured=lambda:measurement,clock=lambda:clock[0],
        map_orientation=offline_orientation_mapper(('z','-x','y')))
    def step(seq,grip=1,value=None,valid=True):
        clock[0]=1+seq*.02;measurement['at']=clock[0]
        adapter.receive_stick(stick(seq=seq,**{axis:sign if value is None else value}),clock[0])
        data=adapter.process_base(base(seq=seq,grip=grip,valid=valid),clock[0])
        result=engine.accept(ControllerPacket.parse(data),clock[0],('127.0.0.1',40000))
        if result is not None and follow:measurement['state']['pose']=list(result)
        return result
    step(0,0,0);step(1,0,0);step(2,1,0)
    return engine,step,measurement,clock


@pytest.mark.parametrize('axis,index',[('ly',0),('lx',1),('ry',2)])
@pytest.mark.parametrize('sign',[-1,1])
def test_hold_one_grip_moves_over_half_meter_without_25cm_reset(axis,index,sign):
    engine,step,state,clock=engine_setup(axis,sign)
    previous=[0.]*6
    for seq in range(3,603):
        target=step(seq)
        assert engine.status=='ACTIVE'
        assert math.dist(target[:3],previous[:3])<=.001+1e-10
        previous=target
    assert sign*target[index]>.59
    assert abs(target[index]-sign*.6)<1e-8
    assert math.dist(target[:3],state['state']['pose'][:3])<1e-12
    assert step(603,0,0) is None and engine.status=='READY'


def test_lagging_feedback_does_not_let_target_run_away():
    engine,step,state,clock=engine_setup(follow=False)
    for seq in range(3,503):
        target=step(seq)
        assert math.dist(target[:3],state['state']['pose'][:3])<=LEAD_M+1e-12
    assert math.isclose(target[0],.005,abs_tol=1e-12)
    target=step(503,value=-1)
    assert target[0]<.005  # Reverse is immediately usable, no accumulated demand.


def test_tracking_loss_stops_and_held_grip_cannot_rearm():
    engine,step,state,clock=engine_setup()
    step(3);assert step(4,valid=False) is None
    assert engine.status=='WAIT_RELEASE'
    assert step(5) is None
    step(6,0,0);assert step(7,value=0) is not None


def test_release_and_new_press_reanchor_without_jump():
    engine,step,state,clock=engine_setup()
    for seq in range(3,10):step(seq)
    actual=list(state['state']['pose']);step(10,0,0)
    assert step(11,value=0)==actual


def test_duplicate_does_not_rewind_virtual_input_or_refresh_reference():
    engine,step,state,clock=engine_setup();step(3)
    before=engine._input
    packet=ControllerPacket('base-1',3,1+3*.033,(999,999,999),(0,0,0,1),1,True,True)
    assert engine.accept(packet,clock[0],('127.0.0.1',40000)) is None
    assert engine._input==before
    assert step(4,value=0)==state['state']['pose']


def test_stale_measured_state_disarms_instead_of_advancing():
    engine,step,state,clock=engine_setup();step(3)
    engine._measured=lambda:{'state':state['state'],'at':clock[0]-.3}
    assert step(4) is None and engine.status=='WAIT_RELEASE'


class Plant:
    def __init__(self):self.state={'joint':[0.]*7,'pose':[0.]*6};self.moves=[];self.stops=0
    def rm_movej_follow(self,joints):
        self.moves.append(list(joints));self.state['joint']=list(joints);self.state['pose'][0]=joints[0]*.002
        return 0
    def rm_set_arm_slow_stop(self):self.stops+=1;return 0
    def rm_set_arm_stop(self):self.stops+=1;return 0


class Solver:
    uses_remote_ik=True
    def calculate_arm_angle(self,joints):return 0.
    def solve(self,seed,pose,angle):return [seed[0]+.5,*seed[1:]]


def test_joint_motion_passes_30deg_without_regrip_and_stops_at_absolute_margin():
    plant=Plant();clock=[1.]
    controller=ContinuousMotionController(plant,lambda:copy.deepcopy(plant.state),Solver(),lambda:clock[0],
        bounds=[[-95.]*7,[95.]*7],intent_provider=lambda:True)
    controller.prepare(copy.deepcopy(plant.state))
    for i in range(190):
        clock[0]+=.02
        target=plant.state['pose'].copy();target[0]+=.001
        result=controller.send_target(target,clock[0])
    assert plant.state['joint'][0]>90 and len(plant.moves)==189
    assert result.outcome==MotionOutcome.HOLD and plant.state['joint'][0]<95
    assert controller._limits.max_joint_step==3
    controller.stop('Grip released');assert plant.stops==1


def test_no_progress_is_detected_without_raising_tracking_tolerance():
    plant=Plant();clock=[1.]
    plant.rm_movej_follow=lambda joints:0  # Simulated stalled feedback.
    controller=ContinuousMotionController(plant,lambda:copy.deepcopy(plant.state),Solver(),lambda:clock[0],
        bounds=[[-95.]*7,[95.]*7],intent_provider=lambda:True)
    controller.prepare(copy.deepcopy(plant.state))
    with pytest.raises(LiveControlFault,match='no measured progress'):
        for _ in range(100):
            clock[0]+=.02;controller.send_target([.005,0,0,0,0,0],clock[0])
    assert controller._limits.max_position_error==.015


def test_controller_and_sdk_absolute_limits_are_intersected_without_writes():
    arm=SimpleNamespace(rm_get_joint_min_pos=lambda:(0,[-100]*7),rm_get_joint_max_pos=lambda:(0,[100]*7))
    algo=SimpleNamespace(rm_algo_get_joint_min_limit=lambda:[-90]*7,rm_algo_get_joint_max_limit=lambda:[90]*7)
    assert read_joint_bounds(arm,algo)==[[-85]*7,[85]*7]
    arm.rm_get_joint_min_pos=lambda:(-2,[])
    with pytest.raises(LiveControlFault):read_joint_bounds(arm,algo)


@pytest.mark.parametrize('confirmation,expected_moves',[('MOVE_RM75_CONTINUOUS',2),('MOVE_RM75_JOYSTICK',0)])
def test_full_entry_fake_sdk_confirmation_stop_and_new_log(tmp_path,confirmation,expected_moves):
    import rm75_continuous_live as entry
    import rm75_target_preview as stable
    from test_rm75_live_integration import ControllerArm,LiveSocket,FakeStateBroadcaster
    events=[];clock=[0.];arm=ControllerArm(events)
    arm.rm_get_joint_min_pos=lambda:(0,[-170.]*7)
    arm.rm_get_joint_max_pos=lambda:(0,[170.]*7)
    frame={'name':'base','pose':[0.]*6}
    arm.rm_get_current_work_frame=lambda:(0,frame)
    arm.rm_get_current_tool_frame=lambda:(0,frame)
    algo=SimpleNamespace(rm_algo_get_joint_min_limit=lambda:[-180.]*7,
        rm_algo_get_joint_max_limit=lambda:[180.]*7,rm_algo_set_workframe=lambda f:None,
        rm_algo_set_toolframe=lambda f:None,rm_algo_euler2quaternion=_euler_to_quaternion,
        rm_algo_quaternion2euler=_quaternion_to_euler)
    sdk=SimpleNamespace(algo=algo,params_type=object,frame_type=lambda **kw:kw)
    adapter=bridge.JoystickAdapter();rows=[]
    for seq,grip in enumerate([0,0,1,1,1,0]):
        now=1+seq*.02;adapter.receive_stick(stick(seq=seq,ly=1 if seq==3 else 0),now)
        rows.append((now,adapter.process_base(base(seq=seq,grip=grip),now),('127.0.0.1',40000)))
    udp=LiveSocket(rows,clock,events)
    path=tmp_path/'continuous.jsonl'
    args=stable.build_parser().parse_args(['--live','--live-6dof','--bind','127.0.0.1','--port','5008','--control-log',str(path)])
    solver=Solver();solver.solve=lambda seed,pose,angle:list(seed)
    with patch.object(entry,'SdkRm75Kinematics',return_value=solver):
        code=entry.run_continuous(args,arm_factory=lambda:arm,sdk_factory=lambda:sdk,
            socket_factory=lambda *a:udp,clock=lambda:clock[0],
            input_func=lambda prompt:confirmation,stdin_isatty=lambda:True,
            state_factory=lambda port:FakeStateBroadcaster(events))
    assert code==(0 if expected_moves else 1)
    if not expected_moves:
        assert arm.moves==[] and 'udp-bind' not in events
        assert arm.closed==1
        return
    assert len(arm.moves)==expected_moves and 'slow-stop' in events
    records=[json.loads(line) for line in path.read_text().splitlines()]
    assert records[0]['per_grip_joint_budget'] is None
    assert records[1]['record']=='setup'
    assert records[-1]['status']=='READY'
    from rm75_continuous_replay import replay_log
    assert replay_log(path)==6


def test_position_jump_not_hidden_even_when_angular_hold():
    engine,step,state,clock=engine_setup();step(3)
    packet=ControllerPacket('base-1',4,1+4*.033,(100,100,100),(0,0,0,1),1,True,True)
    assert engine.accept(packet,1.08,('127.0.0.1',40000)) is None
    assert engine.status=='WAIT_RELEASE'


def test_orientation_mapper_respects_work_rotation_for_base_axis():
    from rm75_continuous_live import orientation_mapper
    from rm75_preview_core import _quaternion_matrix_wxyz,_matrix_multiply,_transpose
    rotation=[[0.,-1,0],[1.,0,0],[0,0,1.]]
    algo=SimpleNamespace(rm_algo_euler2quaternion=_euler_to_quaternion,rm_algo_quaternion2euler=_quaternion_to_euler)
    target=orientation_mapper(algo,('z','-x','y'),rotation)((0,0,0,1),(0,0,math.sin(.01/2),math.cos(.01/2)),[0,0,0])
    actual=_quaternion_matrix_wxyz(tuple(_euler_to_quaternion(target)))
    rx=[[1.,0,0],[0,math.cos(.01),-math.sin(.01)],[0,math.sin(.01),math.cos(.01)]]
    expected=_matrix_multiply(_matrix_multiply(_transpose(rotation),rx),rotation)
    assert max(abs(a-b) for rowa,rowb in zip(actual,expected) for a,b in zip(rowa,rowb))<1e-12


def test_per_command_3deg_limit_is_not_removed_with_grip_budget():
    plant=Plant();clock=[1.];solver=Solver()
    solver.solve=lambda seed,pose,angle:[seed[0]+4,*seed[1:]]
    motion=ContinuousMotionController(plant,lambda:copy.deepcopy(plant.state),solver,lambda:clock[0],
        bounds=[[-95.]*7,[95.]*7],intent_provider=lambda:True)
    motion.prepare(copy.deepcopy(plant.state))
    result=motion.send_target([.001,0,0,0,0,0],clock[0])
    assert result.outcome==MotionOutcome.HOLD and plant.moves==[]
    assert '3' in result.reason


def test_absolute_hold_has_no_windup_and_reverse_resumes_without_regrip():
    engine,step,measurement,clock=engine_setup(follow=False)
    plant=Plant();measurement['state']=plant.state
    solver=Solver()
    solver.solve=lambda seed,pose,angle:[pose[0]/.002,*seed[1:]]
    motion=ContinuousMotionController(plant,lambda:copy.deepcopy(plant.state),solver,lambda:clock[0],
        bounds=[[-5.]*7,[5.]*7],intent_provider=lambda:engine.joystick_input_requested)
    motion.prepare(copy.deepcopy(plant.state))
    for seq in range(3,43):
        checkpoint=engine.command_state()
        target=step(seq)
        result=motion.send_target(target,clock[0])
        if result.outcome==MotionOutcome.HOLD:
            engine.hold(result.reason,clock[0],restore=checkpoint)
    assert engine.status=='HOLD' and 4<=plant.state['joint'][0]<5
    accepted_x=plant.state['pose'][0]
    target=step(43,value=-1)
    result=motion.send_target(target,clock[0])
    assert result.outcome==MotionOutcome.SENT
    assert engine.status=='ACTIVE' and plant.state['pose'][0]<accepted_x


def test_base_translation_is_transformed_to_rotated_work_frame():
    engine,step,measurement,clock=engine_setup(follow=False)
    engine._base_to_work=[[0.,1,0],[-1.,0,0],[0,0,1.]]
    assert step(3)[:3]==pytest.approx([0.,-.001,0.])


def test_unexpected_pose_jump_still_disarms_before_lead_clamping():
    engine,step,state,clock=engine_setup();step(3)
    state['state']['pose'][0]+=.03
    assert step(4) is None and engine.status=='WAIT_RELEASE'
    assert '0.015m' in engine.reason


def test_full_entry_stalled_feedback_stops_without_loosening_error_guard(tmp_path):
    import rm75_continuous_live as entry
    import rm75_target_preview as stable
    from test_rm75_live_integration import ControllerArm,LiveSocket,FakeStateBroadcaster
    events=[];clock=[0.];arm=ControllerArm(events)
    arm.rm_get_joint_min_pos=lambda:(0,[-170.]*7);arm.rm_get_joint_max_pos=lambda:(0,[170.]*7)
    frame={'name':'base','pose':[0.]*6}
    arm.rm_get_current_work_frame=lambda:(0,frame);arm.rm_get_current_tool_frame=lambda:(0,frame)
    algo=SimpleNamespace(rm_algo_get_joint_min_limit=lambda:[-180.]*7,rm_algo_get_joint_max_limit=lambda:[180.]*7,
        rm_algo_set_workframe=lambda f:None,rm_algo_set_toolframe=lambda f:None,
        rm_algo_euler2quaternion=_euler_to_quaternion,rm_algo_quaternion2euler=_quaternion_to_euler)
    sdk=SimpleNamespace(algo=algo,params_type=object,frame_type=lambda **kw:kw)
    adapter=bridge.JoystickAdapter();rows=[]
    for seq in range(75):
        now=1+seq*.02;grip=1 if 2<=seq<74 else 0
        adapter.receive_stick(stick(seq=seq,ly=1 if seq>=3 else 0),now)
        rows.append((now,adapter.process_base(base(seq=seq,grip=grip),now),('127.0.0.1',40000)))
    udp=LiveSocket(rows,clock,events);path=tmp_path/'stalled.jsonl'
    args=stable.build_parser().parse_args(['--live','--live-6dof','--bind','127.0.0.1','--port','5008','--control-log',str(path)])
    solver=Solver();solver.solve=lambda seed,pose,angle:list(seed)
    with patch.object(entry,'SdkRm75Kinematics',return_value=solver):
        assert entry.run_continuous(args,arm_factory=lambda:arm,sdk_factory=lambda:sdk,socket_factory=lambda *a:udp,
            clock=lambda:clock[0],input_func=lambda prompt:entry.CONFIRMATION,stdin_isatty=lambda:True,
            state_factory=lambda port:FakeStateBroadcaster(events))==0
    frames=[json.loads(line) for line in path.read_text().splitlines() if '"record":"frame"' in line]
    assert any(r.get('motion_outcome')=='DISARM' and 'no measured progress' in r['reason'] for r in frames)
    assert events.count('slow-stop')==1 and events.index('slow-stop')<events.index('robot-delete')
    from rm75_continuous_replay import replay_log
    assert replay_log(path)==75
