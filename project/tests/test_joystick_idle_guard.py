"""No commands for Grip alone; preserve convergence, limits and new log replay."""
import copy
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"Assets/Scripts"))
sys.path.insert(0,str(Path(__file__).resolve().parent))
import json
import math
from unittest.mock import patch
import pytest
from rm75_joystick_motion import JoystickMotionController
from rm75_live_control import LiveControlFault,MotionOutcome,RecoverableLimit
from rm75_preview_core import ControllerPacket,PreviewEngine
from rm75_joystick_preview import joystick_engine_class
from rm75_joystick_live import live_joystick_config,JoystickLiveLogWriter
from test_rm75_joystick_bridge import base,stick,module as bridge
from test_joystick_live import execute
from test_rm75_live_integration import ControllerArm

STATE=dict(joint=[0.]*7,pose=[.1,.2,.3,2.004,.039,-1.737])


def controller():
    events=[];clock=[1.];arm=ControllerArm(events)
    class IK:
        uses_remote_ik=True
        def __init__(self):self.solves=0
        def calculate_arm_angle(self,joints):return 0.
        def solve(self,seed,target,angle):self.solves+=1;return list(seed)
    ik=IK();reads=[]
    def read():reads.append(True);return copy.deepcopy(STATE)
    motion=JoystickMotionController(arm,read,ik,lambda:clock[0])
    motion.prepare(copy.deepcopy(STATE))
    return motion,arm,ik,clock,reads


def test_500_stationary_targets_send_no_joint_commands_or_remote_ik():
    motion,arm,ik,clock,reads=controller()
    for _ in range(500):
        clock[0]+=.02
        result=motion.send_target(STATE['pose'],clock[0])
        assert result.outcome==MotionOutcome.UNCHANGED
    assert arm.moves==[] and ik.solves==0 and reads==[]
    assert motion.has_sent is False and motion.waiting_for_intent
    assert motion.telemetry()['engagement_commands_sent']==0
    motion.stop('Grip released')
    assert 'slow-stop' not in arm.event_log  # No motion was sent to stop.


def test_orientation_roundtrip_noise_is_ignored_but_real_rotation_is_not():
    motion,arm,ik,clock,reads=controller()
    target=STATE['pose'].copy();target[3]+=2e-7
    motion.send_target(target,clock[0]);assert not arm.moves
    target[3]+=.001  # Intent smaller than old 0.3deg deadband must still work.
    motion.send_target(target,clock[0])
    assert len(arm.moves)==1 and ik.solves==1


def test_real_translation_starts_and_unchanged_target_keeps_converging():
    motion,arm,ik,clock,reads=controller()
    target=STATE['pose'].copy();target[0]+=.001
    for _ in range(5):
        clock[0]+=.02;motion.send_target(target,clock[0])
    assert len(arm.moves)==5 and ik.solves==5 and len(reads)==5
    assert motion.telemetry()['engagement_commands_sent']==5
    assert motion.measured_state==STATE
    assert motion.measured_at==clock[0]
    motion.stop('Grip released')
    motion.prepare(copy.deepcopy(STATE))
    motion.send_target(STATE['pose'],clock[0])
    assert len(arm.moves)==5 and motion.engagement_commands_sent==0


@pytest.mark.parametrize('now',[float('nan'),True,.7])
def test_idle_guard_does_not_skip_timestamp_validation(now):
    motion,arm,ik,clock,reads=controller()
    with pytest.raises(LiveControlFault):motion.send_target(STATE['pose'],now)
    assert not arm.moves


def test_full_loop_grip_cycles_without_stick_never_move():
    adapter=bridge.JoystickAdapter();events=[]
    for seq,grip in enumerate([0,0,1,1,1,0,1,1,1,0]):
        now=1+seq*.033;adapter.receive_stick(stick(seq=seq),now)
        events.append((now,adapter.process_base(base(seq=seq,grip=grip),now),('127.0.0.1',40000)))
    result=execute(events)
    assert result['code']==0 and result['arm'].moves==[]
    assert result['ik'].poses==[]


def make_engine(anti_windup=True):
    adapter=bridge.JoystickAdapter();engine=joystick_engine_class(PreviewEngine,anti_windup=anti_windup)(live_joystick_config(),lambda:[0.]*6)
    def step(seq,axis):
        now=1+seq*.02;adapter.receive_stick(stick(seq=seq,ly=axis),now)
        payload=adapter.process_base(base(seq=seq,grip=0 if seq<2 else 1),now)
        result=engine.accept(ControllerPacket.parse(payload),now,('127.0.0.1',40000))
        return now,result
    step(0,0);step(1,0);step(2,0)
    return engine,step


def test_joint_hold_drops_rejected_demand_without_changing_robot_reference():
    engine,step=make_engine();step(3,1)
    reference_pose=engine._reference_pose.copy()
    checkpoint=engine.command_state()
    # Simulate 2 seconds of rejected requests while operator keeps pushing.
    for seq in range(4,104):
        now,target=step(seq,1)
        engine.hold('IK joint excursion limit',now,restore=checkpoint)
        assert engine._reference_pose==reference_pose
        assert engine._target==list(checkpoint.target)
    now,target=step(104,-1)
    assert target[0]<checkpoint.target[0]  # Immediate retreat; no 2s windup to undo.
    assert math.isclose(checkpoint.target[0]-target[0],.001,abs_tol=1e-10)


def test_position_radius_hold_still_bounded_and_can_retreat():
    engine,step=make_engine()
    for seq in range(3,340):step(seq,1)
    assert engine.status=='HOLD'
    assert math.hypot(*engine._target[:3])<=.25
    assert engine._reference_pose==[0.]*6
    before=engine._target.copy()
    _,target=step(340,-1)
    assert engine.status=='ACTIVE' and target[0]<before[0]
    assert math.hypot(*target[:3])<=.25


def test_cumulative_joint_limit_is_not_reanchored_or_weakened():
    motion,arm,ik,clock,reads=controller()
    origin=list(motion._reference_solution)
    ik.solve=lambda seed,pose,angle:[30.1,0,0,0,0,0,0]
    # A current seed near boundary avoids the separate 3deg step guard.
    state=copy.deepcopy(STATE);state['joint'][0]=29
    motion._read_state=lambda:state
    target=STATE['pose'].copy();target[0]+=.001
    for _ in range(10):assert motion.send_target(target,clock[0]).outcome==MotionOutcome.HOLD
    assert motion._reference_solution==origin
    assert arm.moves==[] and motion.waiting_for_intent
    assert motion.last_limit_detail.startswith('IK joint excursion')


def test_new_headers_versioned_and_guard_does_not_change_old_reader_mode(tmp_path):
    from rm75_joystick_replay import replay_log
    path=tmp_path/'new.jsonl';config=live_joystick_config()
    with JoystickLiveLogWriter(path,config):pass
    header=json.loads(path.read_text().splitlines()[0])
    assert header['joystick_engine_version']==2
    # Header-only file is enough to validate config/engine selection.
    assert replay_log(path).config.radius==.25


def test_real_entry_factory_wires_idle_guard_and_measured_telemetry(tmp_path):
    import sys
    from types import SimpleNamespace
    from test_rm75_live_integration import LiveSocket
    from rm75_log_replay import offline_orientation_mapper
    from rm75_joystick_replay import replay_log
    import rm75_joystick_live as live
    import rm75_target_preview as stable
    adapter=bridge.JoystickAdapter();rows=[]
    for seq,grip in enumerate([0,0,1,1,1,1,0]):
        now=1+seq*.033
        adapter.receive_stick(stick(seq=seq,ly=1 if seq==4 else 0),now)
        rows.append((now,adapter.process_base(base(seq=seq,grip=grip),now),('127.0.0.1',40000)))
    events=[];clock=[0.];arm=ControllerArm(events);udp=LiveSocket(rows,clock,events)
    class IK:
        uses_remote_ik=True
        def calculate_arm_angle(self,joints):return 0.
        def solve(self,seed,target,angle):return list(seed)
    fake_sdk=SimpleNamespace(Algo=lambda *a:object(),rm_force_type_e=SimpleNamespace(RM_MODEL_RM_ISF_E=1),
        rm_robot_arm_model_e=SimpleNamespace(RM_MODEL_RM_75_E=1),rm_inverse_kinematics_params_t=object)
    real_run=stable.run
    def offline_run(args,arm_factory,socket_factory,provided_clock,**kwargs):
        return real_run(args,lambda:arm,lambda *a:udp,lambda:clock[0],
            input_func=lambda prompt:live.CONFIRMATION,stdin_isatty=lambda:True,
            live_factory=kwargs['live_factory'],orientation_factory=offline_orientation_mapper)
    path=tmp_path/'telemetry.jsonl'
    with patch.dict(sys.modules,{'Robotic_Arm.rm_robot_interface':fake_sdk}), \
         patch.object(stable,'run',side_effect=offline_run), \
         patch.object(stable,'SdkRm75Kinematics',return_value=IK()):
        assert live.main(['--control-log',str(path)])==0
    records=[json.loads(line) for line in path.read_text().splitlines()]
    active=[r for r in records[1:] if r.get('status')=='ACTIVE']
    assert active[0]['engagement_commands_sent']==0
    assert active[0]['waiting_for_joystick_input'] is True
    assert active[0]['motion_has_sent'] is False
    assert active[-1]['engagement_commands_sent']==2
    assert active[-1]['measured_state']==dict(joint=arm.state[1]['joint'],pose=arm.state[1]['pose'])
    assert active[-1]['measured_at']>=active[0]['measured_at']
    assert len(arm.moves)==2
    assert replay_log(path).frames[-1].status=='READY'


def test_explicit_input_guard_does_not_mistake_pose_conversion_error_for_intent():
    motion,arm,ik,clock,reads=controller()
    motion._intent_provider=lambda:False
    target=STATE['pose'].copy();target[3]+=.001
    motion.send_target(target,clock[0])
    assert not arm.moves and ik.solves==0
    motion._intent_provider=lambda:True
    motion.send_target(target,clock[0])
    assert len(arm.moves)==1


def test_anti_windup_logs_replay_with_identical_hold_and_retreat(tmp_path):
    from rm75_joystick_replay import replay_log
    adapter=bridge.JoystickAdapter()
    engine=joystick_engine_class(PreviewEngine)(live_joystick_config(),lambda:[0.]*6)
    peer=('127.0.0.1',40000);path=tmp_path/'limits.jsonl'
    with JoystickLiveLogWriter(path,engine.config) as writer:
        for seq in range(12):
            now=1+seq*.02;axis=1 if seq<9 else -1
            adapter.receive_stick(stick(seq=seq,ly=axis if seq>=3 else 0),now)
            payload=adapter.process_base(base(seq=seq,grip=0 if seq<2 else 1),now)
            checkpoint=engine.command_state() if engine.engaged else None
            target=engine.accept(ControllerPacket.parse(payload),now,peer);outcome=None
            if 4<=seq<=8:
                engine.hold('IK joint excursion limit',now,restore=checkpoint)
                target=None;outcome='HOLD'
            writer.record_frame(payload,peer,now,completed=now,
                reference_pose=[0.]*6 if seq==2 else None,target=target,
                status=engine.status,reason=engine.reason,motion_outcome=outcome)
    result=replay_log(path)
    assert result.frames[8].status=='HOLD'
    assert result.frames[9].status=='ACTIVE'
    assert result.frames[9].target[0]<result.frames[3].target[0]
