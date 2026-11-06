"""Exercise the new live wiring exclusively with a fake arm/socket/IK backend."""
import contextlib
import io
import math
import socket
from unittest.mock import patch
import pytest
from test_rm75_joystick_bridge import base, stick, module as bridge
from test_rm75_live_integration import ControllerArm, LiveSocket
from rm75_live_control import LiveMotionController, LiveControlFault, RecoverableLimit, LiveLimits
from rm75_joystick_motion import JoystickMotionController
from rm75_preview_core import _quaternion_matrix_wxyz
from rm75_log_replay import offline_orientation_mapper
import rm75_joystick_live as live
import rm75_target_preview as stable


def stream(axis='ly',sign=1,tail=None):
    adapter=bridge.JoystickAdapter()
    result=[]
    for seq,grip in enumerate([0,0,1,1,1,0]):
        now=1+seq*.033
        adapter.receive_stick(stick(seq=seq,**{axis:sign if seq==3 else 0}),now)
        result.append((now,adapter.process_base(base(seq=seq,grip=grip),now),('127.0.0.1',40000)))
    return result if tail is None else result[:5]+tail


def execute(events, confirmation=live.CONFIRMATION, tty=True, failure=None, log=None):
    clock=[0.];actions=[];arm=ControllerArm(actions)
    class FakeRemoteIK:
        uses_remote_ik=True
        def __init__(self):self.poses=[]
        def calculate_arm_angle(self,joints):return 179.5
        def solve(self,seed,pose,arm_angle):
            self.poses.append(list(pose))
            if failure is not None and len(self.poses)==2:raise failure
            return list(seed)
    ik=FakeRemoteIK()
    motion=JoystickMotionController(arm,lambda:arm.state[1],ik,lambda:clock[0])
    class LocalSocket(LiveSocket):
        def bind(self,address):
            assert address==('127.0.0.1',5008)
            super().bind(address)
    udp=LocalSocket(events,clock,actions)
    prompts=[];created=[]
    def typed(prompt):prompts.append(prompt);actions.append('confirmed');return confirmation
    def fake_main(argv):
        args=stable.build_parser().parse_args(argv)
        def factory(*a):created.append(True);return motion
        return stable.run(args,lambda:arm,lambda *a:udp,lambda:clock[0],
            input_func=typed,stdin_isatty=lambda:tty,live_factory=factory,
            orientation_factory=lambda axes:offline_orientation_mapper(axes))
    before=(stable.PreviewEngine,stable.PreviewConfig,stable.ControlLogWriter,stable.confirm_live)
    out=io.StringIO()
    with patch.object(stable,'main',side_effect=fake_main),contextlib.redirect_stdout(out):
        code=live.main(['--control-log',str(log)] if log else [])
    assert (stable.PreviewEngine,stable.PreviewConfig,stable.ControlLogWriter,stable.confirm_live)==before
    return dict(code=code,output=out.getvalue(),arm=arm,ik=ik,motion=motion,
                udp=udp,actions=actions,prompts=prompts,created=created)


@pytest.mark.parametrize('axis,index',[('ly',0),('lx',1),('ry',2)])
@pytest.mark.parametrize('sign',[-1,1])
def test_translation_both_signs_center_holds_and_grip_release_stops(axis,index,sign):
    result=execute(stream(axis,sign))
    assert result['code']==0
    poses=result['ik'].poses
    assert len(poses)==2
    assert math.isclose(poses[0][index]-result['arm'].state[1]['pose'][index],sign*.05*.033,abs_tol=1e-12)
    assert poses[0]==poses[1]  # Centered input has no position-filter tail.
    assert len(result['arm'].moves)==2
    assert 'slow-stop' in result['actions']
    assert result['actions'].index('confirmed') < result['actions'].index('udp-bind')
    assert result['actions'].index('slow-stop') < result['actions'].index('robot-delete')
    assert live.CONFIRMATION in result['prompts'][0]


@pytest.mark.parametrize('sign',[-1,1])
def test_base_x_rotation_with_nonzero_reference_pose(sign):
    result=execute(stream('rx',sign))
    poses=result['ik'].poses
    from rm75_log_replay import _euler_to_quaternion as euler_to_quaternion
    reference=_quaternion_matrix_wxyz(tuple(euler_to_quaternion(result['arm'].state[1]['pose'][3:])))
    actual=_quaternion_matrix_wxyz(tuple(euler_to_quaternion(poses[0][3:])))
    t=sign*.05*.033
    rx=[[1,0,0],[0,math.cos(t),-math.sin(t)],[0,math.sin(t),math.cos(t)]]
    expected=[[sum(rx[i][k]*reference[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    assert max(abs(actual[i][j]-expected[i][j]) for i in range(3) for j in range(3))<1e-10
    assert poses[0]==pytest.approx(poses[1],abs=1e-11)


@pytest.mark.parametrize('confirmation,tty',[('MOVE_RM75_6DOF',True),('',True),('MOVE_RM75_JOYSTICK',False)])
def test_no_motion_without_new_interactive_confirmation(confirmation,tty):
    result=execute(stream(),confirmation,tty)
    assert result['code']==1
    assert result['arm'].moves==[] and not result['created'] and not result['udp'].bound


@pytest.mark.parametrize('tail',[
    [(1.5,None)],
    [(1.165,b'{}',('127.0.0.1',40000))],
    [(1.165,base(seq=5,grip=1,valid=False),('127.0.0.1',40000))],
    [(1.165,base(seq=5,grip=1),('127.0.0.1',40001))],
])
def test_loss_malformed_tracking_and_source_change_stop_sent_motion(tail):
    result=execute(stream(tail=tail))
    assert result['code']==0
    assert len(result['arm'].moves)==2
    assert 'slow-stop' in result['actions']
    assert result['motion'].has_sent is False


def test_ik_fault_disarms_without_sending_failed_candidate():
    result=execute(stream(),failure=LiveControlFault('simulated kinematic fault'))
    assert result['code']==0
    assert len(result['arm'].moves)==1
    assert 'slow-stop' in result['actions']
    assert 'FAULT' in result['output']


def test_ik_hold_preserves_accepted_command_and_resumes():
    result=execute(stream(),failure=RecoverableLimit('simulated singularity/limit HOLD'))
    assert result['code']==0
    assert 'HOLD' in result['output']
    assert len(result['arm'].moves)==1


def test_live_log_is_independent_and_replays_without_robot(tmp_path):
    from rm75_joystick_replay import replay_log
    from rm75_log_replay import replay_log as original_replay
    path=tmp_path/'joystick-live.jsonl'
    result=execute(stream(),log=path)
    assert result['code']==0
    replayed=replay_log(path)
    assert replayed.frames[-1].status=='READY'
    with pytest.raises(ValueError):original_replay(path)


def test_user_selected_config_and_original_other_limits_unchanged():
    config=live.live_joystick_config(scale=.2,axes=('z','-x','y'),radius=.05,speed=.02,
        timeout=.25,jump_limit=.1)
    assert config.radius==.25 and config.speed==.05 and config.timeout==.25
    assert config.require_verified_tracking
    assert math.isclose(config.orientation_radius,math.radians(10))
    assert math.isclose(config.orientation_jump_limit,math.radians(3))
    assert config.orientation_speed==.05
    limits=LiveLimits()
    assert limits.max_joint_step==3 and limits.max_joint_excursion==30
    assert limits.max_command_age==.25 and limits.max_state_read_time==.25


def test_live_cli_cannot_override_ports_mapping_or_limits():
    with patch.object(stable,'main') as entry:
        for flags in [['--port','5005'],['--speed','1'],['--axes','x,y,z']]:
            with pytest.raises(SystemExit):live.main(flags)
        entry.assert_not_called()
