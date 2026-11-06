"""Independent requested quantities: 50cm total span, 5cm/s, preserved gates."""
import json
import math
import pytest
from test_rm75_joystick_bridge import base,stick,module as bridge
from rm75_preview_core import ControllerPacket,PreviewConfig
from rm75_joystick_preview import joystick_engine_class
from rm75_joystick_live import live_joystick_config
from rm75_preview_core import PreviewEngine
from rm75_joystick_settings import INPUT_RATE_HZ


def trajectory(axis,sign,seconds=9):
    adapter=bridge.JoystickAdapter()
    engine=joystick_engine_class(PreviewEngine)(live_joystick_config(),lambda:[0]*6)
    rows=[]
    for seq in range(int(seconds*50)+3):
        now=1+seq*.02
        adapter.receive_stick(stick(seq=seq,**{axis:sign}),now)
        payload=adapter.process_base(base(seq=seq,grip=0 if seq<2 else 1),now)
        target=engine.accept(ControllerPacket.parse(payload),now,('127.0.0.1',40000))
        rows.append((now,target,engine.status))
    return rows,engine


@pytest.mark.parametrize('axis,index',[('ly',0),('lx',1),('ry',2)])
@pytest.mark.parametrize('sign',[-1,1])
def test_25cm_radius_and_5cms_are_enforced_on_each_axis(axis,index,sign):
    rows,engine=trajectory(axis,sign)
    assert INPUT_RATE_HZ==50
    assert engine.config.radius==.25 and engine.config.speed==.05
    accepted=[(time,target) for time,target,status in rows if target is not None]
    assert math.isclose(sign*accepted[52][1][index],.05,abs_tol=.002)
    assert max(math.hypot(*target[:3]) for _,target in accepted)<=.25+1e-12
    assert max(abs(target[index]) for _,target in accepted)>.249
    assert any(status=='HOLD' for _,_,status in rows)
    for (t0,a),(t1,b) in zip(accepted,accepted[1:]):
        assert math.dist(a[:3],b[:3])<=.05*(t1-t0)+1e-12


def test_diagonal_composite_speed_is_5cms_not_per_axis():
    adapter=bridge.JoystickAdapter()
    adapter.receive_stick(stick(lx=1,ly=1,ry=1),1)
    adapter.process_base(base(),1.01)
    adapter.process_base(base(seq=1,grip=1),1.02)
    out=json.loads(adapter.process_base(base(seq=2,grip=1),1.04))
    assert math.isclose(math.hypot(out['x'],out['y'],out['z'])*.2/.02,.05,abs_tol=1e-12)


def test_other_safety_limits_unchanged():
    from rm75_live_control import LiveLimits
    config=live_joystick_config();limits=LiveLimits()
    assert config.timeout==.25 and config.jump_limit==.1
    assert config.require_verified_tracking
    assert config.orientation_radius==math.radians(10)
    assert config.orientation_jump_limit==math.radians(3)
    assert config.orientation_speed==.05
    assert limits.max_joint_step==3 and limits.max_joint_excursion==30
    assert limits.monitor_interval==.2 and limits.max_command_age==.25


def test_sender_rate_reports_measured_rate_not_nominal_or_latency():
    meter=bridge.SenderRate()
    assert meter.label()=='?'
    for seq in range(80):meter.observe('pico',seq,seq/50)
    assert float(meter.label())==50
    meter.observe('pico',0,100.)
    assert float(meter.label())==50  # Replay does not refresh the window.
    meter.observe('other-pico',0,1.)
    assert meter.label()=='?'
    for seq in range(1,31):meter.observe('other-pico',seq,1+seq/30)
    assert float(meter.label())==30  # Detect an old APK instead of claiming 50Hz.
