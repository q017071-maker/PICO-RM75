"""Joystick mode boundaries against the deployed stable core; no robot calls."""
import io
import json
import math
from unittest.mock import patch
import pytest
from test_rm75_joystick_bridge import module as bridge, base, stick, unpack
from rm75_preview_core import ControllerPacket, PreviewEngine, PreviewConfig
import rm75_joystick_preview as joystick


def ready():
    a = bridge.JoystickAdapter()
    a.receive_stick(stick(ly=1), 1)
    a.process_base(base(), 1.01)
    a.process_base(base(seq=1, grip=1), 1.02)
    return a


@pytest.mark.parametrize('flag', ['device_valid', 'position_valid', 'rotation_valid',
                                  'grip_valid', 'tracked_valid', 'tracked'])
def test_unverified_base_disarms_and_requires_new_release(flag):
    a = ready()
    data = unpack(base(seq=2, grip=1)); data.pop(flag)
    out = unpack(a.process_base(json.dumps(data).encode(), 1.05))
    assert not out['valid'] and flag not in out
    assert not unpack(a.process_base(base(seq=3, grip=1), 1.08))['valid']
    a.process_base(base(seq=4, grip=0), 1.10)
    assert unpack(a.process_base(base(seq=5, grip=1), 1.13))['valid']


def test_metadata_and_flags_are_original_not_bridge_clock():
    a = ready()
    source = unpack(base(seq=2, grip=1))
    out = unpack(a.process_base(json.dumps(source).encode(), 1.05))
    for key in ('session', 'sequence', 'sent_time', 'grip', 'trigger', 'device_valid',
                'position_valid', 'rotation_valid', 'grip_valid', 'tracked_valid', 'tracked'):
        assert out[key] == source[key]
    assert 'tracking_verified' not in out
    assert ControllerPacket.parse(json.dumps(out).encode()).tracking_verified


@pytest.mark.parametrize('flags', [['--live'], ['--live-6dof'],
    ['--live-orientation-axis', 'x'], ['--scale', '.4'], ['--axes', 'x,y,z'],
    ['--timeout', '1'], ['--speed', '.1']])
def test_unsafe_options_rejected_before_robot_or_socket(flags):
    with patch.object(joystick.preview, 'main') as run:
        with pytest.raises(ValueError): joystick.main(flags)
        run.assert_not_called()


def test_monkeypatch_restored_even_when_readonly_entry_raises():
    before = (joystick.preview.PreviewEngine, joystick.preview.PreviewConfig,
              joystick.preview.ControlLogWriter)
    with patch.object(joystick.preview, 'main', side_effect=RuntimeError('fake error')):
        with pytest.raises(RuntimeError, match='fake error'):
            joystick.main(['--bind', '127.0.0.1', '--port', '5008', '--orientation-preview'])
    assert (joystick.preview.PreviewEngine, joystick.preview.PreviewConfig,
            joystick.preview.ControlLogWriter) == before


def test_direct_localhost_unverified_packet_never_arms():
    cls = joystick.joystick_engine_class(PreviewEngine)
    engine = cls(PreviewConfig(), lambda: [0]*6)
    peer = ('127.0.0.1', 1234)
    for seq, grip in enumerate([0, 0, 1]):
        data = unpack(base(seq=seq, grip=grip)); data.pop('tracked_valid')
        assert engine.accept(ControllerPacket.parse(json.dumps(data).encode()), 1+seq*.03, peer) is None
    assert engine.status == 'WAIT_RELEASE'
    assert engine.config.orientation_radius == math.radians(10)
    assert engine.config.orientation_speed == .05


def test_independent_log_replay_and_original_log_isolation(tmp_path):
    import rm75_log_replay as stable_replay
    from rm75_joystick_replay import replay_log
    config = joystick.joystick_config(PreviewConfig(radius=.25,speed=.05))
    engine = joystick.joystick_engine_class(PreviewEngine)(config, lambda: [.1,.2,.3,0,0,0])
    adapter = bridge.JoystickAdapter()
    log = tmp_path/'joystick.jsonl'
    peer = ('127.0.0.1', 1234)
    with joystick.JoystickLogWriter(log, config) as writer:
        for seq, grip in enumerate([0,0,1,1,1,0]):
            now=1+seq*.03
            adapter.receive_stick(stick(seq=seq, ly=1 if seq<4 else 0), now)
            payload=adapter.process_base(base(seq=seq,grip=grip), now)
            target=engine.accept(ControllerPacket.parse(payload), now, peer)
            writer.record_frame(payload, peer, now, completed=now,
                reference_pose=[.1,.2,.3,0,0,0] if seq==2 else None,
                status=engine.status, reason=engine.reason, target=target)
    result = replay_log(log)
    assert len(result.frames) == 6
    assert result.frames[3].target == result.frames[4].target
    with pytest.raises(ValueError): stable_replay.replay_log(log)


def test_full_readonly_io_with_fake_robot_and_state_return(tmp_path, capsys):
    from test_rm75_target_preview import FakeArm, FakeStateBroadcaster, FakeSocket
    from rm75_log_replay import offline_orientation_mapper
    from rm75_joystick_replay import replay_log
    clock=[1.0]
    adapter=bridge.JoystickAdapter()
    events=[]
    for seq, grip in enumerate([0,0,1,1,1,0]):
        now=1+seq*.05
        adapter.receive_stick(stick(seq=seq,ly=1 if seq==3 else 0),now)
        events.append((now, adapter.process_base(base(seq=seq,grip=grip),now)))
    class LocalSocket(FakeSocket):
        def bind(self, address):
            assert address==('127.0.0.1',5008)
        def recvfrom(self, size):
            payload, _=super().recvfrom(size)
            return payload, ('127.0.0.1',40000)
    sock=LocalSocket(events, clock)
    arm=FakeArm()  # Provides read APIs only; any movement method would fail.
    state=FakeStateBroadcaster()
    log=tmp_path/'readonly.jsonl'
    def fake_entry(argv):
        args=joystick.preview.build_parser().parse_args(argv)
        return joystick.preview.run(args,lambda:arm,lambda *a:sock,lambda:clock[0],
            live_factory=lambda *a:pytest.fail('read-only must never create motion controller'),
            state_factory=lambda port:state,
            orientation_factory=lambda axes:offline_orientation_mapper(axes))
    with patch.object(joystick.preview,'main',side_effect=fake_entry):
        assert joystick.main(['--bind','127.0.0.1','--port','5008',
                              '--orientation-preview','--control-log',str(log)])==0
    assert arm.closed==1 and sock.closed and state.closed
    assert any(peer=='127.0.0.1' for _,peer in state.published)
    result=replay_log(log)
    assert result.frames[2].status=='ACTIVE'
    assert result.frames[-1].status=='READY'
    assert result.frames[3].target[0]>result.frames[2].target[0]
    assert result.frames[3].target==result.frames[4].target
    assert 'PREVIEW ONLY' in capsys.readouterr().out
