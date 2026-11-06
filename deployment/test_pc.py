"""Deployment boundaries only: no NetworkManager writes, ADB calls or robot access."""
import importlib.util
from pathlib import Path
import subprocess

import pytest

spec = importlib.util.spec_from_file_location('rm75_pc_deploy', Path(__file__).with_name('pc.py'))
pc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pc)


def network_rows(wifi='192.168.50.10'):
    return [{'ifname': 'wifi0', 'addr_info': [
        {'scope': 'global', 'family': 'inet', 'local': wifi, 'prefixlen': 24}]}]


def fake_network(monkeypatch, rows, devices='eth0:ethernet\nwifi0:wifi\n', existing=''):
    calls = []
    monkeypatch.setattr(pc, 'addresses', lambda: rows)
    monkeypatch.setattr(pc, 'is_wifi_interface', lambda name: name == 'wifi0')
    monkeypatch.setattr(pc, 'output', lambda argv: devices if 'device' in argv else existing)
    monkeypatch.setattr(pc, 'call', lambda argv, **kw: calls.append(list(argv)))
    return calls


def test_wifi_detection_excludes_ethernet(monkeypatch):
    monkeypatch.setattr(pc, 'is_wifi_interface', lambda name: name == 'wifi0')
    rows = network_rows() + [{'ifname': 'eth0', 'addr_info': [
        {'scope': 'global', 'family': 'inet', 'local': '192.168.1.100', 'prefixlen': 24}]}]
    assert pc.wifi_ip(rows) == '192.168.50.10'


def test_overlapping_wifi_network_is_rejected_before_changes(monkeypatch):
    calls = fake_network(monkeypatch, network_rows('192.168.1.5'))
    with pytest.raises(RuntimeError, match='重叠'):
        pc.configure_ethernet()
    assert calls == []


def test_multiple_ethernet_devices_require_selection(monkeypatch):
    calls = fake_network(monkeypatch, network_rows(), 'eth0:ethernet\neth1:ethernet\n')
    with pytest.raises(RuntimeError, match='不唯一'):
        pc.configure_ethernet()
    assert calls == []


def test_ethernet_profile_preserves_wifi_default_route(monkeypatch):
    calls = fake_network(monkeypatch, network_rows())
    pc.configure_ethernet()
    assert calls[0][:4] == ['nmcli', 'connection', 'add', 'type']
    assert calls[0][calls[0].index('ipv4.never-default') + 1] == 'yes'
    assert calls[0][calls[0].index('ipv6.never-default') + 1] == 'yes'
    assert calls[0][calls[0].index('ipv4.addresses') + 1] == '192.168.1.100/24'
    assert 'ipv4.gateway' not in calls[0]
    assert calls[-1] == ['nmcli', 'connection', 'up', 'RM75-latest-eth0']
    assert all('delete' not in command for command in calls)


def test_existing_profile_is_updated_in_place(monkeypatch):
    calls = fake_network(monkeypatch, network_rows(), existing='RM75-latest-eth0\n')
    pc.configure_ethernet()
    assert calls[0][2] == 'modify'
    assert calls[0][calls[0].index('ipv4.gateway') + 1] == ''


def test_robot_address_cannot_be_assigned_to_pc(monkeypatch):
    calls = fake_network(monkeypatch, network_rows())
    with pytest.raises(ValueError, match='相同'):
        pc.configure_ethernet(pc_cidr='192.168.1.18/24')
    assert calls == []


@pytest.mark.parametrize('value', ['0.0.0.0', '127.0.0.1', '224.0.0.1', '255.255.255.255', 'example.com'])
def test_endpoint_validation_rejects_non_lan_hosts(value):
    with pytest.raises(ValueError):
        pc.valid_host(value)


def test_no_argument_launcher_only_prints_help(monkeypatch, capsys):
    monkeypatch.setattr(pc, 'call', lambda *a, **kw: pytest.fail('default must not run anything'))
    monkeypatch.setattr(pc, 'control', lambda *a, **kw: pytest.fail('default must not connect'))
    assert pc.main([]) == 0
    assert 'live-6dof' in capsys.readouterr().out


def test_adb_multiple_devices_rejected(monkeypatch):
    monkeypatch.setattr(pc, 'output', lambda argv: 'List of devices attached\none\tdevice\ntwo\tdevice\n')
    with pytest.raises(RuntimeError, match='唯一'):
        pc.adb_command()


def test_runtime_does_not_fallback_to_system_python(monkeypatch, tmp_path):
    monkeypatch.setattr(pc, 'PYTHON', tmp_path / 'missing')
    with pytest.raises(RuntimeError, match='虚拟环境'):
        pc.runtime_python()


def test_adb_configuration_writes_then_reads_endpoint_before_launch(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(pc, 'adb_command', lambda serial: ['adb', '-s', 'test-device'])

    def run(argv, **kw):
        calls.append(list(argv))
        if 'push' in argv:
            path = Path(argv[argv.index('push') + 1])
            assert path.read_text() == '192.168.50.10:5005\n'

    def read(argv):
        if 'packages' in argv:
            return 'package:' + pc.APP_ID
        if 'dumpsys' in argv:
            return f'versionCode={pc.APP_VERSION_CODE} minSdk=29'
        return '192.168.50.10:5005\n'

    monkeypatch.setattr(pc, 'call', run)
    monkeypatch.setattr(pc, 'output', read)
    pc.configure_pico('192.168.50.10')
    assert 'force-stop' in calls[0]
    assert 'push' in calls[2]
    assert 'monkey' in calls[3]


def test_failed_endpoint_readback_prevents_app_launch(monkeypatch):
    calls = []
    monkeypatch.setattr(pc, 'adb_command', lambda serial: ['adb'])
    monkeypatch.setattr(pc, 'call', lambda argv, **kw: calls.append(argv))
    def read(argv):
        if 'packages' in argv:
            return pc.APP_ID
        if 'dumpsys' in argv:
            return f'versionCode={pc.APP_VERSION_CODE} minSdk=29'
        return 'bad'
    monkeypatch.setattr(pc, 'output', read)
    with pytest.raises(RuntimeError, match='不一致'):
        pc.configure_pico('192.168.50.10')
    assert not any('monkey' in argv for argv in calls)


def test_old_apk_is_rejected_before_writing_config(monkeypatch):
    calls = []
    monkeypatch.setattr(pc, 'adb_command', lambda serial: ['adb'])
    monkeypatch.setattr(pc, 'call', lambda argv, **kw: calls.append(argv))
    monkeypatch.setattr(pc, 'output', lambda argv:
                        pc.APP_ID if 'packages' in argv else 'versionCode=1 minSdk=29')
    with pytest.raises(RuntimeError, match='旧 APK'):
        pc.configure_pico('192.168.50.10')
    assert calls == []


def test_project_interpreter_ignores_ros_pythonpath(monkeypatch, tmp_path):
    calls = []
    fake_python = tmp_path / 'python'
    fake_python.touch()
    monkeypatch.setattr(pc, 'PYTHON', fake_python)
    monkeypatch.setattr(pc, 'output', lambda argv: calls.append(argv) or '{"sdk":"1.1.6"}')
    pc.runtime_python()
    assert calls[0][1:3] == ['-E', '-s']
    assert pc.project_python_args('-m', 'pip', 'check') == [
        pc.PYTHON, '-E', '-s', '-m', 'pip', 'check']


def test_python_process_excludes_ros_pythonpath(tmp_path):
    import os
    import sys
    result = subprocess.run([sys.executable, '-E', '-s', '-c',
                             'import sys; print(repr(sys.path))'],
                            env=dict(os.environ, PYTHONPATH=str(tmp_path)),
                            capture_output=True, text=True, check=True)
    assert str(tmp_path) not in result.stdout


def test_unity_open_uses_current_project_and_native_configurator(monkeypatch, tmp_path):
    monkeypatch.setattr(pc, 'PROJECT', tmp_path / 'latest-project')
    monkeypatch.setattr(pc, 'LOGS', tmp_path / 'logs')
    monkeypatch.setattr(pc, 'UNITY_DATA', tmp_path / 'Editor/Data')
    pc.UNITY_DATA.parent.mkdir(parents=True)
    (pc.UNITY_DATA.parent / 'Unity').touch()
    calls = []

    class Process:
        def wait(self, timeout):
            raise subprocess.TimeoutExpired('Unity', timeout)

    def launch(argv, **kwargs):
        calls.append(argv)
        return Process()

    monkeypatch.setattr(pc.subprocess, 'Popen', launch)
    assert pc.open_unity() == 0
    args = calls[0]
    assert args[args.index('-projectPath') + 1] == str(pc.PROJECT)
    assert args[args.index('-executeMethod') + 1] == 'Rm75ProjectSetup.ConfigureAndOpen'
    assert '-quit' not in args and '-batchmode' not in args


def test_unity_early_failure_is_reported_without_claiming_open(monkeypatch, tmp_path):
    monkeypatch.setattr(pc, 'PROJECT', tmp_path / 'latest-project')
    monkeypatch.setattr(pc, 'LOGS', tmp_path / 'logs')
    monkeypatch.setattr(pc, 'UNITY_DATA', tmp_path / 'Editor/Data')
    pc.UNITY_DATA.parent.mkdir(parents=True)
    (pc.UNITY_DATA.parent / 'Unity').touch()

    class Process:
        def wait(self, timeout):
            return 199

    monkeypatch.setattr(pc.subprocess, 'Popen', lambda *args, **kwargs: Process())
    with pytest.raises(RuntimeError, match='199'):
        pc.open_unity()


def test_control_is_blocked_during_source_update_before_any_sdk_or_robot_use(monkeypatch, tmp_path):
    monkeypatch.setattr(pc, 'ROOT', tmp_path)
    (tmp_path / 'deployment').mkdir()
    (tmp_path / 'deployment/update-in-progress.json').write_text('{}')
    monkeypatch.setattr(pc, 'runtime_python', lambda: pytest.fail('must not load SDK during update'))
    with pytest.raises(RuntimeError, match='更新与验证'):
        pc.control('live-6dof', '192.168.1.18')


def test_old_apk_not_installed_without_matching_native_build_report(monkeypatch, tmp_path):
    monkeypatch.setattr(pc, 'LOGS', tmp_path)
    monkeypatch.setattr(pc, 'APK', tmp_path/'old.apk')
    pc.APK.write_bytes(b'old build')
    monkeypatch.setattr(pc, 'adb_command', lambda serial: ['adb'])
    monkeypatch.setattr(pc, 'call', lambda *a, **kw: pytest.fail('must not install stale APK'))
    with pytest.raises(RuntimeError, match='新版 APK'):
        pc.configure_pico('192.168.50.10', install=True)


def test_matching_build_report_checks_apk_and_sources(monkeypatch, tmp_path):
    import hashlib
    import json
    monkeypatch.setattr(pc, 'LOGS', tmp_path)
    monkeypatch.setattr(pc, 'APK', tmp_path/'new.apk')
    pc.APK.write_bytes(b'new build')
    monkeypatch.setattr(pc, 'apk_sources_hash', lambda: 'current-sources')
    report = dict(project=str(pc.PROJECT), bundleVersionCode=pc.APP_VERSION_CODE,
                  joystickIncluded=True, apkSha256=hashlib.sha256(pc.APK.read_bytes()).hexdigest(),
                  sourcesSha256='current-sources')
    path=tmp_path/'apk-build.json'; path.write_text(json.dumps(report))
    pc.require_current_apk()
    report['sourcesSha256']='stale'; path.write_text(json.dumps(report))
    with pytest.raises(RuntimeError, match='不一致'): pc.require_current_apk()


def test_joystick_offline_uses_no_sdk_or_control(monkeypatch):
    calls=[]
    monkeypatch.setattr(pc,'call',lambda argv,**kw: calls.append(argv))
    monkeypatch.setattr(pc,'runtime_python',lambda: pytest.fail('must not import SDK'))
    monkeypatch.setattr(pc,'control',lambda *a: pytest.fail('must not connect robot'))
    assert pc.main(['joystick-offline']) == 0
    assert str(calls[0][-1]).endswith('scripts/rm75_joystick_offline.py')


def test_both_modes_share_lock_and_joystick_only_binds_local5008(monkeypatch, tmp_path):
    monkeypatch.setattr(pc,'ROOT',tmp_path)
    monkeypatch.setattr(pc,'LOGS',tmp_path)
    monkeypatch.setattr(pc,'runtime_python',lambda: pc.PYTHON)
    locks=[]; binds=[]; execs=[]
    monkeypatch.setattr(pc.fcntl,'flock',lambda fd,mode: locks.append(mode))
    class FakeSocket:
        def __enter__(self): return self
        def __exit__(self,*a): pass
        def bind(self,address): binds.append(address)
    monkeypatch.setattr(pc.socket,'socket',lambda *a: FakeSocket())
    monkeypatch.setattr(pc.os,'execv',lambda path,args: execs.append(args))
    pc.control('joystick-preview','192.168.1.18')
    pc.control('orientation-preview','192.168.1.18')
    assert len(locks)==2 and all(x & pc.fcntl.LOCK_EX for x in locks)
    assert binds==[('127.0.0.1',5008),('0.0.0.0',5005)]
    assert str(execs[0][4]).endswith('rm75_joystick_preview.py')
    assert '--live' not in execs[0] and '--orientation-preview' in execs[0]


def test_held_mode_lock_blocks_other_mode_before_network(monkeypatch, tmp_path):
    monkeypatch.setattr(pc,'ROOT',tmp_path)
    monkeypatch.setattr(pc,'LOGS',tmp_path)
    monkeypatch.setattr(pc,'runtime_python',lambda: pc.PYTHON)
    def held(*args): raise BlockingIOError('already held')
    monkeypatch.setattr(pc.fcntl,'flock',held)
    monkeypatch.setattr(pc.socket,'socket',lambda *a: pytest.fail('must not reach network'))
    for mode in ['joystick-preview','orientation-preview','live-6dof']:
        with pytest.raises(RuntimeError,match='正在运行'): pc.control(mode,'192.168.1.18')


def test_stale_unity_lock_does_not_block_editor_launch(monkeypatch, tmp_path):
    monkeypatch.setattr(pc, 'PROJECT', tmp_path / 'project')
    monkeypatch.setattr(pc, 'LOGS', tmp_path / 'logs')
    monkeypatch.setattr(pc, 'UNITY_DATA', tmp_path / 'Editor/Data')
    pc.UNITY_DATA.parent.mkdir(parents=True)
    (pc.UNITY_DATA.parent / 'Unity').touch()
    lock = pc.PROJECT / 'Temp/UnityLockfile'
    lock.parent.mkdir(parents=True)
    lock.write_text('interrupted session')
    calls = []
    class Process:
        def wait(self, timeout):
            raise subprocess.TimeoutExpired('Unity', timeout)
    monkeypatch.setattr(pc.subprocess, 'Popen', lambda argv, **kw: calls.append(argv) or Process())
    assert pc.open_unity() == 0
    assert len(calls) == 1
    assert lock.read_text() == 'interrupted session'


def build_paths(monkeypatch, tmp_path):
    monkeypatch.setattr(pc, 'PROJECT', tmp_path / 'project')
    monkeypatch.setattr(pc, 'LOGS', tmp_path / 'logs')
    monkeypatch.setattr(pc, 'APK', pc.PROJECT / 'Build/current.apk')
    pc.APK.parent.mkdir(parents=True)
    pc.APK.write_bytes(b'previous APK')
    lock = pc.PROJECT / 'Temp/UnityLockfile'
    lock.parent.mkdir(parents=True)
    lock.write_text('interrupted session')
    return lock


def test_stale_unity_lock_build_delegates_to_unity_and_keeps_existing_files(monkeypatch, tmp_path):
    from types import SimpleNamespace
    lock = build_paths(monkeypatch, tmp_path)
    previous_pending = pc.APK.parent / 'PICO_RM75_Teleop.building.apk'
    previous_pending.write_bytes(b'previous partial output')
    calls = []
    def build(argv, **kw):
        calls.append(argv)
        pending = Path(kw['env']['RM75_BUILD_PATH'])
        assert pending.parent != pc.APK.parent
        pending.write_bytes(b'new APK')
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(pc.subprocess, 'run', build)
    def verify():
        assert pc.APK.read_bytes() == b'new APK'
    monkeypatch.setattr(pc, 'require_current_apk', verify)
    assert pc.build_apk() == 0
    assert calls[0][calls[0].index('-projectPath') + 1] == str(pc.PROJECT)
    assert lock.read_text() == 'interrupted session'
    assert previous_pending.read_bytes() == b'previous partial output'


def test_native_build_failure_keeps_previous_apk_and_does_not_claim_editor_open(monkeypatch, tmp_path):
    from types import SimpleNamespace
    lock = build_paths(monkeypatch, tmp_path)
    calls = []
    monkeypatch.setattr(pc.subprocess, 'run', lambda argv, **kw:
                        calls.append(argv) or SimpleNamespace(returncode=199))
    with pytest.raises(RuntimeError, match='199') as failure:
        pc.build_apk()
    assert '已在 Unity 中打开' not in str(failure.value)
    assert len(calls) == 1
    assert pc.APK.read_bytes() == b'previous APK'
    assert lock.exists()


def test_concurrent_launcher_build_is_rejected_by_live_flock(monkeypatch, tmp_path):
    build_paths(monkeypatch, tmp_path)
    pc.LOGS.mkdir()
    monkeypatch.setattr(pc.subprocess, 'run', lambda *a, **kw: pytest.fail('second build must not launch'))
    with (pc.LOGS / 'unity-build.lock').open('a') as held:
        pc.fcntl.flock(held, pc.fcntl.LOCK_EX | pc.fcntl.LOCK_NB)
        with pytest.raises(RuntimeError, match='build-apk 构建入口正在运行'):
            pc.build_apk()


def test_joystick_bridge_cli_without_ip_uses_discovery_and_no_sdk(monkeypatch,tmp_path):
    monkeypatch.setattr(pc,'ROOT',tmp_path)
    monkeypatch.setattr(pc,'LOGS',tmp_path/'logs')
    monkeypatch.setattr(pc,'runtime_python',lambda:pytest.fail('no SDK needed'))
    calls=[]
    def execute(exe,args):
        calls.append(args)
        raise SystemExit(0)
    monkeypatch.setattr(pc.os,'execv',execute)
    with pytest.raises(SystemExit) as result:
        pc.main(['joystick-bridge'])
    assert result.value.code==0
    assert len(calls)==1
    assert str(calls[0][4]).endswith('rm75_joystick_bridge.py')
    assert '--base-ip' not in calls[0]


def test_joystick_live_launcher_uses_same_lock_and_local_entry(monkeypatch,tmp_path):
    monkeypatch.setattr(pc,'ROOT',tmp_path)
    monkeypatch.setattr(pc,'LOGS',tmp_path/'logs')
    monkeypatch.setattr(pc,'runtime_python',lambda:pc.PYTHON)
    binds=[];calls=[]
    class Socket:
        def __enter__(self):return self
        def __exit__(self,*a):pass
        def bind(self,address):binds.append(address)
    monkeypatch.setattr(pc.socket,'socket',lambda *a:Socket())
    monkeypatch.setattr(pc.os,'execv',lambda path,args:calls.append(args))
    pc.control('joystick-live','192.168.1.18')
    assert binds==[('127.0.0.1',5008)]
    assert str(calls[0][4]).endswith('rm75_joystick_live.py')
    assert '--control-log' in calls[0]
    assert not any('MOVE_RM75' in str(arg) for arg in calls[0])


def test_continuous_entry_uses_shared_motion_lock_and_local_input(monkeypatch,tmp_path):
    monkeypatch.setattr(pc,'ROOT',tmp_path);monkeypatch.setattr(pc,'LOGS',tmp_path/'logs')
    monkeypatch.setattr(pc,'runtime_python',lambda:pc.PYTHON)
    binds=[];calls=[]
    class Socket:
        def __enter__(self):return self
        def __exit__(self,*a):pass
        def bind(self,address):binds.append(address)
    monkeypatch.setattr(pc.socket,'socket',lambda *a:Socket())
    monkeypatch.setattr(pc.os,'execv',lambda exe,args:calls.append(args))
    pc.control('joystick-continuous','192.168.1.18')
    assert binds==[('127.0.0.1',5008)]
    assert str(calls[0][4]).endswith('rm75_continuous_live.py')
    assert '--control-log' in calls[0]
