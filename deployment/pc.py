#!/usr/bin/env python3
"""Local launcher for the latest project; only explicit control commands use a robot."""
import argparse
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / 'project'
SCRIPTS = PROJECT / 'Assets/Scripts'
LOGS = ROOT / 'deployment/logs'
PYTHON = PROJECT / '.venv/bin/python'
UNITY_DATA = Path(os.environ.get('UNITY_EDITOR_DATA',
    str(Path.home() / 'Unity/Hub/Editor/2022.3.62f3/Editor/Data')))
APP_ID = 'com.DefaultCompany.PICO_RM75_Teleop'
APK = PROJECT / 'Build/PICO_RM75_Teleop.apk'
APP_VERSION_CODE = 20261107


def apk_sources_hash():
    paths = [p.relative_to(PROJECT).as_posix() for p in (PROJECT / 'Assets').rglob('*')
             if p.is_file() and p.suffix in ('.cs', '.unity', '.prefab')]
    paths += ['Packages/manifest.json', 'Packages/packages-lock.json',
              'ProjectSettings/ProjectSettings.asset']
    digest = hashlib.sha256()
    for name in sorted(paths):
        digest.update((name + '\0').encode())
        digest.update((PROJECT / name).read_bytes())
    return digest.hexdigest()


def require_current_apk():
    path = LOGS / 'apk-build.json'
    if not APK.is_file() or not path.is_file():
        raise RuntimeError('尚无双摇杆新版 APK 构建记录；先运行 build-apk 或 Unity 菜单 RM75 > Build Android APK。')
    report = json.loads(path.read_text())
    if (report.get('project') != str(PROJECT) or
            report.get('bundleVersionCode') != APP_VERSION_CODE or
            report.get('joystickIncluded') is not True or
            report.get('apkSha256') != hashlib.sha256(APK.read_bytes()).hexdigest() or
            report.get('sourcesSha256') != apk_sources_hash()):
        raise RuntimeError('APK 与当前含摇杆组件的 Unity 源码不一致；请重新构建。')


def call(argv, **kwargs):
    return subprocess.run([str(a) for a in argv], check=True, **kwargs)


def output(argv):
    return call(argv, capture_output=True, text=True, timeout=15).stdout


def project_python_args(*args):
    # Ignore ROS PYTHONPATH / PYTHONHOME and user site packages, retaining the
    # script directory so runtime modules can import their sibling modules.
    return [PYTHON, '-E', '-s', *args]


def addresses():
    return json.loads(output(['ip', '-j', '-4', 'address', 'show']))


def is_wifi_interface(name):
    return (Path('/sys/class/net') / name / 'wireless').is_dir()


def wifi_ip(rows):
    candidates = []
    for row in rows:
        if is_wifi_interface(row['ifname']):
            candidates.extend(a['local'] for a in row.get('addr_info', [])
                              if a.get('scope') == 'global' and a.get('family') == 'inet')
    if len(candidates) != 1:
        raise RuntimeError('无法唯一确定 Wi-Fi IPv4；先连接 Wi-Fi，或用 --host 明确指定电脑地址。')
    return candidates[0]


def valid_host(value):
    address = ipaddress.IPv4Address(value)
    if address.is_unspecified or address.is_loopback or address.is_multicast or address.is_reserved:
        raise ValueError('需要电脑的局域网 IPv4 地址')
    return str(address)


def runtime_python():
    if not PYTHON.exists():
        raise RuntimeError('最新项目没有虚拟环境；请运行 bash 准备电脑.sh。')
    data = json.loads(output(project_python_args('-c',
        "import importlib.metadata as m,json; print(json.dumps({'sdk':m.version('robotic-arm')}))")))
    if data['sdk'] != '1.1.6':
        raise RuntimeError('需要 robotic-arm==1.1.6；请运行 bash 准备电脑.sh。')
    return PYTHON


def sdk_api_check():
    code = """
from Robotic_Arm.rm_robot_interface import RoboticArm, Algo
import json
required = {
    'RoboticArm': (RoboticArm, ('rm_create_robot_arm', 'rm_get_robot_info',
        'rm_get_current_arm_state', 'rm_get_arm_run_mode', 'rm_delete_robot_arm',
        'rm_get_joint_min_pos', 'rm_get_joint_max_pos', 'rm_get_current_work_frame', 'rm_get_current_tool_frame',
        'rm_algo_inverse_kinematics', 'rm_movej_follow', 'rm_set_arm_slow_stop', 'rm_set_arm_stop')),
    'Algo': (Algo, ('rm_algo_calculate_arm_angle_from_config_rm75',
        'rm_algo_universal_singularity_analyse', 'rm_algo_get_joint_min_limit',
        'rm_algo_get_joint_max_limit', 'rm_algo_euler2quaternion', 'rm_algo_quaternion2euler',
        'rm_algo_ik_remote_init', 'rm_algo_ik_remote', 'rm_algo_pos2matrix', 'rm_algo_set_dq_weight',
        'rm_algo_set_workframe', 'rm_algo_set_toolframe')),
}
missing = [label+'.'+name for label,(cls,names) in required.items()
           for name in names if not callable(getattr(cls,name,None))]
from Robotic_Arm.rm_robot_interface import rm_force_type_e, rm_Mat_t
if not hasattr(rm_force_type_e, 'RM_MODEL_RM_ISF_E'):
    missing.append('rm_force_type_e.RM_MODEL_RM_ISF_E')
print('SDK_API_CHECK='+json.dumps(missing))
"""
    runtime_python()
    text = output(project_python_args('-c', code))
    report = next((line.removeprefix('SDK_API_CHECK=') for line in text.splitlines()
                  if line.startswith('SDK_API_CHECK=')), None)
    if report is None:
        raise RuntimeError('SDK 接口检查没有返回结果。')
    missing = json.loads(report)
    if missing:
        raise RuntimeError('SDK 缺少当前代码需要的接口: ' + ', '.join(missing))


def doctor(offline=False):
    failed = []

    def check(label, ok, detail=''):
        print(f'{"OK" if ok else "MISSING"}: {label} {detail}', flush=True)
        if not ok:
            failed.append(label)

    print('项目:', PROJECT)
    print('未连接 Pico 或机械臂；只检查 SDK 类接口，不实例化设备对象。')
    try:
        runtime_python()
        check('RealMan SDK 1.1.6 / 项目虚拟环境', True)
    except (RuntimeError, subprocess.SubprocessError, ValueError) as exc:
        check('RealMan SDK 1.1.6 / 项目虚拟环境', False,
              '运行 bash 准备电脑.sh 补齐依赖')
    else:
        try:
            sdk_api_check()
            check('RealMan 原生库导入和所需 API', True)
        except (RuntimeError, subprocess.SubprocessError, ValueError) as exc:
            check('RealMan 原生库导入和所需 API', False, str(exc))
    check('Unity 2022.3.62f3', (UNITY_DATA.parent / 'Unity').is_file())
    android = UNITY_DATA / 'PlaybackEngines/AndroidPlayer'
    for rel in ('SDK/platform-tools/adb', 'NDK/source.properties', 'OpenJDK/bin/java'):
        check('Android ' + rel, (android / rel).is_file())
    manifest = json.loads((PROJECT / 'Packages/manifest.json').read_text())
    package = manifest['dependencies']['com.unity.xr.picoxr'].removeprefix('file:')
    package_path = (PROJECT / 'Packages' / package).resolve()
    check('PICO SDK 3.4.0', (package_path / 'package.json').is_file(), str(package_path))
    report_path = LOGS / 'unity-project-config.json'
    report = json.loads(report_path.read_text()) if report_path.is_file() else {}
    check('Unity 内场景/模型配置已校验',
          report.get('project') == str(PROJECT) and report.get('modelValidated') is True,
          str(report_path))
    check('Unity 内双摇杆场景配置已执行',
          report.get('project') == str(PROJECT) and
          report.get('joystickConfigured') is True and report.get('joystickPort') == 5007,
          '构建菜单会自动配置；源码/API 编译通过不能代替原生场景配置。')
    check('Unity 两路输入已配置为 50 Hz', report.get('inputRateHz') == 50,
          '实际收包频率查看桥接器 base_hz / stick_hz。')
    try:
        require_current_apk()
        check('双摇杆新版 APK 已构建', True, str(APK))
    except RuntimeError as exc:
        check('双摇杆新版 APK 已构建', False, str(exc))
    deployed_manifest = ROOT / 'deployment/deployed-source-manifest.json'
    if deployed_manifest.is_file():
        deployed = json.loads(deployed_manifest.read_text())
        baseline = deployed['files']
        label = '控制源码与已部署版本清单一致（' + deployed['commit'][:12] + '）'
    else:
        baseline = json.loads((ROOT / 'deployment/backups/control-source-sha256.json').read_text())
        label = '控制源码与初次部署清单一致'
    check(label, all(
        hashlib.sha256((SCRIPTS / name).read_bytes()).hexdigest() == digest
        for name, digest in baseline.items()))
    if not offline:
        try:
            print('Wi-Fi IPv4:', wifi_ip(addresses()))
            print('机器人路由:', output(['ip', 'route', 'get', '192.168.1.18']).strip())
        except (RuntimeError, subprocess.SubprocessError, ValueError):
            check('网络查询', False, '在普通 Ubuntu 终端运行；当前会话可能禁止访问网卡。')
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
                udp.bind(('0.0.0.0', 5005))
            check('UDP 5005 可用', True)
        except OSError as exc:
            check('UDP 5005 可用', False, str(exc))
    return int(bool(failed))


def configure_ethernet(interface=None, robot_ip='192.168.1.18', pc_cidr='192.168.1.100/24'):
    rows = addresses()
    robot = ipaddress.IPv4Address(robot_ip)
    pc = ipaddress.IPv4Interface(pc_cidr)
    if robot not in pc.network or pc.ip == robot:
        raise ValueError('电脑有线地址必须与机器人同网段，且不能与机器人相同。')
    for row in rows:
        if is_wifi_interface(row['ifname']):
            for addr in row.get('addr_info', []):
                if addr.get('scope') == 'global':
                    wifi = ipaddress.IPv4Interface(f"{addr['local']}/{addr['prefixlen']}")
                    if wifi.network.overlaps(pc.network):
                        raise RuntimeError('Wi-Fi 与机械臂有线网段重叠。先把 Wi-Fi 换到其他网段，再配置有线。')
    devices = output(['nmcli', '-t', '-f', 'DEVICE,TYPE', 'device', 'status']).splitlines()
    ethernet = [line.rsplit(':', 1)[0] for line in devices if line.endswith(':ethernet')]
    if interface is None:
        if len(ethernet) != 1:
            raise RuntimeError('有线网卡不唯一；用 --interface 明确指定。候选：' + ', '.join(ethernet))
        interface = ethernet[0]
    if interface not in ethernet:
        raise ValueError('指定接口不是 NetworkManager 管理的有线网卡。')
    name = 'RM75-latest-' + interface
    names = output(['nmcli', '-g', 'NAME', 'connection', 'show']).splitlines()
    if name not in names:
        call(['nmcli', 'connection', 'add', 'type', 'ethernet', 'ifname', interface,
              'con-name', name, 'connection.autoconnect', 'yes',
              'connection.autoconnect-priority', '100', 'ipv4.method', 'manual',
              'ipv4.addresses', str(pc), 'ipv4.never-default', 'yes',
              'ipv6.method', 'link-local', 'ipv6.never-default', 'yes'])
    else:
        call(['nmcli', 'connection', 'modify', name, 'ipv4.method', 'manual',
              'ipv4.addresses', str(pc), 'ipv4.never-default', 'yes', 'ipv4.gateway', '',
              'connection.autoconnect', 'yes', 'connection.autoconnect-priority', '100',
              'ipv6.method', 'link-local', 'ipv6.never-default', 'yes'])
    call(['nmcli', 'connection', 'up', name])
    print(f'已准备 {name}: {pc}，机器人 {robot}:8080；有线不接管默认路由。')
    print('插线后若尚未启用，可运行: nmcli connection up ' + name)
    print('原连接配置未删除；恢复时先停用此配置，再启用原配置。')


def adb_command(serial=None):
    adb = shutil.which('adb') or str(UNITY_DATA / 'PlaybackEngines/AndroidPlayer/SDK/platform-tools/adb')
    rows = output([adb, 'devices']).splitlines()[1:]
    devices = [line.split()[0] for line in rows if len(line.split()) >= 2 and line.split()[1] == 'device']
    if serial is None:
        if len(devices) != 1:
            raise RuntimeError('需要唯一已授权的 ADB 设备；用 USB 连接 Pico，并在头显允许 USB 调试。')
        serial = devices[0]
    if serial not in devices:
        raise RuntimeError('指定的 ADB 设备未连接或未授权。')
    return [adb, '-s', serial]


def configure_pico(host=None, serial=None, install=False):
    host = valid_host(host or wifi_ip(addresses()))
    adb = adb_command(serial)
    if install:
        require_current_apk()
        call(adb + ['install', '-r', str(APK)])
    if APP_ID not in output(adb + ['shell', 'pm', 'list', 'packages', APP_ID]):
        raise RuntimeError('未安装项目 APK；先运行 install-apk。')
    info = output(adb + ['shell', 'dumpsys', 'package', APP_ID])
    version = re.search(r'\bversionCode=(\d+)\b', info)
    if version is None or int(version[1]) != APP_VERSION_CODE:
        raise RuntimeError('头显仍是旧 APK，不支持地址配置文件；先运行 install-apk 安装本次构建。')
    remote_dir = '/sdcard/Android/data/' + APP_ID + '/files'
    call(adb + ['shell', 'am', 'force-stop', APP_ID])
    call(adb + ['shell', 'mkdir', '-p', remote_dir])
    with tempfile.TemporaryDirectory(prefix='rm75-endpoint-') as tmp:
        local = Path(tmp) / 'ubuntu_udp.txt'
        local.write_text(f'{host}:5005\n')
        call(adb + ['push', str(local), remote_dir + '/ubuntu_udp.txt'])
    actual = output(adb + ['shell', 'cat', remote_dir + '/ubuntu_udp.txt']).strip()
    if actual != f'{host}:5005':
        raise RuntimeError('Pico 配置文件回读与目标不一致，请检查 ADB/存储权限。')
    call(adb + ['shell', 'monkey', '-p', APP_ID, '-c', 'android.intent.category.LAUNCHER', '1'])
    print(f'Pico 目标已设为 {host}:5005，已重新打开 PICO_RM75_Teleop。')
    print('以后 Wi-Fi IP 变化只需重跑 configure-pico，无需重建 APK。')


def probe(seconds):
    sys.path.insert(0, str(SCRIPTS))
    from rm75_preview_core import ControllerPacket
    counts = dict(received=0, valid=0, verified=0, malformed=0)
    sources = set()
    grip_release = grip_press = False
    last_report = 0.0
    started = time.monotonic()
    print(f'只监听 UDP 5005，持续 {seconds:g} 秒；不加载机械臂 SDK。请松开再按下 Grip，并缓慢移动/转动右手柄。', flush=True)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.bind(('0.0.0.0', 5005))
        udp.settimeout(.2)
        while time.monotonic() - started < seconds:
            try:
                data, peer = udp.recvfrom(4096)
            except socket.timeout:
                continue
            counts['received'] += 1
            sources.add(peer)
            try:
                p = ControllerPacket.parse(data)
            except ValueError:
                counts['malformed'] += 1
                continue
            counts['valid'] += int(p.valid)
            counts['verified'] += int(p.tracking_verified)
            grip_release |= p.valid and p.grip <= .3
            grip_press |= p.valid and p.grip >= .7
            if time.monotonic() - last_report >= 1:
                print(f'{peer[0]} valid={p.valid} tracking_verified={p.tracking_verified} '
                      f'xyz={p.position} quaternion={p.rotation} grip={p.grip:.2f}', flush=True)
                last_report = time.monotonic()
    print(json.dumps(dict(counts, grip_release=grip_release, grip_press=grip_press,
                          source_count=len(sources)), ensure_ascii=False))
    return 0 if counts['valid'] and counts['verified'] and len(sources) == 1 else 1


def tests():
    py = PYTHON if PYTHON.exists() else Path(sys.executable)
    # The system interpreter has pytest for offline preparation before pip is available.
    if subprocess.run([str(py), '-E', '-s', '-c', 'import pytest'], capture_output=True).returncode:
        py = Path(sys.executable)
    call([py, '-E', '-s', '-B', '-m', 'pytest', '-q', '-p', 'no:cacheprovider', 'tests'], cwd=PROJECT)
    call([py, '-E', '-s', '-B', '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
          str(ROOT / 'deployment/test_pc.py')], cwd=ROOT)
    for name in ('run_pico_sender_checks.sh', 'run_rm75_digital_twin_checks.sh',
                 'run_joystick_sender_checks.sh'):
        call(['bash', PROJECT / 'tests' / name])
    return 0


def build_apk():
    LOGS.mkdir(parents=True, exist_ok=True)
    # Lock existence is not ownership: a crash can leave UnityLockfile behind.
    # Only Unity itself decides whether its project lock is held. Our own flock
    # merely prevents concurrent invocations of this launcher from sharing logs.
    with (LOGS / 'unity-build.lock').open('a') as guard:
        try:
            fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('已有 build-apk 构建入口正在运行，请等待其完成。') from exc
        return _build_apk()


def _build_apk():
    APK.parent.mkdir(parents=True, exist_ok=True)
    unity = UNITY_DATA.parent / 'Unity'
    log = LOGS / 'unity-android-build.log'
    if (PROJECT / 'Temp/UnityLockfile').exists():
        print('检测到 UnityLockfile；文件可能残留，将由 Unity 检查实际工程占用。', flush=True)
    # An independent output path avoids deleting an interrupted build's files.
    with tempfile.TemporaryDirectory(prefix='rm75-apk-', dir=APK.parent) as scratch:
        pending = Path(scratch) / APK.name
        result = subprocess.run([str(unity), '-batchmode', '-nographics', '-quit',
            '-projectPath', str(PROJECT), '-buildTarget', 'Android',
            '-executeMethod', 'Rm75CommandLineBuild.BuildAndroid', '-logFile', str(log)],
            env=dict(os.environ, RM75_BUILD_PATH=str(pending)),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if result.returncode or not pending.is_file():
            raise RuntimeError(f'Unity APK 构建未完成（退出码 {result.returncode}），请查看 {log}')
        pending.replace(APK)
    require_current_apk()
    print('最新 APK:', APK)
    print('SHA256:', hashlib.sha256(APK.read_bytes()).hexdigest())
    return 0


def open_unity():
    LOGS.mkdir(parents=True, exist_ok=True)
    unity = UNITY_DATA.parent / 'Unity'
    if not unity.is_file():
        raise RuntimeError('未找到 Unity 2022.3.62f3 编辑器。')
    if (PROJECT / 'Temp/UnityLockfile').exists():
        print('检测到 UnityLockfile；文件可能残留，将由 Unity 检查实际工程占用。', flush=True)
    log = LOGS / 'unity-open-latest.log'
    with (LOGS / 'unity-open-console.log').open('w') as console:
        process = subprocess.Popen([str(unity), '-projectPath', str(PROJECT),
            '-buildTarget', 'Android', '-executeMethod', 'Rm75ProjectSetup.ConfigureAndOpen',
            '-logFile', str(log)], stdout=console, stderr=subprocess.STDOUT,
            start_new_session=True)
    try:
        code = process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        print('已发出打开最新 Unity 工程的请求。项目:', PROJECT)
        print('导入后自动配置 Android/PICO 和 SampleScene，不进入 Play 模式。')
        print('配置成功后生成:', LOGS / 'unity-project-config.json')
        return 0
    raise RuntimeError(f'Unity 打开请求退出（代码 {code}）。在普通 Ubuntu 终端运行 '
                       f'bash 打开Unity.sh；日志: {log}')


def control(mode, robot_ip):
    if (ROOT / 'deployment/update-in-progress.json').exists():
        raise RuntimeError('源码更新与验证尚未完成，暂不启动预览或实控。请查看更新日志。')
    robot_ip = valid_host(robot_ip)
    py = runtime_python()
    LOGS.mkdir(parents=True, exist_ok=True)
    lock = open(LOGS / 'input.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise RuntimeError('已有接收/预览/控制入口正在运行，请先在原终端退出。') from exc
    # Fail before opening the robot if a legacy receiver owns the input port.
    joystick = mode in ('joystick-preview', 'joystick-live', 'joystick-continuous')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.bind(('127.0.0.1', 5008) if joystick else ('0.0.0.0', 5005))
    os.set_inheritable(lock.fileno(), True)
    extra = {
        'preview': [], 'orientation-preview': ['--orientation-preview'],
        'live': ['--live'], 'live-6dof': ['--live', '--live-6dof'],
        'joystick-preview': ['--bind', '127.0.0.1', '--port', '5008', '--orientation-preview'],
        'joystick-live': [],
        'joystick-continuous': [],
    }[mode]
    if mode == 'joystick-continuous':
        print('连续平移实控：取消每次Grip累计行程预算，保留绝对限位和停止保护；须新的交互确认。', flush=True)
    elif mode.startswith('live') or mode == 'joystick-live':
        print('即将进入实控入口；保留程序原有交互确认和全部保护参数。', flush=True)
    log = LOGS / (time.strftime('%Y%m%d-%H%M%S') + f'-{mode}-{os.getpid()}.jsonl')
    script = {'joystick-preview': 'rm75_joystick_preview.py',
              'joystick-live': 'rm75_joystick_live.py',
              'joystick-continuous': 'rm75_continuous_live.py'}.get(mode, 'rm75_target_preview.py')
    os.execv(str(py), [str(py), '-E', '-s', '-u', str(SCRIPTS / script),
        '--robot-ip', robot_ip, '--control-log', str(log), *extra])


def joystick_bridge(pico_ip=None):
    if (ROOT / 'deployment/update-in-progress.json').exists():
        raise RuntimeError('源码更新与验证尚未完成。')
    if pico_ip is not None:
        pico_ip = valid_host(pico_ip)
    # No SDK import: use the system Python only for the standard-library bridge.
    LOGS.mkdir(parents=True, exist_ok=True)
    lock = open(LOGS / 'joystick-bridge.lock', 'w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    os.set_inheritable(lock.fileno(), True)
    args = [sys.executable, '-E', '-s', '-u', str(SCRIPTS / 'rm75_joystick_bridge.py')]
    if pico_ip is not None:
        args += ['--base-ip', pico_ip, '--stick-ip', pico_ip, '--state-destination-ip', pico_ip]
    os.execv(sys.executable, args)


def main(argv=None):
    parser = argparse.ArgumentParser(description='最新 PICO → RM75 项目启动入口；无参数只显示帮助。')
    sub = parser.add_subparsers(dest='command')
    p = sub.add_parser('doctor', help='检查依赖和本地网络，不连接设备')
    p.add_argument('--offline', action='store_true', help='不查询网络或绑定端口')
    sub.add_parser('test', help='Python 和 C# 离线验证，不连接设备')
    sub.add_parser('joystick-offline', help='双摇杆四轴模拟：无 SDK、网络或机械臂连接')
    p = sub.add_parser('joystick-bridge', help='只收发 UDP，5005+5007→5008，5006 反馈；不连接机械臂')
    p.add_argument('--pico-ip', help='可选：固定 PICO Wi-Fi IPv4；省略时自动识别并锁定来源')
    sub.add_parser('build-apk', help='只构建最新 APK，不安装、不运行')
    sub.add_parser('open-unity', help='打开最新 Unity 项目并配置 Android/PICO/场景，不进入 Play 模式')
    p = sub.add_parser('network', help='列出地址、路由和端口，不修改设置')
    p = sub.add_parser('configure-ethernet', help='创建机械臂专用有线配置，不连接机械臂')
    p.add_argument('--interface')
    p.add_argument('--robot-ip', default='192.168.1.18')
    p.add_argument('--pc-cidr', default='192.168.1.100/24')
    for command in ('install-apk', 'configure-pico'):
        p = sub.add_parser(command, help='通过已授权 ADB 安装/设置 Pico APK 的电脑目标地址')
        p.add_argument('--host', help='电脑 Wi-Fi IPv4；省略则自动检测')
        p.add_argument('--serial', help='多个 ADB 设备时指定序号')
    p = sub.add_parser('input', help='只检查 Pico 手柄数据，完全不连接机械臂')
    p.add_argument('--seconds', type=float, default=15)
    for mode in ('preview', 'orientation-preview', 'live', 'live-6dof', 'joystick-preview', 'joystick-live', 'joystick-continuous'):
        p = sub.add_parser(mode, help=('连接机械臂：' + ('实控，须交互确认' if mode.startswith('live') or mode in ('joystick-live','joystick-continuous') else '只读预览')))
        p.add_argument('--robot-ip', default='192.168.1.18')
    args = parser.parse_args(argv)
    try:
        if args.command is None:
            parser.print_help()
            return 0
        if args.command == 'doctor':
            return doctor(args.offline)
        if args.command == 'test':
            return tests()
        if args.command == 'joystick-offline':
            call([sys.executable, '-E', '-s', '-u', PROJECT / 'scripts/rm75_joystick_offline.py'])
            return 0
        if args.command == 'joystick-bridge':
            return joystick_bridge(args.pico_ip)
        if args.command == 'build-apk':
            return build_apk()
        if args.command == 'open-unity':
            return open_unity()
        if args.command == 'network':
            call(['ip', '-brief', 'address'])
            call(['ip', 'route'])
            call(['ss', '-lun'])
        elif args.command == 'configure-ethernet':
            configure_ethernet(args.interface, args.robot_ip, args.pc_cidr)
        elif args.command in ('install-apk', 'configure-pico'):
            configure_pico(args.host, args.serial, install=args.command == 'install-apk')
        elif args.command == 'input':
            if not 0 < args.seconds <= 300:
                raise ValueError('--seconds 必须在 0 到 300 秒之间')
            return probe(args.seconds)
        else:
            control(args.command, args.robot_ip)
        return 0
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
        print('ERROR:', exc, file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\n已退出。')
        return 130


if __name__ == '__main__':
    sys.exit(main())
