"""Explicitly confirmed joystick motion, reusing the unchanged stable safety loop.

Importing this module never opens a robot or socket. Only main() may enter the
stable live loop, which still requires an interactive typed confirmation and a
fresh Grip release/press. The independent preview keeps rejecting live flags.
"""
import argparse
from dataclasses import replace
import sys

import rm75_target_preview as stable
from rm75_preview_core import PreviewConfig, PreviewEngine
from rm75_joystick_preview import (JoystickLogWriter, joystick_config, joystick_engine_class,
                                    JOYSTICK_ENGINE_VERSION)
from rm75_joystick_settings import POSITION_RADIUS_M, TRANSLATION_SPEED_M_S
from rm75_udp_latest import SocketModuleProxy
from rm75_joystick_motion import JoystickMotionController


CONFIRMATION = 'MOVE_RM75_JOYSTICK'
JOYSTICK_LIVE_LOG_TYPE = 'rm75_joystick_live_log'


def live_joystick_config(*args, **kwargs):
    # The user-selected joystick profile is independent of pose-follow limits.
    return replace(joystick_config(PreviewConfig(*args, **kwargs)),
                   radius=POSITION_RADIUS_M, speed=TRANSLATION_SPEED_M_S)


class JoystickLiveLogWriter(JoystickLogWriter):
    def __init__(self, *args, motion=None, **kwargs):
        self._motion = motion
        super().__init__(*args, **kwargs)

    def _write(self, value):
        # Skip the preview writer's distinct type, preserving original writer IO.
        from rm75_log_replay import ControlLogWriter
        record = dict(value, type=JOYSTICK_LIVE_LOG_TYPE)
        if record.get('record') == 'header':
            record['joystick_engine_version'] = JOYSTICK_ENGINE_VERSION
            record['joystick_idle_guard'] = True
        elif self._motion is not None and self._motion.get('controller') is not None:
            record.update(self._motion['controller'].telemetry())
            if self._motion.get('engine') is not None:
                record['joystick_input_requested'] = self._motion['engine'].joystick_input_requested
        ControlLogWriter._write(self, record)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', default='192.168.1.18')
    parser.add_argument('--robot-port', type=int, default=8080)
    parser.add_argument('--control-log')
    args = parser.parse_args(argv)
    original = (stable.PreviewEngine, stable.PreviewConfig,
                stable.ControlLogWriter, stable.confirm_live, stable.socket,
                stable.LiveMotionController)
    motion = {}

    def engine_factory(*args, **kwargs):
        engine = joystick_engine_class(PreviewEngine)(*args, **kwargs)
        motion['engine'] = engine
        return engine

    def motion_factory(*args, **kwargs):
        controller = JoystickMotionController(*args, intent_provider=lambda:
            bool(motion.get('engine') and motion['engine'].joystick_input_requested), **kwargs)
        motion['controller'] = controller
        return controller

    def confirm(input_func, stdin_isatty, expected=None):
        return original[3](input_func, stdin_isatty, expected=CONFIRMATION)

    forwarded = ['--live', '--live-6dof', '--bind', '127.0.0.1', '--port', '5008',
                 '--robot-ip', args.robot_ip, '--robot-port', str(args.robot_port)]
    if args.control_log:
        forwarded += ['--control-log', args.control_log]
    print(f'JOYSTICK LIVE | 右 Grip 使能；平移最高 {TRANSLATION_SPEED_M_S:.3f}m/s、'
          f'半径 {POSITION_RADIUS_M:.2f}m；'
          '基坐标 X 旋转最高 0.050rad/s、范围 10deg。', flush=True)
    print('Grip仅使能，首次拨动摇杆后才发送运动；新速度/修复尚待现场复核。', flush=True)
    try:
        stable.PreviewEngine = engine_factory
        stable.PreviewConfig = live_joystick_config
        stable.ControlLogWriter = lambda *a, **kw: JoystickLiveLogWriter(*a, motion=motion, **kw)
        stable.confirm_live = confirm
        stable.socket = SocketModuleProxy(original[4])
        stable.LiveMotionController = motion_factory
        return stable.main(forwarded)
    finally:
        (stable.PreviewEngine, stable.PreviewConfig,
         stable.ControlLogWriter, stable.confirm_live, stable.socket,
         stable.LiveMotionController) = original


if __name__ == '__main__':
    sys.exit(main())
