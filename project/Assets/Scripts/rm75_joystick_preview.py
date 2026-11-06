"""Independent read-only joystick preview; reuses original I/O and protections.

The stable pose-follow entry point and source files remain unchanged. This
entry point rejects live operation before opening any socket or robot handle.
"""
import sys
import math
from dataclasses import replace
import rm75_target_preview as preview
import rm75_log_replay as replay
from rm75_preview_core import PreviewConfig, PreviewEngine
from rm75_joystick_settings import POSITION_RADIUS_M, TRANSLATION_SPEED_M_S
from rm75_udp_latest import SocketModuleProxy

JOYSTICK_LOG_TYPE = 'rm75_joystick_preview_log'
JOYSTICK_ENGINE_VERSION = 2


def joystick_config(config):
    return replace(config, require_verified_tracking=True,
                   orientation_radius=math.radians(10),
                   orientation_jump_limit=math.radians(3), orientation_speed=.05)


class JoystickLogWriter(replay.ControlLogWriter):
    def _write(self, value):
        # Ordinary pose-follow replay must reject these unfiltered trajectories.
        record = dict(value, type=JOYSTICK_LOG_TYPE)
        if record.get('record') == 'header':
            record['joystick_engine_version'] = JOYSTICK_ENGINE_VERSION
        super()._write(record)


def joystick_engine_class(core_engine, *, anti_windup=True):
    class JoystickPreviewEngine(core_engine):
        def __init__(self, config, *args, **kwargs):
            super().__init__(joystick_config(config), *args, **kwargs)
            self.joystick_input_requested = False

        def accept(self, packet, now, peer):
            # Joystick input is already a bounded integrated velocity target.
            # Pose-follow filtering would accumulate a moving target after the
            # sticks return to zero. Bypass only that filter in this mode.
            if hasattr(self, '_filtered_position'):
                self._filtered_position = packet.position
            result = super().accept(packet, now, peer)
            self.joystick_input_requested = False
            if self.engaged and self._reference_position is not None:
                position_input = math.dist(packet.position, self._reference_position) > 1e-12
                direct = max(abs(a-b) for a,b in zip(packet.rotation, self._reference_rotation))
                opposite = max(abs(a+b) for a,b in zip(packet.rotation, self._reference_rotation))
                self.joystick_input_requested = position_input or min(direct, opposite) > 1e-12
            return result

        def hold(self, reason, now=None, restore=None):
            super().hold(reason, now, restore)
            geometric_position_hold = str(reason).startswith('Target outside reference radius')
            if (anti_windup and (restore is not None or geometric_position_hold) and
                    self._filtered_position is not None and self._reference_pose is not None):
                # Discard rejected translation demand while keeping the robot
                # reference pose, workspace radius and joint reference intact.
                # Reversing can retreat immediately; forward remains subject
                # to the same 30deg cumulative joint guard.
                reference = list(self._reference_position)
                for robot_axis, axis in enumerate(self.config.axes):
                    source_axis = 'xyz'.index(axis.lstrip('-'))
                    sign = -1 if axis.startswith('-') else 1
                    offset = self._target[robot_axis] - self._reference_pose[robot_axis]
                    reference[source_axis] = self._filtered_position[source_axis] - offset/(self.config.scale*sign)
                self._reference_position = tuple(reference)
    return JoystickPreviewEngine


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    args = preview.build_parser().parse_args(argv)
    if args.live:
        raise ValueError('Joystick integration is read-only; live operation is not enabled')
    if args.live_6dof or args.live_orientation_axis:
        raise ValueError('Joystick integration is read-only; live flags are forbidden')
    if (args.axes != 'z,-x,y' or args.scale != .2 or args.radius != .03 or
            args.speed != .01 or args.timeout != .25 or args.jump_limit != .1):
        raise ValueError('Joystick preview does not allow overriding mapping or protection options')
    if args.bind != '127.0.0.1' or args.port != 5008:
        raise ValueError('Joystick preview requires --bind 127.0.0.1 --port 5008')
    if not args.orientation_preview:
        raise ValueError('Joystick preview requires --orientation-preview for base-X Roll')
    # Only this dedicated process selects the independent engine. Original
    # entry points retain their own unchanged PreviewEngine and defaults.
    original = preview.PreviewEngine, preview.PreviewConfig, preview.ControlLogWriter, preview.socket
    try:
        preview.PreviewEngine = joystick_engine_class(PreviewEngine)
        preview.PreviewConfig = lambda *a, **kw: replace(
            joystick_config(PreviewConfig(*a, **kw)),
            radius=POSITION_RADIUS_M, speed=TRANSLATION_SPEED_M_S)
        preview.ControlLogWriter = JoystickLogWriter
        preview.socket = SocketModuleProxy(original[3])
        return preview.main(argv)
    finally:
        preview.PreviewEngine, preview.PreviewConfig, preview.ControlLogWriter, preview.socket = original


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except ValueError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        raise SystemExit(1)
