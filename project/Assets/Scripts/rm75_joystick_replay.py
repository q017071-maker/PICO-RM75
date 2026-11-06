"""Robot-free replay for the independently identified joystick preview logs."""
import sys
import json
import rm75_log_replay as replay
from rm75_joystick_preview import JOYSTICK_LOG_TYPE, joystick_engine_class


def sdk_orientation_mapper(axes, project_axis=None):
    # Only local mathematical Algo APIs are used; no RoboticArm is created.
    from Robotic_Arm.rm_robot_interface import Algo, rm_force_type_e, rm_robot_arm_model_e
    from rm75_preview_core import map_relative_orientation
    algo = Algo(rm_robot_arm_model_e.RM_MODEL_RM_75_E, rm_force_type_e.RM_MODEL_RM_ISF_E)
    return lambda reference, current, rpy: map_relative_orientation(
        reference, current, rpy, axes, algo.rm_algo_euler2quaternion,
        algo.rm_algo_quaternion2euler, project_axis=project_axis)


def replay_log(source, *, sdk_orientation=False):
    with open(source, encoding='utf-8') as stream:
        header = json.loads(stream.readline())
        kind = header.get('type')
    if kind not in (JOYSTICK_LOG_TYPE, 'rm75_joystick_live_log'):
        raise ValueError('Expected an independently identified joystick log')
    engine_version = header.get('joystick_engine_version', 1)
    if type(engine_version) is not int or engine_version not in (1, 2):
        raise ValueError('Unsupported joystick engine version')
    engine, log_type, mapper = replay.PreviewEngine, replay.LOG_TYPE, replay.offline_orientation_mapper
    try:
        replay.PreviewEngine = joystick_engine_class(engine, anti_windup=engine_version >= 2)
        replay.LOG_TYPE = kind
        if sdk_orientation:
            replay.offline_orientation_mapper = sdk_orientation_mapper
        return replay.replay_log(source)
    finally:
        replay.PreviewEngine, replay.LOG_TYPE, replay.offline_orientation_mapper = engine, log_type, mapper


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log')
    parser.add_argument('--sdk-orientation', action='store_true',
                        help='Match RealMan float32 orientation math for field logs; Algo only, no robot connection')
    args = parser.parse_args(argv)
    result = replay_log(args.log, sdk_orientation=args.sdk_orientation)
    print(f'Joystick log replay passed: {len(result.frames)} frames; no robot connection or sockets.')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError, ImportError, RuntimeError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        raise SystemExit(1)
