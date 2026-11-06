"""Robot-free replay of continuous target planning using recorded feedback."""
import argparse
import json
import math
from pathlib import Path
from types import SimpleNamespace
from rm75_continuous_core import ContinuousEngine
from rm75_continuous_live import LOG_TYPE,orientation_mapper
from rm75_preview_core import ControllerPacket,PreviewConfig,_quaternion_matrix_wxyz,_transpose
from rm75_log_replay import _validated_frame,_euler_to_quaternion,_quaternion_to_euler


def replay_log(path, *, sdk_orientation=False):
    records=[json.loads(line) for line in Path(path).read_text().splitlines()]
    if not records or records[0].get('type')!=LOG_TYPE:
        raise ValueError('Not a continuous joystick log')
    if records[0].get('continuous_engine_version')!=1:
        raise ValueError('Unsupported continuous engine version')
    setup=next((r for r in records if r.get('record')=='setup'),None)
    if setup is None:raise ValueError('Missing controller setup record')
    config_values=dict(records[0]['config']);config_values['axes']=tuple(config_values['axes'])
    config=PreviewConfig(**config_values)
    rotation=_quaternion_matrix_wxyz(tuple(_euler_to_quaternion(setup['work_frame']['pose'][3:])))
    algo=SimpleNamespace(rm_algo_euler2quaternion=_euler_to_quaternion,rm_algo_quaternion2euler=_quaternion_to_euler)
    if sdk_orientation:
        from Robotic_Arm.rm_robot_interface import Algo,rm_robot_arm_model_e,rm_force_type_e
        algo=Algo(rm_robot_arm_model_e.RM_MODEL_RM_75_E,rm_force_type_e.RM_MODEL_RM_ISF_E)
    current=[None];clock=[0.]
    engine=ContinuousEngine(config,lambda:current[0]['reference_pose'],
        measured=lambda:current[0]['planning_measurement'],clock=lambda:clock[0],
        base_to_work=_transpose(rotation),map_orientation=orientation_mapper(algo,config.axes,rotation))
    previous=None;count=0
    for line,record in enumerate(records[1:],2):
        if record.get('type')!=LOG_TYPE or record.get('version')!=1:
            raise ValueError(f'Invalid record at {line}')
        if record.get('record')=='setup':continue
        received,completed,payload=_validated_frame(record,line,previous);previous=received
        current[0]=record;clock[0]=received
        checkpoint=engine.command_state() if engine.engaged else None
        target=None
        if payload is None:engine.tick(received)
        else:
            try:
                packet=ControllerPacket.parse(payload)
                target=engine.accept(packet,received,tuple(record['peer']))
            except ValueError as exc:engine.disarm(f'Invalid packet: {exc}')
        engine.tick(completed)
        if completed-received>=config.timeout:
            engine.disarm('Reference read timeout; release grip before retrying');target=None
        if record['motion_outcome']=='HOLD' and engine.engaged:
            engine.hold(record['reason'],received,restore=checkpoint);target=None
        elif record['motion_outcome']=='DISARM':
            engine.disarm(record['reason']);target=None
        if engine.status!=record['status']:
            raise ValueError(f'Status mismatch at {line}: {engine.status} vs {record["status"]}')
        if record['target'] is not None:
            if target is None or any(not math.isclose(a,b,abs_tol=1e-12) for a,b in zip(target,record['target'])):
                raise ValueError(f'Target mismatch at {line}')
        count+=1
    return count


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log');parser.add_argument('--sdk-orientation',action='store_true')
    args=parser.parse_args()
    print('Continuous planning replay passed:',replay_log(args.log,sdk_orientation=args.sdk_orientation),'frames; no robot connection.')


if __name__=='__main__':main()
