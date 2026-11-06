"""Explicit continuous joystick entry; no device activity on import."""
import argparse
import builtins
from dataclasses import replace
import re
import socket
import sys
import time
from types import SimpleNamespace

import rm75_target_preview as stable
from rm75_preview_core import (PreviewConfig, map_relative_orientation,
    _matrix_multiply, _transpose, _quaternion_matrix_wxyz, _matrix_quaternion_wxyz)
from rm75_log_replay import ControlLogWriter, _euler_to_quaternion
from rm75_live_control import SdkRm75Kinematics
from rm75_joystick_settings import TRANSLATION_SPEED_M_S
from rm75_udp_latest import LatestInputSocket
from rm75_state_broadcast import StateBroadcaster
from rm75_continuous_core import ContinuousEngine, ContinuousMotionController, read_joint_bounds, LEAD_M

CONFIRMATION='MOVE_RM75_CONTINUOUS'
LOG_TYPE='rm75_continuous_joystick_log'


def orientation_mapper(algo, axes, rotation):
    def mapped(reference, current, rpy):
        base_delta=map_relative_orientation(reference,current,[0.,0.,0.],axes,
            algo.rm_algo_euler2quaternion,algo.rm_algo_quaternion2euler)
        base_matrix=_quaternion_matrix_wxyz(tuple(algo.rm_algo_euler2quaternion(base_delta)))
        work_delta=_matrix_multiply(_matrix_multiply(_transpose(rotation),base_matrix),rotation)
        start=_quaternion_matrix_wxyz(tuple(algo.rm_algo_euler2quaternion(rpy)))
        return list(algo.rm_algo_quaternion2euler(list(_matrix_quaternion_wxyz(_matrix_multiply(work_delta,start)))))
    return mapped


def run_continuous(args, *, arm_factory, sdk_factory, socket_factory=socket.socket,
                   clock=time.monotonic, input_func=input, stdin_isatty=sys.stdin.isatty,
                   state_factory=None):
    if not args.control_log:
        raise ValueError('Continuous mode requires a control log')
    context={}
    original=(stable.PreviewEngine,stable.PreviewConfig,stable.ReadOnlyRobot,
              stable.ControlLogWriter,stable.confirm_live)
    old_print=stable.__dict__.get('print')
    had_print='print' in stable.__dict__

    class Robot(original[2]):
        def read_state(self):
            state=super().read_state()
            context['measurement']={'state':state,'at':clock()}
            return state

        def __enter__(self):
            super().__enter__()
            try:
                sdk=context['sdk'];algo=sdk.algo
                context['bounds']=read_joint_bounds(self.arm,algo)
                for kind in ('work','tool'):
                    code,frame=getattr(self.arm,'rm_get_current_'+kind+'_frame')()
                    if type(code) is not int or code!=0 or not isinstance(frame,dict):
                        raise RuntimeError(f'Cannot read current {kind} frame')
                    pose=stable.validate_pose(frame.get('pose'))
                    getattr(algo,'rm_algo_set_'+kind+'frame')(sdk.frame_type(frame_name=frame['name'],pose=tuple(pose)))
                    context[kind+'_frame']=frame
                rotation=_quaternion_matrix_wxyz(tuple(_euler_to_quaternion(context['work_frame']['pose'][3:])))
                context['rotation']=rotation
                context['writer']._write(dict(type=LOG_TYPE,version=1,record='setup',
                    absolute_joint_bounds_deg=context['bounds'],joint_margin_deg=5.,
                    work_frame=context['work_frame'],tool_frame=context['tool_frame']))
                builtins.print('ABSOLUTE JOINT LIMITS (5deg margin):',context['bounds'],flush=True)
                return self
            except BaseException:
                self.close()
                raise

    class Writer(ControlLogWriter):
        def __init__(self,*a,**kw):
            context['writer']=self
            super().__init__(*a,**kw)
        def _write(self,value):
            record=dict(value,type=LOG_TYPE)
            if record.get('record')=='header':
                record.update(continuous_engine_version=1,per_grip_translation_limit=None,
                    per_grip_joint_budget=None,radius_reference='latest_measured_TCP_lead',
                    tcp_lead_limit_m=LEAD_M,initial_input_guard=True)
            elif record.get('record')=='frame':
                engine=context.get('engine');controller=context.get('controller')
                if engine is not None:
                    record['planning_measurement']=engine.planning_measurement
                if controller is not None:
                    record.update(controller.telemetry())
            super()._write(record)

    def engine_factory(config,read_pose,**kw):
        engine=ContinuousEngine(config,read_pose,measured=lambda:context['measurement'],clock=clock,
            base_to_work=_transpose(context['rotation']),**kw)
        context['engine']=engine
        return engine

    def config_factory(*a,**kw):
        return replace(PreviewConfig(*a,**kw),radius=LEAD_M,speed=TRANSLATION_SPEED_M_S,
            require_verified_tracking=True)

    def orientation_factory(axes):
        sdk=sdk_factory();context['sdk']=sdk
        return lambda reference,current,rpy:orientation_mapper(sdk.algo,axes,context['rotation'])(reference,current,rpy)

    def motion_factory(arm,read_state,provided_clock):
        sdk=context['sdk'];solver=SdkRm75Kinematics(sdk.algo,sdk.params_type,ik_owner=arm)
        controller=ContinuousMotionController(arm,read_state,solver,provided_clock,
            bounds=context['bounds'],intent_provider=lambda:context['engine'].joystick_input_requested)
        context['controller']=controller
        return controller

    def confirm(typed,isatty,expected=None):
        return original[4](typed,isatty,expected=CONFIRMATION)

    def policy_print(*values,**kw):
        if len(values)==1 and isinstance(values[0],str):
            text=values[0]
            if text.startswith('LIVE 6DOF ARMED'):
                text=text.replace('LIVE 6DOF ARMED','CONTINUOUS TCP ARMED').replace('radius=','lead_limit=')
            elif text.startswith('LIVE LIMITS'):
                text=re.sub(r' joint_excursion=[^ ]+','',text)
            values=(text,)
        builtins.print(*values,**kw)

    try:
        stable.PreviewEngine,stable.PreviewConfig,stable.ReadOnlyRobot=engine_factory,config_factory,Robot
        stable.ControlLogWriter,stable.confirm_live,stable.print=Writer,confirm,policy_print
        if state_factory is None:
            state_factory=lambda port:StateBroadcaster(socket_factory(socket.AF_INET,socket.SOCK_DGRAM),clock,port=port)
        builtins.print('CONTINUOUS TCP | 无Grip累计平移/关节行程预算；'
            f'{TRANSLATION_SPEED_M_S:.3f}m/s；目标超前最多{LEAD_M*1000:g}mm（不是总行程限制）。',flush=True)
        builtins.print('保留Grip/绝对关节限位5deg裕量/单步3deg/奇异性0.01/失联和跟踪保护；新设备需现场验收。',flush=True)
        return stable.run(args,arm_factory,socket_factory,clock,input_func=input_func,
            stdin_isatty=stdin_isatty,live_factory=motion_factory,state_factory=state_factory,
            orientation_factory=orientation_factory)
    finally:
        (stable.PreviewEngine,stable.PreviewConfig,stable.ReadOnlyRobot,
         stable.ControlLogWriter,stable.confirm_live)=original
        if had_print:stable.print=old_print
        else:stable.__dict__.pop('print',None)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip',default='192.168.1.18')
    parser.add_argument('--control-log',required=True)
    args=parser.parse_args(argv)
    forwarded=stable.build_parser().parse_args(['--live','--live-6dof','--bind','127.0.0.1','--port','5008',
        '--robot-ip',args.robot_ip,'--control-log',args.control_log])
    def arm_factory():
        from Robotic_Arm.rm_robot_interface import RoboticArm,rm_thread_mode_e
        return RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    def sdk_factory():
        from Robotic_Arm.rm_robot_interface import (Algo,rm_robot_arm_model_e,rm_force_type_e,
            rm_frame_t,rm_inverse_kinematics_params_t)
        return SimpleNamespace(algo=Algo(rm_robot_arm_model_e.RM_MODEL_RM_75_E,rm_force_type_e.RM_MODEL_RM_ISF_E),
            frame_type=rm_frame_t,params_type=rm_inverse_kinematics_params_t)
    return run_continuous(forwarded,arm_factory=arm_factory,sdk_factory=sdk_factory,
        socket_factory=lambda *a:LatestInputSocket(socket.socket(*a)))


if __name__=='__main__':
    sys.exit(main())
