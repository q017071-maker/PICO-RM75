"""Continuous TCP translation: real input increments, measured lead, absolute joints.

No robot SDK imports or device connections. The original packet/Grip/rotation
gate and original motion safety implementation are reused by composition.
"""
from dataclasses import replace
import math
from rm75_preview_core import PreviewEngine, PreviewCommandState
from rm75_live_control import (LiveControlFault, RecoverableLimit, MotionOutcome,
                               _finite_list, _state)
from rm75_joystick_motion import JoystickMotionController

LEAD_M = .005


class ContinuousEngine(PreviewEngine):
    def __init__(self, config, read_pose, *, measured, clock, map_orientation=None,
                 base_to_work=None):
        super().__init__(config, read_pose, map_orientation=map_orientation)
        self._measured = measured
        self._clock = clock
        self._base_to_work = base_to_work or [[1.,0,0],[0,1.,0],[0,0,1.]]
        self._input = None
        self._input_time = None
        self._xyz = None
        self._started = False
        self.joystick_input_requested = False
        self.planning_measurement = None

    def disarm(self, reason):
        super().disarm(reason)
        self._input = self._input_time = self._xyz = None
        self._started = False
        self.joystick_input_requested = False

    def command_state(self):
        state = super().command_state()
        return PreviewCommandState(tuple([*self._xyz, *state.target[3:]]), state.limited_rotation)

    def hold(self, reason, now=None, restore=None):
        super().hold(reason, now, restore)
        if restore is not None:
            self._xyz = list(restore.target[:3])

    def accept(self, packet, now, peer):
        self.planning_measurement = None
        previous_source, previous_sequence, previous_sent = self._source, self._sequence, self._sent_time
        was_engaged = self.engaged
        old_input, old_time = self._input, self._input_time
        # Reuse the source, sequence, time, tracking, Grip and orientation gate.
        # XYZ is generated below from input increments, never from a Grip radius.
        target = super().accept(replace(packet, position=(0.,0.,0.)), now, peer)
        if ((peer, packet.session) == previous_source and
                (packet.sequence <= previous_sequence or packet.sent_time <= previous_sent)):
            return None
        if ((peer, packet.session) != self._source or packet.sequence != self._sequence
                or packet.sent_time != self._sent_time):
            return None
        if not self.engaged:
            return None
        self._input, self._input_time = packet.position, now
        if was_engaged and old_input is not None and math.dist(packet.position, old_input) > self.config.jump_limit:
            self.disarm('Controller position jump; release Grip')
            return None
        if target is None:
            return None
        if not was_engaged or self._xyz is None:
            self._xyz = list(target[:3])
            self.joystick_input_requested = False
            return target
        if old_input is None or old_time is None:
            self.disarm('Continuous input reference missing; release Grip')
            return None
        delta = dict(zip('xyz', (a-b for a,b in zip(packet.position, old_input))))
        base_step = [delta[axis.lstrip('-')]*self.config.scale*(-1 if axis.startswith('-') else 1)
                     for axis in self.config.axes]
        step = [sum(row[j]*base_step[j] for j in range(3)) for row in self._base_to_work]
        distance = math.hypot(*step)
        maximum = self.config.speed*min(.05,max(0.,now-old_time))
        if distance > maximum and distance:
            step = [v*maximum/distance for v in step]
        rotation_input = min(max(abs(a-b) for a,b in zip(packet.rotation,self._reference_rotation)),
                             max(abs(a+b) for a,b in zip(packet.rotation,self._reference_rotation))) > 1e-12
        self.joystick_input_requested = math.hypot(*step) > 1e-12 or rotation_input
        self.planning_measurement = None
        if self._started or self.joystick_input_requested:
            sample = self._measured()
            state = _state(sample['state'])
            age = self._clock()-sample['at']
            if not math.isfinite(age) or not 0 <= age < self.config.timeout:
                self.disarm('Measured state stale; release Grip')
                return None
            self.planning_measurement = {'state':state,'at':sample['at']}
            actual = state['pose'][:3]
            if not self._started:
                self._xyz = list(actual)
                self._started = True
            if math.dist(self._xyz, actual) > .015:
                self.disarm('Robot position error exceeds 0.015m; release Grip')
                return None
            proposed = [a+b for a,b in zip(self._xyz,step)]
            old_error = math.dist(self._xyz,actual)
            new_error = math.dist(proposed,actual)
            if new_error > LEAD_M and new_error > old_error:
                if old_error > LEAD_M:
                    proposed = list(self._xyz)  # Wait for feedback, do not wind up.
                else:
                    proposed = [a+(b-a)*LEAD_M/new_error for a,b in zip(actual,proposed)]
            self._xyz = proposed
        self._target = [*self._xyz,*target[3:]]
        return list(self._target)


def read_joint_bounds(arm, algo, margin=5.):
    """Intersect controller and model absolute limits; never write joint limits."""
    bounds=[]
    for getter, model_getter, label in (
            ('rm_get_joint_min_pos','rm_algo_get_joint_min_limit','minimum'),
            ('rm_get_joint_max_pos','rm_algo_get_joint_max_limit','maximum')):
        code, values = getattr(arm,getter)()
        if type(code) is not int or code != 0:
            raise LiveControlFault(f'Cannot read controller joint {label}: {code}')
        values=_finite_list(values,7,label)
        model=_finite_list(getattr(algo,model_getter)(),7,'model '+label)
        bounds.append([max(a,b)+margin if label=='minimum' else min(a,b)-margin
                       for a,b in zip(values,model)])
    if any(a>=b for a,b in zip(*bounds)):
        raise LiveControlFault('Invalid or conflicting absolute joint limits')
    return bounds


class AbsoluteKinematics:
    """Use the current measured seed as a per-command reference, not a Grip budget."""
    def __init__(self, solver, owner, bounds):
        if not solver.uses_remote_ik:
            raise LiveControlFault('Continuous mode requires dedicated remote IK')
        self.solver,self.owner,self.bounds=solver,owner,bounds
        self.uses_remote_ik=True

    def calculate_arm_angle(self, joints):
        return self.solver.calculate_arm_angle(joints)

    def solve(self, seed, pose, angle):
        result = _finite_list(self.solver.solve(seed,pose,angle),7,'IK solution')
        for i,(q,lo,hi) in enumerate(zip(result,*self.bounds),1):
            if q <= lo or q >= hi:
                raise RecoverableLimit(f'J{i} absolute joint limit with safety margin; reverse to retreat')
        # This mode deliberately replaces the old per-Grip 30deg budget with
        # absolute bounds above. The original 3deg measured-seed step check is
        # still executed by LiveMotionController immediately after this return.
        self.owner._reference_solution=list(seed)
        return result


class ContinuousMotionController(JoystickMotionController):
    def __init__(self, arm, read_state, kinematics, clock, *, bounds, **kwargs):
        super().__init__(arm,read_state,kinematics,clock,**kwargs)
        self._kinematics=AbsoluteKinematics(kinematics,self,bounds)
        self._progress=None

    def prepare(self, state):
        super().prepare(state)
        self._progress=None

    def _hold(self, reason):
        self._progress=None
        return super()._hold(reason)

    def send_target(self, pose, now):
        result=super().send_target(pose,now)
        if result.outcome != MotionOutcome.HOLD and self.has_sent and self.measured_state:
            actual=self.measured_state['pose'][:3]
            error=math.dist(pose[:3],actual)
            if error < .0005:
                self._progress=None
            elif self._progress is None:
                self._progress=[self._clock(),list(actual),0.,min(.0002,error*.1)]
            else:
                p=self._progress;p[2]=max(p[2],math.dist(actual,p[1]))
                if self._clock()-p[0]>=1.:
                    if p[2]<p[3]:
                        raise LiveControlFault('TCP made no measured progress for 1s; release Grip and inspect')
                    self._progress=None
        return result
