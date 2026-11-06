"""Joystick-only intent guard; original IK, joint and stop checks remain intact."""
import math
from rm75_live_control import (LiveMotionController, LiveControlFault, MotionOutcome,
                               MotionResult, _state, _finite_list, _orientation_distance)


class JoystickMotionController(LiveMotionController):
    # Ignore mathematical round-trip noise only, not intentional stick input.
    # Existing LiveLimits.deadband values are not changed.
    INTENT_POSITION_EPS = 1e-9
    INTENT_ORIENTATION_EPS = 1e-6
    def __init__(self, arm, read_state, kinematics, clock, *args, intent_provider=None, **kwargs):
        self.measured_state = None
        self.measured_at = None
        self.commands_sent = 0
        self.engagement_commands_sent = 0
        self.waiting_for_intent = True
        self.last_limit_detail = None
        self._intent_provider = intent_provider

        def observed_read():
            result = read_state()
            self.measured_state = _state(result)
            self.measured_at = clock()
            return result

        super().__init__(arm, observed_read, kinematics, clock, *args, **kwargs)
        self._intent_origin = None

    def prepare(self, state):
        self.waiting_for_intent = True
        self.engagement_commands_sent = 0
        super().prepare(state)
        self._intent_origin = _state(state)['pose']
        self.measured_state = _state(state)
        self.measured_at = self._clock()
        self.waiting_for_intent = True
        self.last_limit_detail = None

    def send_target(self, pose, now):
        if not self.prepared:
            raise LiveControlFault('Live motion is not prepared')
        target = _finite_list(pose, 6, 'Live target pose')
        if type(now) not in (int, float) or not math.isfinite(now):
            raise LiveControlFault('Target receive time must be finite')
        age = self._clock() - now
        if not math.isfinite(age) or age < 0 or age >= self._limits.max_command_age:
            raise LiveControlFault(f'Target is stale: age {age!r}s')
        has_intent = (math.dist(target[:3], self._intent_origin[:3]) >= self.INTENT_POSITION_EPS or
                      _orientation_distance(target, self._intent_origin) >= self.INTENT_ORIENTATION_EPS)
        if self.waiting_for_intent and self._intent_provider is not None:
            try:
                has_intent = self._intent_provider()
            except Exception as exc:
                raise LiveControlFault(f'Joystick intent read failed: {exc}') from exc
            if type(has_intent) is not bool:
                raise LiveControlFault('Joystick intent must be boolean')
        if self.waiting_for_intent and not has_intent:
            # Grip alone must not send a reference-pose correction. This is only
            # a pre-motion guard: after a real input, unchanged targets still
            # use continuous remote IK until the robot converges as before.
            self.last_result = MotionResult(MotionOutcome.UNCHANGED,
                                            'Grip armed; waiting for joystick input')
            return self.last_result
        result = super().send_target(target, now)
        if result.outcome != MotionOutcome.HOLD:
            self.waiting_for_intent = False
            self.commands_sent += 1
            self.engagement_commands_sent += 1
        return result

    def _hold(self, reason):
        self.last_limit_detail = str(reason)
        if str(reason).startswith('IK joint excursion '):
            reason = (f'IK joint excursion {self._limits.max_joint_excursion:g}deg limit: reverse stick to retreat; '
                      'release/repress Grip to establish a new reference')
        return super()._hold(reason)

    def telemetry(self):
        return dict(measured_state=self.measured_state, measured_at=self.measured_at,
                    commands_sent=self.commands_sent,
                    engagement_commands_sent=self.engagement_commands_sent,
                    motion_prepared=self.prepared, motion_has_sent=self.has_sent,
                    waiting_for_joystick_input=self.waiting_for_intent,
                    last_limit_detail=self.last_limit_detail)
