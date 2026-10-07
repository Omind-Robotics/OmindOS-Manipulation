"""Transport-independent task contract shared by offline and ROS2 examples.

Distances are metres, angles radians. Base goals use map; arm trajectories use
the robot's own joint names. Navigation and manipulation are sequential in v0.1.
"""
from dataclasses import dataclass
from enum import Enum
import math


class Phase(str, Enum):
    IDLE = 'idle'
    NAVIGATING = 'navigating'
    READY = 'ready'
    MANIPULATING = 'manipulating'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    CANCELED = 'canceled'


JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


def joint_names(side):
    if side not in ('left', 'right'):
        raise ValueError('side must be left or right')
    return [side + '_' + name for name in JOINTS]


def trajectory_joint_names(include_waist=False):
    return (['waist_pitch_joint'] if include_waist else []) + joint_names('left') + joint_names('right')


def validate_trajectory(names, points):
    """Accept exactly the legacy 12-joint or waist-first 13-joint contract."""
    if list(names) not in (trajectory_joint_names(), trajectory_joint_names(True)):
        raise ValueError('Expected left-then-right 12 joints, optionally preceded by waist_pitch_joint')
    if not points or len(points) > 10000:
        raise ValueError('Trajectory must contain 1..10000 points')
    last = -1.0
    for point in points:
        t, q = point['time_s'], point['positions']
        if not math.isfinite(t) or t <= last or t < 0:
            raise ValueError('Trajectory times must be finite, nonnegative and increasing')
        if len(q) != len(names) or not all(math.isfinite(v) for v in q):
            raise ValueError('Expected one finite position per named joint')
        last = t
    return points


def validate_playback(plan):
    """Schema binds the exact joint layout; waist limits travel with its plan."""
    from community.waist_config import WaistConfig
    schemas = {'omindos.community.playback.v0.1': False,
               'omindos.community.playback.waist_pitch.v1': True}
    if plan.get('schema') not in schemas or plan.get('simulation_only') is not True:
        raise ValueError('Only Community simulation playback plans are accepted')
    waist = schemas[plan['schema']]
    if plan['joint_names'] != trajectory_joint_names(waist):
        raise ValueError('Playback schema and joint layout do not match')
    validate_trajectory(plan['joint_names'], plan['points'])
    BaseGoal(**plan['base_goal'])
    if waist:
        config = WaistConfig(**plan['waist_config'])
        lo, hi = config.limits_rad
        if any(not lo <= point['positions'][0] <= hi for point in plan['points']):
            raise ValueError('Playback exceeds configured waist limits')
        if abs(plan['points'][0]['positions'][0] - config.transport_rad) > 1e-9:
            raise ValueError('Playback must start at the configured waist transport angle')
    return plan


@dataclass(frozen=True)
class BaseGoal:
    x: float
    y: float
    yaw: float
    frame_id: str = 'map'

    def __post_init__(self):
        if self.frame_id != 'map' or not all(math.isfinite(v) for v in (self.x, self.y, self.yaw)):
            raise ValueError('Base goal must be finite and in map')


class TaskGate:
    """Only explicit successful results advance the task. Failure never falls through."""
    def __init__(self):
        self.phase = Phase.IDLE
        self.history = [self.phase.value]

    def _set(self, phase):
        self.phase = phase
        self.history.append(phase.value)

    def start_navigation(self):
        if self.phase != Phase.IDLE:
            raise RuntimeError('Navigation requires an idle task')
        self._set(Phase.NAVIGATING)

    def navigation_result(self, success, stopped):
        if self.phase != Phase.NAVIGATING:
            raise RuntimeError('Unexpected navigation result')
        self._set(Phase.READY if success and stopped else Phase.FAILED)

    def start_manipulation(self):
        if self.phase != Phase.READY:
            raise RuntimeError('Manipulation requires successful navigation and a stopped base')
        self._set(Phase.MANIPULATING)

    def manipulation_result(self, success):
        if self.phase != Phase.MANIPULATING:
            raise RuntimeError('Unexpected manipulation result')
        self._set(Phase.SUCCEEDED if success else Phase.FAILED)

    def cancel(self):
        if self.phase in (Phase.SUCCEEDED, Phase.FAILED, Phase.CANCELED):
            raise RuntimeError('Task is already terminal')
        self._set(Phase.CANCELED)

    def fail(self):
        if self.phase not in (Phase.SUCCEEDED, Phase.FAILED, Phase.CANCELED):
            self._set(Phase.FAILED)
