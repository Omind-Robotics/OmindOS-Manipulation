"""ROS-independent, fail-closed arrival evidence for Navigation Community v0.1.

All receipt times use one monotonic clock, relative to goal publication. This is
an application gate, not an acknowledgement/cancel API or a physical stop.
"""
import math

TRACE_SCHEMA = 'omindos.community.navigation_trace.v1'


def finite_vector(value, size, name):
    if len(value) != size or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in value):
        raise ValueError(f'{name}: expected {size} finite numbers')
    return list(value)


def angle_delta(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


class ArrivalGate:
    """Require a post-goal matching path and fresh, stable stopped observations."""
    def __init__(self, goal, position_tolerance=.25, settle_s=.6, max_age_s=.5):
        self.goal = finite_vector(goal, 2, 'goal xy')
        if not all(math.isfinite(v) and v > 0 for v in (position_tolerance, settle_s, max_age_s)):
            raise ValueError('Gate tolerances must be positive')
        self.position_tolerance, self.settle_s, self.max_age_s = position_tolerance, settle_s, max_age_s
        self.last_time, self.stable_since, self.anchor = -1., None, None

    def observe(self, sample):
        now, pose_time, cmd_time, path_time = finite_vector(
            [sample[k] for k in ('time_s', 'pose_time_s', 'cmd_time_s', 'path_time_s')], 4, 'times')
        if now <= self.last_time or now < 0:
            raise ValueError('Observation times must increase from goal publication')
        if now - self.last_time > self.max_age_s:
            self.stable_since, self.anchor = None, None
        self.last_time = now
        pose = finite_vector(sample['pose'], 3, 'pose x,y,yaw')
        cmd = finite_vector(sample['twist'], 6, 'twist linear xyz, angular xyz')
        endpoint = finite_vector(sample['path_end'], 2, 'path endpoint')
        valid = (sample.get('frame_id') == 'map'
                 and 0 <= pose_time <= now and now - pose_time <= self.max_age_s
                 and 0 <= cmd_time <= now and now - cmd_time <= self.max_age_s
                 and 0 <= path_time <= now
                 and math.dist(endpoint, self.goal) <= .25
                 and math.dist(pose[:2], self.goal) <= self.position_tolerance
                 and max(abs(v) for v in cmd) <= .001)
        if not valid:
            self.stable_since, self.anchor = None, None
            return False
        if (self.anchor is None or math.dist(pose[:2], self.anchor[:2]) > .01
                or abs(angle_delta(pose[2], self.anchor[2])) > .02):
            self.stable_since, self.anchor = now, pose
        return now - self.stable_since >= self.settle_s


def validate_trace(trace):
    if (trace.get('schema') != TRACE_SCHEMA or trace.get('simulation_only') is not True
            or trace.get('frame_id') != 'map'):
        raise ValueError('Expected a map-frame simulation navigation trace')
    gate = ArrivalGate(trace['goal_xy'])
    samples = trace['samples']
    if not samples or len(samples) > 20000:
        raise ValueError('Expected 1..20000 navigation observations')
    arrived = False
    previous = None
    for sample in samples:
        arrived = gate.observe(sample)
        if previous:
            dt = sample['time_s'] - previous['time_s']
            # Reject discontinuities / sparse logs before collision replay. These
            # are generous simulation bounds, not measured hardware capabilities.
            if (dt > .5 or math.dist(sample['pose'][:2], previous['pose'][:2]) > 1.5 * dt + .02
                    or abs(angle_delta(sample['pose'][2], previous['pose'][2])) > 3 * dt + .02):
                raise ValueError('Navigation trace has a gap or discontinuous base motion')
        previous = sample
    if not arrived or trace.get('status') != 'arrived':
        raise ValueError('Navigation did not finish with fresh arrival and stop evidence')
    return trace
