"""Standalone grid A* and planar kinematics. No ROS or hardware transport.

This reference planner is independent of nav3d_native. Occupied and unknown
cells are obstacles; a circular envelope covers the complete transport pose.
"""
import heapq
import itertools
import math
import numpy as np
from community.navigation_contract import angle_delta, finite_vector, TRACE_SCHEMA, validate_trace


class Grid:
    def __init__(self, data, resolution, origin):
        self.data = np.asarray(data)
        self.origin = np.array(finite_vector(origin, 2, 'grid origin'), dtype=float)
        if (self.data.ndim != 2 or min(self.data.shape) < 2 or self.data.size > 1000000
                or not np.isfinite(self.data).all() or np.any(self.data != self.data.astype(int))
                or np.any((self.data < -1) | (self.data > 100))
                or not math.isfinite(resolution) or resolution <= 0):
            raise ValueError('Invalid occupancy grid')
        self.resolution = float(resolution)
        self.height, self.width = self.data.shape
        self.blocked = (self.data < 0) | (self.data >= 50)
        self.radius = 0.

    def inflate(self, radius):
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError('Footprint radius must be positive')
        result = Grid(self.data.copy(), self.resolution, self.origin)
        result.radius = radius
        # Cell-centre separation minus both half diagonals bounds the closest
        # possible points in two cells. Entire accepted cells are collision free.
        cells = int(math.ceil(radius / self.resolution + math.sqrt(2)))
        for dy in range(-cells, cells + 1):
            for dx in range(-cells, cells + 1):
                if math.hypot(dx, dy) * self.resolution > radius + math.sqrt(2) * self.resolution:
                    continue
                sy0, sy1 = max(0, -dy), min(self.height, self.height - dy)
                sx0, sx1 = max(0, -dx), min(self.width, self.width - dx)
                if sy0 < sy1 and sx0 < sx1:
                    result.blocked[sy0+dy:sy1+dy, sx0+dx:sx1+dx] |= self.blocked[sy0:sy1, sx0:sx1]
        yy, xx = np.indices(self.data.shape)
        r = self.resolution
        result.blocked |= (xx*r < radius) | ((self.width-xx-1)*r < radius)
        result.blocked |= (yy*r < radius) | ((self.height-yy-1)*r < radius)
        return result

    def cell(self, xy):
        x, y = np.floor((np.asarray(xy) - self.origin) / self.resolution).astype(int)
        return int(x), int(y)

    def center(self, cell):
        return self.origin + (np.array(cell) + .5) * self.resolution

    def free(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height and not self.blocked[y, x]

    def segment_free(self, a, b):
        # Supercover: visit every crossed cell and both sides of a grid corner.
        a, b = np.asarray(a), np.asarray(b)
        delta = b - a
        cuts = [0., 1.]
        for axis in range(2):
            if abs(delta[axis]) < 1e-12:
                continue
            lo, hi = sorted(((a[axis]-self.origin[axis])/self.resolution,
                             (b[axis]-self.origin[axis])/self.resolution))
            for k in range(math.ceil(lo), math.floor(hi)+1):
                t = (self.origin[axis] + k*self.resolution-a[axis])/delta[axis]
                if 0 <= t <= 1:
                    cuts.append(t)
        cuts = sorted(set(cuts))
        for t in cuts + [(a+b)/2 for a, b in zip(cuts, cuts[1:])]:
            p = a + t * delta
            for dx, dy in ((0, 0), (-1e-9, -1e-9), (-1e-9, 1e-9), (1e-9, -1e-9), (1e-9, 1e-9)):
                if not self.free(self.cell(p + [dx, dy])):
                    return False
        return True

    def plan(self, start, goal):
        start = finite_vector(start, 2, 'start xy')
        goal = finite_vector(goal, 2, 'goal xy')
        first, last = self.cell(start), self.cell(goal)
        if not self.segment_free(start, self.center(first)) or not self.segment_free(self.center(last), goal):
            raise ValueError('Start or goal is outside the free whole-robot footprint area')
        queue, cost, parent = [(0., first)], {first: 0.}, {}
        while queue:
            _, current = heapq.heappop(queue)
            if current == last:
                break
            for dx, dy in itertools.product((-1, 0, 1), repeat=2):
                if dx == dy == 0:
                    continue
                nxt = (current[0]+dx, current[1]+dy)
                if not self.free(nxt):
                    continue
                if dx and dy and (not self.free((current[0]+dx, current[1]))
                                  or not self.free((current[0], current[1]+dy))):
                    continue
                candidate = cost[current] + math.hypot(dx, dy)
                if candidate < cost.get(nxt, math.inf):
                    cost[nxt], parent[nxt] = candidate, current
                    heapq.heappush(queue, (candidate + math.dist(nxt, last), nxt))
        if last not in cost:
            raise ValueError('No collision-free chassis route for this transport footprint')
        cells = [last]
        while cells[-1] != first:
            cells.append(parent[cells[-1]])
        points = [np.array(start)] + [self.center(c) for c in reversed(cells)] + [np.array(goal)]
        simplified, index = [points[0]], 0
        while index < len(points)-1:
            nxt = len(points)-1
            while nxt > index+1 and not self.segment_free(points[index], points[nxt]):
                nxt -= 1
            if not self.segment_free(points[index], points[nxt]):
                raise ValueError('Route crossed a blocked cell')
            simplified.append(points[nxt])
            index = nxt
        return simplified

    def to_dict(self):
        return dict(resolution=self.resolution, origin=self.origin.tolist(), data=self.data.tolist())


def kinematic_trace(points, start_yaw=0., goal_yaw=0., dt=.1, speed=.4, turn_rate=.7):
    if (not all(math.isfinite(v) for v in (start_yaw, goal_yaw, dt, speed, turn_rate))
            or not 0 < dt <= .5 or not 0 < speed <= 1 or not 0 < turn_rate <= 2):
        raise ValueError('Invalid planar kinematic limits')
    if len(points) < 2:
        raise ValueError('Route needs at least two points')
    points = [np.array(finite_vector(p, 2, 'waypoint')) for p in points]
    pose, samples = [*points[0], start_yaw], []

    def append(v=0., w=0.):
        t = round(len(samples)*dt, 8)
        samples.append(dict(time_s=t, pose_time_s=t, cmd_time_s=t, path_time_s=0.,
                            frame_id='map', pose=list(map(float, pose)),
                            twist=[v, 0., 0., 0., 0., w], path_end=points[-1].tolist()))

    def turn(yaw):
        delta = angle_delta(yaw, pose[2])
        n = max(1, math.ceil(abs(delta)/(turn_rate*dt)))
        for _ in range(n):
            pose[2] = angle_delta(pose[2]+delta/n, 0.)
            append(w=delta/n/dt)

    append()
    for target in points[1:]:
        start = np.array(pose[:2])
        delta = target-start
        if np.linalg.norm(delta) < 1e-10:
            continue
        turn(math.atan2(delta[1], delta[0]))
        n = max(1, math.ceil(np.linalg.norm(delta)/(speed*dt)))
        for i in range(1, n+1):
            pose[:2] = start + i/n*delta
            append(v=float(np.linalg.norm(delta)/n/dt))
    turn(goal_yaw)
    for _ in range(math.ceil(.8/dt)+1):
        append()
    return validate_trace(dict(schema=TRACE_SCHEMA, simulation_only=True, frame_id='map',
                               backend='standalone_astar', status='arrived',
                               goal_xy=points[-1].tolist(), samples=samples))
