"""Optional topic adapter for the released omindos/navigation:0.1 simulator.

Run inside the Navigation runtime, then replay its JSON with rm75_navigation on
a normal Python host. No Nav2 Action, goal cancellation or hardware stop implied.
"""
import argparse
import json
import math
from pathlib import Path
import time
from community.navigation_contract import ArrivalGate, TRACE_SCHEMA, finite_vector, validate_trace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', nargs=2, type=float, default=[-1.6, -2.5], metavar=('X', 'Y'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=80.)
    parser.add_argument('--startup-timeout', type=float, default=30.)
    args = parser.parse_args()
    finite_vector(args.goal, 2, 'goal')
    if not all(math.isfinite(v) and v > 0 for v in (args.timeout, args.startup_timeout)):
        parser.error('timeouts must be finite and positive')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.unlink(missing_ok=True)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Path as NavPath, OccupancyGrid

    def stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec / 1e9

    def yaw_of(q):
        values = finite_vector([q.x, q.y, q.z, q.w], 4, 'quaternion')
        if abs(sum(v*v for v in values)-1.) > .001 or abs(q.x)+abs(q.y) > 1e-5:
            raise ValueError('Expected normalized planar orientation')
        return math.atan2(2*q.w*q.z, 1-2*q.z*q.z)

    class Collector(Node):
        def __init__(self):
            super().__init__('omindos_manipulation_navigation_adapter')
            self.started, self.goal_stamp = None, None
            self.pose, self.cmd, self.grid, self.path = None, None, None, None
            self.pose_time = self.cmd_time = self.path_time = -1.
            self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(PoseStamped, '/localization/pose', self.on_pose, 10)
            self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
            self.create_subscription(NavPath, '/plan', self.on_path, latched)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)

        def elapsed(self):
            return time.monotonic()-self.started if self.started is not None else -1.

        def on_pose(self, msg):
            age = self.get_clock().now().nanoseconds/1e9-stamp_seconds(msg.header.stamp)
            if msg.header.frame_id != 'map' or not -.05 <= age <= .5:
                return
            self.pose = finite_vector([msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation)], 3, 'pose')
            self.pose_time = self.elapsed()

        def on_cmd(self, msg):
            self.cmd = finite_vector([msg.linear.x, msg.linear.y, msg.linear.z,
                                      msg.angular.x, msg.angular.y, msg.angular.z], 6, 'twist')
            self.cmd_time = self.elapsed()

        def on_path(self, msg):
            if (self.started is None or msg.header.frame_id != 'map' or not msg.poses
                    or stamp_seconds(msg.header.stamp) < self.goal_stamp):
                return
            self.path = [finite_vector([p.pose.position.x, p.pose.position.y], 2, 'path point') for p in msg.poses]
            self.path_time = self.elapsed()

        def on_map(self, msg):
            if (msg.header.frame_id != 'map' or abs(yaw_of(msg.info.origin.orientation)) > 1e-6
                    or msg.info.width * msg.info.height != len(msg.data)
                    or msg.info.width < 2 or msg.info.height < 2 or len(msg.data) > 1000000
                    or not math.isfinite(msg.info.resolution) or msg.info.resolution <= 0):
                raise ValueError('Expected bounded, axis-aligned map occupancy grid')
            finite_vector([msg.info.origin.position.x, msg.info.origin.position.y], 2, 'map origin')
            if any(v < -1 or v > 100 for v in msg.data):
                raise ValueError('Invalid occupancy cell')
            w = msg.info.width
            self.grid = dict(resolution=msg.info.resolution,
                             origin=[msg.info.origin.position.x, msg.info.origin.position.y],
                             data=[list(msg.data[i:i+w]) for i in range(0, len(msg.data), w)])

    result = dict(schema=TRACE_SCHEMA, simulation_only=True, frame_id='map',
                  backend='omindos_navigation_0.1_topics', goal_xy=args.goal,
                  runtime='omindos/navigation:0.1', status='failed', samples=[])
    rclpy.init()
    node = Collector()
    try:
        startup = time.monotonic()
        while (node.pose is None or node.cmd is None or node.grid is None
               or node.goal_pub.get_subscription_count() < 1):
            if time.monotonic()-startup > args.startup_timeout:
                raise RuntimeError('Navigation runtime did not provide map/pose/cmd and a goal subscriber')
            rclpy.spin_once(node, timeout_sec=.05)
        node.started = time.monotonic()
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = node.get_clock().now().to_msg()
        node.goal_stamp = stamp_seconds(goal.header.stamp)
        goal.pose.position.x, goal.pose.position.y = args.goal
        goal.pose.orientation.w = 1.
        node.goal_pub.publish(goal)  # one task; do not repeatedly overwrite another goal
        gate, next_sample = ArrivalGate(args.goal), 0.
        while node.elapsed() < args.timeout:
            rclpy.spin_once(node, timeout_sec=.02)
            now = node.elapsed()
            if now < next_sample:
                continue
            next_sample = now+.1
            sample = dict(time_s=now, pose_time_s=node.pose_time, cmd_time_s=node.cmd_time,
                          path_time_s=node.path_time, frame_id='map', pose=node.pose[:], twist=node.cmd[:],
                          path_end=node.path[-1][:] if node.path else args.goal[:])
            result['samples'].append(sample)
            if gate.observe(sample):
                result.update(status='arrived', map=node.grid, final_path_xy=node.path)
                validate_trace(result)
                break
        else:
            raise RuntimeError('Arrival/stop gate timed out; no manipulation authorized')
    except Exception as exc:
        result.update(status='failed', error=str(exc))
        raise
    finally:
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(dict(status=result['status'], samples=len(result['samples']), output=str(args.output))))


if __name__ == '__main__':
    main()
