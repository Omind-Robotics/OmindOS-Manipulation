"""Read-only ROS2 observations for UR3e commissioning; never send motion goals.

Run with the sourced ROS2 Humble system Python, outside the simulation venv.
An observed controller and joint stream do not identify or certify real hardware.
"""
import argparse
import json
import math
from pathlib import Path
import time

from community.interface import JOINTS


def inspect_joint_state(names, positions, velocities, stamp_s, now_s, expected, max_age=1.0):
    """Reorder by names and reject stale/malformed telemetry, without commanding it."""
    if len(set(names)) != len(names) or len(names) != len(positions):
        raise ValueError('Joint names must be unique and match positions')
    if len(set(expected)) != 6 or not all(name in names for name in expected):
        raise ValueError('Missing one or more expected UR3e joints')
    if not all(math.isfinite(v) for v in (stamp_s, now_s)) or stamp_s <= 0:
        raise ValueError('Joint timestamp is missing or nonfinite')
    age = now_s - stamp_s
    if age < -.1 or age > max_age:
        raise ValueError('Joint timestamp is stale or ahead of this ROS clock')
    indexes = [names.index(name) for name in expected]
    q = [positions[i] for i in indexes]
    if not all(math.isfinite(v) for v in q):
        raise ValueError('Nonfinite joint position')
    velocity = None
    if velocities:
        if len(velocities) != len(names):
            raise ValueError('Velocity array does not match joint names')
        velocity = [velocities[i] for i in indexes]
        if not all(math.isfinite(v) for v in velocity):
            raise ValueError('Nonfinite joint velocity')
    return dict(joint_names=list(expected), positions_rad=q, velocities_rad_s=velocity,
                stamp_ros_s=stamp_s, source_age_s=age,
                stationary_sample=None if velocity is None else max(abs(v) for v in velocity) < .001)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint-topic', default='/joint_states')
    parser.add_argument('--joint-prefix', default='')
    parser.add_argument('--arm-action', default='/scaled_joint_trajectory_controller/follow_joint_trajectory')
    parser.add_argument('--controller-manager', default='/controller_manager')
    parser.add_argument('--controller', default='scaled_joint_trajectory_controller')
    parser.add_argument('--timeout', type=float, default=15)
    parser.add_argument('--output', type=Path, default=Path('community-results/hardware/ur3e-observation.json'))
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from control_msgs.action import FollowJointTrajectory
        from controller_manager_msgs.srv import ListControllers
    except ImportError as exc:
        parser.exit(2, f'ROS2 dependency unavailable: {exc}. Source ROS2 Humble and use its system Python.\n')

    expected = [args.joint_prefix + joint for joint in JOINTS]
    rclpy.init()
    node = Node('omindos_ur3e_observer')
    latest = dict(sample=None, received=0., error='No joint message received',
                  distinct_samples=0, last_stamp=None)
    controllers = None

    def observe(message):
        try:
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            sample = inspect_joint_state(list(message.name), list(message.position), list(message.velocity),
                stamp, node.get_clock().now().nanoseconds / 1e9, expected)
            if latest['last_stamp'] != stamp:
                latest['distinct_samples'] += 1
            latest.update(sample=sample, received=time.monotonic(), error=None, last_stamp=stamp)
        except (ValueError, TypeError) as exc:
            latest.update(sample=None, error=str(exc), distinct_samples=0, last_stamp=None)

    node.create_subscription(JointState, args.joint_topic, observe, qos_profile_sensor_data)
    # Discovery only: no send_goal, trajectory publication, driver startup or dashboard control.
    arm = ActionClient(node, FollowJointTrajectory, args.arm_action)
    listing = node.create_client(ListControllers, args.controller_manager.rstrip('/') + '/list_controllers')
    future = None
    interrupted = False
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=.1)
            if future is None and listing.service_is_ready():
                future = listing.call_async(ListControllers.Request())
            if future is not None and future.done() and controllers is None:
                try:
                    controllers = [dict(name=c.name, state=c.state, type=c.type)
                                   for c in future.result().controller]
                except Exception:
                    controllers = []
            # Observe several timestamps; a latched old packet cannot satisfy this.
            active = any(c['name'] == args.controller and c['state'] == 'active' for c in controllers or [])
            if latest['distinct_samples'] >= 3 and latest['sample'] and active and arm.server_is_ready():
                break
    except KeyboardInterrupt:
        interrupted = True
    finally:
        sample = latest['sample']
        fresh = bool(sample and time.monotonic() - latest['received'] <= 1.0 and
                     -.1 <= node.get_clock().now().nanoseconds / 1e9 - sample['stamp_ros_s'] <= 1.0)
        active = any(c['name'] == args.controller and c['state'] == 'active' for c in controllers or [])
        action_found = arm.server_is_ready()
        observed = fresh and latest['distinct_samples'] >= 3 and active and action_found and not interrupted
        report = dict(schema='omindos.ur3e.observation.v0.1', status='observed' if observed else 'incomplete',
            read_only=True, physical_robot_verified=False, ready_for_motion=False,
            joint_topic=args.joint_topic, joint_prefix=args.joint_prefix, arm_action=args.arm_action,
            action_server_discovered=action_found, controller_active=active, controllers=controllers,
            fresh_joint_stream=fresh, distinct_joint_samples=latest['distinct_samples'],
            sample=sample, telemetry_error=latest['error'], interrupted=interrupted,
            gripper='not checked; model and transport must be confirmed onsite')
        arm.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    payload = json.dumps(report, indent=2, allow_nan=False)
    print(payload)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(payload + '\n', encoding='utf-8')
    return 0 if observed else 1


if __name__ == '__main__':
    raise SystemExit(main())
