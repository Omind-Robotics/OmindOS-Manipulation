"""Exercise the observer against ROS2 fixtures, recording any motion requests."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers

from community.interface import JOINTS


class ReadOnlyRosTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('omindos_observer_test_fixture')
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.motion_requests = 0
        self.stale = False
        self.publisher = self.node.create_publisher(JointState, '/observer_test/joint_states', 10)

        def publish():
            message = JointState()
            message.header.stamp = self.node.get_clock().now().to_msg()
            if self.stale:
                message.header.stamp.sec -= 60
            message.name = list(JOINTS)
            message.position = [0., -.7, .9, .3, .6, 0.]
            message.velocity = [0.] * 6
            self.publisher.publish(message)

        def reject_motion(_):
            self.motion_requests += 1
            return GoalResponse.REJECT

        def list_controllers(request, response):
            controller = ControllerState()
            controller.name = 'observer_test_controller'
            controller.state = 'active'
            controller.type = 'test/FixtureOnly'
            response.controller = [controller]
            return response

        self.timer = self.node.create_timer(.05, publish)
        self.service = self.node.create_service(ListControllers, '/observer_test/list_controllers', list_controllers)
        self.action = ActionServer(self.node, FollowJointTrajectory, '/observer_test/follow_joint_trajectory',
            execute_callback=lambda _: FollowJointTrajectory.Result(), goal_callback=reject_motion)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.executor.shutdown()
        self.thread.join(timeout=3)
        self.action.destroy()
        self.node.destroy_node()
        rclpy.shutdown()

    def run_observer(self, timeout):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'observed.json'
            run = subprocess.run([sys.executable, '-m', 'community.hardware.ur3e_preflight',
                '--joint-topic', '/observer_test/joint_states',
                '--arm-action', '/observer_test/follow_joint_trajectory',
                '--controller-manager', '/observer_test', '--controller', 'observer_test_controller',
                '--timeout', str(timeout), '--output', str(output)],
                cwd=Path(__file__).resolve().parents[2], capture_output=True, text=True, timeout=timeout+10)
            self.assertTrue(output.exists(), run.stdout + run.stderr)
            return run.returncode, json.loads(output.read_text())

    def test_discovery_and_telemetry_never_send_motion(self):
        code, report = self.run_observer(10)
        self.assertEqual(code, 0, report)
        self.assertEqual(report['status'], 'observed')
        self.assertTrue(report['read_only'])
        self.assertFalse(report['ready_for_motion'])
        self.assertFalse(report['physical_robot_verified'])
        self.assertEqual(self.motion_requests, 0)

    def test_stale_telemetry_is_not_accepted(self):
        self.stale = True
        code, report = self.run_observer(4)
        self.assertEqual(code, 1, report)
        self.assertEqual(report['status'], 'incomplete')
        self.assertFalse(report['fresh_joint_stream'])
        self.assertEqual(self.motion_requests, 0)


if __name__ == '__main__':
    unittest.main()
