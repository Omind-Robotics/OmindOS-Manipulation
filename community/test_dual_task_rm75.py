"""Full WRS model/collision integration; run with requirements-community.txt."""
import unittest
import numpy as np
import modeling.collision_model as cm
from community.dual_task import PlanningFailure, plan_robot_object_motion
from community.dual_task_motion import run_task
from community.pitch_robot import MobileDualPitch


class RM75TaskIntegrationTests(unittest.TestCase):
    def test_real_model_task_path_and_state_restoration(self):
        robot, plan = run_task()
        start = robot.get_jnt_values(robot.component).copy()
        self.assertEqual(len(plan['joint_names']), 15)
        self.assertLess(plan['max_tcp_position_error_m'], .001)
        self.assertLess(plan['max_tcp_rotation_error_rad'], .01)
        for q in plan['positions_rad']:
            robot.fk(robot.component, q)
            self.assertFalse(robot.is_collided())
        robot.fk(robot.component, start)
        plan_robot_object_motion(robot, start, plan['object_poses'][:1], plan['object_to_tcp'])
        np.testing.assert_array_equal(robot.get_jnt_values(robot.component), start)

    def test_environment_obstacle_rejects_task_and_restores(self):
        robot = MobileDualPitch()
        start = robot.get_jnt_values(robot.component).copy()
        grasps = []
        for name in ('lft_arm', 'rgt_arm'):
            p, r = robot.get_gl_tcp(name)
            t = np.eye(4); t[:3, 3], t[:3, :3] = p, r
            grasps.append(t)
        obstacle = cm.gen_box(extent=[2, 2, 3])
        obstacle.set_pos([0, 0, 1])
        with self.assertRaises(PlanningFailure):
            plan_robot_object_motion(robot, start, [np.eye(4)], grasps, obstacles=[obstacle])
        np.testing.assert_array_equal(robot.get_jnt_values(robot.component), start)


if __name__ == '__main__':
    unittest.main()
