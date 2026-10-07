"""Geometry regressions for a waist that pitches above a stationary chassis."""
import math
import unittest
import numpy as np
import modeling.collision_model as cm
from community.legacy_pitch_robot import MobileDualPitch
from community.mobile_dual_arm import navigate
from community.interface import BaseGoal


def rotation_y(q):
    c, s = math.cos(q), math.sin(q)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class PitchTests(unittest.TestCase):
    def setUp(self):
        self.robot = MobileDualPitch()

    def test_both_tcps_rotate_about_configured_pivot_with_base_yaw(self):
        robot = self.robot
        yaw = .4
        base_r = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                           [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        robot.set_base([.4, -.2, 0], base_r)
        initial = {side: robot.get_gl_tcp(side) for side in ('lft_arm', 'rgt_arm')}
        chassis_pos = robot.chassis.lnks[0]['gl_pos'].copy()
        pivot = robot.pos + base_r @ np.array(robot.waist_config.pivot_m)
        rotation = base_r @ rotation_y(.2) @ base_r.T
        robot.set_waist_pitch(.2)
        for side, (pos, rot) in initial.items():
            new_pos, new_rot = robot.get_gl_tcp(side)
            np.testing.assert_allclose(new_pos, pivot + rotation @ (pos - pivot), atol=1e-9)
            np.testing.assert_allclose(new_rot, rotation @ rot, atol=1e-9)
        np.testing.assert_allclose(robot.chassis.lnks[0]['gl_pos'], chassis_pos)
        np.testing.assert_allclose(robot.chassis.lnks[0]['gl_rotmat'], base_r)

    def test_attachment_follows_waist_and_remains_in_collision_scene(self):
        robot = self.robot
        obj = cm.gen_box(extent=[.01, .01, .01])
        pos, rot = robot.get_gl_tcp('lft_arm')
        obj.set_pos(pos)
        obj.set_rotmat(rot)
        robot.hold('lft_arm', obj)
        robot.set_waist_pitch(.15)
        info = robot.lft_oih_infos[0]
        np.testing.assert_allclose(info['gl_pos'], robot.get_gl_tcp('lft_arm')[0], atol=1e-9)
        probe = cm.gen_box(extent=[.02, .02, .02])
        probe.set_pos(info['gl_pos'])
        self.assertTrue(robot.is_collided([probe]))
        robot.release('lft_arm', obj)
        self.assertEqual(robot.lft_oih_infos, [])

    def test_chassis_and_torso_obstacle_checks_follow_correct_frames(self):
        robot = self.robot
        self.assertFalse(robot.is_collided())
        chassis_obstacle = cm.gen_box(extent=[.02, .02, .02])
        chassis_obstacle.set_pos(robot.chassis.lnks[0]['gl_pos'])
        self.assertTrue(robot.is_collided([chassis_obstacle]))
        pivot = np.array(robot.waist_config.pivot_m)
        upper_point = np.array(robot.waist_config.torso_center_m) + [0, 0, .25]
        torso_obstacle = cm.gen_box(extent=[.01, .01, .01])
        torso_obstacle.set_pos(pivot + rotation_y(.2) @ (upper_point - pivot))
        self.assertFalse(robot.is_collided([torso_obstacle]))
        robot.set_waist_pitch(.2)
        self.assertFalse(robot.is_collided())
        self.assertTrue(robot.is_collided([chassis_obstacle]))
        self.assertTrue(robot.is_collided([torso_obstacle]))
        robot.set_waist_pitch(.3)
        self.assertTrue(robot.is_collided())  # Folded arms approach the reference chassis.

    def test_limits_reject_before_mutation_and_navigation_requires_transport_pose(self):
        robot = self.robot
        start = robot.get_jnt_values(robot.component)
        for angle in (float('nan'), .36, -.36):
            bad = start.copy()
            bad[0] = angle
            with self.assertRaises(ValueError):
                robot.fk(robot.component, bad)
            np.testing.assert_array_equal(robot.get_jnt_values(robot.component), start)
        robot.set_waist_pitch(.1)
        with self.assertRaises(RuntimeError):
            navigate(robot, BaseGoal(.1, 0, 0))

    def test_copy_moves_independently_and_rejects_arm_limits(self):
        robot = self.robot
        clone = robot.copy()
        q = clone.get_jnt_values(clone.component)
        q[0] = .2
        clone.fk(clone.component, q)
        self.assertEqual(robot.waist_pitch, 0.)
        self.assertFalse(clone.is_collided())
        self.assertEqual(clone.get_jnt_ranges(clone.component).shape, (13, 2))
        q[2] = 100
        with self.assertRaises(ValueError):
            clone.fk(clone.component, q)


if __name__ == '__main__':
    unittest.main()
