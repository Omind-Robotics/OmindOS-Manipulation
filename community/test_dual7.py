"""Behavior checks for real 7+7+1 kinematics, independent of ROS2."""
import copy
import math
import unittest
from pathlib import Path
import numpy as np
import modeling.collision_model as cm
from community.pitch_robot import MobileDualPitch
from community.seven_axis_config import load_arm_config, arm_config_dict, validate_arm_config
from community.common import plan_path, seed_rng

GENERIC_CONFIG = Path(__file__).resolve().parents[1] / 'configs/simulation/dual-seven-axis.reference.json'


def axis_rotation(axis, angle):
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*k + (1-math.cos(angle))*(k @ k)


class DualSevenTests(unittest.TestCase):
    def setUp(self):
        self.robot = MobileDualPitch(arm_configs=load_arm_config(GENERIC_CONFIG))

    def test_independent_fk_with_base_and_waist_for_both_seven_axis_chains(self):
        r = self.robot
        base_rotation = axis_rotation([0,0,1], .4)
        r.set_base([.2, -.1, .05], base_rotation)
        q = r.get_jnt_values(r.component)
        q += np.array([.12] + [.04, -.06, .03, .09, -.08, .05, .21]*2)
        r.fk(r.component, q)
        self.assertEqual(r.get_jnt_ranges(r.component).shape, (15, 2))
        for side, key, values in [('lft','left',q[1:8]), ('rgt','right',q[8:])]:
            cfg = r.arm_configs[key]
            waist_rotation = axis_rotation([0,1,0], q[0])
            pivot = np.array(r.waist_config.pivot_m)
            # The reference mount has no rotation. Compute each axis/translation independently.
            rotation = base_rotation @ waist_rotation
            pos = r.pos + base_rotation @ pivot + rotation @ (np.array(cfg.mount_m)-pivot)
            for axis, angle, length in zip(cfg.axes, values, cfg.lengths_m):
                rotation = rotation @ axis_rotation(axis, angle)
                pos += rotation @ np.array([length,0,0])
            rotation = rotation @ axis_rotation([0,1,0], math.pi/2)
            hand = getattr(r, side+'_hnd')
            pos += rotation @ hand.jaw_center_pos
            rotation = rotation @ hand.jaw_center_rotmat
            actual_pos, actual_rotation = r.get_gl_tcp(side+'_arm')
            np.testing.assert_allclose(actual_pos, pos, atol=1e-9)
            np.testing.assert_allclose(actual_rotation, rotation, atol=1e-9)
        np.testing.assert_allclose(r.chassis.rotmat, base_rotation)

    def test_every_arm_joint_changes_tcp_and_does_not_move_other_arm(self):
        r = self.robot
        start = r.get_jnt_values(r.component)
        for side, indices, other in [('lft',range(1,8),'rgt'), ('rgt',range(8,15),'lft')]:
            r.fk(r.component, start)
            p0, rot0 = r.get_gl_tcp(side+'_arm')
            other_pose = r.get_gl_tcp(other+'_arm')
            for index in indices:
                q = start.copy()
                q[index] += .13
                r.fk(r.component, q)
                p, rot = r.get_gl_tcp(side+'_arm')
                self.assertGreater(np.linalg.norm(p-p0) + np.linalg.norm(rot-rot0), .01)
                p_other, rot_other = r.get_gl_tcp(other+'_arm')
                np.testing.assert_allclose(p_other, other_pose[0], atol=1e-9)
                np.testing.assert_allclose(rot_other, other_pose[1], atol=1e-9)

    def test_invalid_dimensions_nonfinite_and_joint_limits_are_atomic(self):
        r = self.robot
        start = r.get_jnt_values(r.component)
        bad_inputs = [start[:-1], np.r_[start,0], [0.]*13]
        for index, value in [(0,.36), (7,9), (14,9), (3,float('nan')), (8,True)]:
            q = list(start)
            q[index] = value
            bad_inputs.append(q)
        for q in bad_inputs:
            with self.assertRaises(ValueError):
                r.fk(r.component, q)
            np.testing.assert_array_equal(r.get_jnt_values(r.component), start)
        self.assertEqual(len(r.get_jnt_values('both_arm')),14)
        self.assertEqual(len(r.get_jnt_values('lft_arm')),7)

    def test_both_seventh_links_and_body_are_in_collision_scene(self):
        r = self.robot
        self.assertFalse(r.is_collided())
        for link in [r.chassis.lnks[0], r.torso.lnks[0], r.lft_arm.lnks[7], r.rgt_arm.lnks[7]]:
            obstacle = cm.gen_box(extent=[.01]*3)
            obstacle.set_pos(link['gl_pos'])
            self.assertTrue(r.is_collided([obstacle]))

    def test_cross_arm_collision_and_nonadjacent_self_collision(self):
        data = arm_config_dict(load_arm_config(GENERIC_CONFIG))
        for side in ('left','right'):
            data[side]['mount_m'] = [.6, .38 if side=='left' else -.38, 1.38]
        r = MobileDualPitch(arm_configs=validate_arm_config(data))
        self.assertFalse(r.is_collided())
        q = np.zeros(15)
        q[1], q[8] = -math.pi/2, math.pi/2
        r.fk(r.component,q)
        self.assertTrue(r.is_collided())
        q = np.zeros(15)
        q[4], q[6] = 2.4, 1.6
        r.fk(r.component,q)
        self.assertTrue(r.is_collided())

    def test_attachment_follows_seventh_joint_and_waist_and_releases_at_tcp(self):
        r = self.robot
        obj = cm.gen_box(extent=[.01]*3)
        pos, rot = r.get_gl_tcp('lft_arm')
        obj.set_pos(pos + rot @ np.array([.025,0,0]))
        obj.set_rotmat(rot)
        r.hold('lft_arm',obj)
        q = r.get_jnt_values(r.component)
        q[0],q[7] = .1,.3
        r.fk(r.component,q)
        pos,rot = r.get_gl_tcp('lft_arm')
        target = pos + rot @ np.array([.025,0,0])
        np.testing.assert_allclose(r.lft_oih_infos[0]['gl_pos'],target,atol=1e-9)
        probe = cm.gen_box(extent=[.01]*3)
        probe.set_pos(target)
        self.assertTrue(r.is_collided([probe]))
        r.release('lft_arm',obj)
        np.testing.assert_allclose(obj.get_pos(),target,atol=1e-6)
        self.assertEqual(r.lft_oih_infos,[])

    def test_copy_ik_and_planning_use_seven_axis_dimensions(self):
        r = self.robot
        clone = r.copy()
        start = clone.get_jnt_values(clone.component)
        goal = start.copy()
        goal[0] = .1
        goal[7],goal[14] = .2,-.2
        seed_rng(7)
        samples = plan_path(clone, clone.component, start, goal)
        self.assertTrue(all(len(q)==15 for q in samples))
        np.testing.assert_array_equal(r.get_jnt_values(r.component),start)
        target_q = r.get_jnt_values('lft_arm') + [.03,-.04,.02,.05,-.02,.04,.03]
        r.fk('lft_arm',target_q)
        pos,rot = r.get_gl_tcp('lft_arm')
        r.fk('lft_arm',start[1:8])
        solution = r.ik('lft_arm',pos,rot,seed_jnt_values=start[1:8],local_minima='end')
        self.assertIsNotNone(solution)
        self.assertEqual(len(solution),7)
        r.fk('lft_arm',solution)
        actual_pos,actual_rot = r.get_gl_tcp('lft_arm')
        np.testing.assert_allclose(actual_pos,pos,atol=.003)
        np.testing.assert_allclose(actual_rot,rot,atol=.01)

    def test_repeated_release_preserves_remaining_object_and_collision_pairs(self):
        r = self.robot
        right = cm.gen_box(extent=[.01]*3)
        right.set_pos(r.get_gl_tcp('rgt_arm')[0])
        r.hold('rgt_arm', right)
        left = cm.gen_box(extent=[.01]*3)
        for _ in range(12):  # More than the spare collision bit masks without recycling.
            left.set_pos(r.get_gl_tcp('lft_arm')[0])
            r.hold('lft_arm', left)
            r.release('lft_arm', left)
            self.assertEqual(len(r.rgt_oih_infos), 1)
            self.assertFalse(r.is_collided())
            probe = cm.gen_box(extent=[.01]*3)
            probe.set_pos(r.rgt_oih_infos[0]['gl_pos'])
            self.assertTrue(r.is_collided([probe]))
        r.release('rgt_arm', right)
        self.assertEqual(r.rgt_oih_infos, [])

    def test_configuration_rejects_invalid_axes_dimensions_and_limits(self):
        good = arm_config_dict(load_arm_config(GENERIC_CONFIG))
        for field, value in [('lengths_m',[.1]*6),('axes',[[0,0,0]]*7),
                             ('limits_rad',[[1,-1]]*7),('link_width_m',-.1),
                             ('mount_m',[0,float('nan'),0]),('home_rad',[9]*7)]:
            bad = copy.deepcopy(good)
            bad['left'][field] = value
            with self.subTest(field=field),self.assertRaises(ValueError):
                validate_arm_config(bad)


if __name__ == '__main__':
    unittest.main()
