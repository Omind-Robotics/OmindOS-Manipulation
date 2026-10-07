"""RM75-specific checks against the independent official modified-DH table."""
import copy
import gzip
import hashlib
import json
import math
import unittest
import numpy as np
import modeling.collision_model as cm
from community.pitch_robot import MobileDualPitch
from community.rm75_config import MODEL_DIR
from community.seven_axis_config import arm_config_dict, load_arm_config, validate_arm_config
from community.common import plan_path, seed_rng


def rot(axis, angle):
    v = np.eye(3)[axis]
    x, y, z = v
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*k + (1-np.cos(angle))*(k@k)


def rpy(angles):
    return rot(2, angles[2]) @ rot(1, angles[1]) @ rot(0, angles[0])


def mdh(q):
    """RM75-B official MDH, metres, exact right angles; no URDF/WRS dependency."""
    t = np.eye(4)
    for angle, alpha, d in zip(q, [0, -90, 90, -90, 90, -90, 90],
                              [.2405, 0, .256, 0, .210, 0, .144]):
        rx = rot(0, math.radians(alpha))
        a = np.eye(4)
        a[:3, :3] = rx @ rot(2, angle)
        a[:3, 3] = rx @ np.array([0, 0, d])
        t = t @ a
    return t


class RM75Tests(unittest.TestCase):
    def test_random_fk_matches_manufacturer_mdh_with_base_waist_and_mounts(self):
        r = MobileDualPitch(enable_cc=False)
        r.set_base([.12, -.2, .03], rpy([.1, -.2, .4]))
        rng = np.random.default_rng(75)
        for _ in range(20):
            q = rng.uniform(*r.get_jnt_ranges(r.component).T)
            r.fk(r.component, q)
            pitch = rot(1, q[0])
            pivot = np.array(r.waist_config.pivot_m)
            for side, values in [('left', q[1:8]), ('right', q[8:])]:
                cfg = r.arm_configs[side]
                root = np.eye(4)
                root[:3, :3] = r.rotmat @ pitch @ rpy(cfg.mount_rpy)
                root[:3, 3] = r.pos + r.rotmat @ (pivot + pitch @ (np.array(cfg.mount_m)-pivot))
                expected = root @ mdh(values)
                arm = r.lft_arm if side == 'left' else r.rgt_arm
                # Manufacturer URDF rounds ±pi/2 to ±1.5708; compare with exact MDH.
                np.testing.assert_allclose(arm.jnts[-1]['gl_posq'], expected[:3, 3], atol=1e-5)
                np.testing.assert_allclose(arm.jnts[-1]['gl_rotmatq'], expected[:3, :3], atol=3e-5)

    def test_zero_pose_reach_and_manufacturer_limits(self):
        r = MobileDualPitch(enable_cc=False)
        arm = r.lft_arm
        arm.fix_to(np.zeros(3), np.eye(3), np.zeros(7))
        np.testing.assert_allclose(arm.jnts[-1]['gl_posq'], [0, 0, .8505], atol=4e-6)
        # Published degree limits, with the source URDF's conservative radian rounding.
        np.testing.assert_allclose(np.rad2deg(np.asarray(arm.get_jnt_ranges())[:, 1]),
                                   [178, 130, 178, 135, 178, 128, 360], atol=.2)
        q = r.get_jnt_values(r.component)
        q[7], q[14] = 6.28, -6.28
        r.fk(r.component, q)
        bad = q.copy(); bad[14] = -6.29
        with self.assertRaises(ValueError):
            r.fk(r.component, bad)
        np.testing.assert_array_equal(r.get_jnt_values(r.component), q)

    def test_mesh_integrity_and_all_eight_links_in_each_collision_scene(self):
        manifest = json.loads((MODEL_DIR/'mesh-sha256.json').read_text())
        for name, digest in manifest.items():
            data = gzip.decompress((MODEL_DIR/'meshes'/(name+'.gz')).read_bytes())
            self.assertEqual(hashlib.sha256(data).hexdigest(), digest)
        r = MobileDualPitch()
        self.assertFalse(r.is_collided())
        for arm in (r.lft_arm, r.rgt_arm):
            for link in arm.lnks:
                center = link['collisionmodel'].objtrm.bounds.mean(axis=0)
                probe = cm.gen_box(extent=[.01]*3)
                probe.set_pos(link['gl_pos'] + link['gl_rotmat'] @ center)
                self.assertTrue(r.is_collided([probe]), link['name'])

    def test_every_joint_changes_tcp_and_other_arm_stays_fixed(self):
        r = MobileDualPitch(enable_cc=False)
        start = r.get_jnt_values(r.component)
        for own, other, indices in [('lft_arm', 'rgt_arm', range(1,8)),
                                     ('rgt_arm', 'lft_arm', range(8,15))]:
            r.fk(r.component, start)
            p0, r0 = r.get_gl_tcp(own)
            po, ro = r.get_gl_tcp(other)
            for index in indices:
                q = start.copy(); q[index] += .1
                r.fk(r.component, q)
                p1, r1 = r.get_gl_tcp(own)
                self.assertGreater(np.linalg.norm(p1-p0)+np.linalg.norm(r1-r0), .01)
                np.testing.assert_allclose(r.get_gl_tcp(other)[0], po, atol=1e-9)
                np.testing.assert_allclose(r.get_gl_tcp(other)[1], ro, atol=1e-9)

    def test_collision_free_planning_copy_and_ik(self):
        original = MobileDualPitch()
        r = original.copy()
        start = r.get_jnt_values(r.component)
        goal = start.copy()
        goal[0], goal[7], goal[14] = .1, .2, -.2
        seed_rng(7)
        path = plan_path(r, r.component, start, goal)
        self.assertTrue(all(len(q) == 15 for q in path))
        np.testing.assert_array_equal(original.get_jnt_values(r.component), start)
        target_q = start[1:8] + [.03, -.04, .02, .05, -.02, .04, .03]
        r.fk('lft_arm', target_q)
        pos, rotation = r.get_gl_tcp('lft_arm')
        solution = r.ik('lft_arm', pos, rotation, seed_jnt_values=start[1:8], local_minima='end')
        self.assertIsNotNone(solution)
        r.fk('lft_arm', solution)
        np.testing.assert_allclose(r.get_gl_tcp('lft_arm')[0], pos, atol=.003)
        np.testing.assert_allclose(r.get_gl_tcp('lft_arm')[1], rotation, atol=.01)

    def test_cross_arm_collision_and_folded_self_collision(self):
        data = arm_config_dict(load_arm_config())
        data['right'] = copy.deepcopy(data['left'])
        r = MobileDualPitch(arm_configs=validate_arm_config(data))
        self.assertTrue(r.is_collided())
        r = MobileDualPitch()
        r.fk('lft_arm', [0, 0, 0, 2.3, 0, 2.1, 0])
        self.assertTrue(r.is_collided())

    def test_attachment_tracks_waist_and_wrist_then_releases(self):
        r = MobileDualPitch()
        obj = cm.gen_box(extent=[.01]*3)
        obj.set_pos(r.get_gl_tcp('lft_arm')[0])
        r.hold('lft_arm', obj)
        q = r.get_jnt_values(r.component)
        q[0], q[7] = .1, .3
        r.fk(r.component, q)
        target = r.get_gl_tcp('lft_arm')[0]
        np.testing.assert_allclose(r.lft_oih_infos[0]['gl_pos'], target, atol=1e-9)
        r.release('lft_arm', obj)
        np.testing.assert_allclose(obj.get_pos(), target, atol=1e-6)
        self.assertFalse(r.is_collided())

    def test_config_roundtrip_rejects_unknown_variant_and_invalid_home(self):
        good = arm_config_dict(load_arm_config())
        self.assertEqual(arm_config_dict(validate_arm_config(good)), good)
        for field, value in [('model', 'RM75-6F'), ('home_rad', [0]*6),
                             ('home_rad', [9]*7), ('mount_rpy', [0, 0, float('nan')])]:
            bad = copy.deepcopy(good); bad['left'][field] = value
            with self.assertRaises(ValueError):
                validate_arm_config(bad)


if __name__ == '__main__':
    unittest.main()
