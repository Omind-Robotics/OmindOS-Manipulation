"""Check requested 15-joint endpoints, imported models and chassis stop gating."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from sdk.omindos_client import ManipulationClient
from community.rm75_config import MODEL_DIR, RM75Config
from community.pitch_robot import MobileDualPitch
from community.urdf_model import load_model

ROOT = Path(__file__).resolve().parents[1]


class CustomerTests(unittest.TestCase):
    def test_customer_object_motion_and_collision(self):
        with ManipulationClient() as client:
            meta = client.wait(client.get_mobile_model())
            self.assertEqual(meta['status'], 'succeeded', meta)
            start = meta['result']['home_positions_rad']
            grasps = meta['result']['home_tcp_poses']
            identity = np.eye(4)
            goal = identity.copy(); goal[:3, 3] = [.015, 0., .01]
            result = client.wait(client.plan_mobile_object_motion(
                start, [identity.tolist(), goal.tolist(), identity.tolist()], grasps), timeout=120)
            self.assertEqual(result['status'], 'succeeded', result)
            plan = result['result']
            self.assertEqual(plan['dof'], 15)
            self.assertFalse(plan['timed'])
            self.assertFalse(plan['hardware_executed'])
            self.assertGreater(len(plan['positions_rad']), 2)
            self.assertLess(plan['max_tcp_position_error_m'], .001)
            self.assertLess(plan['max_relative_position_error_m'], .002)
            blocked = client.wait(client.plan_mobile_object_motion(start, [identity.tolist()], grasps,
                obstacles=[dict(size_m=[3., 3., 3.], position_m=[0., 0., 1.])]))
            self.assertEqual(blocked['status'], 'failed', blocked)

    def test_custom_15_joint_path_and_fixed_base(self):
        with ManipulationClient() as client:
            meta = client.wait(client.get_mobile_model())
            self.assertEqual(meta['status'], 'succeeded', meta)
            start = meta['result']['home_positions_rad']
            goal = start.copy()
            goal[0], goal[7], goal[14] = .1, .2, -.2
            result = client.wait(client.plan_mobile_joint_motion(start, goal, base_pose=[.2, -.1, .3]))
            self.assertEqual(result['status'], 'succeeded', result)
            self.assertEqual(result['result']['base_pose'], [.2, -.1, .3])
            np.testing.assert_allclose(result['result']['positions_rad'][0], start)
            np.testing.assert_allclose(result['result']['positions_rad'][-1], goal)
            self.assertTrue(all(len(q) == 15 for q in result['result']['positions_rad']))
            blocked = client.wait(client.plan_mobile_joint_motion(start, goal, obstacles=[
                dict(size_m=[3., 3., 3.], position_m=[0., 0., 1.])]))
            self.assertEqual(blocked['status'], 'failed', blocked)

    def test_mobile_navigation_and_custom_target(self):
        with ManipulationClient() as client:
            meta = client.wait(client.get_mobile_model())
            goal = meta['result']['home_positions_rad']
            goal[0], goal[7], goal[14] = .1, .2, -.2
            scene = json.loads((ROOT/'configs/simulation/rm75-navigation.reference.json').read_text())
            result = client.wait(client.plan_mobile_task(scene=scene, target_positions=goal))
            self.assertEqual(result['status'], 'succeeded', result)
            plan = result['result']
            self.assertEqual(plan['dof'], 15)
            self.assertEqual(plan['phase_history'], ['idle', 'navigating', 'ready', 'manipulating', 'succeeded'])
            np.testing.assert_allclose(plan['manipulation_positions_rad'][-1], goal)
            bad = copy.deepcopy(plan['navigation'])
            bad['samples'][-1]['twist'][0] = .1
            failed = client.wait(client.plan_mobile_task(navigation_trace=bad, target_positions=goal))
            self.assertEqual(failed['status'], 'failed', failed)

    def test_external_urdf_changes_kinematics_and_limits(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'custom.urdf'
            tree = ET.parse(MODEL_DIR/'RM75-B.urdf')
            for mesh in tree.findall('.//mesh'):
                mesh.set('filename', str(MODEL_DIR/'meshes'/Path(mesh.get('filename')).name)+'.gz')
            first = tree.find("joint[@name='joint_1']")
            first.find('origin').set('xyz', '0 0 0.30')
            first.find('limit').set('upper', '2.0')
            tree.write(path)
            default = MobileDualPitch(enable_cc=False)
            config = default.arm_configs['left'].to_dict()
            config.update(model='URDF-7R', urdf_path=str(path))
            custom = MobileDualPitch(arm_configs=dict(default.arm_configs, left=RM75Config(**config)), enable_cc=False)
            self.assertAlmostEqual(custom.lft_arm.get_jnt_ranges()[0][1], 2.)
            self.assertGreater(np.linalg.norm(custom.get_gl_tcp('lft_arm')[0]-default.get_gl_tcp('lft_arm')[0]), .05)
            first.set('type', 'prismatic')
            tree.write(path)
            with self.assertRaises(ValueError):
                load_model(path)

    def test_invalid_shape_cannot_start_worker(self):
        with ManipulationClient() as client:
            with self.assertRaises(ValueError):
                client.plan_mobile_object_motion([0]*15, [np.eye(4).tolist()], [])
            with self.assertRaises(ValueError):
                client.plan_mobile_object_motion([0]*15, [[1, 2, 3]], [np.eye(4).tolist()]*2)
            with self.assertRaises(ValueError):
                client.plan_mobile_joint_motion([0]*14, [0]*15)
            with self.assertRaises(ValueError):
                client.plan_mobile_task()
            with self.assertRaises(ValueError):
                client.plan_mobile_task(scene={}, navigation_trace={})


if __name__ == '__main__':
    unittest.main()
