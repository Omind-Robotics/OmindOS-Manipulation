"""Numerical task-space contracts; no rendering or hardware dependencies."""
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from community.dual_task import (DualTaskPlanner, PlanningFailure, TrackingSettings,
                                 interpolate, inverse, pose_error, transform,
                                 plan_robot_object_motion)


def translation(x=0, y=0, z=0):
    t = np.eye(4); t[:3, 3] = [x, y, z]
    return t


def linear_fk(q):
    # Both TCP x positions depend on the SAME waist q[0].
    return [translation(q[0] + q[1], .2 + q[2], q[3]),
            translation(q[0] + q[4], -.2 + q[5], q[6])]


class CoupledTaskTests(unittest.TestCase):
    def planner(self, collision=lambda q: True, limits=None, settings=None):
        return DualTaskPlanner(linear_fk, limits if limits is not None else [[-1, 1]]*7,
                               collision, settings)

    def test_shared_waist_required_by_both_arms(self):
        limits = [[-1, 1]] + [[-.01, .01]]*6
        q = self.planner(limits=limits).solve([translation(.3, .2), translation(.3, -.2)], np.zeros(7))
        self.assertGreater(q[0], .28)
        for pose, target in zip(linear_fk(q), [translation(.3, .2), translation(.3, -.2)]):
            self.assertLess(pose_error(pose, target)[0], 1e-4)

    def test_rigid_path_and_relative_transform(self):
        grasps = [translation(0, .2), translation(0, -.2)]
        plan = self.planner().plan_object_motion(np.zeros(7),
                [np.eye(4), translation(.04, .02, .01)], grasps)
        self.assertGreater(plan['validated_samples'], 8)
        self.assertFalse(plan['timed'])
        self.assertFalse(plan['hardware_executed'])
        self.assertLess(plan['max_relative_position_error_m'], 1e-6)
        for q, obj in zip(plan['positions_rad'], plan['object_poses']):
            for actual, grasp in zip(linear_fk(q), grasps):
                self.assertLess(pose_error(actual, np.array(obj) @ grasp)[0], 1e-6)

    def test_collision_between_clear_endpoints_is_rejected(self):
        # A long object step makes an edge cross the thin blocked band.
        settings = TrackingSettings(translation_step_m=.1, joint_step_rad=.001)
        planner = self.planner(collision=lambda q: not (.018 < q[0]+q[1] < .022), settings=settings)
        with self.assertRaises(PlanningFailure):
            planner.plan_object_motion(np.zeros(7), [np.eye(4), translation(.04)],
                                       [translation(0, .2), translation(0, -.2)])

    def test_initial_collision_and_wrong_grasp_start_are_rejected(self):
        with self.assertRaises(PlanningFailure):
            self.planner(collision=lambda q: False).plan_object_motion(np.zeros(7), [np.eye(4)],
                                [translation(0, .2), translation(0, -.2)])
        with self.assertRaises(PlanningFailure):
            self.planner().plan_object_motion(np.zeros(7), [translation(.1)],
                                [translation(0, .2), translation(0, -.2)])

    def test_unreachable_goal_fails_with_no_partial_plan(self):
        with self.assertRaises(PlanningFailure):
            self.planner().solve([translation(9, .2), translation(9, -.2)], np.zeros(7))

    def test_rotation_at_pi_is_not_mistaken_for_zero(self):
        target = np.eye(4)
        target[:3, :3] = Rotation.from_rotvec([math.pi, 0, 0]).as_matrix()
        middle = interpolate(np.eye(4), target, .5)
        self.assertAlmostEqual(pose_error(np.eye(4), target)[1], math.pi)
        self.assertAlmostEqual(pose_error(np.eye(4), middle)[1], math.pi/2)
        with self.assertRaises(PlanningFailure):
            self.planner().solve([target, target], np.zeros(7))

    def test_bounds_nonfinite_and_bad_rotations_rejected(self):
        for bad in ([[0, 0]], [[1, -1]], [[0, float('inf')]]):
            with self.assertRaises(ValueError):
                self.planner(limits=bad)
        for bad in ([0]*6, [float('nan')]*7, [2]*7):
            with self.assertRaises(ValueError):
                self.planner().solve(linear_fk(np.zeros(7)), bad)
        bad = np.eye(4); bad[0, 0] = -1
        with self.assertRaises(ValueError):
            transform(bad)
        for field in ('joint_step_rad', 'ik_position_m', 'max_samples'):
            with self.assertRaises(ValueError):
                TrackingSettings(**{field: 0})
        with self.assertRaises(ValueError):
            self.planner(collision=None)
        with self.assertRaises(ValueError):
            self.planner(collision=lambda q: None).solve(linear_fk(np.zeros(7)), np.zeros(7))

    def test_sample_budget_rejects_large_requests(self):
        with self.assertRaises(PlanningFailure):
            self.planner(settings=TrackingSettings(max_samples=2)).plan_object_motion(np.zeros(7),
                [np.eye(4), translation(.04)], [translation(0, .2), translation(0, -.2)])

    def test_relative_grasp_constraint_stricter_than_individual_tcp_tolerance(self):
        grasps = [translation(0, .2005), translation(0, -.2005)]
        # Each TCP is within 1 mm of its individual target but their separation
        # is wrong by 1 mm; a 0.1 mm relative grasp constraint must reject it.
        with self.assertRaises(PlanningFailure):
            self.planner(settings=TrackingSettings(relative_position_m=.0001)).plan_object_motion(
                np.zeros(7), [np.eye(4)], grasps)

    def test_adapter_restores_robot_on_success_and_failure(self):
        class Robot:
            component = 'waist_and_arms'
            cc = object()
            pos = np.zeros(3)
            rotmat = np.eye(3)
            q = np.zeros(7)
            blocked = False
            def get_jnt_values(self, component): return self.q.copy()
            def get_jnt_ranges(self, component): return np.array([[-1, 1]]*7)
            def fk(self, component, q): self.q = np.array(q)
            def is_collided(self, obstacle_list): return self.blocked
            def get_gl_tcp(self, arm):
                t = linear_fk(self.q)[0 if arm == 'lft_arm' else 1]
                return t[:3, 3], t[:3, :3]
        r = Robot()
        # Adapter is explicitly for the 15-DoF assembly, not a generic robot.
        r.get_jnt_ranges = lambda component: np.array([[-1, 1]]*15)
        r.q = np.zeros(15)
        start = r.q.copy()
        plan_robot_object_motion(r, start, [np.eye(4), translation(.02)],
                                 [translation(0, .2), translation(0, -.2)])
        np.testing.assert_array_equal(r.q, start)
        r.blocked = True
        with self.assertRaises(PlanningFailure):
            plan_robot_object_motion(r, start, [np.eye(4)],
                                     [translation(0, .2), translation(0, -.2)])
        np.testing.assert_array_equal(r.q, start)


class ManufacturerKinematicsTests(unittest.TestCase):
    def test_pinned_rm75_15dof_coupled_ik_and_object_path(self):
        root = Path(__file__).resolve().parents[1]
        urdf = ET.parse(root/'community/models/realman_rm75/RM75-B.urdf').getroot()
        arms = json.loads((root/'configs/simulation/dual-rm75.reference.json').read_text())
        waist = json.loads((root/'configs/simulation/rm75-waist.reference.json').read_text())
        joints = [urdf.find(f"joint[@name='joint_{i}']") for i in range(1, 8)]
        limits = [waist['limits_rad']] + [[float(j.find('limit').get(k)) for k in ('lower', 'upper')]
                                               for j in joints]*2
        start = np.array([waist['home_rad'], *arms['left']['home_rad'], *arms['right']['home_rad']])

        def poses(q):
            pitch = Rotation.from_rotvec([0, q[0], 0]).as_matrix()
            pivot = np.array(waist['pivot_m'])
            result = []
            for side, values in [('left', q[1:8]), ('right', q[8:])]:
                t = np.eye(4)
                t[:3, :3] = pitch @ Rotation.from_euler('xyz', arms[side]['mount_rpy']).as_matrix()
                t[:3, 3] = pivot + pitch @ (np.array(arms[side]['mount_m']) - pivot)
                for value, joint in zip(values, joints):
                    origin = joint.find('origin'); step = np.eye(4)
                    rpy = list(map(float, origin.get('rpy').split()))
                    step[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
                    step[:3, 3] = list(map(float, origin.get('xyz').split()))
                    axis = np.array(list(map(float, joint.find('axis').get('xyz').split())))
                    step[:3, :3] = step[:3, :3] @ Rotation.from_rotvec(axis*value).as_matrix()
                    t = t @ step
                result.append(t)
            return result

        # This fixture independently computes pinned URDF FK; actual WRS collision
        # geometry and TCP offset are exercised by test_dual_task_rm75 in full CI.
        planner = DualTaskPlanner(poses, limits, lambda q: True)
        goal = start + np.array([.015, .01, -.01, .005, .01, -.01, .005, .01,
                                 -.01, -.01, -.005, .01, .01, .005, -.01])
        solution = planner.solve(poses(goal), start)
        self.assertEqual(len(solution), 15)
        for actual, target in zip(poses(solution), poses(goal)):
            p, r = pose_error(actual, target)
            self.assertLess(p, 1e-4); self.assertLess(r, .001)
        plan = planner.plan_object_motion(start, [np.eye(4), translation(.01, 0, .005)], poses(start))
        self.assertLess(plan['max_tcp_position_error_m'], .001)
        self.assertLess(plan['max_relative_position_error_m'], .002)
        self.assertGreater(plan['validated_samples'], 2)


if __name__ == '__main__':
    unittest.main()
