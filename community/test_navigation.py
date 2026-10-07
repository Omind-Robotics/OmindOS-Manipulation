"""Meaningful success/failure coverage for the mobile manipulation handoff."""
import copy
import json
import math
import unittest
from unittest.mock import patch
import numpy as np
from community.base_navigation import Grid, kinematic_trace
from community.navigation_contract import ArrivalGate, validate_trace


class NavigationContractTests(unittest.TestCase):
    def setUp(self):
        self.trace = kinematic_trace([[0., 0.], [1., 0.]])

    def test_arrival_and_all_twist_axes(self):
        validate_trace(self.trace)
        for axis in range(6):
            trace = copy.deepcopy(self.trace)
            for sample in trace['samples']:
                sample['twist'][axis] = .1
            with self.assertRaises(ValueError):
                validate_trace(trace)

    def test_wrong_or_pre_goal_path(self):
        for key, value in [('path_end', [3., 4.]), ('path_time_s', -1.)]:
            trace = copy.deepcopy(self.trace)
            for sample in trace['samples']:
                sample[key] = value
            with self.assertRaises(ValueError):
                validate_trace(trace)

    def test_stale_pose_and_command(self):
        for key in ('pose_time_s', 'cmd_time_s'):
            trace = copy.deepcopy(self.trace)
            for sample in trace['samples']:
                sample[key] = 0.
            with self.assertRaises(ValueError):
                validate_trace(trace)

    def test_no_settle_across_missing_observations(self):
        sample = copy.deepcopy(self.trace['samples'][-1])
        gate = ArrivalGate([1., 0.])
        for t in [0., 2., 2.1, 2.2]:
            sample.update(time_s=t, pose_time_s=t, cmd_time_s=t)
            self.assertFalse(gate.observe(sample))

    def test_motion_drift_and_frame_rejected(self):
        trace = copy.deepcopy(self.trace)
        for i, sample in enumerate(trace['samples'][-9:]):
            sample['pose'][2] = i*.018
        with self.assertRaises(ValueError):
            validate_trace(trace)
        for sample in self.trace['samples']:
            sample['frame_id'] = 'odom'
        with self.assertRaises(ValueError):
            validate_trace(self.trace)

    def test_actual_terminal_yaw_is_retained(self):
        trace = kinematic_trace([[0., 0.], [1., 0.]], goal_yaw=.8)
        self.assertAlmostEqual(validate_trace(trace)['samples'][-1]['pose'][2], .8)

    def test_corrupt_trace_rejected(self):
        for mutation in ('nan', 'gap', 'teleport', 'failed'):
            trace = copy.deepcopy(self.trace)
            if mutation == 'nan': trace['samples'][-1]['pose'][0] = math.nan
            if mutation == 'gap': trace['samples'] = trace['samples'][::10]
            if mutation == 'teleport': trace['samples'][3]['pose'][0] += 2.
            if mutation == 'failed': trace['status'] = 'failed'
            with self.assertRaises(ValueError):
                validate_trace(trace)


class GridTests(unittest.TestCase):
    def test_detour_and_whole_footprint(self):
        data = np.zeros((60, 60), dtype=int)
        data[23:37, 28:32] = 100
        grid = Grid(data, .1, [-3., -3.]).inflate(.5)
        self.assertFalse(grid.segment_free([-2., 0.], [2., 0.]))
        path = grid.plan([-2., 0.], [2., 0.])
        self.assertGreater(len(path), 2)
        for a, b in zip(path, path[1:]):
            self.assertTrue(grid.segment_free(a, b))
        self.assertFalse(grid.free(grid.cell([.4, 0.])))

    def test_unknown_wall_and_outside_goal(self):
        data = np.zeros((30, 30), dtype=int)
        data[:, 15] = -1
        grid = Grid(data, .1, [0, 0]).inflate(.2)
        for goal in ([2., 1.5], [4., 1.5]):
            with self.assertRaises(ValueError):
                grid.plan([.7, 1.5], goal)

    def test_narrow_chassis_only_passage_rejected(self):
        data = np.zeros((50, 50), dtype=int)
        data[:, 20:25] = 100
        data[20:30, 20:25] = 0
        grid = Grid(data, .1, [0, 0])
        grid.inflate(.2).plan([1., 2.5], [4., 2.5])
        with self.assertRaises(ValueError):
            grid.inflate(.79).plan([1., 2.5], [4., 2.5])

    def test_corner_cutting_rejected(self):
        grid = Grid([[0, 100, 0], [100, 0, 0], [0, 0, 0]], 1., [0, 0])
        self.assertFalse(grid.segment_free([.5, .5], [1.5, 1.5]))
        with self.assertRaises(ValueError):
            grid.plan([.5, .5], [2.5, 2.5])


class MobileTaskTests(unittest.TestCase):
    def test_full_task_preserves_transport_and_base_frame(self):
        from community.rm75_navigation import run_task, DEFAULT_SCENE, transport_radius, set_pose
        from community.common import seed_rng
        seed_rng(7)
        robot, _, plan = run_task(json.loads(DEFAULT_SCENE.read_text()))
        self.assertEqual(plan['phase_history'], ['idle', 'navigating', 'ready', 'manipulating', 'succeeded'])
        self.assertEqual(plan['dof'], 15)
        self.assertEqual(len(plan['transport_positions_rad']), 15)
        self.assertGreater(plan['validated_base_samples'], 100)
        self.assertTrue(np.allclose(robot.pos, [4.8, 1.5, 0.]))
        robot.fk(robot.component, plan['transport_positions_rad'])
        set_pose(robot, [0., 0., 0.])
        initial_tcp = robot.get_gl_tcp('lft_arm')[0]
        radius = transport_radius(robot)
        set_pose(robot, [2., 3., math.pi/2])
        self.assertAlmostEqual(transport_radius(robot), radius)
        self.assertTrue(np.allclose(robot.get_gl_tcp('lft_arm')[0], [2.-initial_tcp[1], 3.+initial_tcp[0], initial_tcp[2]]))

    def test_failed_navigation_never_calls_arm_planner(self):
        from community.rm75_navigation import run_task, DEFAULT_SCENE
        scene = json.loads(DEFAULT_SCENE.read_text())
        scene['goal'] = [1.8, 0., 0.]
        with patch('community.rm75_navigation.plan_path') as planner:
            with self.assertRaises(ValueError):
                run_task(scene)
            planner.assert_not_called()

    def test_imported_trace_requires_clearance(self):
        from community.rm75_navigation import run_task
        trace = kinematic_trace([[0., 0.], [1., 0.]])
        data = np.zeros((60, 60), dtype=int)
        data[30, 30] = 100
        trace['map'] = Grid(data, .1, [-3, -3]).to_dict()
        with patch('community.rm75_navigation.plan_path') as planner:
            with self.assertRaisesRegex(RuntimeError, 'clearance'):
                run_task(navigation_trace=trace)
            planner.assert_not_called()


if __name__ == '__main__':
    unittest.main()
