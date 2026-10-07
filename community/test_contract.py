"""Regression checks for failure handling and independently sampled paths."""
import unittest
import copy
from community.interface import (BaseGoal, TaskGate, Phase, joint_names, validate_trajectory,
                                 trajectory_joint_names, validate_playback)
from community.waist_config import WaistConfig, load_waist_config


class ContractTests(unittest.TestCase):
    def test_waist_playback_checks_layout_schema_limits_and_transport_pose(self):
        plan = dict(schema='omindos.community.playback.waist_pitch.v1', simulation_only=True,
                    base_goal=dict(x=0, y=0, yaw=0), waist_config=load_waist_config().to_dict(),
                    joint_names=trajectory_joint_names(True),
                    points=[dict(time_s=0, positions=[0.] * 13),
                            dict(time_s=1, positions=[.1] * 13)])
        self.assertIs(validate_playback(plan), plan)
        for mutation in ('schema', 'order', 'length', 'limit', 'start', 'simulation'):
            bad = copy.deepcopy(plan)
            if mutation == 'schema':
                bad['schema'] = 'omindos.community.playback.v0.1'
            elif mutation == 'order':
                bad['joint_names'] = bad['joint_names'][1:] + bad['joint_names'][:1]
            elif mutation == 'length':
                bad['points'][1]['positions'].pop()
            elif mutation == 'limit':
                bad['points'][1]['positions'][0] = .5
            elif mutation == 'start':
                bad['points'][0]['positions'][0] = .1
            else:
                bad['simulation_only'] = False
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                validate_playback(bad)

    def test_waist_config_rejects_invalid_geometry_and_limits(self):
        for key, value in [('limits_rad', [1, -1]), ('pivot_m', [0, float('nan'), 0]),
                           ('chassis_size_m', [1, 0, 1]), ('home_rad', .5),
                           ('transport_rad', False), ('simulation_only', False)]:
            config = load_waist_config().to_dict()
            config[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                WaistConfig(**config)

    def test_success_order(self):
        task = TaskGate()
        task.start_navigation()
        task.navigation_result(True, True)
        task.start_manipulation()
        task.manipulation_result(True)
        self.assertEqual(task.phase, Phase.SUCCEEDED)

    def test_failed_or_moving_base_blocks_arm(self):
        for success, stopped in [(False, True), (True, False), (False, False)]:
            task = TaskGate()
            task.start_navigation()
            task.navigation_result(success, stopped)
            with self.assertRaises(RuntimeError):
                task.start_manipulation()

    def test_cancel_blocks_late_result(self):
        task = TaskGate()
        task.start_navigation()
        task.cancel()
        with self.assertRaises(RuntimeError):
            task.navigation_result(True, True)
        with self.assertRaises(RuntimeError):
            task.start_manipulation()

    def test_goal_rejects_invalid_frame_and_nan(self):
        for args in [(0, 0, 0, 'odom'), (float('nan'), 0, 0, 'map')]:
            with self.assertRaises(ValueError):
                BaseGoal(*args)

    def test_trajectory_rejects_reordered_joints_nan_and_time_reversal(self):
        names = joint_names('left') + joint_names('right')
        good = [dict(time_s=0, positions=[0] * 12), dict(time_s=1, positions=[.1] * 12)]
        self.assertEqual(validate_trajectory(names, good), good)
        for n, points in [(names[::-1], good), (names, good[::-1]),
                          (names, [dict(time_s=0, positions=[float('nan')] * 12)]),
                          (names, [dict(time_s=0, positions=[0] * 6)]), (names, [])]:
            with self.assertRaises(ValueError):
                validate_trajectory(n, points)


if __name__ == '__main__':
    unittest.main()
