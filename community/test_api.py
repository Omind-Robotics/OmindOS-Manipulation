"""Public API checks: real planning, failure propagation, cancellation and cleanup."""
import unittest

from community.api import ManipulationClient

START = [0., -.7, .9, .3, .6, 0.]
GOAL = [.45, -.8, 1.1, .45, .8, -.15]


class ApiTests(unittest.TestCase):
    def test_custom_motion_returns_requested_endpoints_and_dense_path(self):
        with ManipulationClient() as client:
            task = client.plan_joint_motion(START, GOAL, obstacles=[
                dict(size_m=[.12, .12, .3], position_m=[.25, -.25, .15])])
            status = client.wait(task)
            self.assertEqual(status['status'], 'succeeded', status)
            points = status['result']['positions_rad']
            self.assertEqual(points[0], START)
            self.assertEqual(points[-1], GOAL)
            self.assertGreater(len(points), 2)
            for a, b in zip(points, points[1:]):
                self.assertLessEqual(max(abs(x-y) for x, y in zip(a, b)), .025 + 1e-9)
            self.assertEqual(client.stop(task)['status'], 'succeeded')

    def test_blocked_scene_cannot_report_success(self):
        with ManipulationClient() as client:
            task = client.plan_joint_motion(START, GOAL, obstacles=[
                dict(size_m=[2, 2, 2], position_m=[0, 0, 0])])
            status = client.wait(task)
            self.assertEqual(status['status'], 'failed', status)
            self.assertIsNone(status['result'])
            self.assertEqual(status['error']['code'], 'worker_failed')

    def test_joint_limits_fail_before_planning(self):
        with ManipulationClient() as client:
            status = client.wait(client.plan_joint_motion(START, [100] * 6))
            self.assertEqual(status['status'], 'failed', status)
            self.assertIn('joint limits', status['error']['details'])

    def test_custom_box_grasps_return_poses(self):
        with ManipulationClient() as client:
            status = client.wait(client.plan_grasps([.02, .06, .04], jaw_width_m=.065))
            self.assertEqual(status['status'], 'succeeded', status)
            result = status['result']
            self.assertGreater(result['candidate_count'], 0)
            self.assertEqual(result['frame_id'], 'object')
            for grasp in result['candidates']:
                self.assertEqual(grasp['jaw_width_m'], .065)
                self.assertEqual(len(grasp['jaw_center_rotation']), 3)

    def test_invalid_parameters_are_rejected_before_submission(self):
        with ManipulationClient() as client:
            for goal in ([0] * 5, [float('nan')] * 6, ['0'] * 6, [True] * 6):
                with self.assertRaises(ValueError):
                    client.plan_joint_motion(START, goal)
            for size, width in [([0, .1, .1], .065), ([.02, .06, .04], .1)]:
                with self.assertRaises(ValueError):
                    client.plan_grasps(size, jaw_width_m=width)
            with self.assertRaises(ValueError):
                client.run_demo('../anything')

    def test_cancel_reaps_worker_and_is_terminal(self):
        with ManipulationClient() as client:
            task = client.run_demo('integrated_task')
            self.assertEqual(client.stop(task)['status'], 'canceled')
            self.assertEqual(client.wait(task)['status'], 'canceled')
            self.assertIsNotNone(client._tasks[task]['process'].poll())

    def test_timeout_terminates_worker(self):
        with ManipulationClient() as client:
            task = client.run_demo('integrated_task')
            status = client.wait(task, timeout=.000001)
            self.assertEqual(status['status'], 'failed')
            self.assertEqual(status['error']['code'], 'timeout')
            self.assertIsNotNone(client._tasks[task]['process'].poll())

    def test_close_stops_work_and_disallows_new_tasks(self):
        client = ManipulationClient()
        try:
            task = client.run_demo('integrated_task')
            client.close()
            self.assertEqual(client.get_status(task)['status'], 'canceled')
            with self.assertRaises(RuntimeError):
                client.run_demo('quick_start')
        finally:
            client.close()


if __name__ == '__main__':
    unittest.main()
