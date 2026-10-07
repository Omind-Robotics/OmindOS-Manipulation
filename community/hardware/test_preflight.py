"""Telemetry checks for commissioning. These tests do not validate hardware."""
import unittest

from community.interface import JOINTS
from community.hardware.ur3e_preflight import inspect_joint_state


class TelemetryTests(unittest.TestCase):
    def test_reordered_joints_preserve_identity(self):
        names = list(JOINTS)[::-1]
        result = inspect_joint_state(names, list(range(6)), [0.] * 6, 100., 100.2, JOINTS)
        self.assertEqual(result['positions_rad'], [5, 4, 3, 2, 1, 0])
        self.assertTrue(result['stationary_sample'])

    def test_missing_velocity_is_unknown(self):
        result = inspect_joint_state(list(JOINTS), [0.] * 6, [], 100., 100.2, JOINTS)
        self.assertIsNone(result['stationary_sample'])

    def test_moving_sample_is_not_stationary(self):
        result = inspect_joint_state(list(JOINTS), [0.] * 6, [.02] * 6, 100., 100.2, JOINTS)
        self.assertFalse(result['stationary_sample'])

    def test_stale_future_and_missing_timestamps_fail(self):
        for stamp in (98., 102., 0., float('nan')):
            with self.assertRaises(ValueError):
                inspect_joint_state(list(JOINTS), [0.] * 6, [], stamp, 100., JOINTS)

    def test_wrong_joint_mapping_fails(self):
        for names in (list(JOINTS)[:-1], ['wrong'] + list(JOINTS)[1:], [JOINTS[0]] * 6):
            with self.assertRaises(ValueError):
                inspect_joint_state(names, [0.] * len(names), [], 100., 100., JOINTS)

    def test_nonfinite_values_fail(self):
        for q, v in [([float('inf')] * 6, []), ([0.] * 6, [float('nan')] * 6), ([0.] * 6, [0.])]:
            with self.assertRaises(ValueError):
                inspect_joint_state(list(JOINTS), q, v, 100., 100., JOINTS)


if __name__ == '__main__':
    unittest.main()
