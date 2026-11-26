import math
import unittest

import numpy as np
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from welding_sim.joint_mapping import convert_trajectory, replay_branch, ur10_to_jaka
from welding_sim.profiles import JAKA, UR_JOINTS


class JointReplayTests(unittest.TestCase):
    def test_replay_copies_motion_and_derivatives_with_a_constant_mount_correction(self):
        start = [-1., 1.6, -1.3, 1.25, 1.57, 0.]
        finish = [-1., 1.8, -1.1, 1.35, 1.77, .2]
        actual = [-1., -1.6, 1.3, -1.25, -1.57, -3 * math.pi / 4]
        bounds = [(-math.tau, math.tau)] * 6
        offsets = replay_branch(start, actual, [start, finish], bounds)
        original = RobotTrajectory()
        path = original.joint_trajectory
        path.joint_names = list(reversed(JAKA.joints))
        path.points = [JointTrajectoryPoint(positions=list(reversed(start))),
                       JointTrajectoryPoint(positions=list(reversed(finish)), velocities=[.2]*6,
                                            accelerations=[.4]*6)]
        path.points[1].time_from_start.sec = 1
        converted = convert_trajectory(original, UR_JOINTS, offsets, 2.).joint_trajectory
        self.assertEqual(converted.joint_names, list(UR_JOINTS))
        np.testing.assert_allclose(ur10_to_jaka(converted.points[0].positions), start)
        np.testing.assert_allclose(ur10_to_jaka(converted.points[1].positions), finish)
        np.testing.assert_allclose(converted.points[1].velocities, [.1,-.1,-.1,-.1,.1,.1])
        np.testing.assert_allclose(converted.points[1].accelerations, [.1,-.1,-.1,-.1,.1,.1])
        self.assertEqual(converted.points[1].time_from_start.sec, 2)
        self.assertEqual(path.joint_names, list(reversed(JAKA.joints)))

    def test_single_revolution_branch_preserves_continuity_and_rejects_bounds(self):
        limits = [(-math.tau, math.tau)] * 6
        start = [0.] * 6
        finish = [0.,0.,0.,0.,0.,-5.]
        ur = [0.,0.,0.,0.,0.,-2.]
        offsets = replay_branch(start, ur, [start, finish], limits)
        self.assertTrue(all(-math.tau <= p + offsets[5] <= math.tau
                            for p in (start[5], finish[5])))
        with self.assertRaisesRegex(RuntimeError, 'elbow_joint'):
            replay_branch(start, ur, [start, [0.,0.,14.,0.,0.,0.]], limits)
