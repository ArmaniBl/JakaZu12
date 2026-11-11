"""Feedback freshness and pre-dispatch validation without sending action goals."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from moveit_msgs.msg import RobotState, RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from welding_sim.runner import WeldingRunner


def state(position):
    return RobotState(joint_state=JointState(name=['wrist_1_joint'], position=[float(position)]))


def trajectory(position):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = ['wrist_1_joint']
    result.joint_trajectory.points = [JointTrajectoryPoint(positions=[float(position)])]
    return result


class ExecutionStartTests(unittest.TestCase):
    def test_fresh_state_waits_for_feedback_after_call(self):
        stub = SimpleNamespace(latest_state=state(0).joint_state, received_at=9.9,
                               initial_scene=SimpleNamespace(robot_state=RobotState()))
        def receive(*args, **kwargs):
            stub.latest_state = state(.1).joint_state
            stub.received_at = 10.1
        with patch('welding_sim.runner.time.monotonic', return_value=10), \
                patch('welding_sim.runner.rclpy.spin_once', side_effect=receive) as spin:
            actual = WeldingRunner.fresh_state(stub)
        self.assertEqual(actual.joint_state.position[0], .1)
        spin.assert_called_once()

    def test_waits_until_positions_stop_drifting(self):
        clock = [0.0]
        def fresh():
            clock[0] += .1
            return state(min(clock[0], .5)*.2)
        stub = SimpleNamespace(fresh_state=fresh)
        with patch('welding_sim.runner.time.monotonic', side_effect=lambda: clock[0]):
            result = WeldingRunner.settled_state(stub)
        self.assertGreaterEqual(clock[0], .8)
        self.assertAlmostEqual(result.joint_state.position[0], .1)

    def test_continuing_motion_times_out(self):
        clock = [0.0]
        def fresh():
            clock[0] += .1
            return state(clock[0])
        with patch('welding_sim.runner.time.monotonic', side_effect=lambda: clock[0]):
            with self.assertRaisesRegex(RuntimeError, 'did not settle'):
                WeldingRunner.settled_state(SimpleNamespace(fresh_state=fresh), timeout=.8)

    def test_start_comparison_matches_joint_names(self):
        actual = RobotState(joint_state=JointState(name=['other', 'wrist_1_joint'], position=[8.0, .02]))
        self.assertAlmostEqual(WeldingRunner.start_errors(actual, trajectory(0))['wrist_1_joint'], .02)
        with self.assertRaisesRegex(RuntimeError, 'Missing or invalid'):
            WeldingRunner.start_errors(RobotState(), trajectory(0))

    def stub(self, settled, feedback, moveit):
        return SimpleNamespace(
            report={}, cartesian_targets={'pass': ['waypoint']},
            settled_state=Mock(side_effect=[state(q) for q in settled]),
            fresh_state=Mock(side_effect=[state(q) for q in feedback]),
            call=Mock(side_effect=[SimpleNamespace(scene=SimpleNamespace(robot_state=state(q))) for q in moveit]),
            cartesian=Mock(side_effect=lambda current, _: trajectory(current.joint_state.position[0])),
            start_errors=WeldingRunner.start_errors,
            get_logger=lambda: SimpleNamespace(warning=Mock()))

    def test_replans_if_feedback_changes_while_preparing_path(self):
        stub = self.stub([0, .02], [.02, .02], [.02, .02])
        result = WeldingRunner.prepare_execution(stub, 'pass', trajectory(0))
        self.assertAlmostEqual(result.joint_trajectory.points[0].positions[0], .02)
        self.assertEqual(stub.cartesian.call_count, 2)
        self.assertEqual(len(stub.report['execution_start_checks']), 2)

    def test_persistent_drift_is_rejected_after_bounded_replans(self):
        stub = self.stub([0, .02, .04], [.02, .04, .06], [.02, .04, .06])
        with self.assertRaisesRegex(RuntimeError, 'Trajectory was not sent'):
            WeldingRunner.prepare_execution(stub, 'pass', trajectory(0))
        self.assertEqual(stub.cartesian.call_count, 3)

    def test_moveit_state_is_checked_even_when_feedback_matches(self):
        stub = self.stub([0, 0, 0], [0, 0, 0], [.02, .02, .02])
        with self.assertRaisesRegex(RuntimeError, 'wrist_1_joint'):
            WeldingRunner.prepare_execution(stub, 'pass', trajectory(0))
        self.assertEqual(stub.report['execution_start_checks'][0]['feedback_errors']['wrist_1_joint'], 0)
        self.assertAlmostEqual(stub.report['execution_start_checks'][0]['moveit_errors']['wrist_1_joint'], .02)

    def test_return_pose_is_replanned_from_actual_feedback(self):
        stub = self.stub([.02], [.02], [.02])
        stub.cartesian_targets = {}
        stub.joint_targets = {'workpiece:return': {'wrist_1_joint': .5}}
        stub.joint_motion = Mock(return_value=trajectory(.02))
        WeldingRunner.prepare_execution(stub, 'workpiece:return', trajectory(0))
        self.assertEqual(stub.joint_motion.call_args.args[0].joint_state.position[0], .02)
        self.assertEqual(stub.joint_motion.call_args.args[1], {'wrist_1_joint': .5})
        stub.cartesian.assert_not_called()


if __name__ == '__main__':
    unittest.main()
