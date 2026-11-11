"""Pose coherence, missing TF and stale feedback for browser robot display."""
import unittest
from unittest.mock import Mock

from geometry_msgs.msg import TransformStamped
from tf2_ros import LookupException

from welding_sim.robot_view import FRAMES, robot_snapshot


class RobotViewTests(unittest.TestCase):
    def buffer(self):
        def lookup(target, source, stamp):
            result = TransformStamped()
            result.header.frame_id = target
            result.child_frame_id = source
            result.header.stamp.sec = 10
            result.transform.translation.x = -.1 * FRAMES.index(source)
            result.transform.rotation.w = 1.0
            return result
        return Mock(lookup_transform=Mock(side_effect=lookup))

    def test_all_links_share_the_tip_time_and_world_frame(self):
        buffer = self.buffer()
        result = robot_snapshot(buffer, True, now_sec=10.2)
        self.assertTrue(result['available'])
        self.assertEqual([p['frame'] for p in result['points']], list(FRAMES))
        self.assertEqual(result['points'][0]['position'], [0, 0, 0])
        self.assertAlmostEqual(result['points'][-1]['position'][0], -.7)
        for call in buffer.lookup_transform.call_args_list[1:]:
            self.assertEqual(call.args[0], 'world')
            self.assertEqual(call.args[2].nanoseconds, 10_000_000_000)

    def test_stale_joint_feedback_does_not_show_cached_pose(self):
        buffer = self.buffer()
        result = robot_snapshot(buffer, False)
        self.assertFalse(result['available'])
        self.assertEqual(result['points'], [])
        buffer.lookup_transform.assert_not_called()

    def test_missing_link_and_stale_tf_hide_whole_model(self):
        buffer = self.buffer()
        self.assertFalse(robot_snapshot(buffer, True, now_sec=12)['available'])
        buffer.lookup_transform.side_effect = LookupException('missing transform')
        result = robot_snapshot(buffer, True)
        self.assertFalse(result['available'])
        self.assertEqual(result['points'], [])

    def test_invalid_transform_is_not_sent_to_browser(self):
        buffer = self.buffer()
        message = TransformStamped()
        message.transform.translation.x = float('nan')
        buffer.lookup_transform.side_effect = None
        buffer.lookup_transform.return_value = message
        self.assertFalse(robot_snapshot(buffer, True)['available'])


if __name__ == '__main__':
    unittest.main()
