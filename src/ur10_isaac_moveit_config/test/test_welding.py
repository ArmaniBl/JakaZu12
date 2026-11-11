"""Geometry invariants and rejection of incomplete Cartesian plans."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, RobotState, RobotTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from shape_msgs.msg import SolidPrimitive

from welding_sim.geometry import box_edges, face_contour, rotation, select_edges
from welding_sim.runner import WeldingRunner, object_edges


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.edges = box_edges([0.3, 0.3, 0.2], [-1.0, 0, 0.1])

    def test_box_topology_and_bounds(self):
        self.assertEqual(len({e.id for e in self.edges}), 12)
        vertices = {tuple(p.round(8)) for e in self.edges for p in (e.start, e.end)}
        self.assertEqual(len(vertices), 8)
        for vertex in vertices:
            self.assertIn(vertex[0], (-1.15, -0.85))
            self.assertIn(vertex[1], (-0.15, 0.15))
            self.assertIn(vertex[2], (0.0, 0.2))
            self.assertEqual(sum(np.allclose(vertex, p) for e in self.edges for p in (e.start, e.end)), 3)

    def test_selection_and_subrange(self):
        self.assertEqual(len(select_edges(self.edges, face="z_plus")), 4)
        edge = select_edges(self.edges, ["x_plus__z_plus"])[0]
        seam, tool, q = edge.path(0.2, 0.8, 0.1, 0.005, math.pi/2)
        np.testing.assert_allclose(seam[[0, -1]], [[-0.85, -0.09, 0.2], [-0.85, 0.09, 0.2]])
        np.testing.assert_allclose(np.linalg.norm(tool-seam, axis=1), 0.1)
        self.assertLessEqual(np.max(np.linalg.norm(np.diff(tool, axis=0), axis=1)), 0.005 + 1e-12)
        np.testing.assert_allclose(rotation(q)[:, 2], -edge.outward, atol=1e-12)

    def test_rotated_primitive_and_object_poses_compose(self):
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.pose.position.x = -1.0
        obj.pose.orientation.z = math.sin(math.pi/4)
        obj.pose.orientation.w = math.cos(math.pi/4)
        primitive_pose = Pose()
        primitive_pose.orientation.w = 1.0
        primitive_pose.position.x = 0.4
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.3, 0.3, 0.2])]
        obj.primitive_poses = [primitive_pose]
        edges = object_edges(obj)
        center = np.mean([p for e in edges for p in (e.start, e.end)], axis=0)
        np.testing.assert_allclose(center, [-1, 0.4, 0], atol=1e-12)
        self.assertAlmostEqual(edges[0].length, 0.2)

    def test_invalid_selection_and_geometry_rejected(self):
        with self.assertRaises(ValueError):
            select_edges(self.edges, ["not_an_edge"])
        for dimensions in ([0, 1, 1], [-1, 1, 1], [float("nan"), 1, 1]):
            with self.assertRaises(ValueError):
                box_edges(dimensions)
        with self.assertRaises(ValueError):
            self.edges[0].path(0.8, 0.2)

    def test_partial_cartesian_plan_is_rejected(self):
        result = GetCartesianPath.Response()
        result.error_code.val = 1
        result.fraction = 0.99
        stub = SimpleNamespace(args=SimpleNamespace(step=0.005, scaling=0.05, speed=0.02),
                               report={}, call=lambda *args: result)
        with self.assertRaisesRegex(RuntimeError, "Incomplete Cartesian path"):
            WeldingRunner.cartesian(stub, RobotState(), [])

    def test_face_contour_is_closed_and_visits_all_corners(self):
        contour = face_contour(self.edges, "z_plus")
        self.assertAlmostEqual(contour.length, 1.2)
        seam, tool, q = contour.path(step=0.007)
        np.testing.assert_allclose(seam[0], seam[-1])
        np.testing.assert_allclose(seam[:, 2], 0.2)
        np.testing.assert_allclose(tool[:, 2], 0.3)
        for vertex in contour.vertices:
            self.assertTrue(any(np.allclose(vertex, p) for p in seam))
        self.assertTrue(all(np.count_nonzero(np.abs(delta) > 1e-10) == 1 for delta in np.diff(seam, axis=0)))
        np.testing.assert_allclose(rotation(q)[:, 2], [0, 0, -1], atol=1e-12)

    def test_contour_subrange_is_fraction_of_whole_perimeter(self):
        contour = face_contour(self.edges, "z_plus")
        seam, _, _ = contour.path(0.2, 0.8, step=0.007)
        self.assertAlmostEqual(np.linalg.norm(np.diff(seam, axis=0), axis=1).sum(), 0.72)
        self.assertFalse(np.allclose(seam[0], seam[-1]))

    def test_tcp_speed_limit_stretches_time_and_derivatives(self):
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ["joint"]
        trajectory.joint_trajectory.points = [
            JointTrajectoryPoint(positions=[0.0], velocities=[1.0], accelerations=[2.0]),
            JointTrajectoryPoint(positions=[1.0], velocities=[1.0], accelerations=[2.0],
                                 time_from_start=Duration(sec=1)),
        ]
        def fk(service, name, request):
            result = GetPositionFK.Response()
            result.error_code.val = 1
            pose = PoseStamped()
            pose.pose.position.x = request.robot_state.joint_state.position[0]*0.04
            result.pose_stamped = [pose]
            return result
        stub = SimpleNamespace(args=SimpleNamespace(speed=0.02), report={}, call=fk)
        WeldingRunner.limit_cartesian_speed(stub, RobotState(), trajectory)
        last = trajectory.joint_trajectory.points[-1]
        duration = last.time_from_start.sec + last.time_from_start.nanosec/1e9
        self.assertLessEqual(0.04/duration, 0.02)
        self.assertAlmostEqual(last.velocities[0]*duration, 1.0)
        self.assertAlmostEqual(last.accelerations[0]*duration**2, 2.0)


class PublisherDiscoveryTests(unittest.TestCase):
    def test_waits_for_delayed_isaac_discovery(self):
        stub = SimpleNamespace(
            report={}, get_logger=lambda: SimpleNamespace(info=Mock()),
            get_publishers_info_by_topic=Mock(side_effect=[[],
                [SimpleNamespace(node_name="joint_state_publisher")],
                [SimpleNamespace(node_name="_Graph_ROS_JointStates_PublisherJointState")]]))
        with patch("welding_sim.runner.rclpy.spin_once") as spin:
            WeldingRunner.wait_for_isaac_publisher(stub)
        self.assertEqual(spin.call_count, 2)
        self.assertEqual(stub.report["isaac_joint_state_publishers"],
                         ["_Graph_ROS_JointStates_PublisherJointState"])

    def test_rejects_other_publishers_after_timeout(self):
        stub = SimpleNamespace(report={}, get_publishers_info_by_topic=lambda _: [
            SimpleNamespace(node_name="joint_state_publisher")])
        with patch("welding_sim.runner.rclpy.spin_once") as spin:
            with self.assertRaisesRegex(RuntimeError, "Discovered publishers: joint_state_publisher"):
                WeldingRunner.wait_for_isaac_publisher(stub, timeout=0)
        spin.assert_not_called()


if __name__ == "__main__":
    unittest.main()
