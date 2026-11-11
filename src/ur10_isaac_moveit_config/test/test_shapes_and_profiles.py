"""Non-box geometry and native JAKA names, without connecting to hardware."""
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import CollisionObject, RobotState, RobotTrajectory
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from trajectory_msgs.msg import JointTrajectoryPoint

from welding_sim.geometry import mesh_geometry, cylinder_geometry, rotation
from welding_sim.profiles import ISAAC, JAKA, load_profile
from welding_sim.return_pose import load_return_pose
from welding_sim.runner import WeldingRunner, object_geometry
from welding_sim.web_control import Settings, geometry_payload, Controller


def planar():
    return mesh_geometry([[0, 0, 0], [.3, 0, 0], [.3, .2, 0], [0, .2, 0]], [[0, 1, 2], [0, 2, 3]])


class ShapesTests(unittest.TestCase):
    def test_cylinder_circle_and_partial_arc(self):
        g = cylinder_geometry([.2, .15], [-1, 0, .1])
        c = g.contour('z_plus')
        seam, tool, q = c.path(.1, .6, .03, .005)
        self.assertAlmostEqual(c.length, .3*math.pi)
        np.testing.assert_allclose(np.linalg.norm(seam[:, :2]-[-1, 0], axis=1), .15)
        np.testing.assert_allclose(seam[:, 2], .2)
        np.testing.assert_allclose(tool[:, 2], .23)
        self.assertLessEqual(np.linalg.norm(np.diff(tool, axis=0), axis=1).max(), .005)
        np.testing.assert_allclose(rotation(q)[:, 2], [0, 0, -1], atol=1e-12)
        full = c.path()[0]
        np.testing.assert_allclose(full[0], full[-1], atol=1e-12)
        bottom = g.contour('z_minus')
        np.testing.assert_allclose(rotation(bottom.path()[2])[:, 2], [0, 0, 1], atol=1e-12)

    def test_cylinder_rotated_object_pose(self):
        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.pose.orientation.y = math.sin(math.pi/4)
        obj.pose.orientation.w = math.cos(math.pi/4)
        p = Pose(); p.orientation.w = 1.; p.position.z = .4
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[.2, .15])]
        obj.primitive_poses = [p]
        g = object_geometry(obj)
        np.testing.assert_allclose(g.contour('z_plus').center, [.5, 0, 0], atol=1e-12)
        np.testing.assert_allclose(g.contour('z_plus').outward, [1, 0, 0], atol=1e-12)

    def test_mesh_combines_triangles_and_removes_diagonal(self):
        g = planar()
        self.assertEqual(len(g.contours), 1)
        self.assertEqual(len(g.edges), 4)
        c = g.contour('face_0')
        self.assertAlmostEqual(c.length, 1.)
        np.testing.assert_allclose(c.outward, [0, 0, 1])
        seam, tool, _ = c.path(.2, .9, .03, .005)
        self.assertAlmostEqual(np.linalg.norm(np.diff(seam, axis=0), axis=1).sum(), .7)
        np.testing.assert_allclose(tool-seam, np.tile([0, 0, .03], (len(seam), 1)))

    def test_triangle_and_concave_polygon(self):
        g = mesh_geometry([[0, 0, 0], [.2, 0, 0], [0, .2, 0]], [[0, 1, 2]])
        seam, _, _ = g.contour('face_0').path(step=.002)
        self.assertAlmostEqual(np.linalg.norm(np.diff(seam, axis=0), axis=1).sum(), .4+math.sqrt(.08))
        vertices = [[0,0,0],[2,0,0],[2,1,0],[1,1,0],[1,2,0],[0,2,0]]
        g = mesh_geometry(vertices, [[0,1,3],[1,2,3],[0,3,5],[3,4,5]])
        self.assertEqual(len(g.edges), 6)
        self.assertAlmostEqual(g.contour('face_0').length, 8.)

    def test_reject_bad_meshes_and_dimensions(self):
        for dims in ([.2], [.2, 0], [math.nan, .1]):
            with self.assertRaises(ValueError): cylinder_geometry(dims)
        for vertices, triangles in (([[0,0,0],[1,0,0],[2,0,0]], [[0,1,2]]),
                                     ([[0,0,0],[1,0,0],[0,1,0]], [[0,1,4]]),
                                     ([[0,0,0],[1,0,0],[0,1,0],[1,1,0]], [[0,1,2],[0,1,3]])):
            with self.assertRaises(ValueError): mesh_geometry(vertices, triangles)
        with self.assertRaises(ValueError): planar().contour('z_plus')

    def test_mesh_collision_object_and_web_catalog(self):
        obj = CollisionObject(); obj.header.frame_id = 'world'; obj.pose.orientation.w = 1.
        p = Pose(); p.position.x = -1.; p.orientation.w = 1.
        obj.meshes = [Mesh(vertices=[Point(x=0.,y=0.,z=0.), Point(x=.3,y=0.,z=0.), Point(x=0.,y=.2,z=0.)],
                           triangles=[MeshTriangle(vertex_indices=[0,1,2])])]
        obj.mesh_poses = [p]
        g = object_geometry(obj)
        payload = geometry_payload(g, Settings())
        self.assertEqual(payload['selected_face'], 'face_0')
        self.assertEqual(payload['shape'], 'mesh')
        self.assertEqual(len(payload['faces'][0]['vertices']), 3)
        ring = geometry_payload(cylinder_geometry([.2,.15]), Settings(selection='edge'))
        self.assertGreater(len(ring['edges'][0]['vertices']), 30)
        self.assertAlmostEqual(ring['length'], .3*math.pi)


class ProfileTests(unittest.TestCase):
    def test_names_and_strict_feedback(self):
        self.assertEqual(JAKA.group, 'jaka_zu12')
        self.assertEqual(JAKA.tip, 'Link_6')
        self.assertTrue(JAKA.accepts(JointState(name=list(JAKA.joints), position=[0.]*6)))
        self.assertFalse(JAKA.accepts(JointState(name=list(ISAAC.joints), position=[0.]*6)))
        self.assertFalse(JAKA.accepts(JointState(name=list(JAKA.joints)+['joint_1'], position=[0.]*7)))
        self.assertTrue(JAKA.publisher_matches('moveit_server'))
        self.assertFalse(JAKA.publisher_matches('_Graph_ROS'))

    def test_custom_names_and_return_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'profile.json'
            path.write_text(json.dumps({'tip':'tcp', 'frames':['Link_0', 'tcp']}))
            self.assertEqual(load_profile('jaka_real', path).tip, 'tcp')
            path.write_text(json.dumps({'real':False}))
            with self.assertRaises(ValueError): load_profile('jaka_real', path)
            path.write_text(json.dumps({'units':'rad', 'joint_positions':dict.fromkeys(JAKA.joints, .1)}))
            self.assertEqual(len(load_return_pose(path, JAKA.joints)['joint_positions']), 6)
            with self.assertRaises(ValueError): load_return_pose(path, ISAAC.joints)

    def test_cartesian_and_fk_use_jaka_frames(self):
        response = GetCartesianPath.Response(); response.error_code.val = 1; response.fraction = 1.
        stub = SimpleNamespace(profile=JAKA, args=SimpleNamespace(step=.005, scaling=.05,speed=.02),
                               call=Mock(return_value=response), report={}, limit_cartesian_speed=Mock())
        WeldingRunner.cartesian(stub, RobotState(), [])
        request = stub.call.call_args.args[2]
        self.assertEqual((request.group_name,request.link_name), ('jaka_zu12','Link_6'))
        trajectory = RobotTrajectory(); trajectory.joint_trajectory.joint_names=list(JAKA.joints)
        trajectory.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.]*6)]
        fk = GetPositionFK.Response(); fk.error_code.val = 1
        from geometry_msgs.msg import PoseStamped
        fk.pose_stamped = [PoseStamped()]
        stub.call = Mock(return_value=fk)
        state=RobotState(joint_state=JointState(name=list(JAKA.joints),position=[0.]*6))
        WeldingRunner.limit_cartesian_speed(stub,state,trajectory)
        self.assertEqual(stub.call.call_args.args[2].fk_link_names, ['Link_6'])

    def test_joint_goals_never_drop_unknown_names(self):
        result = GetMotionPlan.Response(); result.motion_plan_response.error_code.val=1
        state=RobotState(joint_state=JointState(name=list(JAKA.joints),position=[0.]*6))
        stub=SimpleNamespace(profile=JAKA,args=SimpleNamespace(scaling=.05), latest_state=state.joint_state,
                             call=Mock(return_value=result))
        WeldingRunner.joint_motion(stub,state,dict.fromkeys(JAKA.joints,.1))
        request = stub.call.call_args.args[2].motion_plan_request
        self.assertEqual(request.group_name, 'jaka_zu12')
        self.assertEqual([j.joint_name for j in request.goal_constraints[0].joint_constraints],list(JAKA.joints))
        stub.call.reset_mock()
        with self.assertRaises(RuntimeError): WeldingRunner.joint_motion(stub,state,dict.fromkeys(ISAAC.joints,0.))
        stub.call.assert_not_called()

    def test_real_panel_does_not_launch_isaac(self):
        with tempfile.TemporaryDirectory() as tmp:
            controller=Controller(tmp,Path(tmp)/'weld.py',profile=JAKA)
            with self.assertRaisesRegex(ValueError, 'run_real.sh'): controller.scene(True)
            self.assertTrue(controller.status()['robot_profile']['real'])

    def test_real_execution_requires_only_selected_publisher(self):
        stub = SimpleNamespace(profile=JAKA, report={}, get_publishers_info_by_topic=Mock(
            return_value=[SimpleNamespace(node_name='moveit_server')]))
        WeldingRunner.wait_for_robot_publisher(stub, timeout=0.)
        stub.get_publishers_info_by_topic.assert_called_with('/joint_states')
        stub.get_publishers_info_by_topic.return_value.append(SimpleNamespace(node_name='_Graph_ROS'))
        with self.assertRaises(RuntimeError): WeldingRunner.wait_for_robot_publisher(stub, timeout=0.)

    def test_mesh_hole_is_rejected(self):
        vertices = [[0,0,0],[3,0,0],[3,3,0],[0,3,0],[1,1,0],[2,1,0],[2,2,0],[1,2,0]]
        triangles = [[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
        with self.assertRaisesRegex(ValueError, 'holes'): mesh_geometry(vertices,triangles)


if __name__ == '__main__': unittest.main()
