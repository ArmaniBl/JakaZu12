"""Saved joint targets remain fixed and are reached through collision-aware planning."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, RobotState, RobotTrajectory
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from welding_sim.geometry import box_edges
from welding_sim.return_pose import JOINTS, load_return_pose
from welding_sim.runner import WeldingRunner
from welding_sim.profiles import JAKA

TARGETS=dict(zip(JOINTS,[0.,-2.0944,-1.0472,-1.5708,1.5708,0.]))
def state(values=TARGETS):return RobotState(joint_state=JointState(name=list(values),position=list(values.values())))
def trajectory(value):
    result=RobotTrajectory();result.joint_trajectory.joint_names=list(JOINTS)
    result.joint_trajectory.points=[JointTrajectoryPoint(positions=[float(value)]*6)]
    return result

class ReturnPoseTests(unittest.TestCase):
    def test_saved_pose_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'return.json'
            path.write_text(json.dumps({'units':'rad','joint_positions':TARGETS}))
            self.assertEqual(load_return_pose(path)['joint_positions'],TARGETS)
            for positions in ({}, {**TARGETS,'wrist_1_joint':True}, {**TARGETS,'wrist_2_joint':float('nan')}):
                path.write_text(json.dumps({'units':'rad','joint_positions':positions}))
                with self.assertRaises(ValueError):load_return_pose(path)

    def test_joint_planning_uses_saved_angles_exactly(self):
        result=GetMotionPlan.Response();result.motion_plan_response.error_code.val=1
        stub=SimpleNamespace(args=SimpleNamespace(scaling=.5),latest_state=state().joint_state,call=Mock(return_value=result))
        WeldingRunner.joint_motion(stub,state(),TARGETS)
        request=stub.call.call_args.args[2].motion_plan_request
        self.assertEqual({j.joint_name:j.position for j in request.goal_constraints[0].joint_constraints},TARGETS)
        self.assertEqual(request.max_velocity_scaling_factor,.5)

    def stub(self, finish):
        obj=CollisionObject();obj.id='welding_workpiece';obj.header.frame_id='world';obj.pose.orientation.w=1.
        pose=Pose();pose.position.x=-1.;pose.position.z=.1;pose.orientation.w=1.
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.3,.3,.2])];obj.primitive_poses=[pose]
        return SimpleNamespace(args=SimpleNamespace(clearance=.05),report={},return_pose={'units':'rad','joint_positions':TARGETS},
            cartesian_targets={},joint_targets={},fresh_state=state,end_state=WeldingRunner.end_state,
            approach=Mock(return_value=trajectory(.1)),cartesian=Mock(return_value=trajectory(.2)),
            joint_motion=Mock(side_effect=[finish]),display=Mock(),get_logger=lambda:SimpleNamespace(info=Mock(),warning=Mock()))

    def paths(self):
        edge=box_edges([.3,.3,.2],[-1,0,.1])[0];seam,tool,q=edge.path(offset=.03)
        return [(edge,seam,tool,q)]

    def test_saved_return_is_last_and_planned_from_retracted_state(self):
        stub=self.stub(trajectory(.3))
        stages=WeldingRunner.plan(stub,self.paths())
        self.assertEqual(stages[-1][0],'workpiece:return')
        self.assertEqual(stub.joint_motion.call_args.args[1],TARGETS)
        self.assertEqual(stub.joint_motion.call_args.args[0].joint_state.position[0],.2)
        self.assertEqual(stub.report['return_pose']['joint_positions'],TARGETS)

    def test_unreachable_return_prevents_a_complete_plan(self):
        stub=self.stub(RuntimeError('return path blocked'))
        with self.assertRaisesRegex(RuntimeError,'return path blocked'):WeldingRunner.plan(stub,self.paths())
        stub.display.publish.assert_not_called()

    def test_real_job_ends_with_vertical_lift_without_return(self):
        stub=self.stub(trajectory(.3));stub.profile=JAKA
        paths=self.paths()
        stages=WeldingRunner.plan_fixed(stub,paths)
        self.assertEqual(stages[-1][0], 'workpiece:lift')
        stub.joint_motion.assert_not_called()
        last=paths[-1][2][-1];q=paths[-1][3]
        target=stub.cartesian_targets['workpiece:lift'][0]
        self.assertAlmostEqual(target.position.x,float(last[0]))
        self.assertAlmostEqual(target.position.y,float(last[1]))
        self.assertAlmostEqual(target.position.z,float(last[2])+.25)
        for actual,expected in zip([target.orientation.x,target.orientation.y,target.orientation.z,target.orientation.w],q):
            self.assertAlmostEqual(actual,float(expected))
        self.assertEqual(len(stub.cartesian.call_args_list[0].args[1]),1)
        self.assertEqual(len(stub.cartesian.call_args_list[1].args[1]),len(paths[0][2])-1)

    def test_blocked_real_lift_prevents_execution_plan(self):
        stub=self.stub(trajectory(.3));stub.profile=JAKA
        stub.cartesian.side_effect=[trajectory(.2),trajectory(.2),RuntimeError('lift blocked')]
        with self.assertRaisesRegex(RuntimeError,'lift blocked'):
            WeldingRunner.plan_fixed(stub,self.paths())
        stub.display.publish.assert_not_called()

    def test_return_feedback_is_verified(self):
        stub=SimpleNamespace(return_pose={'joint_positions':TARGETS},report={},settled_state=state)
        WeldingRunner.verify_return_pose(stub)
        self.assertTrue(all(e==0 for e in stub.report['return_pose_errors_rad'].values()))
        stub.settled_state=lambda:state({**TARGETS,'wrist_1_joint':TARGETS['wrist_1_joint']+.02})
        with self.assertRaisesRegex(RuntimeError,'not reached'):WeldingRunner.verify_return_pose(stub)

if __name__=='__main__':unittest.main()
