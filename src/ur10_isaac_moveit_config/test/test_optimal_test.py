"""Compare full routes without invoking ROS services or robot actions."""
from copy import deepcopy
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState, RobotTrajectory
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from welding_sim import optimal_test, runner
from welding_sim.profiles import JAKA
from welding_sim.geometry import rotation


def trajectory(begin,end):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = list(JAKA.joints)
    result.joint_trajectory.points = [
        JointTrajectoryPoint(positions=[begin]*6,time_from_start=Duration(sec=0)),
        JointTrajectoryPoint(positions=[end]*6,time_from_start=Duration(sec=2))]
    return result


class FakeRunner(optimal_test.OptimalTestRunner):
    def __init__(self):
        self.args = NS(tilt=0.,tilt_direction=0.)
        self.profile = JAKA
        self.initial_scene = NS(world=NS(collision_objects=[]),robot_state=NS(attached_collision_objects=[]))
        self.final_scene = deepcopy(self.initial_scene)
        self.state = RobotState(joint_state=JointState(name=list(JAKA.joints),position=[0.]*6))
        self.report = {'selection':[{}]}
        self.published = []
        self.seen_orientations = []
        self.display = NS(publish=self.published.append)
        self.max_candidates,self.search_seconds = 4,180.
        self.get_logger = lambda:NS(info=lambda message:None)
        self.settled_state = lambda:deepcopy(self.state)

    def call(self,service,name,request):
        if service == GetPositionFK:
            pose = PoseStamped()
            pose.pose.orientation.z,pose.pose.orientation.w = np.sin(.35),np.cos(.35)
            return NS(error_code=NS(val=1),pose_stamped=[pose])
        return NS(scene=self.final_scene)


class OptimalTestTests(unittest.TestCase):
    def run_search(self,node,actual=None):
        index = [0]
        def plan_fixed(node,paths):
            node.seen_orientations.append(paths[0][3])
            i = index[0];index[0]+=1
            if i==2:raise RuntimeError('Completion lift failed')
            end = [2.,.5,0.,.1][i]
            finish = end+([.1,.1,0.,3.][i])
            node.report['status']='planned'
            node.cartesian_targets={'selected':i}
            node.joint_targets={}
            node.display.publish('candidate must stay hidden')
            return [('approach',trajectory(0.,end)),('lift',trajectory(end,finish))]
        with tempfile.TemporaryDirectory() as directory:
            urdf = Path(directory)/'urdf';urdf.mkdir()
            (urdf/'jaka_zu12.urdf').write_text('<robot>'+''.join(
                f'<joint name="{j}"><limit lower="-6.28" upper="6.28"/></joint>' for j in JAKA.joints)+'</robot>')
            with patch.object(optimal_test,'get_package_share_directory',return_value=directory), \
                 patch.object(runner.WeldingRunner,'plan_fixed',new=plan_fixed), \
                 patch.object(runner.WeldingRunner,'fresh_state',return_value=actual or node.state):
                return node.plan([(NS(),None,np.zeros((2,3)),np.array([0.,0.,0.,1.]))])

    def test_best_complete_route_is_selected_and_only_it_is_published(self):
        node = FakeRunner()
        result = self.run_search(node)
        self.assertEqual(node.report['optimization']['chosen_index'],1)
        self.assertEqual(node.report['optimization']['attempts'][2]['status'],'failed')
        self.assertEqual(node.cartesian_targets,{'selected':1})
        self.assertEqual(len(node.published),1)
        self.assertEqual(len(result),2)
        self.assertGreater(node.report['optimization']['cost_reduction_percent'],0)

    def test_moving_robot_invalidates_comparison(self):
        node = FakeRunner()
        actual = deepcopy(node.state);actual.joint_state.position=[.1]*6
        with self.assertRaisesRegex(RuntimeError,'Robot moved'):
            self.run_search(node,actual)
        self.assertFalse(node.published)

    def test_scene_change_invalidates_comparison(self):
        node = FakeRunner();node.final_scene.world.collision_objects=[NS(id='new obstacle')]
        with self.assertRaisesRegex(RuntimeError,'Collision geometry changed'):
            self.run_search(node)
        self.assertFalse(node.published)

    def test_execution_is_disabled(self):
        with self.assertRaisesRegex(RuntimeError,'cannot execute'):
            FakeRunner().execute([])

    def test_roll_search_preserves_working_axis(self):
        node = FakeRunner();node.args.tilt=25.;node.args.tilt_direction=60.
        self.run_search(node)
        normals = [rotation(q)[:,2] for q in node.seen_orientations]
        for normal in normals[1:]:
            np.testing.assert_allclose(normal,normals[0],atol=1e-10)

    def test_equivalent_wrist_goal_avoids_full_turn_within_limits(self):
        node = FakeRunner()
        node.seed_limits = dict.fromkeys(JAKA.joints,(-6.28,6.28));node.seed_index=0
        captured = []
        def call(service,name,request):
            self.assertEqual(service,GetPositionIK)
            self.assertTrue(request.ik_request.avoid_collisions)
            goal = deepcopy(node.state);goal.joint_state.position[-1]=6.
            return NS(error_code=NS(val=1),solution=goal)
        node.call=call
        node.joint_motion=lambda state,goals:captured.append(goals)
        node.approach(node.state,np.zeros(3),np.array([0.,0.,0.,1.]))
        self.assertAlmostEqual(captured[0]['joint_6'],6.-2*np.pi)

    def test_fixed_orientation_does_not_search_tool_roll(self):
        node = FakeRunner();node.args.orientation_mode='fixed'
        self.run_search(node)
        self.assertTrue(all(a['roll_delta_deg']==0. for a in node.report['optimization']['attempts']))

    def test_launcher_forces_plan_even_if_execute_is_requested(self):
        def check(**kwargs):
            self.assertEqual(optimal_test.sys.argv[-2:],['--mode','plan'])
            return 0
        with patch.object(optimal_test.sys,'argv',['optimal_test.py','--mode','execute']), \
             patch.object(runner,'main',side_effect=check):
            self.assertEqual(optimal_test.main(),0)


if __name__ == '__main__':
    unittest.main()
