import unittest
from types import SimpleNamespace as NS
from copy import deepcopy
from unittest.mock import Mock
from moveit_msgs.msg import RobotState,RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from welding_sim.runner import WeldingRunner
from welding_sim.profiles import JAKA
from welding_sim.geometry import FaceContour
from test_table_limits import TableLimitsTests
import numpy as np
class BestRouteTests(unittest.TestCase):
 def fixture(self):
  state=RobotState(joint_state=JointState(name=list(JAKA.joints),position=[0.]*6))
  scene=TableLimitsTests().scene();published=[]
  fake=NS(args=NS(orientation_mode='fixed',tilt=0.,tilt_direction=0.,max_tilt=45.,orientation_timeout=120.),profile=JAKA,
   initial_scene=scene,settled_state=lambda:deepcopy(state),fresh_state=lambda:deepcopy(state),report={},
   display=NS(publish=published.append),get_logger=lambda:NS(info=lambda _:None),call=lambda *args:NS(scene=deepcopy(scene)))
  calls=[]
  def plan(paths):
   cost=[3.,1.,2.,4.][len(calls)];calls.append(cost)
   t=RobotTrajectory();t.joint_trajectory.joint_names=list(JAKA.joints)
   a=JointTrajectoryPoint(positions=[0.]*6);b=JointTrajectoryPoint(positions=[cost,0.,0.,0.,0.,0.]);b.time_from_start.sec=1
   t.joint_trajectory.points=[a,b];fake.report['candidate_cost']=cost;fake.cartesian_targets={'chosen':cost};fake.joint_targets={}
   return [('complete',t)]
  fake.plan_fixed=plan
  paths=[(None,np.zeros((2,3)),np.zeros((2,3)),np.array([0.,0.,0.,1.]))]
  return fake,paths,published
 def test_selects_best_complete_route_instead_of_first_and_restores_targets(self):
  f,p,published=self.fixture();stages=WeldingRunner.plan_best(f,p)
  self.assertEqual(stages[0][1].joint_trajectory.points[-1].positions[0],1.)
  self.assertEqual(f.cartesian_targets,{'chosen':1.});self.assertEqual(len(published),1)
  self.assertEqual(f.report['optimization']['complete_candidates'],4)
 def test_scene_change_invalidates_selected_plan(self):
  f,p,published=self.fixture();f.call=lambda *args:NS(scene=NS(world=NS(collision_objects=[]),robot_state=f.initial_scene.robot_state))
  with self.assertRaisesRegex(RuntimeError,'scene changed'):WeldingRunner.plan_best(f,p)
  self.assertFalse(published)
