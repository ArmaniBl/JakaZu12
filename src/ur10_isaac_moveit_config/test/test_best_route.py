import unittest
from types import SimpleNamespace as NS
from copy import deepcopy
from unittest.mock import Mock, patch
from moveit_msgs.msg import RobotState,RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from welding_sim.runner import WeldingRunner
from welding_sim.profiles import ISAAC, JAKA
from welding_sim.geometry import FaceContour
from test_table_limits import TableLimitsTests
import numpy as np
class BestRouteTests(unittest.TestCase):
 def setUp(self):
  self.clearance = Mock()
  self.clearance.minimum.return_value = dict(clearance_m=.020, link='Link_5', stage='complete')
  patcher = patch('welding_sim.runner.TableClearance', return_value=self.clearance)
  patcher.start()
  self.addCleanup(patcher.stop)

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
  fake.remote_parameter=Mock()
  fake.planning_floor=None
  fake.check_world_collisions_enabled=Mock()
  fake.check_body_path=Mock()
  fake.end_state=WeldingRunner.end_state
  paths=[(None,np.zeros((2,3)),np.zeros((2,3)),np.array([0.,0.,0.,1.]))]
  return fake,paths,published
 def test_selects_best_complete_route_instead_of_first_and_restores_targets(self):
  f,p,published=self.fixture();stages=WeldingRunner.plan_best(f,p)
  self.assertEqual(stages[0][1].joint_trajectory.points[-1].positions[0],1.)
  self.assertEqual(f.cartesian_targets,{'chosen':1.});self.assertEqual(len(published),1)
  self.assertEqual(f.report['optimization']['complete_candidates'],4)
  self.assertIsNone(f.optimization_start)
  f.check_body_path.assert_called_once_with('complete',stages[0][1],published[0].trajectory_start.joint_state)
 def test_low_clearance_candidate_is_rejected_and_search_continues(self):
  f,p,published=self.fixture()
  self.clearance.minimum.side_effect = [
   dict(clearance_m=margin, link='Link_5', stage='complete') for margin in (.025,.004,.020,.018)]
  stages=WeldingRunner.plan_best(f,p)
  self.assertEqual(stages[0][1].joint_trajectory.points[-1].positions[0],2.)
  self.assertEqual(f.report['chosen_orientation']['table_clearance']['clearance_m'],.020)
  self.assertEqual(f.report['orientation_attempts'][1]['status'],'failed')

 def test_collision_failure_prevents_publishing_selected_route(self):
  f,p,published=self.fixture()
  f.check_body_path.side_effect=RuntimeError('Whole-body collision check failed')
  with self.assertRaisesRegex(RuntimeError,'Whole-body collision'):
   WeldingRunner.plan_best(f,p)
  self.assertFalse(published)
 def test_collision_on_best_candidate_uses_next_validated_route(self):
  f,p,published=self.fixture()
  f.check_body_path.side_effect=[RuntimeError('Whole-body collision check failed'),None]
  stages=WeldingRunner.plan_best(f,p)
  self.assertEqual(stages[0][1].joint_trajectory.points[-1].positions[0],2.)
  self.assertEqual(f.cartesian_targets,{'chosen':2.})
  self.assertEqual(len(f.report['validation_failures']),1)
  self.assertEqual(len(published),1)
 def test_scene_change_invalidates_selected_plan(self):
  f,p,published=self.fixture();f.call=lambda *args:NS(scene=NS(world=NS(collision_objects=[]),robot_state=f.initial_scene.robot_state))
  with self.assertRaisesRegex(RuntimeError,'scene changed'):WeldingRunner.plan_best(f,p)
  self.assertFalse(published)

 def test_isaac_optimizes_without_jaka_workspace_walls(self):
  f,p,published=self.fixture()
  f.profile=ISAAC
  f.args.orientation_mode='auto'
  f.initial_scene.world.collision_objects.clear()
  for method in ('settled_state','fresh_state'):
   original=getattr(f,method)
   def sim_state(original=original):
    state=original()
    state.joint_state.name=list(ISAAC.joints)
    return state
   setattr(f,method,sim_state)
  plan=f.plan_fixed
  def sim_plan(paths):
   stages=plan(paths)
   stages[0][1].joint_trajectory.joint_names=list(ISAAC.joints)
   return stages
  f.plan_fixed=sim_plan
  options=[(0.,0.,float(roll)) for roll in range(4)]
  with patch('welding_sim.runner.candidates',return_value=options):
   stages=WeldingRunner.plan_best(f,p)
  self.assertEqual(stages[0][1].joint_trajectory.points[-1].positions[0],1.)
  self.assertEqual(len(published),1)
