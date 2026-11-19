import unittest
from copy import deepcopy
from moveit_msgs.msg import PlanningScene,ObjectColor
from welding_sim.table_limits import wall_specs,require_walls
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
class TableLimitsTests(unittest.TestCase):
 def test_moveit_normalized_pose_is_equivalent(self):
  s=self.scene()
  for o in s.world.collision_objects:
   o.pose=deepcopy(o.primitive_poses[0]);o.primitive_poses[0]=Pose();o.primitive_poses[0].orientation.w=1.
  require_walls(s)
 def test_composed_rotation_and_translation_are_checked(self):
  import numpy as np
  s=self.scene()
  for o in s.world.collision_objects:
   p=o.primitive_poses[0].position
   x,y=p.x,p.y
   o.pose.orientation.z=2**-.5;o.pose.orientation.w=2**-.5
   p.x=y;p.y=-x
   o.primitive_poses[0].orientation.z=-2**-.5;o.primitive_poses[0].orientation.w=2**-.5
  require_walls(s)
  s.world.collision_objects[0].pose.position.x=.001
  with self.assertRaises(RuntimeError):require_walls(s)
 def scene(self):
  s=PlanningScene()
  for name,dimensions,center in wall_specs():
   o=CollisionObject(id=name);o.header.frame_id='world';o.pose.orientation.w=1.
   p=Pose();p.orientation.w=1.;p.position.x,p.position.y,p.position.z=center
   o.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=dimensions)];o.primitive_poses=[p];s.world.collision_objects.append(o)
  return s
 def test_inner_boundaries_are_table_plus_ten_centimetres(self):
  s=self.scene();require_walls(s)
  for o,axis,sign,expected in zip(s.world.collision_objects,[0,0,1,1],[1,-1,1,-1],[-.54,.46,-1.165,.235]):
   p=o.primitive_poses[0].position;v=[p.x,p.y,p.z]
   self.assertAlmostEqual(v[axis]+sign*o.primitives[0].dimensions[axis]/2,expected)
 def test_missing_or_changed_walls_are_rejected(self):
  s=self.scene();s.world.collision_objects.pop()
  with self.assertRaises(RuntimeError):require_walls(s)
  s=self.scene();s.world.collision_objects[0].pose.position.x=.2
  with self.assertRaises(RuntimeError):require_walls(s)
