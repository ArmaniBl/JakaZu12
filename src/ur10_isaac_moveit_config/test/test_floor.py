"""The floor blocks motion but cannot be selected for welding."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
from moveit_msgs.msg import AllowedCollisionMatrix,AllowedCollisionEntry
from welding_sim.environment import simulation_floor,allow_base_support
from welding_sim.runner import object_geometry
from welding_sim.runner import WeldingRunner
from welding_sim.profiles import ISAAC
from welding_sim.web_control import Settings

class FloorTests(unittest.TestCase):
    def test_floor_top_and_selection_exclusion(self):
        obj=simulation_floor()
        self.assertEqual(obj.id,'floor')
        self.assertEqual(obj.primitive_poses[0].position.z+obj.primitives[0].dimensions[2]/2,0.)
        with self.assertRaises(ValueError):object_geometry(obj)
        with self.assertRaises(ValueError):Settings.parse({'object':'floor'})

    def test_only_fixed_base_contact_allowed_existing_matrix_preserved(self):
        matrix=AllowedCollisionMatrix(entry_names=['forearm_link','wrist_1_link'],entry_values=[
            AllowedCollisionEntry(enabled=[False,True]),AllowedCollisionEntry(enabled=[True,False])])
        result=allow_base_support(matrix)
        self.assertEqual(len(matrix.entry_names),2)
        self.assertTrue(result.entry_values[0].enabled[1])
        floor=result.entry_names.index('floor');base=result.entry_names.index('base_link_inertia')
        self.assertTrue(result.entry_values[floor].enabled[base])
        self.assertFalse(result.entry_values[floor].enabled[0])
        self.assertFalse(result.entry_values[floor].enabled[1])
        self.assertEqual(allow_base_support(result),result)

    def test_simulation_planning_rejects_missing_floor(self):
        stub=SimpleNamespace(profile=ISAAC,args=SimpleNamespace(mode='plan'),
            call=Mock(return_value=SimpleNamespace(scene=SimpleNamespace(world=SimpleNamespace(collision_objects=[])))))
        with patch('welding_sim.runner.time.monotonic',side_effect=[0.,21.]):
            with self.assertRaisesRegex(RuntimeError,'floor is absent'):WeldingRunner.read_object(stub)

if __name__=='__main__':unittest.main()
