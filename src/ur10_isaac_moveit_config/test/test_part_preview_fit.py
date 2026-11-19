"""Known CAD/scan poses test automatic fitting without robot or camera access."""
import json
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from welding_sim.part_preview_fit import fit_preview,top_surface
from welding_sim.workpiece_model import preview_object,PREVIEW_TYPE

MODEL=Path(__file__).resolve().parents[2]/'jaka_zu12_moveit_config/config/workpiece_models/slotted-88x40x5.json'


class PartPreviewTests(unittest.TestCase):
    def setUp(self):
        self.model=json.loads(MODEL.read_text())
        self.vertices,self.surface=top_surface(self.model['vertices_mm'],self.model['triangles'])

    def test_startup_is_real_mesh_not_cube(self):
        obj=preview_object(self.model)
        self.assertEqual(obj.id,'welding_workpiece')
        self.assertEqual(obj.type.key,PREVIEW_TYPE)
        self.assertFalse(obj.primitives)
        self.assertEqual(len(obj.meshes),1)
        np.testing.assert_allclose(np.ptp(self.vertices,axis=0),[.04,.088,.005],atol=1e-7)

    def test_recovers_pose_with_background_and_noise(self):
        rng=np.random.default_rng(13)
        rotation=Rotation.from_euler('xyz',[3.,-4.,32.],degrees=True).as_matrix()
        translation=np.array([-.3,-.7,.01])
        measured=self.surface@rotation.T+translation+rng.normal(0,.00015,self.surface.shape)
        measured=measured[rng.choice(len(measured),int(len(measured)*.85),False)]
        background=np.column_stack([rng.uniform(-.12,-.09,150),rng.uniform(-.71,-.68,150),rng.normal(.015,.0003,150)])
        transform,quality=fit_preview(np.vstack([measured,background]),self.model['vertices_mm'],self.model['triangles'])
        self.assertLess(np.linalg.norm(transform[:3,3]-translation),.002)
        error=Rotation.from_matrix(transform[:3,:3]@rotation.T).magnitude()
        self.assertLess(np.degrees(error),3.)
        self.assertGreater(quality['model_coverage'],.9)
        self.assertFalse(quality['execution_ready'])

    def test_mismatched_surface_does_not_move_model(self):
        scan=self.surface.copy();scan[:,:2]*=4
        with self.assertRaises(ValueError):
            fit_preview(scan,self.model['vertices_mm'],self.model['triangles'])

    def test_small_partial_strip_is_not_a_full_match(self):
        measured=self.surface[self.surface[:,0]>.014]
        with self.assertRaises(ValueError):
            fit_preview(measured,self.model['vertices_mm'],self.model['triangles'])


if __name__=='__main__':unittest.main()
