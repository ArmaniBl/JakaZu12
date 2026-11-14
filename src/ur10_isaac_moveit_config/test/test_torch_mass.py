"""Known mass integrals and parallel-axis theorem, independent of USD/PhysX."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'isaac'))
from torch_mass import mesh_mass,combine

class MassTests(unittest.TestCase):
    def cube(self,shift=(0,0,0)):
        vertices=np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                           [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]],dtype=float)+shift
        faces=[[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
               [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
        return vertices[faces]

    def test_cube_mass_and_inertia_independent_of_translation_and_winding(self):
        for shift in ((0,0,0),(2,-3,4)):
            for triangles in (self.cube(shift),self.cube(shift)[:,::-1]):
                mass,center,inertia=mesh_mass(triangles,3.)
                self.assertAlmostEqual(mass,24.)
                np.testing.assert_allclose(center,shift,atol=1e-12)
                np.testing.assert_allclose(inertia,np.eye(3)*16,atol=1e-10)

    def test_parallel_axis_combination(self):
        mass,center,inertia=combine([(1.,np.array([-1.,0,0]),np.zeros((3,3))),
                                     (1.,np.array([1.,0,0]),np.zeros((3,3)))])
        self.assertEqual(mass,2.)
        np.testing.assert_allclose(center,[0,0,0])
        np.testing.assert_allclose(inertia,np.diag([0.,2.,2.]))

    def test_invalid_mass_geometry_rejected(self):
        with self.assertRaises(ValueError):mesh_mass(np.zeros((1,3,3)),1.)
        with self.assertRaises(ValueError):mesh_mass(self.cube(),-1.)

if __name__=='__main__':unittest.main()
