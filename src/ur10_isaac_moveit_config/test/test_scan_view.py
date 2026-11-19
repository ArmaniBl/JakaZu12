"""View leveling preserves metric geometry and the assigned camera heading."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from scan_view import level_rotation, table_height_anchor, yaw_about_point, scaled_preview_points


class ScanViewTests(unittest.TestCase):
    def test_measured_scale_preserves_shape_and_anchor(self):
        anchor=np.array([.04,-.08,.44]);points=anchor+np.array([[0.,0.,0.],[.2,0.,0.],[0.,.12,0.]])
        before=points.copy();factor=1.1178180784627967
        result=scaled_preview_points(points,anchor,factor)
        np.testing.assert_array_equal(points,before)
        np.testing.assert_allclose(result[0],[0,0,0])
        np.testing.assert_allclose(np.linalg.norm(np.diff(result,axis=0),axis=1),
            factor*np.linalg.norm(np.diff(points,axis=0),axis=1))
        for invalid in [0,-1,float('nan'),float('inf')]:
            with self.assertRaises(ValueError):scaled_preview_points(points,anchor,invalid)

    def reference(self):
        rng = np.random.default_rng(4)
        angle = np.radians(38.)
        normal = np.array([0.,np.cos(angle),-np.sin(angle)])
        x = np.array([1.,0.,0.])
        y = np.cross(normal,x)
        uv = rng.uniform(-.1,.1,(1200,2))
        plane = uv[:,0,None]*x+uv[:,1,None]*y+np.array([.02,-.03,.4])
        return plane,normal

    def test_level_plane_and_preserve_all_distances(self):
        points, normal = self.reference()
        result = level_rotation(points,-90.)
        rotation = np.array(result['camera_to_world_rotation'])
        np.testing.assert_allclose(rotation@normal,[0.,0.,1.],atol=1e-10)
        np.testing.assert_allclose(rotation.T@rotation,np.eye(3),atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(rotation),1.)
        moved = points@rotation.T
        self.assertLess(np.ptp(moved[:,2]),1e-10)
        np.testing.assert_allclose(np.linalg.norm(np.diff(points,axis=0),axis=1),
                                   np.linalg.norm(np.diff(moved,axis=0),axis=1),atol=1e-12)
        # Optical forward now points along increasing table rows (world +X), down.
        np.testing.assert_allclose(rotation@np.array([0.,0.,1.]),
                                   [np.cos(np.radians(38.)),0.,-np.sin(np.radians(38.))],atol=1e-10)

    def test_outliers_do_not_tilt_reference(self):
        points, normal = self.reference()
        outliers = np.random.default_rng(5).uniform(-.2,.2,(200,3))
        result = level_rotation(np.vstack([points,outliers]),-90.)
        np.testing.assert_allclose(np.array(result['camera_to_world_rotation'])@normal,[0.,0.,1.],atol=.002)

    def test_wrong_reference_is_rejected(self):
        rng = np.random.default_rng(6)
        with self.assertRaises(ValueError):
            level_rotation(rng.uniform(-.2,.2,(1500,3)),-90.)

    def test_table_height_changes_only_z_without_deforming_geometry(self):
        points,_ = self.reference()
        rotation = np.array(level_rotation(points,-90.)['camera_to_world_rotation'])
        anchor = points.mean(0)+.06*rotation[2]
        before = (points-anchor)@rotation.T
        updated,report = table_height_anchor(points,anchor,rotation)
        after = (points-updated)@rotation.T
        self.assertAlmostEqual(report['world_z_correction_m'],.06)
        self.assertAlmostEqual(float(np.percentile(after[:,2],2)),0.)
        np.testing.assert_allclose(after[:,:2],before[:,:2],atol=1e-12)
        np.testing.assert_allclose(np.diff(after,axis=0),np.diff(before,axis=0),atol=1e-12)

    def test_yaw_preserves_reference_height_and_geometry(self):
        points,_ = self.reference()
        rotation = np.array(level_rotation(points,-90.)['camera_to_world_rotation'])
        translation = np.array([-.3,-.7,0.])
        pivot = points[0]@rotation.T+translation
        before = points@rotation.T+translation
        adjusted,center = yaw_about_point(rotation,translation,-57.,pivot)
        after = points@adjusted.T+center
        np.testing.assert_allclose(after[0],pivot,atol=1e-12)
        np.testing.assert_allclose(before[:,2],after[:,2],atol=1e-12)
        np.testing.assert_allclose(np.linalg.norm(np.diff(before,axis=0),axis=1),
                                   np.linalg.norm(np.diff(after,axis=0),axis=1),atol=1e-12)


if __name__ == '__main__':
    unittest.main()
