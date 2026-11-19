"""Bounded stereo preview correction with held-out measured correspondences."""
import importlib.util
from pathlib import Path
import unittest
import cv2
import numpy as np

HELPER=Path(__file__).resolve().parents[3]/'integrations/stereo_refinement.py'
spec=importlib.util.spec_from_file_location('stereo_refinement',HELPER)
stereo=importlib.util.module_from_spec(spec);spec.loader.exec_module(stereo)


class StereoRefinementTests(unittest.TestCase):
    def setUp(self):
        self.k=np.array([[3000.,0,1296],[0,3000.,972],[0,0,1.]])
        self.cameras=[dict(K=self.k.tolist(),distortion=np.zeros(5).tolist()) for _ in range(2)]
        x,y=np.meshgrid(np.linspace(-.06,.06,60),np.linspace(-.04,.04,45))
        self.xyz=np.c_[x.ravel(),y.ravel(),(.5+.002*np.sin(x*40)).ravel()]
        self.translation=np.array([-.1,0,0])

    def pixels(self, angle):
        rng=np.random.default_rng(5)
        values=[]
        for rv,tv in [(np.zeros(3),np.zeros(3)),(np.deg2rad([angle,0.,0.]),self.translation)]:
            points=cv2.projectPoints(self.xyz,rv,tv,self.k,np.zeros(5))[0][:,0]
            values.append(points+rng.normal(0,.08,points.shape))
        return values

    def test_small_angular_mismatch_recovers_real_points_without_moving_baseline(self):
        pixels=self.pixels(.1);baseline=self.translation.copy()
        rotation,report=stereo.refine_preview_rotation(pixels,self.cameras,np.eye(3),self.translation)
        xyz,positive,error=stereo.triangulate(pixels,self.cameras,rotation,self.translation)
        self.assertTrue(positive.all());self.assertLess(np.percentile(error,95),1.5)
        self.assertLess(np.median(np.linalg.norm(xyz-self.xyz,axis=1)),.002)
        np.testing.assert_array_equal(self.translation,baseline)
        self.assertLess(report['angle_deg'],.2)
        self.assertGreater(report['validation_pairs'],1000)
        self.assertFalse(report['execution_ready'])
        self.assertEqual(report['filled_or_cad_generated_points'],0)

    def test_valid_geometry_is_not_refitted(self):
        with self.assertRaises(ValueError):
            stereo.refine_preview_rotation(self.pixels(0),self.cameras,np.eye(3),self.translation)

    def test_large_plate_coverage_keeps_geometry_quality_and_metric_depth(self):
        x,y=np.meshgrid(np.linspace(-.1,.1,90),np.linspace(-.06,.06,60))
        self.xyz=np.c_[x.ravel(),y.ravel(),np.full(x.size,.44)]
        pixels=self.pixels(.115)
        rotation,report=stereo.refine_preview_rotation(pixels,self.cameras,np.eye(3),self.translation)
        xyz,positive,error=stereo.triangulate(pixels,self.cameras,rotation,self.translation)
        self.assertLess(report['validation_p95_px'],1.5)
        self.assertGreater(np.mean(positive&(error<=2)),.97)
        self.assertLess(np.median(np.linalg.norm(xyz-self.xyz,axis=1)),.002)

    def test_large_mismatch_is_rejected(self):
        with self.assertRaises(ValueError):
            stereo.refine_preview_rotation(self.pixels(2),self.cameras,np.eye(3),self.translation)

    def test_sparse_or_unrelated_pairs_are_rejected(self):
        pixels=self.pixels(.1)
        with self.assertRaises(ValueError):
            stereo.refine_preview_rotation([p[:100] for p in pixels],self.cameras,np.eye(3),self.translation)
        np.random.default_rng(2).shuffle(pixels[1])
        with self.assertRaises(ValueError):
            stereo.refine_preview_rotation(pixels,self.cameras,np.eye(3),self.translation)


if __name__=='__main__':unittest.main()
