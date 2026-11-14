"""Tilt bounds, unchanged TCP path, retries including return and safe rejection."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
import numpy as np
from welding_sim.geometry import rotation
from welding_sim.orientation import candidates,tilted_orientation
from welding_sim.runner import WeldingRunner
from welding_sim.web_control import Settings,geometry_payload
from welding_sim.geometry import box_geometry

class OrientationTests(unittest.TestCase):
    def test_tilt_angle_and_orthonormal_rotation(self):
        for tilt in (0,15,30,60):
            for direction in (0,90,230):
                r=rotation(tilted_orientation([0,0,0,1],tilt,direction,70))
                np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-12)
                self.assertAlmostEqual(r[2,2],np.cos(np.deg2rad(tilt)))
        with self.assertRaises(ValueError):tilted_orientation([0,0,0,1],61)

    def test_candidates_respect_limit_and_are_bounded(self):
        values=list(candidates(10,20,35))
        self.assertEqual(values[0],(10,20,0))
        self.assertTrue(all(0<=t<=35 for t,_,_ in values))
        self.assertEqual(len(values),len(set(values)))
        self.assertLess(len(values),70)

    def stub(self,mode='auto'):
        return SimpleNamespace(args=SimpleNamespace(orientation_mode=mode,tilt=10.,tilt_direction=20.,
            max_tilt=45.,orientation_timeout=5.),report={},get_logger=lambda:SimpleNamespace(info=Mock()))

    def paths(self):
        return [(SimpleNamespace(id='contour:test'),np.array([[0.,0,0],[1.,0,0]]),
                 np.array([[0.,0,.03],[1.,0,.03]]),np.array([0.,0,0,1.]))]

    def test_retry_after_failed_return_keeps_tcp_path_and_clears_old_targets(self):
        node=self.stub();paths=self.paths();calls=[]
        def plan(stub,candidate):
            np.testing.assert_array_equal(candidate[0][1],paths[0][1])
            np.testing.assert_array_equal(candidate[0][2],paths[0][2])
            self.assertEqual(stub.cartesian_targets,{})
            calls.append(candidate)
            if len(calls)==1:
                stub.cartesian_targets['partial']='must be cleared'
                raise RuntimeError('return blocked')
            return [('workpiece:return','trajectory')]
        with patch.object(WeldingRunner,'plan_fixed',side_effect=plan):
            result=WeldingRunner.plan(node,paths)
        self.assertEqual(result[-1][0],'workpiece:return')
        self.assertEqual(len(node.report['orientation_attempts']),2)
        self.assertEqual(node.report['chosen_orientation']['status'],'planned')
        self.assertIsNone(node.orientation_deadline)

    def test_fixed_rejects_without_retry_and_budget_expires_without_planning(self):
        with patch.object(WeldingRunner,'plan_fixed',side_effect=RuntimeError('blocked')) as plan:
            with self.assertRaisesRegex(RuntimeError,'blocked'):WeldingRunner.plan(self.stub('fixed'),self.paths())
            self.assertEqual(plan.call_count,1)
        with patch.object(WeldingRunner,'plan_fixed') as plan,patch('welding_sim.runner.time.monotonic',side_effect=[0.,10.]):
            with self.assertRaises(RuntimeError):WeldingRunner.plan(self.stub(),self.paths())
            plan.assert_not_called()

    def test_web_validation_and_preview_preserve_positions(self):
        for data in ({'tilt':46,'max_tilt':45},{'orientation_mode':'other'},{'orientation_timeout':0},
                     {'max_tilt':61},{'tilt':True},{'tilt_direction':float('nan')}):
            with self.assertRaises(ValueError):Settings.parse(data)
        g=box_geometry([.3,.3,.2])
        flat=geometry_payload(g,Settings())
        tilted=geometry_payload(g,Settings(tilt=30))
        self.assertEqual(flat['seam'],tilted['seam']);self.assertEqual(flat['tool'],tilted['tool'])
        self.assertNotEqual(flat['orientation'],tilted['orientation'])
        args=Settings.parse({'tilt':30,'orientation_mode':'fixed'}).argv('/tmp/weld.py','plan','/tmp/output')
        self.assertIn('--orientation-mode',args);self.assertIn('--tilt',args)

if __name__=='__main__':unittest.main()
