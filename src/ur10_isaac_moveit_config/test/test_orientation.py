"""Tilt bounds, unchanged TCP path, retries including return and safe rejection."""
import unittest
from types import SimpleNamespace
import numpy as np
from welding_sim.geometry import rotation
from welding_sim.orientation import (
    aimed_orientations, candidates, orbit_orientations, pipe_plate_orientations, radial_orientations,
    route_candidates, swept_orientations, tilted_orientation,
)
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

    def test_radial_tilt_tracks_the_joint_without_changing_tool_roll_at_closure(self):
        up = np.array([0.,0.,1.])
        seam = np.array([[1.,0.,0.],[0.,1.,0.],[-1.,0.,0.],[0.,-1.,0.],[1.,0.,0.]])
        orientations = radial_orientations([1.,0.,0.,0.], seam, np.zeros(3), up, 30., 90.)
        axes = np.array([rotation(q)[:,2] for q in orientations])
        np.testing.assert_allclose(axes, -.5 * seam - np.sqrt(3) / 2 * up, atol=1e-12)
        np.testing.assert_allclose(orientations[0], orientations[-1], atol=1e-12)

    def test_aimed_tilt_points_toward_the_joint_and_respects_limit(self):
        seam = np.array([[0., 0., 0.], [1., 0., 0.]])
        points = seam + [[.003, 0., .010], [0., .030, .010]]
        q = aimed_orientations([1., 0., 0., 0.], seam, points, 45., 0., 90.)
        axes = np.array([rotation(item)[:, 2] for item in q])
        aim = seam[0] - points[0]
        np.testing.assert_allclose(axes[0], aim / np.linalg.norm(aim), atol=1e-12)
        np.testing.assert_allclose(axes[1], [0., -np.sqrt(.5), -np.sqrt(.5)], atol=1e-12)

    def test_orbit_keeps_torch_relative_to_the_pipe_and_closes(self):
        seam = np.array([[1., 0., 0.], [0., 1., 0.], [-1., 0., 0.],
                         [0., -1., 0.], [1., 0., 0.]])
        up = np.array([0., 0., 1.])
        tool = orbit_orientations([1., 0., 0., 0.], seam, np.zeros(3), up, 30., -90.)
        rotations = np.array([rotation(q) for q in tool])
        np.testing.assert_allclose(rotations[:, :, 2], -.5 * seam - np.sqrt(3) / 2 * up, atol=1e-12)
        np.testing.assert_allclose(rotations[0], rotations[-1], atol=1e-12)
        np.testing.assert_allclose(rotations[1], [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]] @ rotations[0],
                                   atol=1e-12)

    def test_pipe_posture_is_sampled_by_angle_and_stays_within_tilt_limit(self):
        angles = np.radians(np.arange(361))
        seam = np.column_stack((-np.sin(angles), np.cos(angles), np.zeros(len(angles))))
        nominal = [1., 0., 0., 0.]
        q = pipe_plate_orientations(nominal, seam, np.zeros(3), np.array([0., 0., 1.]))
        axes = np.array([rotation(item)[:, 2] for item in q])
        self.assertTrue(np.all(axes[:, 2] <= -np.cos(np.radians(35.)) + 1e-10))
        self.assertLess(np.max(np.linalg.norm(np.diff(axes, axis=0), axis=1)), .08)
        self.assertFalse(np.allclose(axes[0], axes[-1]))

    def test_ring_search_preserves_selected_samples_including_partial_arcs(self):
        q = np.array([1., 0., 0., 0.])
        edge = SimpleNamespace(id='seam_pipe_base', kind='weld', radial_offset_center=np.zeros(3))
        closed = np.array([[1., 0., 0.], [0., 1., 0.], [-1., 0., 0.],
                           [0., -1., 0.], [1., 0., 0.]])
        options = list(candidates(0., 0., 45.))
        for seam in (closed, closed[:3]):
            points = seam + [0., 0., .01]
            variants = route_candidates([(edge, seam, points, q)], options, 45.,
                                        allow_reordering=True, near_joint=True)
            profile_orders = [order for mode, order, *_ in variants if mode == 'pipe_plate']
            expected = ['loop_quarter_1', 'loop_quarter_1_reverse'] if len(seam) == len(closed) else []
            self.assertEqual(profile_orders, expected)
            for _, _, [(_, joint, tcp, _)], tilt, _, _ in variants:
                self.assertLessEqual(tilt, 45.)
                self.assertEqual(set(map(tuple, seam)), set(map(tuple, joint)))
                np.testing.assert_allclose(tcp - joint, np.tile([0., 0., .01], (len(tcp), 1)))

    def test_swept_tilt_changes_direction_with_distance_along_the_seam(self):
        seam = np.array([[0.,0.,0.],[.1,0.,0.],[.3,0.,0.],[.6,0.,0.]])
        orientations = swept_orientations([0.,0.,0.,1.], seam, 30., 0., 90.)
        axes = np.array([rotation(q)[:,2] for q in orientations])
        angle = np.radians(30.) * np.cos(np.pi * seam[:,0] / .6)
        np.testing.assert_allclose(axes, np.column_stack((np.sin(angle), np.zeros(4), np.cos(angle))), atol=1e-12)

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
