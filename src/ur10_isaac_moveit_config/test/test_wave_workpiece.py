"""Imported CAD weld path survives scene transport, transforms and selection."""
from copy import deepcopy
import json
from pathlib import Path
import unittest
from types import SimpleNamespace
import numpy as np
from welding_sim.workpiece_model import preview_object,apply_model_seams
from welding_sim.runner import object_geometry, WeldingRunner, PROBE_SPEED_M_S, PROBE_STANDOFF_M
from welding_sim.web_control import geometry_payload,Settings
from welding_sim.geometry import rotation

MODEL=Path(__file__).resolve().parents[2]/'jaka_zu12_moveit_config/config/workpiece_models/05-wave-butt.json'


class WaveWorkpieceTests(unittest.TestCase):
    def setUp(self):
        self.model=json.loads(MODEL.read_text())
        self.obj=preview_object(self.model)

    def test_complete_collision_mesh_and_open_center_seam(self):
        vertices=np.array(self.model['vertices_mm'])
        np.testing.assert_allclose(np.ptp(vertices,axis=0),[200,120,6])
        self.assertEqual(len(self.obj.meshes[0].triangles),len(self.model['triangles']))
        geometry=object_geometry(self.obj)
        payload=geometry_payload(geometry,Settings(face='face_1'))
        self.assertEqual([s['id'] for s in payload['seams']],['seam_center'])
        self.assertEqual(payload['selected_face'],'seam_center')
        self.assertFalse(any(f['id']=='seam_center' for f in payload['faces']))
        points=np.array(payload['seam'])
        self.assertFalse(np.allclose(points[0],points[-1]))
        self.assertAlmostEqual(payload['length'],.143331602378414,places=9)
        self.assertEqual(len(self.model['seams'][0]['vertices_mm']),161)

    def test_local_seam_follows_object_and_mesh_pose(self):
        self.obj.pose.position.x=.1;self.obj.pose.position.y=-.2
        self.obj.pose.orientation.z=2**-.5;self.obj.pose.orientation.w=2**-.5
        self.obj.mesh_poses[0].position.x=.02
        geometry=object_geometry(self.obj);seam=geometry.contour('seam_center')
        local=np.array(self.model['seams'][0]['vertices_mm'])*.001
        R=rotation([0,0,2**-.5,2**-.5])
        expected=(local+[.02,0,0])@R.T+[.1,-.2,0]
        np.testing.assert_allclose(seam.vertices,expected,atol=1e-12)

    def test_partial_path_is_sent_to_runner_without_closing_the_seam(self):
        settings=Settings(face='seam_center',begin=.25,end=.75,step=.001)
        payload=geometry_payload(object_geometry(self.obj),settings)
        points=np.array(payload['seam']);tool=np.array(payload['tool'])
        self.assertAlmostEqual(payload['length'],.143331602378414/2)
        self.assertLess(np.max(np.linalg.norm(np.diff(points,axis=0),axis=1)),.001001)
        np.testing.assert_allclose(tool-points,np.tile([0,0,.03],(len(points),1)),atol=1e-10)
        argv=settings.argv('/weld.py','plan','/tmp/test')
        self.assertEqual(argv[argv.index('--face')+1],'seam_center')

    def test_probe_preview_alternates_between_both_plates(self):
        payload = geometry_payload(object_geometry(self.obj),
                                   Settings(face='seam_center', begin=.2, end=.8))
        contacts = payload['probe_contacts']
        self.assertEqual([(p['station'], p['side']) for p in contacts],
                         [(1, 'A'), (1, 'B'), (2, 'A'), (2, 'B')])
        self.assertNotEqual(contacts[0]['surface'], contacts[1]['surface'])
        self.assertEqual([p['surface'] for p in contacts[:2]],
                         [p['surface'] for p in contacts[2:]])
        np.testing.assert_allclose([p['position_m'][2] for p in contacts], .006)

    def test_probe_plan_uses_contact_and_retract_in_order(self):
        payload = geometry_payload(object_geometry(self.obj), Settings(face='seam_center'))
        calls = []
        planner = SimpleNamespace(args=SimpleNamespace(clearance=.03), cartesian_targets={}, report={},
                                  approach=lambda *args: object(), end_state=lambda state, _: state)

        def cartesian(state, waypoints, *, speed_limit):
            self.assertEqual(speed_limit, PROBE_SPEED_M_S)
            calls.append(waypoints[0].position.z)
            return object()

        planner.cartesian = cartesian
        edge = SimpleNamespace(id='seam_center')
        _, stages = WeldingRunner.plan_probe(planner, object(), edge, payload['probe_contacts'],
                                             [0., 0., 0., 1.])
        expected = [f'seam_center:probe:{point}:{motion}' for point in ('1A', '1B', '2A', '2B')
                    for motion in ('approach', 'contact', 'retract')]
        self.assertEqual([name for name, _ in stages], expected)
        np.testing.assert_allclose(calls, [.006, .036] * 4)

        planner.args.probe_standoff = True
        planner.args.offset = .02
        calls.clear()
        _, stages = WeldingRunner.plan_probe(planner, object(), edge, payload['probe_contacts'],
                                             [0., 0., 0., 1.])
        self.assertEqual([name for name, _ in stages],
                         [name.replace(':contact', ':standoff') for name in expected])
        np.testing.assert_allclose(calls, [.006 + PROBE_STANDOFF_M, .036 + PROBE_STANDOFF_M] * 4)
        self.assertAlmostEqual(planner.report['probing'][-1]['standoff_m'], PROBE_STANDOFF_M)

        def blocked(*args, **kwargs):
            raise RuntimeError('Incomplete Cartesian path')

        planner.cartesian = blocked
        planner.args.probe_standoff = False
        with self.assertRaisesRegex(RuntimeError, '1A contact failed'):
            WeldingRunner.plan_probe(planner, object(), edge, payload['probe_contacts'],
                                     [0., 0., 0., 1.])

    def test_tilted_wave_seam_levels_tcp_without_reducing_clearance(self):
        # Simulate residual pitch from registration of the nominally flat plate.
        self.obj.pose.orientation.x = np.sin(.005)
        self.obj.pose.orientation.w = np.cos(.005)
        geometry = object_geometry(self.obj)
        edge = geometry.contour('seam_center')
        payload = geometry_payload(geometry, Settings(face='seam_center', offset=.008))
        seam, path = np.asarray(payload['seam']), np.asarray(payload['tool'])
        original_path = seam + .008 * edge.outward
        self.assertGreater(np.ptp(seam[:, 2]), .001)
        np.testing.assert_allclose(path[:, :2], original_path[:, :2])
        np.testing.assert_allclose(path[:, 2], original_path[:, 2].max())

        contacts = payload['probe_contacts']
        self.assertEqual(len(contacts), 4)
        original_probes = np.array([np.asarray(p['position_m']) + PROBE_STANDOFF_M * np.asarray(p['normal'])
                                    for p in contacts])
        planner = SimpleNamespace(args=SimpleNamespace(clearance=.03, probe_standoff=True),
                                  cartesian_targets={}, report={}, approach=lambda *args: object(),
                                  end_state=lambda state, _: state, cartesian=lambda *args, **kwargs: object())
        WeldingRunner.plan_probe(planner, object(), edge, contacts, [0., 0., 0., 1.])
        targets = np.asarray(planner.report['probing'][0]['tcp_targets_xyz_m'])
        self.assertGreater(np.ptp(original_probes[:, 2]), .0005)
        np.testing.assert_allclose(targets[:, :2], original_probes[:, :2])
        np.testing.assert_allclose(targets[:, 2], original_probes[:, 2].max())
        planned_z = [poses[0].position.z for name, poses in planner.cartesian_targets.items()
                     if name.endswith(':standoff')]
        np.testing.assert_allclose(planned_z, targets[:, 2])

    def test_metadata_cannot_follow_another_mesh(self):
        geometry=object_geometry(self.obj);changed=deepcopy(self.obj)
        changed.meshes[0].vertices[0].x+=.0001
        with self.assertRaisesRegex(ValueError,'do not belong'):
            apply_model_seams(changed,geometry,np.zeros(3),[0,0,0,1])


if __name__=='__main__':unittest.main()
