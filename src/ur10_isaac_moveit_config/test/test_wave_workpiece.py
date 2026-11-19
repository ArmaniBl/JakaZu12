"""Imported CAD weld path survives scene transport, transforms and selection."""
from copy import deepcopy
import json
from pathlib import Path
import unittest
import numpy as np
from welding_sim.workpiece_model import preview_object,apply_model_seams
from welding_sim.runner import object_geometry
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

    def test_metadata_cannot_follow_another_mesh(self):
        geometry=object_geometry(self.obj);changed=deepcopy(self.obj)
        changed.meshes[0].vertices[0].x+=.0001
        with self.assertRaisesRegex(ValueError,'do not belong'):
            apply_model_seams(changed,geometry,np.zeros(3),[0,0,0,1])


if __name__=='__main__':unittest.main()
