"""Real slotted CAD exposes separate selectable seams without joining loops."""
import json
from pathlib import Path
import unittest
import time
from unittest.mock import patch
import numpy as np
from welding_sim.runner import object_geometry
from welding_sim.workpiece_model import preview_object
from welding_sim.web_control import Controller,Settings,geometry_payload
from welding_sim.profiles import ISAAC, JAKA

ROOT=Path(__file__).resolve().parents[3]


class WebSeamTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=json.loads((Path(__file__).resolve().parent/'fixtures/slotted-88x40x5.json').read_text())
        cls.geometry=object_geometry(preview_object(cls.model))

    def test_upper_catalog_contains_outer_slot_and_round_hole(self):
        payload=geometry_payload(self.geometry,Settings())
        seams=[s for s in payload['seams'] if s['normal'][2]>.5]
        self.assertEqual(len(seams),3)
        self.assertTrue(any('внешний' in s['label'] for s in seams))
        self.assertTrue(any('прорези' in s['label'] for s in seams))
        self.assertTrue(any('отверстия' in s['label'] for s in seams))
        outer=next(s for s in seams if s['kind']=='outer')
        surface=next(f for f in payload['faces'] if f['id']==outer['face_id'])
        self.assertEqual(len(surface['loops']),3)

    def test_each_selected_seam_is_closed_and_sent_to_cli_by_id(self):
        for name,c in self.geometry.contours.items():
            if c.outward[2]<.5:continue
            settings=Settings.parse(dict(selection='face',face=name,step=.001))
            payload=geometry_payload(self.geometry,settings)
            self.assertEqual(payload['selected_face'],name)
            seam=np.array(payload['seam']);tool=np.array(payload['tool'])
            np.testing.assert_allclose(seam[0],seam[-1],atol=1e-10)
            self.assertLess(np.max(np.linalg.norm(np.diff(seam,axis=0),axis=1)),.001001)
            np.testing.assert_allclose(tool-seam,np.tile(.03*c.outward,(len(seam),1)),atol=1e-10)
            argv=settings.argv('/weld.py','plan','/tmp/test')
            self.assertEqual(argv[argv.index('--face')+1],name)

    def test_partial_hole_has_requested_length_without_leaving_loop(self):
        name=next(k for k,c in self.geometry.contours.items() if c.kind=='hole' and c.outward[2]>.5)
        c=self.geometry.contour(name)
        payload=geometry_payload(self.geometry,Settings(face=name,begin=.25,end=.75))
        self.assertAlmostEqual(payload['length'],c.length*.5)
        seam=np.array(payload['seam'])
        self.assertFalse(np.allclose(seam[0],seam[-1]))
        self.assertTrue(np.all(seam>=c.vertices.min(0)-1e-8))
        self.assertTrue(np.all(seam<=c.vertices.max(0)+1e-8))

    def test_cached_cad_can_be_selected_without_connected_moveit(self):
        controller=Controller(ROOT,ROOT/'install/ur10_isaac_moveit_config/lib/ur10_isaac_moveit_config/weld.py',profile=JAKA)
        try:
            with patch('welding_sim.runner.object_geometry',wraps=object_geometry) as parser:
                payload=controller.geometry(Settings())
                controller.geometry(Settings())
                parser.assert_called_once()
            self.assertEqual(payload['source'],'cached_cad')
            self.assertEqual(payload['shape'],'mesh')
            selected=json.loads((ROOT/'src/jaka_zu12_moveit_config/config/workpiece_model.json').read_text())
            self.assertEqual(payload['part_name'],selected['part_name'])
            self.assertTrue(payload['demo'])
        finally:controller.close()


    def test_catalog_meshes_offer_only_complete_relevant_contours(self):
        directory = ROOT/'src/jaka_zu12_moveit_config/config/workpiece_models'
        for part, ids in (
                ('02-tee-gusset', {
                    'seam_tee_base_back', 'seam_tee_base_front_left', 'seam_tee_base_front_right',
                    'seam_gusset_base_left', 'seam_gusset_base_right', 'seam_gusset_base_toe',
                    'seam_gusset_web_left', 'seam_gusset_web_right', 'seam_gusset_web_top',
                }),
                ('04-pipe-plate', {'seam_pipe_base'}),
                ('05-wave-butt', {'seam_center'})):
            with self.subTest(part=part):
                model = json.loads((directory/f'{part}.json').read_text())
                obj = preview_object(model)
                obj.type.key = f'cad:{part}'
                geometry = object_geometry(obj)
                for profile in (ISAAC, JAKA):
                    payload = geometry_payload(geometry, Settings(selection='edge'), profile=profile)
                    self.assertEqual({seam['id'] for seam in payload['seams']}, ids)
                self.assertEqual(payload['edges'], [])
                self.assertGreater(len(payload['faces']), len(ids))
                self.assertGreater(len(payload['tool']), 1)
                controller = Controller(ROOT, ROOT/'weld.py', profile=JAKA)
                controller.last_telemetry = time.monotonic()
                controller.telemetry['moveit'] = True
                controller.object_edges[obj.id] = geometry
                omitted = next(name for name in geometry.contours if name not in ids)
                with patch('welding_sim.web_control.subprocess.Popen') as launch:
                    for settings in (Settings(face=omitted), Settings(selection='edge', edge=geometry.edges[0].id)):
                        with self.assertRaises(ValueError):
                            controller.start('preview', settings)
                    launch.assert_not_called()

    def test_gusset_routes_follow_joint_lines_and_skip_the_covered_wall_span(self):
        path = ROOT/'src/jaka_zu12_moveit_config/config/workpiece_models/02-tee-gusset.json'
        model = json.loads(path.read_text())
        model['initial_pose']['position_m'] = [0., 0., 0.]
        obj = preview_object(model)
        obj.type.key = 'cad:02-tee-gusset'
        geometry = object_geometry(obj)
        obj.type.db = ''  # Scene loaded before the gusset contained authored seams.
        self.assertEqual(object_geometry(obj).selectable_seams, geometry.selectable_seams)

        endpoints = {
            'seam_tee_base_front_left': [[-100, 3, 7], [70, 3, 7]],
            'seam_tee_base_front_right': [[78, 3, 7], [100, 3, 7]],
            'seam_gusset_base_left': [[70, 3, 7], [70, 42, 7]],
            'seam_gusset_web_right': [[78, 3, 8], [78, 3, 47]],
        }
        for name, expected in endpoints.items():
            with self.subTest(seam=name):
                payload = geometry_payload(geometry, Settings(face=name, offset=.008))
                self.assertEqual(payload['selected_face'], name)
                seam, tool = np.array(payload['seam']), np.array(payload['tool'])
                np.testing.assert_allclose(seam[[0,-1]] * 1000, expected)
                np.testing.assert_allclose(np.linalg.norm(tool-seam, axis=1), .008)

    def test_pipe_joint_is_closed_and_offset_away_from_both_surfaces(self):
        path = ROOT/'src/jaka_zu12_moveit_config/config/workpiece_models/04-pipe-plate.json'
        model = json.loads(path.read_text())
        model['initial_pose']['position_m'] = [.12, -.65, .002]
        obj = preview_object(model)
        obj.type.key = 'cad:04-pipe-plate'
        payload = geometry_payload(object_geometry(obj), Settings(face='seam_pipe_base', offset=.008))
        seam, tool = np.asarray(payload['seam']), np.asarray(payload['tool'])
        self.assertEqual(payload['selected_face'], 'seam_pipe_base')
        np.testing.assert_allclose(seam[0], seam[-1], atol=1e-10)
        np.testing.assert_allclose(seam[:, 2], .008)
        radii = np.linalg.norm(seam[:, :2]-[.12,-.65], axis=1)
        np.testing.assert_allclose(radii, .05, atol=1.1e-5)
        np.testing.assert_allclose(tool[:, 2]-seam[:, 2], .008/np.sqrt(2))
        np.testing.assert_allclose(np.linalg.norm(tool[:, :2]-[.12,-.65], axis=1)-radii,
                                   .008/np.sqrt(2))
        np.testing.assert_allclose(np.linalg.norm(tool-seam, axis=1), .008)

    def test_temporary_moveit_timeout_keeps_live_part_and_disables_readiness(self):
        controller = Controller(ROOT, ROOT/'weld.py', profile=JAKA)
        controller.scene_received = True
        controller.object_edges['welding_workpiece'] = self.geometry
        controller.telemetry['objects'] = [{'id': 'welding_workpiece'}]
        payload = controller.geometry(Settings())
        self.assertEqual(payload['source'], 'last_moveit')
        self.assertFalse(payload['demo'])
        self.assertFalse(controller.status()['health']['moveit'])
        self.assertEqual(controller.status()['health']['objects'], [{'id': 'welding_workpiece'}])

    def test_missing_live_part_waits_instead_of_substituting_startup_model(self):
        controller = Controller(ROOT, ROOT/'weld.py', profile=JAKA)
        controller.scene_generation = 1
        self.assertEqual(controller.geometry(Settings()), {'source': 'awaiting_scene', 'scene_generation': 1})
        controller.object_edges['welding_workpiece'] = self.geometry
        payload = controller.geometry(Settings())
        self.assertEqual(payload['scene_generation'], 1)
        self.assertEqual(controller.status()['scene']['generation'], 1)


if __name__=='__main__':unittest.main()
