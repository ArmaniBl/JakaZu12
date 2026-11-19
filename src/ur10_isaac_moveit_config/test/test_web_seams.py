"""Real slotted CAD exposes separate selectable seams without joining loops."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from welding_sim.runner import object_geometry
from welding_sim.workpiece_model import preview_object
from welding_sim.web_control import Controller,Settings,geometry_payload
from welding_sim.profiles import JAKA

ROOT=Path(__file__).resolve().parents[3]


class WebSeamTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=json.loads((ROOT/'src/jaka_zu12_moveit_config/config/workpiece_models/slotted-88x40x5.json').read_text())
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


if __name__=='__main__':unittest.main()
