"""Exercise the adapter installed in the source scanner without its CAD SDK."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace, ModuleType
import sys
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation


class ScannerExportTests(unittest.TestCase):
    def test_export_recovers_camera_frame_and_metric_cad_pose(self):
        path=Path(__file__).resolve().parents[3]/'integrations/kvantron_export.py'
        spec=importlib.util.spec_from_file_location('scan_export_fixture',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        geometry=ModuleType('adaptive_weld.geometry')
        def matrix(position,rotation):
            value=np.eye(4);value[:3,:3]=Rotation.from_euler('xyz',rotation,degrees=True).as_matrix()
            value[:3,3]=position;return value
        geometry.transform_matrix=matrix
        context=dict(points=[[1.,2.,3.]]*100,unit='grid_pitch',synthetic=False,
                     pose=dict(scale=10.,position=[100.,0.,0.],rotation=[0.,0.,0.]),
                     provenance=dict(calibration_sha256='measured-calibration'))
        part=SimpleNamespace(name='measured part',position=[120.,0.,0.],rotation=[0.,0.,0.],
                             mesh=SimpleNamespace(vertices=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]),faces=np.array([[0,1,2]])))
        camera=np.eye(4);camera[2,3]=50.
        with patch.dict(sys.modules,{'adaptive_weld':ModuleType('adaptive_weld'),'adaptive_weld.geometry':geometry}):
            result=module.export_bundle(context,part,'frame',camera,True,'task','capture')
            np.testing.assert_allclose(result['camera_points_mm'][0],[10,20,80])
            np.testing.assert_allclose(np.array(result['camera_T_cad_mm'])[:3,3],[20,0,50])
            again=module.export_bundle(context,part,'frame',camera,True,'task','capture')
            self.assertEqual(result['identity'],again['identity'])
            part.position[0]+=1
            changed=module.export_bundle(context,part,'frame',camera,True,'task','capture')
            self.assertNotEqual(result['identity'],changed['identity'])
            with self.assertRaises(ValueError):module.export_bundle(context,part,'frame',camera,False,'task','capture')


if __name__=='__main__':unittest.main()
