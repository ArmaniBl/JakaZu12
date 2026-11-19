"""Local pose recovery and scene-based web planning, without hardware."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from welding_sim.scan_alignment import align_pose, transform_points
from welding_sim.scan_model import sample_object
from welding_sim.web_control import Controller, Settings
from welding_sim.profiles import JAKA
from welding_sim.geometry import box_edges


class ScanPoseTests(unittest.TestCase):
    def test_translation_and_rotation_of_three_visible_faces(self):
        obj=CollisionObject(id='welding_workpiece');obj.pose.orientation.w=1.
        pose=Pose();pose.orientation.w=1.;pose.position.y=-.6;pose.position.z=.075
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.15,.12,.15])]
        obj.primitive_poses=[pose]
        model,normals=sample_object(obj,spacing=.002)
        correction=np.eye(4);correction[:3,:3]=Rotation.from_euler('xyz',[2,-3,6],degrees=True).as_matrix()
        center=model.mean(0)
        correction[:3,3]=center+np.array([.012,-.008,.003])-correction[:3,:3]@center
        scan=transform_points(model[np.sum(normals,axis=1)>.5][::3],correction)
        recovered,quality=align_pose(scan,model,normals)
        np.testing.assert_allclose(recovered,correction,atol=.0002)
        self.assertLess(quality['p95_m'],.002)

    def test_single_plane_cannot_establish_pose(self):
        x,y=np.meshgrid(np.linspace(0,.2,30),np.linspace(0,.2,30))
        model=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))
        normals=np.tile([0.,0.,1.],(len(model),1))
        with self.assertRaisesRegex(ValueError,'ambiguous'):
            align_pose(model,model,normals)

    def test_web_uses_scene_without_calling_scan_checks(self):
        import tempfile
        import time
        with tempfile.TemporaryDirectory() as tmp:
            controller=Controller(tmp,Path(tmp)/'weld.py',JAKA)
            controller.telemetry.update(moveit=True,feedback=True,controller=True,isaac=True)
            controller.last_telemetry=time.monotonic()
            controller.object_edges['welding_workpiece']=box_edges([.04,.088,.005])
            controller.scan_command=Mock(side_effect=AssertionError('Scan checks must not run'))
            with patch('welding_sim.web_control.subprocess.Popen') as launch,patch('welding_sim.web_control.threading.Thread'):
                launch.return_value.poll.return_value=0
                controller.start('execute',Settings())
                command=launch.call_args.args[0]
                self.assertIn('--use-scene-workpiece',command)
                self.assertNotIn('--scan-report',command)
                self.assertNotIn('--scan-calibration',command)
                controller.scan_command.assert_not_called()
                controller.close()


if __name__=='__main__':unittest.main()
