"""Known measurements exercise registration without a scanner or robot."""
import tempfile
from pathlib import Path
import unittest
import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from welding_sim.scan_alignment import (align_translation, read_cloud, rigid_fit,
                                        transform_points, validate_transform)
from welding_sim.scan_model import sample_object


class ScanAlignmentTests(unittest.TestCase):
    def model(self):
        obj = CollisionObject(id='welding_workpiece')
        obj.pose.orientation.w = 1.
        pose = Pose();pose.orientation.w = 1.;pose.position.y = -.6;pose.position.z = .1
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.3, .2, .2])]
        obj.primitive_poses = [pose]
        return sample_object(obj, spacing=.002)

    def test_known_translation_of_three_visible_faces(self):
        model, normals = self.model()
        mask = np.sum(normals, axis=1) > .5
        delta = np.array([.012, -.009, .006])
        scan = model[mask][::3]+delta
        result = align_translation(scan, model, normals, tolerance=.003)
        np.testing.assert_allclose(result['translation_m'], delta, atol=.0002)

    def test_single_plane_is_ambiguous(self):
        model, normals = self.model()
        with self.assertRaisesRegex(ValueError, 'Ambiguous'):
            align_translation(model[normals[:, 2] > .5], model, normals)

    def test_wrong_shape_is_rejected(self):
        model, normals = self.model()
        with self.assertRaises(ValueError):
            align_translation(model*1.3, model, normals)

    def test_rigid_calibration_and_reflection_rejection(self):
        source = np.array([[0,0,0],[.2,0,0],[0,.2,0],[0,0,.2]])
        matrix = np.eye(4);matrix[:3,3] = [.1,-.6,.2]
        matrix[:3,:3] = [[0,-1,0],[1,0,0],[0,0,1]]
        target = transform_points(source, matrix)
        np.testing.assert_allclose(rigid_fit(source,target),matrix,atol=1e-12)
        matrix[0,0] = 2
        with self.assertRaises(ValueError):validate_transform(matrix)

    def test_ascii_units_and_property_order(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'scan.ply'
            path.write_text('ply\nformat ascii 1.0\nelement vertex 30\n'
                            'property float z\nproperty float x\nproperty float y\n'
                            'end_header\n'+('300 100 200\n'*30))
            points = read_cloud(path,'mm')
            np.testing.assert_allclose(points[0],[.1,.2,.3])
            path.write_text('ply\nformat binary_little_endian 1.0\nelement vertex 30\nend_header\n')
            with self.assertRaises(ValueError):read_cloud(path,'mm')


if __name__ == '__main__':
    unittest.main()
