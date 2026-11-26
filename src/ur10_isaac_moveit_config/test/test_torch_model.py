"""The optional tool changes the chain tip and provides real collision geometry."""
from pathlib import Path
import struct
import json
import subprocess
import unittest
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from welding_sim.profiles import load_profile

class TorchModelTests(unittest.TestCase):
    def setUp(self):
        self.package=Path(get_package_share_directory('ur10_isaac_moveit_config'))

    def expand(self,name,enabled):
        return ET.fromstring(subprocess.check_output(['xacro',str(self.package/'config'/name),
            f'torch:={"true" if enabled else "false"}','name:=ur10','ur_type:=ur10'],text=True))

    def test_optional_urdf_and_semantic_tip(self):
        for enabled in (False,True):
            urdf=self.expand('ur10.urdf.xacro',enabled)
            self.assertEqual(urdf.find("link[@name='welding_tcp']") is not None,enabled)
            srdf=self.expand('ur10.srdf.xacro',enabled)
            tip=srdf.find("group[@name='ur_manipulator']/chain").get('tip_link')
            self.assertEqual(tip,'welding_tcp' if enabled else 'tool0')

    def test_collision_and_profile_match_exported_tool(self):
        urdf=self.expand('ur10.urdf.xacro',True)
        link=urdf.find("link[@name='welding_torch_link']")
        # The original Isaac asset has one more box than the long-tip JAKA tool.
        self.assertEqual(len(link.findall('collision')),76)
        self.assertTrue(all(c.find('geometry/box') is not None for c in link.findall('collision')))
        path=self.package/'meshes/torch/collision.stl'
        data=path.read_bytes();count=struct.unpack('<I',data[80:84])[0]
        self.assertGreater(count,100)
        self.assertEqual(len(data),84+50*count)
        profile=load_profile('isaac_ur10',self.package/'config/ur10_torch_profile.json')
        self.assertEqual(profile.tip,'welding_tcp')
        self.assertEqual(profile.frames[-1],'welding_tcp')

    def test_standard_robot_models_include_torch_without_flags(self):
        for package,urdf_name,srdf_name,group in (
                ('ur10_isaac_moveit_config','ur10.urdf.xacro','ur10.srdf','ur_manipulator'),
                ('jaka_zu12_moveit_config','jaka_zu12.urdf.xacro','jaka_zu12.srdf','jaka_zu12')):
            config=Path(get_package_share_directory(package))/'config'
            mappings=['name:=ur10','ur_type:=ur10'] if package=='ur10_isaac_moveit_config' else []
            robot=ET.fromstring(subprocess.check_output(['xacro',str(config/urdf_name),*mappings],text=True))
            self.assertIsNotNone(robot.find("link[@name='welding_torch_link']/collision"))
            self.assertIsNotNone(robot.find("link[@name='welding_tcp']"))
            semantic=ET.parse(config/srdf_name).getroot()
            self.assertEqual(semantic.find(f"group[@name='{group}']/chain").get('tip_link'),'welding_tcp')

    def test_asset_and_calibrated_tcp_are_independent(self):
        data=json.loads((self.package/'config/torch_geometry.json').read_text())
        self.assertEqual(data['source'], 'JTM_RevC_MB_EVO36_0140528_exposed_tip_50mm.usd')
        self.assertTrue((self.package/'assets'/data['source']).is_file())
        self.assertEqual(data['tcp_definition'], 'preserved calibrated TCP')
        self.assertEqual(data['source'],
                         json.loads((self.package/'config/torch_collision.json').read_text())['source'])
        urdf=self.expand('ur10.urdf.xacro',True)
        origin=urdf.find("joint[@name='torch_tcp_joint']/origin")
        xyz=list(map(float,origin.get('xyz').split()))
        self.assertAlmostEqual(xyz[1],.229114062985,places=10)
        self.assertAlmostEqual(xyz[2],.101956156188,places=10)
        self.assertEqual(origin.attrib, data['tcp_origin'])

    def test_isaac_clearance_uses_its_own_moving_collision_geometry(self):
        from welding_sim.environment import simulation_floor
        from welding_sim.profiles import ISAAC
        from welding_sim.table_clearance import TableClearance
        robot = self.expand('ur10.urdf.xacro', True)
        clearance = TableClearance(ET.tostring(robot), ISAAC, simulation_floor())
        self.assertEqual(clearance.tabletop, 0.)
        self.assertIn('welding_torch_link', clearance.shapes)
        self.assertNotIn('base_link_inertia', clearance.shapes)
        self.assertTrue(any(name == 'flange' for name, *_ in clearance.chain))

    def test_inertial_properties_match_simulation_estimate(self):
        data=json.loads((self.package/'config/torch_dynamics.json').read_text())
        urdf=self.expand('ur10.urdf.xacro',True)
        inertial=urdf.find("link[@name='welding_torch_link']/inertial")
        self.assertAlmostEqual(float(inertial.find('mass').get('value')),data['tool_mass_kg'])
        self.assertTrue(data['estimated'])
        self.assertGreater(data['combined_wrist_mass_kg'],data['tool_mass_kg'])
        self.assertEqual(len(data['gains']),6)
        self.assertTrue(all(g['damping']>0 for g in data['gains']))

if __name__=='__main__':unittest.main()
