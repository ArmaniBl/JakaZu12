"""Scene updates are idle-only and never authorize motion."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
from moveit_msgs.msg import PlanningScene,CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from auto_workpiece import AutoWorkpiece
from welding_sim.workpiece_model import preview_object
from welding_sim.runner import WeldingRunner
from welding_sim.profiles import JAKA

MODEL=Path(__file__).resolve().parents[2]/'jaka_zu12_moveit_config/config/workpiece_model.json'


class FakeNode:
    def __init__(self,model):
        self.scene=PlanningScene()
        floor=CollisionObject(id='floor');floor.header.frame_id='world';floor.pose.orientation.w=1.
        pose=Pose();pose.orientation.w=1.;pose.position.x=-.04;pose.position.y=-.465;pose.position.z=-.05
        floor.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.8,1.2,.1])];floor.primitive_poses=[pose]
        self.scene.world.collision_objects=[floor,preview_object(model)]
        self.applied=[];self.moving=False;self.reads=0;self.change_scene=False
    def create_client(self,*args):return NS(service_is_ready=lambda:True)
    def create_publisher(self,*args):return NS(publish=lambda message:None)
    def get_logger(self):return NS(info=lambda message:None)
    def require_idle(self):
        if self.moving:raise ValueError('Robot is moving')
    def call(self,service,name,request):
        if service==ApplyPlanningScene:
            self.applied.append(request);return NS(success=True)
        self.reads+=1
        scene=deepcopy(self.scene)
        if self.change_scene and self.reads>1:scene.world.collision_objects.append(CollisionObject(id='new_obstacle'))
        return NS(scene=scene)


class AutoWorkpieceTests(unittest.TestCase):
    def test_selected_model_and_weld_metadata_survive_old_scanner_bundle(self):
        model=json.loads((MODEL.parent/'workpiece_models/05-wave-butt.json').read_text())
        path=self.workspace/'src/jaka_zu12_moveit_config/config/workpiece_model.json'
        path.write_text(json.dumps(model))
        self.node.scene.world.collision_objects[1]=preview_object(model)
        bundle=dict(self.bundle,part_name='old-scanner-model.step')
        self.assertTrue(self.auto.update(bundle,np.zeros((100,3))))
        self.assertEqual(self.auto.result['state'],'matched_preview')
        stored=json.loads(path.read_text())
        self.assertEqual(stored['part_name'],'05-wave-butt.step')
        self.assertEqual(stored['seams'],model['seams'])
        self.assertEqual(stored['model_selection'],'operator')
        self.assertEqual(self.node.applied[0].scene.world.collision_objects[0].type.db,
                         preview_object(model).type.db)

    def test_next_startup_selection_does_not_replace_another_live_model(self):
        model=json.loads((MODEL.parent/'workpiece_models/05-wave-butt.json').read_text())
        old=json.loads((Path(__file__).resolve().parent/'fixtures/slotted-88x40x5.json').read_text())
        path=self.workspace/'src/jaka_zu12_moveit_config/config/workpiece_model.json'
        path.write_text(json.dumps(model))
        self.node.scene.world.collision_objects[1]=preview_object(old)
        self.assertTrue(self.auto.update(self.bundle,np.zeros((100,3))))
        self.assertEqual(self.auto.result['state'],'different_model_retained')
        self.assertFalse(self.node.applied)
        self.assertEqual(json.loads(path.read_text()),model)

    def setUp(self):
        self.model=json.loads(MODEL.read_text())
        self.bundle=dict(self.model,capture_id='capture',frame_signature='frame')
        self.node=FakeNode(self.model)
        self.directory=tempfile.TemporaryDirectory()
        self.workspace=Path(self.directory.name)
        (self.workspace/'src/jaka_zu12_moveit_config/config').mkdir(parents=True)
        self.auto=AutoWorkpiece(self.node,self.workspace)
        self.transform=np.eye(4);self.transform[:3,3]=[-.3,-.7,.01]
        self.fit=patch('auto_workpiece.fit_preview',return_value=(self.transform,dict(rmse_m=.001,execution_ready=False)))
        self.fit.start()
    def tearDown(self):self.fit.stop();self.directory.cleanup()

    def test_updates_only_workpiece_and_keeps_unverified_type(self):
        self.assertTrue(self.auto.update(self.bundle,np.zeros((100,3))))
        self.assertEqual(len(self.node.applied),1)
        objects=self.node.applied[0].scene.world.collision_objects
        self.assertEqual([o.id for o in objects],['welding_workpiece'])
        self.assertEqual(objects[0].type.key,'scanner_preview_unregistered')
        self.assertFalse(self.auto.result['execution_ready'])
        self.assertTrue((self.workspace/'scans/auto_workpiece.json').is_file())

    def test_moving_robot_prevents_update(self):
        self.node.moving=True
        self.assertFalse(self.auto.update(self.bundle,np.zeros((100,3))))
        self.assertFalse(self.node.applied)

    def test_changed_collision_scene_prevents_update(self):
        self.node.change_scene=True
        self.assertFalse(self.auto.update(self.bundle,np.zeros((100,3))))
        self.assertFalse(self.node.applied)

    def test_approved_model_is_not_overwritten(self):
        self.node.scene.world.collision_objects[1].type.key=''
        self.assertTrue(self.auto.update(self.bundle,np.zeros((100,3))))
        self.assertFalse(self.node.applied)

    def test_table_penetration_is_rejected(self):
        self.transform[2,3]=-.01
        self.assertTrue(self.auto.update(self.bundle,np.zeros((100,3))))
        self.assertEqual(self.auto.result['state'],'mismatch')
        self.assertFalse(self.node.applied)

    def test_real_execution_rejects_unverified_preview(self):
        fake=NS(args=NS(object='welding_workpiece',mode='execute'),profile=JAKA,
                call=lambda *args:NS(scene=self.node.scene))
        with self.assertRaisesRegex(RuntimeError,'not a validated robot registration'):
            WeldingRunner.read_object(fake)

    def test_real_execution_requires_collision_table(self):
        self.node.scene.world.collision_objects.pop(0)
        fake = NS(args=NS(object='welding_workpiece', mode='execute', use_scene_workpiece=True),
                  profile=JAKA, call=lambda *args: NS(scene=self.node.scene))
        with patch('welding_sim.runner.time.monotonic', side_effect=[0., 21.]):
            with self.assertRaisesRegex(RuntimeError, 'Collision table is absent'):
                WeldingRunner.read_object(fake)

    def test_operator_scene_mode_uses_current_geometry_without_scan_receipt(self):
        fake=NS(args=NS(object='welding_workpiece',mode='execute',use_scene_workpiece=True),profile=JAKA,
                call=lambda *args:NS(scene=self.node.scene))
        edges=WeldingRunner.read_object(fake)
        self.assertTrue(edges)
        self.assertEqual(fake.workpiece.id,'welding_workpiece')


if __name__=='__main__':unittest.main()
