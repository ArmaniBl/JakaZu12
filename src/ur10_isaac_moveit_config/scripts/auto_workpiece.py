"""Update only an unverified CAD preview, while the robot is idle."""
import json
from pathlib import Path
import time

import numpy as np
from moveit_msgs.msg import ObjectColor
from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String

from welding_sim.part_preview_fit import fit_preview
from welding_sim.workpiece_model import preview_object,PREVIEW_TYPE,mesh_signature
from welding_sim.runner import pose_transform


class AutoWorkpiece:
    def __init__(self,node,workspace):
        self.node=node
        self.workspace=Path(workspace)
        self.scene=node.create_client(GetPlanningScene,'/get_planning_scene')
        self.apply=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
        self.publisher=node.create_publisher(String,'/scanner/workpiece_status',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.result=dict(state='waiting_for_cloud',execution_ready=False,preliminary=True)
        self.publisher.publish(String(data=json.dumps(self.result)))

    def status(self,state,**values):
        self.result=dict(state=state,updated_unix=time.time(),execution_ready=False,preliminary=True,**values)
        self.publisher.publish(String(data=json.dumps(self.result)))
        self.node.get_logger().info('Automatic CAD: '+json.dumps(self.result,ensure_ascii=False))

    def update(self,bundle,points):
        """True consumes this scan; False retries when services/idle feedback recover."""
        if not self.scene.service_is_ready() or not self.apply.service_is_ready():
            self.status('waiting_for_moveit',capture_id=bundle['capture_id'])
            return False
        try:
            self.node.require_idle()
        except ValueError as error:
            self.status('waiting_for_idle',reason=str(error),capture_id=bundle['capture_id'])
            return False
        scene=self.node.call(GetPlanningScene,'/get_planning_scene',GetPlanningScene.Request()).scene
        floor=next((o for o in scene.world.collision_objects if o.id=='floor'),None)
        current=next((o for o in scene.world.collision_objects if o.id=='welding_workpiece'),None)
        if floor is None:
            self.status('waiting_for_table',capture_id=bundle['capture_id']);return False
        if current is not None and current.type.key!=PREVIEW_TYPE:
            self.status('approved_model_retained',reason='Automatic preview cannot overwrite an approved or manually placed workpiece')
            return True
        model_path=self.workspace/'src/jaka_zu12_moveit_config/config/workpiece_model.json'
        model=json.loads(model_path.read_text()) if model_path.is_file() else bundle
        if model.get('model_selection')=='operator' and current is not None:
            mesh=current.meshes[0] if len(current.meshes)==1 else None
            if mesh is None or mesh_signature([[v.x,v.y,v.z] for v in mesh.vertices],
                    [list(t.vertex_indices) for t in mesh.triangles])!=mesh_signature(
                        np.asarray(model['vertices_mm'])*.001,model['triangles']):
                self.status('different_model_retained',reason='Selected startup model will apply on next scene launch',capture_id=bundle['capture_id'])
                return True
        try:
            transform,quality=fit_preview(points,model['vertices_mm'],model['triangles'])
            obj=preview_object(model,transform)
            floor_pose=pose_transform(floor.pose)@pose_transform(floor.primitive_poses[0])
            if not np.allclose(floor_pose[:3,2],[0.,0.,1.],atol=1e-5):
                raise ValueError('Automatic preview requires a horizontal table')
            vertices=np.array(model['vertices_mm'])*.001
            world=vertices@transform[:3,:3].T+transform[:3,3]
            local=(world-floor_pose[:3,3])@floor_pose[:3,:3]
            half=np.array(floor.primitives[0].dimensions)/2
            if local[:,2].min()<half[2]-.001 or np.any(np.abs(local[:,:2])>half[:2]+.001):
                raise ValueError('Fitted CAD penetrates or extends outside the support table')
        except (ValueError,KeyError,IndexError) as error:
            self.status('mismatch',reason=str(error),capture_id=bundle['capture_id'])
            return True
        try:self.node.require_idle()
        except ValueError as error:
            self.status('waiting_for_idle',reason=str(error));return False
        latest=self.node.call(GetPlanningScene,'/get_planning_scene',GetPlanningScene.Request()).scene
        if (sorted(latest.world.collision_objects,key=lambda o:o.id)!=sorted(scene.world.collision_objects,key=lambda o:o.id)
                or latest.robot_state.attached_collision_objects!=scene.robot_state.attached_collision_objects):
            self.status('scene_changed',capture_id=bundle['capture_id']);return False
        request=ApplyPlanningScene.Request();request.scene.is_diff=True
        request.scene.world.collision_objects=[obj]
        color=ObjectColor(id=obj.id);color.color.r,color.color.g,color.color.b,color.color.a=.9,.5,.1,1.
        request.scene.object_colors=[color]
        response=self.node.call(ApplyPlanningScene,'/apply_planning_scene',request)
        if not response.success:raise ValueError('MoveIt rejected fitted CAD')
        report=dict(capture_id=bundle['capture_id'],frame_signature=bundle['frame_signature'],
                    part_name=model['part_name'],transform=transform.tolist(),quality=quality,
                    preliminary=True,execution_ready=False)
        report_path=self.workspace/'scans/auto_workpiece.json';report_path.parent.mkdir(parents=True,exist_ok=True)
        temporary=report_path.with_suffix('.tmp');temporary.write_text(json.dumps(report,indent=2));temporary.replace(report_path)
        model=dict(model,schema='jaka-startup-workpiece/v1',units='mm',placement_verified=False,
                   initial_pose=dict(position_m=transform[:3,3].tolist(),quaternion_xyzw=[
                       obj.pose.orientation.x,obj.pose.orientation.y,obj.pose.orientation.z,obj.pose.orientation.w]))
        path=self.workspace/'src/jaka_zu12_moveit_config/config/workpiece_model.json'
        temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(model,indent=2));temporary.replace(path)
        self.status('matched_preview',capture_id=report['capture_id'],part_name=report['part_name'],
                    transform=report['transform'],quality=quality)
        return True
