#!/usr/bin/env python3
"""Convert measured kVantron scan/CAD into a reviewed MoveIt workpiece."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import time
import uuid
import subprocess
from urllib.request import urlopen
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from moveit_msgs.msg import CollisionObject, ObjectColor, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionFK
from sensor_msgs.msg import JointState
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from welding_sim.scan_alignment import validate_transform, transform_points, digest, align_pose
from welding_sim.scan_model import sample_object
from welding_sim.geometry import quaternion
from rangevision import Bridge, signature


def fetch_bundle(url):
    # The configured scanner URL is read-only; redirects are not followed to other protocols.
    if url == 'windows-local':
        root=next((p for p in Path(__file__).resolve().parents if (p/'integrations/read_scan.ps1').is_file()),None)
        if root is None:raise ValueError('Windows scan relay script not found')
        script=subprocess.check_output(['wslpath','-w',str(root/'integrations/read_scan.ps1')],text=True).strip()
        response=subprocess.run(['powershell.exe','-NoProfile','-NonInteractive','-ExecutionPolicy','Bypass',
                                 '-File',script],capture_output=True,timeout=20)
        if response.returncode:
            detail=response.stderr.decode('utf-8',errors='replace').strip()[-2000:]
            try:detail=json.loads(detail).get('error',detail)
            except (ValueError,AttributeError):pass
            raise ValueError(detail)
        raw=response.stdout
    elif not url.startswith(('http://', 'https://')):
        raise ValueError('Scanner URL must use HTTP(S)')
    else:
        with urlopen(url.rstrip('/')+'/api/spectrum/moveit-export', timeout=15) as response:
            raw = response.read(32_000_001)
    if len(raw) > 32_000_000:
        raise ValueError('Scanner export exceeds 32 MB')
    bundle = json.loads(raw)
    if bundle.get('schema') != 'kvantron-moveit/v1' or bundle.get('units') != 'mm':
        raise ValueError('Unsupported scan contract')
    if not 0 <= time.time()-bundle['created_unix'] < 60:
        raise ValueError('Scanner response is stale')
    return bundle


def initial_object(bundle, calibration):
    if (calibration.get('frame_signature') != bundle.get('frame_signature')
            or not calibration.get('frame_signature')):
        raise ValueError('Scanner frame is not bound to robot calibration; calibrate this frame first')
    if (calibration.get('frame') != 'world' or calibration.get('fixed_scanner') is not True
            or not 0 <= calibration.get('validation_max_error_m', 1) <= .001):
        raise ValueError('Independent scanner-to-world calibration is missing or rejected')
    camera = validate_transform(calibration['scanner_to_world'])
    nominal = np.array(bundle['camera_T_cad_mm'], float);nominal[:3, 3] *= .001
    target = camera@validate_transform(nominal)
    vertices = np.asarray(bundle['vertices_mm'], float)*.001
    triangles = np.asarray(bundle['triangles'])
    points = np.asarray(bundle['camera_points_mm'], float)*.001
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or not 4 <= len(vertices) <= 100000
            or triangles.ndim != 2 or triangles.shape[1] != 3 or not 4 <= len(triangles) <= 20000
            or triangles.dtype.kind not in 'iu' or triangles.min() < 0 or triangles.max() >= len(vertices)
            or points.ndim != 2 or points.shape[1] != 3 or not 100 <= len(points) <= 100000
            or not np.isfinite(vertices).all() or not np.isfinite(points).all()):
        raise ValueError('Invalid or excessively complex CAD/scan; simplify the CAD mesh')
    obj = CollisionObject(id='welding_workpiece', operation=CollisionObject.ADD)
    obj.header.frame_id = 'world'
    obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = map(float, target[:3, 3])
    q = quaternion(target[:3, :3])
    obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w = map(float, q)
    pose = Pose();pose.orientation.w = 1.
    obj.mesh_poses = [pose]
    obj.meshes = [Mesh(vertices=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in vertices],
                       triangles=[MeshTriangle(vertex_indices=[int(i) for i in t]) for t in triangles])]
    return obj, transform_points(points, camera), target


def detect(node, bundle, calibration):
    obj, points, nominal = initial_object(bundle, calibration)
    scene = node.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request()).scene
    floor = next((o for o in scene.world.collision_objects if o.id == 'floor'), None)
    if not floor or not floor.primitives or floor.primitives[0].type != SolidPrimitive.BOX:
        raise ValueError('Supporting table collision object is required')
    from welding_sim.runner import pose_transform
    floor_pose = pose_transform(floor.pose)@pose_transform(floor.primitive_poses[0])
    if not np.allclose(floor_pose[:3, 2], [0,0,1], atol=1e-5):
        raise ValueError('Only a horizontal calibrated support table is supported')
    table_z = floor_pose[2,3]+floor.primitives[0].dimensions[2]/2
    model, normals = sample_object(obj, spacing=.002)
    low, high = model.min(0)-.05, model.max(0)+.05
    points = points[np.all((points>=low)&(points<=high),axis=1)&(points[:,2]>table_z+.002)]
    _, unique = np.unique(np.floor(points/.001),axis=0,return_index=True)
    points = points[unique]
    correction, quality = align_pose(points, model, normals)
    target = correction@nominal
    moved = deepcopy(obj)
    moved.pose.position.x, moved.pose.position.y, moved.pose.position.z = map(float,target[:3,3])
    q = quaternion(target[:3,:3])
    moved.pose.orientation.x,moved.pose.orientation.y,moved.pose.orientation.z,moved.pose.orientation.w = map(float,q)
    # Reject a fitted CAD which would penetrate the supporting table.
    vertices = np.array([[v.x,v.y,v.z] for v in moved.meshes[0].vertices])
    world_vertices = transform_points(vertices,target)
    if world_vertices[:,2].min() < table_z-.001:
        raise ValueError('Fitted CAD penetrates the support table')
    inverse_floor = np.linalg.inv(floor_pose)
    on_table = transform_points(world_vertices,inverse_floor)
    half = np.asarray(floor.primitives[0].dimensions[:2])/2
    if np.any(np.abs(on_table[:,:2])>half+.001):
        raise ValueError('Detected part extends outside the support table')
    old = next((o for o in scene.world.collision_objects if o.id == obj.id), None)
    return dict(quality=quality, object=message_to_ordereddict(moved),
                old_object_sha256=signature(old) if old else None, floor_sha256=signature(floor),
                target_xyz_m=target[:3,3].tolist(), target_quaternion=q.tolist())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('info','detect','apply','publish','check','tcp'))
    parser.add_argument('--pose-dir',default='scans/tcp_poses')
    parser.add_argument('--tip',default='welding_tcp')
    parser.add_argument('--frame',default='world')
    parser.add_argument('--scanner-url', default='http://127.0.0.1:8158')
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--report', required=True)
    args = parser.parse_args()
    node = None
    try:
        if args.command == 'info':
            b = fetch_bundle(args.scanner_url)
            print(json.dumps({k:b[k] for k in ('task_id','capture_id','frame_signature','part_name','identity')},ensure_ascii=False));return
        if args.command in ('detect','apply','check') and not Path(args.calibration).is_file():
            raise ValueError('Не выполнена привязка сканера к роботу: отсутствует '+args.calibration+
                '. Калибровка камер candidate.json её не заменяет. Нужны измеренные пары точек '
                'scanner_mm/world_m и отдельные контрольные точки; см. docs/SCAN_TO_WELD.md.')
        rclpy.init(args=[]);node = Bridge()
        if args.command == 'tcp':
            node.require_idle()
            joints=node.samples[-1][1].copy()
            request=GetPositionFK.Request();request.header.frame_id=args.frame
            request.fk_link_names=[args.tip]
            request.robot_state=RobotState(joint_state=JointState(
                name=[f'joint_{i}' for i in range(1,7)],position=joints.tolist()))
            response=node.call(GetPositionFK,'/compute_fk',request)
            if response.error_code.val!=1 or len(response.pose_stamped)!=1 or list(response.fk_link_names)!=[args.tip]:
                raise ValueError('MoveIt could not calculate the requested TCP pose')
            pose=response.pose_stamped[0]
            if pose.header.frame_id!=args.frame:raise ValueError('Unexpected TCP reference frame')
            node.require_idle()
            if np.max(np.abs(node.samples[-1][1]-joints))>.001:
                raise ValueError('Robot changed position during capture; save again')
            values=[pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,
                    pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w]
            if not np.isfinite(values).all():raise ValueError('Nonfinite TCP pose')
            result=dict(schema='jaka-tcp-snapshot/v1',created_unix=time.time(),frame=args.frame,
                        tip=args.tip,position_m=values[:3],quaternion_xyzw=values[3:],
                        joint_names=request.robot_state.joint_state.name,joint_positions_rad=joints.tolist(),
                        source='MoveIt FK from fresh stationary JAKA feedback',
                        camera_point=None,clearance_m=None,calibration_pair_complete=False)
            folder=Path(args.pose_dir);folder.mkdir(parents=True,exist_ok=True)
            file=folder/('tcp-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8]+'.json')
            with file.open('x',encoding='utf-8') as stream:json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
            print(json.dumps(dict(result,file=str(file.resolve())),ensure_ascii=False));return
        if args.command == 'publish':
            report = json.loads(Path(args.report).read_text())
            cloud = np.load(Path(args.report).with_suffix('.npz'))['points']
            node.publish(cloud);return
        # Detection only reads a scene snapshot and writes a proposal. Applying
        # it, validating execution and every motion stage still require feedback.
        if args.command != 'detect':
            node.require_idle()
        bundle = fetch_bundle(args.scanner_url)
        calibration_hash = digest(args.calibration)
        calibration = json.loads(Path(args.calibration).read_text())
        if args.command == 'check':
            approved=json.loads(Path(args.report).read_text())
            if not 0<=time.time()-approved['created_unix']<=300:
                raise ValueError('Detection expired; scan/detect again before welding')
            if bundle['identity']!=approved['identity'] or calibration_hash!=approved['calibration_sha256']:
                raise ValueError('Scanner/CAD/calibration changed; detect again')
            scene=node.call(GetPlanningScene,'/get_planning_scene',GetPlanningScene.Request()).scene
            obj=next((o for o in scene.world.collision_objects if o.id=='welding_workpiece'),None)
            floor=next((o for o in scene.world.collision_objects if o.id=='floor'),None)
            if not obj or signature(obj)!=signature_from_result(approved) or not floor or signature(floor)!=approved['floor_sha256']:
                raise ValueError('Imported workpiece/table changed; detect again')
            print(json.dumps({'valid':True,'identity':bundle['identity']}));return
        result = detect(node,bundle,calibration)
        if digest(args.calibration) != calibration_hash:
            raise ValueError('Calibration changed during detection')
        result.update(created_unix=time.time(),identity=bundle['identity'],capture_id=bundle['capture_id'],
                      part_name=bundle['part_name'],calibration_sha256=calibration_hash,
                      scanner_url=args.scanner_url)
        if args.command == 'detect':
            Path(args.report).parent.mkdir(parents=True,exist_ok=True)
            Path(args.report).write_text(json.dumps(result,allow_nan=False))
            _, points, _ = initial_object(bundle,calibration)
            np.savez_compressed(Path(args.report).with_suffix('.npz'),points=points)
        else:
            approved = json.loads(Path(args.report).read_text())
            if not 0<=time.time()-approved['created_unix']<=300:
                raise ValueError('Detection expired; detect again')
            for key in ('identity','calibration_sha256','old_object_sha256','floor_sha256'):
                if result[key]!=approved[key]:
                    raise ValueError('Scan/CAD/calibration/scene changed; detect again')
            obj = restore_object(approved['object'])
            if signature(obj)!=signature_from_result(result):
                raise ValueError('Registration result changed; detect again')
            node.require_idle()
            scene = node.call(GetPlanningScene,'/get_planning_scene',GetPlanningScene.Request()).scene
            old = next((o for o in scene.world.collision_objects if o.id==obj.id),None)
            floor = next((o for o in scene.world.collision_objects if o.id=='floor'),None)
            if (signature(old) if old else None)!=approved['old_object_sha256'] or not floor or signature(floor)!=approved['floor_sha256']:
                raise ValueError('Scene changed before applying detection')
            request = ApplyPlanningScene.Request();request.scene.is_diff=True;request.scene.robot_state.is_diff=True
            request.scene.world.collision_objects=[obj]
            color = ObjectColor(id=obj.id);color.color.r=.9;color.color.g=.5;color.color.b=.1;color.color.a=1.
            request.scene.object_colors=[color]
            if not node.call(ApplyPlanningScene,'/apply_planning_scene',request).success:
                raise ValueError('MoveIt rejected the scanned workpiece')
        result.pop('object')
        print(json.dumps(result,ensure_ascii=False))
    except (ValueError,KeyError,OSError,TypeError,IndexError,subprocess.SubprocessError) as error:
        parser.exit(1,str(error)+'\n')
    except KeyboardInterrupt:
        pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def signature_from_result(result):
    return signature(restore_object(result['object']))


def restore_object(value):
    # Humble converts the octet operation to a character, not a JSON number.
    fields=dict(value);fields.pop('operation',None)
    obj=CollisionObject();set_message_fields(obj,fields)
    obj.operation=CollisionObject.ADD
    return obj


if __name__=='__main__':main()
