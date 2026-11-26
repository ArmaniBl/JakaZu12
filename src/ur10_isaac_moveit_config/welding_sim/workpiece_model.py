"""Portable startup CAD and explicit unverified-placement marker."""
import numpy as np
import hashlib
import json
import re
from pathlib import Path
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Pose,Point
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh,MeshTriangle
from .geometry import quaternion, rotation, FaceContour
from .table_limits import default_workpiece_base

PREVIEW_TYPE='scanner_preview_unregistered'
SEAM_PREFIX='jaka-weld-seams/v1:'
CATALOG_PREFIX = 'cad:'
WORKPIECE_CATALOG = (
    ('02-tee-gusset', 'Тавровая деталь с косынкой', '200 × 120 × 60 мм'),
    ('04-pipe-plate', 'Труба на пластине', '200 × 120 × 50 мм'),
    ('05-wave-butt', 'Пластина с волнистым стыком', '200 × 120 × 6 мм'),
)


def load_catalog_model(key):
    try:
        package = Path(get_package_share_directory('jaka_zu12_moveit_config'))
    except PackageNotFoundError:
        package = Path(__file__).resolve().parents[2] / 'jaka_zu12_moveit_config'
    return json.loads((package / 'config/workpiece_models' / f'{key}.json').read_text(encoding='utf-8'))


def mesh_signature(vertices, triangles):
    return hashlib.sha256(np.asarray(vertices,dtype='<f8').tobytes()
        +np.asarray(triangles,dtype='<i4').tobytes()).hexdigest()


def apply_model_seams(obj, geometry, center, orientation):
    """Read weld paths carried atomically with their collision mesh and pose."""
    if obj.type.db.startswith(SEAM_PREFIX):
        metadata = json.loads(obj.type.db[len(SEAM_PREFIX):])
    elif any(obj.type.key == CATALOG_PREFIX + key for key, _, _ in WORKPIECE_CATALOG):
        # Scenes loaded before these routes were authored still carry the catalog ID.
        model = load_catalog_model(obj.type.key.removeprefix(CATALOG_PREFIX))
        metadata = dict(mesh_signature=mesh_signature(np.asarray(model['vertices_mm']) * .001,
                                                      model['triangles']),
                        seams=model['seams'])
    else:
        return geometry
    mesh=obj.meshes[0]
    vertices=np.array([[v.x,v.y,v.z] for v in mesh.vertices])
    triangles=[list(t.vertex_indices) for t in mesh.triangles]
    if metadata['mesh_signature']!=mesh_signature(vertices,triangles):
        raise ValueError('Weld paths do not belong to this collision mesh')
    seams=metadata['seams']
    if not isinstance(seams,list) or not 1<=len(seams)<=32:raise ValueError('Invalid weld path catalog')
    transform=rotation(orientation);selected=[]
    for seam in seams:
        name=seam['id'];points=np.asarray(seam['vertices_mm'],dtype=float)*.001
        normal=np.asarray(seam['normal'],dtype=float)
        if (not isinstance(name,str) or not re.fullmatch(r'seam_[A-Za-z0-9_]{1,64}',name)
                or name in geometry.contours or points.ndim!=2 or points.shape[1]!=3
                or not 2<=len(points)<=5000 or not np.isfinite(points).all()
                or normal.shape!=(3,) or not np.isfinite(normal).all()
                or not np.isclose(np.linalg.norm(normal),1.)
                or np.min(np.linalg.norm(np.diff(points,axis=0),axis=1))<1e-9
                or np.any(points<vertices.min(0)-.001) or np.any(points>vertices.max(0)+.001)
                or np.ptp(points@normal)>1e-6):
            raise ValueError('Invalid local weld path')
        radial_center = None
        if 'radial_offset_center_mm' in seam:
            radial_center = np.asarray(seam['radial_offset_center_mm'], dtype=float)*.001
            if (radial_center.shape != (3,) or not np.isfinite(radial_center).all()
                    or not np.allclose((points-radial_center)@normal, 0., atol=1e-6)
                    or np.any(np.linalg.norm(points-radial_center, axis=1) < .001)):
                raise ValueError('Invalid center of radial weld offset')
            radial_center = transform @ radial_center + center
        geometry.contours[name]=FaceContour(name,points@transform.T+center,
            transform@normal,str(seam.get('label',name)),kind='weld',
            level_tcp=name == 'seam_center' and np.allclose(normal, [0., 0., 1.]),
            radial_offset_center=radial_center)
        selected.append(name)
    geometry.selectable_seams=tuple(selected)
    return geometry


def preview_object(model,transform=None):
    vertices=np.asarray(model['vertices_mm'],dtype=float)*.001
    faces=np.asarray(model['triangles'])
    if (vertices.ndim!=2 or vertices.shape[1]!=3 or not 4<=len(vertices)<=100000
            or not np.isfinite(vertices).all() or faces.ndim!=2 or faces.shape[1]!=3
            or faces.dtype.kind not in 'iu' or not 4<=len(faces)<=20000
            or faces.min()<0 or faces.max()>=len(vertices)):
        raise ValueError('Invalid startup CAD mesh')
    obj=CollisionObject(id='welding_workpiece',operation=CollisionObject.ADD)
    obj.header.frame_id='world';obj.type.key=PREVIEW_TYPE
    if transform is None:
        position=model['initial_pose']['position_m'];q=model['initial_pose']['quaternion_xyzw']
    else:
        transform=np.asarray(transform,dtype=float)
        if (transform.shape!=(4,4) or not np.isfinite(transform).all()
                or not np.allclose(transform[3],[0.,0.,0.,1.])
                or not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-6)
                or not np.isclose(np.linalg.det(transform[:3,:3]),1.)):
            raise ValueError('Expected a rigid CAD pose')
        position=transform[:3,3];q=quaternion(transform[:3,:3])
    if not np.isfinite([*position,*q]).all() or not np.isclose(np.linalg.norm(q),1.):
        raise ValueError('Invalid initial CAD pose')
    obj.pose.position.x,obj.pose.position.y,obj.pose.position.z=map(float,position)
    obj.pose.orientation.x,obj.pose.orientation.y,obj.pose.orientation.z,obj.pose.orientation.w=map(float,q)
    local=Pose();local.orientation.w=1.
    obj.mesh_poses=[local]
    obj.meshes=[Mesh(vertices=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in vertices],
                     triangles=[MeshTriangle(vertex_indices=[int(i) for i in f]) for f in faces])]
    if model.get('seams'):
        obj.type.db=SEAM_PREFIX+json.dumps(dict(mesh_signature=mesh_signature(vertices,faces),seams=model['seams']),ensure_ascii=False)
    return obj


def startup_workpiece(model, table_top):
    """Place the startup CAD on the table using its configured orientation."""
    workpiece = preview_object(model)
    workpiece.pose.position.x, workpiece.pose.position.y, workpiece.pose.position.z = default_workpiece_base(table_top)
    return workpiece
