"""Export the supplied torch to URDF meshes and create an Isaac USD overlay.

Requires usd-core and numpy in a separate Python environment, not in ROS.
The original robot scene and torch asset are never modified.
"""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
from torch_tip import working_tip


def stl(path, triangles):
    with path.open('wb') as stream:
        stream.write(b'USD torch export'.ljust(80,b'\0'))
        stream.write(struct.pack('<I',len(triangles)))
        for tri in triangles:
            normal=np.cross(tri[1]-tri[0],tri[2]-tri[0])
            length=np.linalg.norm(normal)
            normal=normal/length if length>1e-15 else np.zeros(3)
            stream.write(struct.pack('<12fH',*normal,*tri.reshape(-1),0))


def origin(matrix):
    # USD uses row vectors; URDF uses column vectors.
    m=np.array(matrix,dtype=float).T
    r=m[:3,:3]
    pitch=math.asin(float(np.clip(-r[2,0],-1,1)))
    if abs(math.cos(pitch))<1e-8:
        roll,yaw=0.,math.atan2(-r[0,1],r[1,1])
    else:
        roll,yaw=math.atan2(r[2,1],r[2,2]),math.atan2(r[1,0],r[0,0])
    return {'xyz':' '.join(f'{v:.12g}' for v in m[:3,3]),
            'rpy':' '.join(f'{v:.12g}' for v in (roll,pitch,yaw))}


def mesh_triangles(source, mount_inverse):
    cache=UsdGeom.XformCache()
    visuals=defaultdict(list);collisions=[];hose_points=[]
    for prim in source.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.IsA(UsdPhysics.Joint):
            raise ValueError('Torch must be a fixed asset without rigid bodies/joints')
        if not prim.IsA(UsdGeom.Mesh):continue
        mesh=UsdGeom.Mesh(prim)
        transform=cache.GetLocalToWorldTransform(prim)*mount_inverse
        points=np.array([transform.Transform(Gf.Vec3d(*map(float,p))) for p in mesh.GetPointsAttr().Get()])
        if '/Hose/' in str(prim.GetPath()):hose_points.extend(points)
        indices=list(mesh.GetFaceVertexIndicesAttr().Get());counts=list(mesh.GetFaceVertexCountsAttr().Get())
        triangles=[];start=0
        for count in counts:
            face=indices[start:start+count];start+=count
            # CAD meshes in this asset are convex polygons/triangles.
            for i in range(1,count-1):triangles.append(points[[face[0],face[i],face[i+1]]])
        if mesh.GetOrientationAttr().Get()==UsdGeom.Tokens.leftHanded:
            triangles=[t[[0,2,1]] for t in triangles]
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collisions.extend(triangles)
            continue
        if mesh.ComputeVisibility()==UsdGeom.Tokens.invisible:continue
        material,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        color=(.45,.48,.5)
        if material:
            for shader in Usd.PrimRange(material.GetPrim()):
                for attribute in shader.GetAttributes():
                    if any(n in attribute.GetName().lower() for n in ('diffuse_color','diffusecolor','base_color')):
                        value=attribute.Get()
                        if isinstance(value,(Gf.Vec3f,Gf.Vec3d)):color=tuple(map(float,value))
        visuals[color].extend(triangles)
    if not collisions or not visuals:raise ValueError('No usable visual/collision geometry')
    return visuals, collisions, hose_points


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',required=True,type=Path)
    parser.add_argument('--scene',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--package',type=Path,default=Path(__file__).resolve().parents[1]/'src/ur10_isaac_moveit_config')
    args=parser.parse_args()
    if args.output.exists():parser.error('Output already exists; choose a new scene name')
    source=Usd.Stage.Open(str(args.source))
    root=source.GetDefaultPrim()
    mount=source.GetPrimAtPath(str(root.GetPath())+'/MountFrame')
    tcp=source.GetPrimAtPath(str(root.GetPath())+'/LowerPivot/Frames/TCP')
    if not mount or not tcp:parser.error('Required MountFrame/TCP is absent')
    if abs(UsdGeom.GetStageMetersPerUnit(source)-1.)>1e-9:parser.error('Expected a torch in meters')
    cache=UsdGeom.XformCache()
    mount_inverse=cache.GetLocalToWorldTransform(mount).GetInverse()
    tcp_matrix,contact_extension=working_tip(source)
    output=args.package/'meshes/torch'
    output.mkdir(parents=True,exist_ok=True)
    visuals,collisions,hose_points=mesh_triangles(source,mount_inverse)
    stl(output/'collision.stl',np.array(collisions))
    xacro=ET.Element('robot',{'xmlns:xacro':'http://www.ros.org/wiki/xacro'})
    macro=ET.SubElement(xacro,'xacro:macro',{'name':'welding_torch','params':'parent'})
    link=ET.SubElement(macro,'link',{'name':'welding_torch_link'})
    for i,(color,triangles) in enumerate(sorted(visuals.items())):
        name=f'visual_{i}.stl';stl(output/name,np.array(triangles))
        visual=ET.SubElement(link,'visual')
        geometry=ET.SubElement(visual,'geometry')
        ET.SubElement(geometry,'mesh',{'filename':'package://ur10_isaac_moveit_config/meshes/torch/'+name})
        material=ET.SubElement(visual,'material',{'name':f'torch_color_{i}'})
        ET.SubElement(material,'color',{'rgba':' '.join(map(str,(*color,1.)))})
    geometry=ET.SubElement(ET.SubElement(link,'collision'),'geometry')
    ET.SubElement(geometry,'mesh',{'filename':'package://ur10_isaac_moveit_config/meshes/torch/collision.stl'})
    hose_center=hose_size=None
    if hose_points:
        hose_points=np.array(hose_points)
        low,high=hose_points.min(axis=0),hose_points.max(axis=0)
        hose_center=(low+high)/2;hose_size=high-low+.002
        collision=ET.SubElement(link,'collision',{'name':'hose_envelope'})
        ET.SubElement(collision,'origin',{'xyz':' '.join(map(str,hose_center)),'rpy':'0 0 0'})
        ET.SubElement(ET.SubElement(collision,'geometry'),'box',{'size':' '.join(map(str,hose_size))})
    joint=ET.SubElement(macro,'joint',{'name':'tool0_torch_mount','type':'fixed'})
    ET.SubElement(joint,'parent',{'link':'${parent}'});ET.SubElement(joint,'child',{'link':'welding_torch_link'})
    ET.SubElement(joint,'origin',{'xyz':'0 0 0','rpy':'0 0 0'})
    ET.SubElement(macro,'link',{'name':'welding_tcp'})
    joint=ET.SubElement(macro,'joint',{'name':'torch_tcp_joint','type':'fixed'})
    ET.SubElement(joint,'parent',{'link':'welding_torch_link'});ET.SubElement(joint,'child',{'link':'welding_tcp'})
    ET.SubElement(joint,'origin',origin(tcp_matrix))
    ET.indent(xacro)
    ET.ElementTree(xacro).write(args.package/'config/welding_torch.xacro',encoding='utf-8',xml_declaration=True)
    report={'source':args.source.name,'mount':'MountFrame','tcp':'WeldingTCP',
            'tcp_definition':'center of forward CopperContactTip face',
            'contact_extension_from_nozzle_m':contact_extension,
            'tcp_origin':origin(tcp_matrix),'collision_triangles':len(collisions),
            'visual_triangles':sum(map(len,visuals.values())),'visual_materials':len(visuals),
            'hose_envelope':{'center':hose_center.tolist(),'size':hose_size.tolist()} if hose_center is not None else None}
    (args.package/'config/torch_geometry.json').write_text(json.dumps(report,indent=2)+'\n')
    # Archive the input for reproducible exports; meshes are self-contained.
    assets=args.package/'assets';assets.mkdir(exist_ok=True)
    shutil.copyfile(args.source,assets/args.source.name)
    scene=Usd.Stage.Open(str(args.scene))
    wrist=scene.GetPrimAtPath('/World/ur10/wrist_3_link')
    if not wrist or not wrist.HasAPI(UsdPhysics.RigidBodyAPI):parser.error('Expected UR10 wrist_3_link rigid body')
    # URDF wrist_3 -> flange -> tool0 has identity net rotation for this UR10.
    # MountFrame is aligned to tool0 and inherits the wrist rigid body in Isaac.
    result=Usd.Stage.CreateNew(str(args.output))
    import os
    result.GetRootLayer().subLayerPaths=[os.path.relpath(args.scene,args.output.parent).replace('\\','/')]
    UsdGeom.SetStageMetersPerUnit(result,1.);UsdGeom.SetStageUpAxis(result,'Z')
    result.SetDefaultPrim(result.GetPrimAtPath('/World'))
    torch=UsdGeom.Xform.Define(result,'/World/ur10/wrist_3_link/WeldingTorch')
    torch.GetPrim().GetReferences().AddReference(os.path.relpath(args.source,args.output.parent).replace('\\','/'))
    torch.ClearXformOpOrder();torch.AddTransformOp().Set(mount_inverse)
    working=UsdGeom.Xform.Define(result,str(torch.GetPath())+'/WeldingTCP')
    working.AddTransformOp().Set(tcp_matrix)
    for prim in Usd.PrimRange(torch.GetPrim()):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
    if hose_center is not None:
        envelope=UsdGeom.Cube.Define(result,str(torch.GetPath())+'/HoseEnvelope')
        envelope.CreateSizeAttr(1.)
        envelope.AddTranslateOp().Set(Gf.Vec3d(*hose_center))
        envelope.AddScaleOp().Set(Gf.Vec3d(*hose_size))
        envelope.CreateVisibilityAttr('invisible')
        UsdPhysics.CollisionAPI.Apply(envelope.GetPrim()).CreateCollisionEnabledAttr(True)
    result.GetRootLayer().Save()
    # Keep every re-export consistent with the current collision and dynamics model.
    import subprocess,sys
    for helper in ('configure_torch_dynamics.py','configure_torch_collision.py'):
        subprocess.run([sys.executable,str(Path(__file__).with_name(helper)),
                        '--source',str(args.source),'--scene',str(args.output),'--package',str(args.package)],check=True)
    print(json.dumps(report,indent=2));print('Scene:',args.output)


if __name__=='__main__':main()
