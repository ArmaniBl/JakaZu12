"""Apply estimated tool mass and mass-aware damping to a backed-up USD overlay.

Never runs a timeline or sends ROS commands. Requires usd-core and numpy.
"""
import argparse
from datetime import datetime
import json,math,shutil,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,UsdShade
from torch_mass import mesh_mass,combine


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',required=True,type=Path)
    parser.add_argument('--scene',required=True,type=Path)
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('torch_dynamics.json'))
    parser.add_argument('--package',type=Path,default=Path(__file__).resolve().parents[1]/'src/ur10_isaac_moveit_config')
    args=parser.parse_args()
    sys.path.insert(0,str(args.package))
    from welding_sim.geometry import rotation,quaternion
    config=json.loads(args.config.read_text())
    if not 0<config['damping_ratio']<=2 or not 60<=config['physics_hz']<=240:parser.error('Invalid dynamics configuration')
    source=Usd.Stage.Open(str(args.source));cache=UsdGeom.XformCache()
    mount=source.GetPrimAtPath(str(source.GetDefaultPrim().GetPath())+'/MountFrame')
    mount_inverse=cache.GetLocalToWorldTransform(mount).GetInverse()
    parts=[];report_parts=[]
    for prim in source.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or prim.HasAPI(UsdPhysics.CollisionAPI):continue
        mesh=UsdGeom.Mesh(prim)
        if mesh.ComputeVisibility()=='invisible':continue
        material,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        material_name=material.GetPrim().GetName() if material else ''
        density=config['density_kg_m3'].get(material_name)
        if density is None:raise ValueError('Unspecified density for '+material_name)
        transform=cache.GetLocalToWorldTransform(prim)*mount_inverse
        points=np.array([transform.Transform(Gf.Vec3d(*map(float,p))) for p in mesh.GetPointsAttr().Get()])
        indices=list(mesh.GetFaceVertexIndicesAttr().Get());triangles=[];offset=0
        for count in mesh.GetFaceVertexCountsAttr().Get():
            face=indices[offset:offset+count];offset+=count
            triangles.extend(points[[face[0],face[i],face[i+1]]] for i in range(1,count-1))
        mass,center,inertia=mesh_mass(triangles,density)
        parts.append((mass,center,inertia))
        report_parts.append({'prim':str(prim.GetPath()),'material':material_name,'density':density,'mass_kg':mass})
    tool_mass,tool_center,tool_inertia=combine(parts)
    measured=config.get('measured_tool_mass_kg')
    if measured is not None:
        if not math.isfinite(measured) or measured<=0:raise ValueError('Measured mass must be positive')
        tool_inertia*=measured/tool_mass;tool_mass=float(measured)
    stage=Usd.Stage.Open(str(args.scene))
    # Read the bare robot from the underlying scene, never from a previous overlay correction.
    base_layer=stage.GetRootLayer().subLayerPaths[0]
    bare=Usd.Stage.Open(str((args.scene.parent/base_layer).resolve()))
    wrist_path='/World/ur10/wrist_3_link'
    base_mass=UsdPhysics.MassAPI(bare.GetPrimAtPath(wrist_path))
    q=base_mass.GetPrincipalAxesAttr().Get();v=q.GetImaginary()
    rot=rotation([*v,q.GetReal()])
    base_tensor=rot@np.diag(base_mass.GetDiagonalInertiaAttr().Get())@rot.T
    # The authored MountFrame and UR10 wrist tool0 have identical axes.
    mass,center,tensor=combine([(float(base_mass.GetMassAttr().Get()),np.array(base_mass.GetCenterOfMassAttr().Get()),base_tensor),
                                (tool_mass,tool_center,tool_inertia)])
    values,axes=np.linalg.eigh(tensor)
    if np.linalg.det(axes)<0:axes[:,0]*=-1
    if values.min()<=0:raise ValueError('Combined inertia is not positive definite')
    backup=args.scene.with_name(args.scene.name+'.bak_dynamics_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    shutil.copy2(args.scene,backup)
    stage.SetEditTarget(stage.GetRootLayer())
    wrist=stage.GetPrimAtPath(wrist_path);api=UsdPhysics.MassAPI.Apply(wrist)
    api.CreateMassAttr(float(mass));api.CreateCenterOfMassAttr(Gf.Vec3f(*center))
    api.CreateDiagonalInertiaAttr(Gf.Vec3f(*values))
    q=quaternion(axes);api.CreatePrincipalAxesAttr(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
    # Solver settings affect only the simulator; CAD geometry and the JAKA SDK remain unchanged.
    articulation=next((p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)),None)
    if articulation:
        for name,value in (('solverPositionIterationCount',config['solver_position_iterations']),('solverVelocityIterationCount',config['solver_velocity_iterations'])):
            attr=articulation.GetAttribute('physxArticulation:'+name)
            if not attr:attr=articulation.CreateAttribute('physxArticulation:'+name,Sdf.ValueTypeNames.UInt)
            attr.Set(value)
    for p in stage.Traverse():
        if p.IsA(UsdPhysics.Scene):p.CreateAttribute('physxScene:timeStepsPerSecond',Sdf.ValueTypeNames.UInt).Set(config['physics_hz'])
    links=['shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link','ee_link']
    joints=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
    world=UsdGeom.XformCache();gains=[]
    for index,name in enumerate(joints):
        joint=stage.GetPrimAtPath('/World/ur10/joints/'+name)
        physics=UsdPhysics.RevoluteJoint(joint)
        child=stage.GetPrimAtPath(str(physics.GetBody1Rel().GetTargets()[0]))
        m=world.GetLocalToWorldTransform(child)
        local_q=physics.GetLocalRot1Attr().Get();v=local_q.GetImaginary()
        local_rot=rotation([*v,local_q.GetReal()])
        axis=np.eye(3)[:,{'X':0,'Y':1,'Z':2}[physics.GetAxisAttr().Get()]]
        axis=np.array(m.TransformDir(Gf.Vec3d(*(local_rot@axis))))
        pivot=np.array(m.Transform(Gf.Vec3d(*physics.GetLocalPos1Attr().Get())))
        effective=0.
        for link in links[index:]:
            body=stage.GetPrimAtPath('/World/ur10/'+link);props=UsdPhysics.MassAPI(body)
            if not props:continue
            body_mass=props.GetMassAttr().Get()
            if not body_mass or body_mass<=0:continue
            mat=world.GetLocalToWorldTransform(body)
            body_rot=np.array(mat)[:3,:3].T
            pq=props.GetPrincipalAxesAttr().Get();pv=pq.GetImaginary()
            moments=np.array(props.GetDiagonalInertiaAttr().Get())
            com=np.array(props.GetCenterOfMassAttr().Get())
            if not np.all(np.isfinite(moments)) or np.any(moments<=0) or not np.all(np.isfinite(com)):continue
            principal=rotation([*pv,pq.GetReal()]) if np.linalg.norm([*pv,pq.GetReal()])>1e-12 else np.eye(3)
            inertia=body_rot@principal@np.diag(moments)@principal.T@body_rot.T
            delta=np.array(mat.Transform(Gf.Vec3d(*props.GetCenterOfMassAttr().Get())))-pivot
            effective+=axis@inertia@axis+body_mass*np.linalg.norm(np.cross(delta,axis))**2
        original=UsdPhysics.DriveAPI.Get(bare.GetPrimAtPath(str(joint.GetPath())),'angular')
        stiffness=float(original.GetStiffnessAttr().Get())
        damping=max(float(original.GetDampingAttr().Get()),2*config['damping_ratio']*math.sqrt(stiffness*effective))
        UsdPhysics.DriveAPI.Get(joint,'angular').CreateDampingAttr(damping)
        gains.append({'joint':name,'effective_inertia_estimate':effective,'stiffness':stiffness,'damping':damping})
    from restore_wrist_visual import restore
    restore(stage)
    stage.GetRootLayer().Save()
    # Match URDF inertial properties; these do not set the real controller's payload.
    path=args.package/'config/welding_torch.xacro';ET.register_namespace('xacro','http://www.ros.org/wiki/xacro')
    tree=ET.parse(path);link=tree.find(".//link[@name='welding_torch_link']")
    for old in link.findall('inertial'):link.remove(old)
    inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'origin',{'xyz':' '.join(map(str,tool_center)),'rpy':'0 0 0'})
    ET.SubElement(inertial,'mass',{'value':str(tool_mass)})
    ET.SubElement(inertial,'inertia',{name:str(tool_inertia[i,j]) for name,i,j in (
        ('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2))})
    ET.indent(tree);tree.write(path,encoding='utf-8',xml_declaration=True)
    result={'estimated':measured is None,'method':'sum of CAD mesh volume integrals with assumed material densities',
        'tool_mass_kg':tool_mass,'tool_center_m':tool_center.tolist(),'tool_inertia_kg_m2':tool_inertia.tolist(),
        'combined_wrist_mass_kg':mass,'combined_wrist_center_m':center.tolist(),'gains':gains,'parts':report_parts,
        'backup':str(backup),'dynamic_validation':'not yet performed'}
    (args.package/'config/torch_dynamics.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('parts','tool_inertia_kg_m2')},indent=2))

if __name__=='__main__':main()
