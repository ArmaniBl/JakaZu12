"""Conservative box union covering every visible CAD component, in both engines."""
import argparse,json,shutil
from datetime import datetime
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf

def envelopes(source,padding=.0005):
    cache=UsdGeom.XformCache();root=str(source.GetDefaultPrim().GetPath())
    inv=cache.GetLocalToWorldTransform(source.GetPrimAtPath(root+'/MountFrame')).GetInverse()
    boxes=[]
    for p in source.Traverse():
        if not p.IsA(UsdGeom.Mesh) or p.HasAPI(UsdPhysics.CollisionAPI):continue
        mesh=UsdGeom.Mesh(p)
        if mesh.ComputeVisibility()=='invisible':continue
        mat=cache.GetLocalToWorldTransform(p)*inv
        pts=np.array([mat.Transform(Gf.Vec3d(*map(float,v))) for v in mesh.GetPointsAttr().Get()])
        low,high=pts.min(axis=0)-padding,pts.max(axis=0)+padding
        boxes.append((str(p.GetPath()),low,high))
    kept=[]
    for name,low,high in sorted(boxes,key=lambda b:float(np.prod(b[2]-b[1])),reverse=True):
        if not any(np.all(low>=lo) and np.all(high<=hi) for _,lo,hi in kept):kept.append((name,low,high))
    # Verify coverage after removing redundant nested bounds.
    for _,low,high in boxes:
        if not any(np.all(low>=lo-1e-10) and np.all(high<=hi+1e-10) for _,lo,hi in kept):
            raise ValueError('Lost CAD component coverage')
    return kept,len(boxes)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',required=True,type=Path);parser.add_argument('--scene',required=True,type=Path)
    parser.add_argument('--package',type=Path,default=Path(__file__).resolve().parents[1]/'src/ur10_isaac_moveit_config')
    args=parser.parse_args();source=Usd.Stage.Open(str(args.source))
    boxes,count=envelopes(source)
    stage=Usd.Stage.Open(str(args.scene));root=stage.GetPrimAtPath('/World/ur10/wrist_3_link/WeldingTorch')
    if not root:parser.error('Mounted WeldingTorch is absent')
    shutil.copy2(args.scene,str(args.scene)+'.bak_collision_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    stage.SetEditTarget(stage.GetRootLayer())
    for p in Usd.PrimRange(root):
        if p.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
    ET.register_namespace('xacro','http://www.ros.org/wiki/xacro')
    path=args.package/'config/welding_torch.xacro';tree=ET.parse(path)
    link=tree.find(".//link[@name='welding_torch_link']")
    for old in link.findall('collision'):link.remove(old)
    for index,(name,low,high) in enumerate(boxes):
        center,size=(low+high)/2,high-low
        cube=UsdGeom.Cube.Define(stage,str(root.GetPath())+f'/CollisionEnvelope/Part_{index}')
        cube.CreateSizeAttr(1.);cube.ClearXformOpOrder()
        cube.AddTranslateOp().Set(Gf.Vec3d(*center));cube.AddScaleOp().Set(Gf.Vec3d(*size))
        cube.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
        collision=ET.SubElement(link,'collision',{'name':f'cad_part_{index}'})
        ET.SubElement(collision,'origin',{'xyz':' '.join(map(str,center)),'rpy':'0 0 0'})
        ET.SubElement(ET.SubElement(collision,'geometry'),'box',{'size':' '.join(map(str,size))})
    ET.indent(tree);tree.write(path,encoding='utf-8',xml_declaration=True)
    stage.GetRootLayer().Save()
    data={'method':'conservative per-component axis-aligned boxes; redundant enclosed boxes removed',
          'visible_components':count,'box_count':len(boxes),'padding_m':.0005,
          'boxes':[{'source':name,'center':((lo+hi)/2).tolist(),'size':(hi-lo).tolist()} for name,lo,hi in boxes]}
    (args.package/'config/torch_collision.json').write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps({k:v for k,v in data.items() if k!='boxes'},indent=2))

if __name__=='__main__':main()
