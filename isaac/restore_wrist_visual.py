"""Refresh the wrist visual without changing rigid-body or joint parameters."""
import argparse
from datetime import datetime
from pathlib import Path
import shutil
from pxr import Usd,UsdGeom


def restore(stage):
    wrist=stage.GetPrimAtPath('/World/ur10/wrist_3_link')
    visuals=stage.GetPrimAtPath('/World/ur10/wrist_3_link/visuals')
    if not wrist or not visuals:raise ValueError('UR10 wrist visual is absent')
    visuals.SetInstanceable(False)
    UsdGeom.Imageable(wrist).CreateVisibilityAttr('inherited')
    UsdGeom.Imageable(visuals).MakeVisible()
    meshes=[]
    for prim in Usd.PrimRange(visuals):
        if prim.IsA(UsdGeom.Mesh):
            mesh=UsdGeom.Mesh(prim)
            mesh.MakeVisible()
            mesh.CreateDoubleSidedAttr(True)
            meshes.append({'path':str(prim.GetPath()),'vertices':len(mesh.GetPointsAttr().Get())})
    if not meshes:raise ValueError('Wrist visuals have no resolved mesh')
    return meshes


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene',type=Path);args=parser.parse_args()
    stage=Usd.Stage.Open(str(args.scene));stage.SetEditTarget(stage.GetRootLayer())
    physics=lambda:{str(p.GetPath())+':'+a.GetName():str(a.Get()) for p in stage.Traverse()
                    for a in p.GetAttributes() if a.GetName().startswith(('physics:','physx','drive:'))}
    before=physics()
    backup=str(args.scene)+'.bak_visual_'+datetime.now().strftime('%Y%m%d_%H%M%S')
    shutil.copy2(args.scene,backup)
    meshes=restore(stage)
    if physics()!=before:raise RuntimeError('Physics properties unexpectedly changed; file not saved')
    stage.GetRootLayer().Save()
    print({'meshes':meshes,'physics_unchanged':True,'backup':backup})

if __name__=='__main__':main()
