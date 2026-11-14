"""Read-only USD check of the fixed attachment and exported TCP (requires usd-core)."""
import argparse
import json
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from torch_tip import working_tip

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene',type=Path)
    args=parser.parse_args()
    stage=Usd.Stage.Open(str(args.scene))
    wrist=stage.GetPrimAtPath('/World/ur10/wrist_3_link')
    root=stage.GetPrimAtPath(str(wrist.GetPath())+'/WeldingTorch')
    mount=stage.GetPrimAtPath(str(root.GetPath())+'/MountFrame')
    tcp=stage.GetPrimAtPath(str(root.GetPath())+'/WeldingTCP')
    assert root and mount and tcp and wrist.HasAPI(UsdPhysics.RigidBodyAPI)
    visual=stage.GetPrimAtPath(str(wrist.GetPath())+'/visuals')
    assert visual and not visual.IsInstance()
    meshes=[p for p in Usd.PrimRange(visual) if p.IsA(UsdGeom.Mesh)]
    assert meshes and all(UsdGeom.Imageable(p).ComputeVisibility()!='invisible' for p in meshes)
    cache=UsdGeom.XformCache()
    relative=lambda prim:cache.GetLocalToWorldTransform(prim)*cache.GetLocalToWorldTransform(wrist).GetInverse()
    assert np.max(np.abs(np.array(relative(mount))-np.eye(4)))<1e-8
    initial=np.array(relative(tcp))
    expected,extension=working_tip(stage,str(root.GetPath()))
    assert np.max(np.abs(initial-np.array(expected)))<1e-8
    collision_count=0
    for prim in Usd.PrimRange(root):
        assert not prim.HasAPI(UsdPhysics.RigidBodyAPI)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():collision_count+=1
    metadata=Path(__file__).resolve().parents[1]/'src/ur10_isaac_moveit_config/config/torch_collision.json'
    expected_count=json.loads(metadata.read_text())['box_count'] if metadata.exists() else 27
    # Disabled original collider meshes remain in the referenced CAD asset.
    assert collision_count==expected_count
    dynamics=metadata.with_name('torch_dynamics.json')
    if dynamics.exists():
        info=json.loads(dynamics.read_text())
        props=UsdPhysics.MassAPI(wrist)
        assert abs(props.GetMassAttr().Get()-info['combined_wrist_mass_kg'])<1e-5
        assert np.all(np.array(props.GetDiagonalInertiaAttr().Get())>0)
    # Perturb the wrist only in an unsaved session layer: no source files change.
    stage.SetEditTarget(stage.GetSessionLayer())
    xform=UsdGeom.Xformable(wrist);xform.ClearXformOpOrder()
    changed=Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1,2,3),37))
    changed.SetTranslateOnly(Gf.Vec3d(.7,-.2,.8))
    xform.AddTransformOp(opSuffix='torch_validation').Set(changed)
    cache.Clear()
    assert np.max(np.abs(np.array(relative(tcp))-initial))<1e-8
    print(json.dumps({'mount_aligned':True,'enabled_colliders':collision_count,'tcp_on_contact_tip':True,
                      'tcp_follows_wrist':True,'tcp_in_wrist_row_matrix':initial.tolist()},indent=2))

if __name__=='__main__':main()
