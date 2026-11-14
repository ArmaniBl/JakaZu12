"""Locate the center of the forward copper contact-tip face in the supplied CAD."""
import numpy as np
from pxr import Gf, UsdGeom


def working_tip(stage, root=None):
    root=root or str(stage.GetDefaultPrim().GetPath())
    nozzle=stage.GetPrimAtPath(root+'/LowerPivot/Frames/NozzleTip')
    contact=stage.GetPrimAtPath(root+'/LowerPivot/Torch/CopperContactTip')
    mount=stage.GetPrimAtPath(root+'/MountFrame')
    if not nozzle or not contact or not mount:
        raise ValueError('Missing NozzleTip, CopperContactTip or MountFrame')
    cache=UsdGeom.XformCache()
    nozzle_matrix=cache.GetLocalToWorldTransform(nozzle)
    transform=cache.GetLocalToWorldTransform(contact)*nozzle_matrix.GetInverse()
    points=np.array([transform.Transform(Gf.Vec3d(*map(float,p)))
                     for p in UsdGeom.Mesh(contact).GetPointsAttr().Get()])
    forward=float(points[:,2].max())
    if not np.all(np.isfinite(points)) or not -.02<forward<.02:
        raise ValueError('Unexpected contact-tip geometry')
    # The authored nozzle frame is on the cylindrical contact tip's axis.
    offset=Gf.Matrix4d().SetTranslate(Gf.Vec3d(0,0,forward))
    return offset*nozzle_matrix*cache.GetLocalToWorldTransform(mount).GetInverse(), forward
