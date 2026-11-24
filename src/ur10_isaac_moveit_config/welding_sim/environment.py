"""Simulation obstacles, separate from weldable geometry."""
from copy import deepcopy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, AllowedCollisionEntry
from shape_msgs.msg import SolidPrimitive


def simulation_floor(top=0.):
    obj=CollisionObject(id='floor',operation=CollisionObject.ADD)
    obj.header.frame_id='world';obj.pose.orientation.w=1.
    pose=Pose();pose.orientation.w=1.;pose.position.z=float(top)-.05
    # Covers the UR10 workspace, unlike the small physical mounting platform.
    obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[10.,10.,.1])]
    obj.primitive_poses=[pose]
    return obj


def allow_base_support(matrix, base_link='base_link_inertia'):
    """Only the stationary mounting base may touch the supporting floor."""
    result=deepcopy(matrix)
    for name in ('floor',base_link):
        if name not in result.entry_names:
            result.entry_names.append(name)
            for row in result.entry_values:row.enabled.append(False)
            result.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(result.entry_names)))
    a,b=(result.entry_names.index(name) for name in ('floor',base_link))
    result.entry_values[a].enabled[b]=True
    result.entry_values[b].enabled[a]=True
    return result
