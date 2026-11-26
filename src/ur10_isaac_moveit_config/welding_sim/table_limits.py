"""Table placement and collision limits."""
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

JAKA_TABLE_TOP_M = .005


DEFAULT_WORKPIECE_CELL = 'S8'


def world_point(config, x_mm, y_mm, z_mm=0.):
    transform = config['world_from_table']
    angle = math.radians(transform['yaw_deg'])
    tx, ty, tz = transform['translation_m']
    return (tx + (math.cos(angle)*x_mm-math.sin(angle)*y_mm)/1000,
            ty + (math.sin(angle)*x_mm+math.cos(angle)*y_mm)/1000,
            tz + z_mm/1000)


def default_workpiece_base(table_top=0.):
    path = Path(get_package_share_directory('ur10_isaac_moveit_config')) / 'config/table_grid.json'
    layout = json.loads(path.read_text())
    grid = layout['grid']
    column, row = DEFAULT_WORKPIECE_CELL[0], int(DEFAULT_WORKPIECE_CELL[1:])
    x_mm = grid['offset_x_mm'] + (ord(column) - ord('A')) * grid['pitch_mm']
    y_mm = grid['offset_y_mm'] + (row - 1) * grid['pitch_mm']
    x, y, _ = world_point(layout, x_mm, y_mm)
    return x, y, table_top


def wall_specs():
    # Physical table X=[-.44,.36], Y=[-1.065,.135], metres.
    xmin,xmax,ymin,ymax=-.54,.46,-1.165,.235
    thickness=2.
    return [
        ('workspace_x_min',[thickness,ymax-ymin+2*thickness,4.],[xmin-thickness/2,(ymin+ymax)/2,1.]),
        ('workspace_x_max',[thickness,ymax-ymin+2*thickness,4.],[xmax+thickness/2,(ymin+ymax)/2,1.]),
        ('workspace_y_min',[xmax-xmin+2*thickness,thickness,4.],[(xmin+xmax)/2,ymin-thickness/2,1.]),
        ('workspace_y_max',[xmax-xmin+2*thickness,thickness,4.],[(xmin+xmax)/2,ymax+thickness/2,1.])]


def require_walls(scene):
    import numpy as np
    from .geometry import rotation
    objects={o.id:o for o in scene.world.collision_objects}
    for name,dimensions,center in wall_specs():
        obj=objects.get(name)
        if obj is None or len(obj.primitives)!=1 or len(obj.primitive_poses)!=1:
            raise RuntimeError('Workspace limits absent; restart run_real.sh')
        def transform(pose):
            q=[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
            p=np.array([pose.position.x,pose.position.y,pose.position.z])
            if not np.isfinite(p).all() or not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q),1.):
                raise RuntimeError('Invalid workspace wall pose: '+name)
            return rotation(q),p
        base_r,base_p=transform(obj.pose)
        local_r,local_p=transform(obj.primitive_poses[0])
        # MoveIt may transfer the primitive pose into CollisionObject.pose.
        # Compare the composed world placement, not its serialization.
        world_p=base_p+base_r@local_p
        world_r=base_r@local_r
        if (not np.allclose(obj.primitives[0].dimensions,dimensions,atol=1e-8,rtol=0)
                or not np.allclose(world_p,center,atol=1e-8,rtol=0)
                or not np.allclose(world_r,np.eye(3),atol=1e-8,rtol=0)
                or obj.meshes or obj.planes
                or obj.header.frame_id!='world' or obj.primitives[0].type!=1):
            raise RuntimeError('Workspace limits changed; restart run_real.sh')
