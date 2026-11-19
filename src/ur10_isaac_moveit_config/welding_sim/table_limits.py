"""Collision walls bounding the whole robot to the tabletop plus 100 mm."""
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
