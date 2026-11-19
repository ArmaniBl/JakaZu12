"""Sample the actual MoveIt workpiece geometry, including its poses."""
import numpy as np
from shape_msgs.msg import SolidPrimitive
from .runner import pose_transform
from .scan_alignment import transform_points


def sample_object(obj, spacing=.001):
    points, normals = [], []
    outer = pose_transform(obj.pose)

    def append(p, n, pose):
        if sum(len(part) for part in points)+len(p) > 1_000_000:
            raise ValueError('Workpiece surface sampling exceeds one million points')
        matrix = outer @ pose_transform(pose)
        points.append(transform_points(p, matrix))
        normals.append(np.asarray(n) @ matrix[:3, :3].T)

    def triangle(a, b, c, pose):
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-12:
            return
        normal /= length
        count = min(500, max(2, int(max(np.linalg.norm(b-a), np.linalg.norm(c-a))/spacing)+1))
        u, v = np.meshgrid(np.linspace(0, 1, count), np.linspace(0, 1, count))
        mask = u+v <= 1
        p = a+u[mask, None]*(b-a)+v[mask, None]*(c-a)
        append(p, np.tile(normal, (len(p), 1)), pose)

    if len(obj.primitives)+len(obj.meshes) != 1:
        raise ValueError('Expected one workpiece primitive or mesh')
    for primitive, pose in zip(obj.primitives, obj.primitive_poses):
        if primitive.type == SolidPrimitive.BOX:
            size = np.array(primitive.dimensions)
            for axis in range(3):
                other = [i for i in range(3) if i != axis]
                grids = [np.linspace(-size[i]/2, size[i]/2,
                                    min(500, max(2, int(size[i]/spacing)+1))) for i in other]
                u, v = np.meshgrid(*grids)
                for sign in (-1, 1):
                    p = np.zeros((u.size, 3));p[:, axis] = sign*size[axis]/2
                    p[:, other[0]], p[:, other[1]] = u.ravel(), v.ravel()
                    n = np.zeros_like(p);n[:, axis] = sign
                    append(p, n, pose)
        elif primitive.type == SolidPrimitive.CYLINDER:
            height, radius = primitive.dimensions
            angles = np.linspace(0, 2*np.pi, min(1500, max(24, int(2*np.pi*radius/spacing))), endpoint=False)
            heights = np.linspace(-height/2, height/2, min(500, max(2, int(height/spacing)+1)))
            a, z = np.meshgrid(angles, heights)
            n = np.column_stack((np.cos(a.ravel()), np.sin(a.ravel()), np.zeros(a.size)))
            p = n*radius;p[:, 2] = z.ravel();append(p, n, pose)
            grid = np.arange(-radius, radius+spacing, spacing)
            x, y = np.meshgrid(grid, grid);mask = x*x+y*y <= radius*radius
            for sign in (-1, 1):
                p = np.column_stack((x[mask], y[mask], np.full(mask.sum(), sign*height/2)))
                n = np.zeros_like(p);n[:, 2] = sign;append(p, n, pose)
        else:
            raise ValueError('Only BOX, CYLINDER and triangle mesh registration supported')
    for mesh, pose in zip(obj.meshes, obj.mesh_poses):
        vertices = np.array([[v.x, v.y, v.z] for v in mesh.vertices])
        if len(mesh.triangles) > 20000:
            raise ValueError('Too many mesh triangles')
        for t in mesh.triangles:
            triangle(*vertices[list(t.vertex_indices)], pose)
    if not points:
        raise ValueError('Empty workpiece geometry')
    return np.concatenate(points), np.concatenate(normals)
