"""Contours for boxes, cylinder caps and connected planar mesh faces."""
from dataclasses import dataclass
from itertools import combinations, product
import math

import numpy as np


FACES = tuple(f"{axis}_{sign}" for axis in "xyz" for sign in ("minus", "plus"))


def rotation(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-12:
        raise ValueError("Invalid object quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def quaternion(matrix):
    # Symmetric eigenproblem also handles rotations near 180 degrees.
    m = np.asarray(matrix)
    k = np.array([
        [m[0,0]-m[1,1]-m[2,2], m[1,0]+m[0,1], m[2,0]+m[0,2], m[2,1]-m[1,2]],
        [m[1,0]+m[0,1], m[1,1]-m[0,0]-m[2,2], m[2,1]+m[1,2], m[0,2]-m[2,0]],
        [m[2,0]+m[0,2], m[2,1]+m[1,2], m[2,2]-m[0,0]-m[1,1], m[1,0]-m[0,1]],
        [m[2,1]-m[1,2], m[0,2]-m[2,0], m[1,0]-m[0,1], np.trace(m)],
    ]) / 3.0
    values, vectors = np.linalg.eigh(k)
    q = vectors[:, np.argmax(values)]
    return q if q[3] >= 0 else -q


@dataclass
class Edge:
    id: str
    faces: tuple
    start: np.ndarray
    end: np.ndarray
    outward: np.ndarray

    @property
    def length(self):
        return float(np.linalg.norm(self.end - self.start))

    def path(self, begin=0.0, finish=1.0, offset=0.10, step=0.005, roll=0.0):
        if not all(math.isfinite(v) for v in (begin, finish, offset, step, roll)):
            raise ValueError("Path parameters must be finite")
        if not 0 <= begin < finish <= 1 or offset <= 0 or step <= 0:
            raise ValueError("Require 0 <= begin < finish <= 1, positive offset and step")
        count = max(1, math.ceil(self.length * (finish - begin) / step))
        seam = np.array([self.start + t * (self.end-self.start)
                         for t in np.linspace(begin, finish, count+1)])
        tangent = (self.end-self.start) / self.length
        z_axis = -self.outward
        y_axis = np.cross(z_axis, tangent)
        x_axis = tangent * math.cos(roll) + y_axis * math.sin(roll)
        y_axis = np.cross(z_axis, x_axis)
        orientation = quaternion(np.column_stack((x_axis, y_axis, z_axis)))
        return seam, seam + offset * self.outward, orientation


def box_edges(dimensions, center=(0, 0, 0), orientation=(0, 0, 0, 1)):
    size = np.asarray(dimensions, dtype=float)
    center = np.asarray(center, dtype=float)
    if size.shape != (3,) or not np.all(np.isfinite(size)) or np.any(size <= 0):
        raise ValueError("Box dimensions must be three positive finite lengths")
    if center.shape != (3,) or not np.all(np.isfinite(center)):
        raise ValueError("Box center must be three finite coordinates")
    rot = rotation(orientation)
    edges = []
    for a, b in combinations(range(3), 2):
        free = next(i for i in range(3) if i not in (a, b))
        for sa, sb in product((-1, 1), repeat=2):
            start = np.zeros(3)
            start[a], start[b], start[free] = sa*size[a]/2, sb*size[b]/2, -size[free]/2
            end = start.copy()
            end[free] = size[free]/2
            normal = np.zeros(3)
            normal[a], normal[b] = sa/math.sqrt(2), sb/math.sqrt(2)
            faces = tuple(f"{'xyz'[i]}_{'plus' if sign > 0 else 'minus'}"
                          for i, sign in ((a, sa), (b, sb)))
            edges.append(Edge("__".join(faces), faces, rot @ start + center,
                              rot @ end + center, rot @ normal))
    return edges


def select_edges(edges, names=(), face=None):
    known = {edge.id for edge in edges}
    unknown = set(names) - known
    if unknown:
        raise ValueError(f"Unknown edges: {', '.join(sorted(unknown))}")
    if face is not None and face not in FACES:
        raise ValueError(f"Unknown face {face}; choose from {', '.join(FACES)}")
    return [edge for edge in edges if edge.id in names or face in edge.faces]


@dataclass
class FaceContour:
    id: str
    vertices: np.ndarray
    outward: np.ndarray
    label: str = ''
    kind: str = 'outer'
    face_id: str = ''
    loops: object = None

    @property
    def length(self):
        return float(np.linalg.norm(np.diff(self.vertices, axis=0), axis=1).sum())

    def path(self, begin=0.0, finish=1.0, offset=0.10, step=0.005, roll=0.0):
        if (not all(math.isfinite(v) for v in (begin, finish, offset, step, roll))
                or not 0 <= begin < finish <= 1 or offset <= 0 or step <= 0):
            raise ValueError("Require 0 <= begin < finish <= 1, positive offset and step")
        lengths = np.linalg.norm(np.diff(self.vertices, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
        lower, upper = begin * self.length, finish * self.length
        count = max(1, math.ceil((upper-lower) / step))
        # Include every corner explicitly: never cut diagonally across the face.
        distances = sorted(set(np.linspace(lower, upper, count+1))
                           | {float(s) for s in cumulative if lower < s < upper})
        distances = [distance for i, distance in enumerate(distances)
                     if i == 0 or distance-distances[i-1] > 1e-10]
        points = []
        for distance in distances:
            segment = min(int(np.searchsorted(cumulative, distance, side="right"))-1, len(lengths)-1)
            fraction = (distance-cumulative[segment]) / lengths[segment]
            points.append(self.vertices[segment] + fraction*(self.vertices[segment+1]-self.vertices[segment]))
        seam = np.array(points)
        x_axis = (self.vertices[1]-self.vertices[0])/lengths[0]
        z_axis = -self.outward
        y_axis = np.cross(z_axis, x_axis)
        x_axis = x_axis*math.cos(roll) + y_axis*math.sin(roll)
        orientation = quaternion(np.column_stack((x_axis, np.cross(z_axis, x_axis), z_axis)))
        return seam, seam + offset*self.outward, orientation


def face_contour(edges, face):
    boundary = select_edges(edges, face=face)
    if len(boundary) != 4:
        raise ValueError("A box face must have four boundary edges")
    outward = np.sum([edge.outward for edge in boundary], axis=0)
    outward /= np.linalg.norm(outward)
    vertices = []
    adjacency = {}
    for edge in boundary:
        ids = []
        for point in (edge.start, edge.end):
            index = next((i for i, p in enumerate(vertices) if np.allclose(p, point, atol=1e-9)), None)
            if index is None:
                index = len(vertices)
                vertices.append(point)
            ids.append(index)
        a, b = ids
        adjacency.setdefault(a, []).append(b)
        adjacency.setdefault(b, []).append(a)
    start = min(range(4), key=lambda i: (np.linalg.norm(vertices[i]), tuple(vertices[i])))
    order = [start]
    while len(order) < 4:
        order.append(next(i for i in adjacency[order[-1]] if i not in order))
    ordered = [vertices[i] for i in order]
    if np.dot(np.cross(ordered[1]-ordered[0], ordered[2]-ordered[1]), outward) < 0:
        ordered = [ordered[0], *reversed(ordered[1:])]
    return FaceContour(f"contour:{face}", np.array([*ordered, ordered[0]]), outward)


@dataclass
class WorkpieceGeometry:
    shape: str
    edges: list
    contours: dict
    selectable_seams: tuple = ()

    def contour(self, name):
        if name not in self.contours:
            raise ValueError(f"Unknown face {name}; choose from {', '.join(self.contours)}")
        return self.contours[name]

    def select(self, names):
        by_id = {e.id: e for e in self.edges}
        unknown = set(names)-set(by_id)
        if unknown:
            raise ValueError(f"Unknown edges: {', '.join(sorted(unknown))}")
        return [by_id[name] for name in dict.fromkeys(names)]


def box_geometry(dimensions, center=(0, 0, 0), orientation=(0, 0, 0, 1)):
    edges = box_edges(dimensions, center, orientation)
    return WorkpieceGeometry('box', edges, {f: face_contour(edges, f) for f in FACES})


@dataclass
class CircleContour:
    id: str
    center: np.ndarray
    radius: float
    basis: np.ndarray
    outward: np.ndarray
    faces: tuple
    label: str = ''
    kind: str = 'outer'
    face_id: str = ''
    loops: object = None

    @property
    def length(self):
        return 2*math.pi*self.radius

    @property
    def vertices(self):
        return self.path(offset=.001, step=min(.005, self.radius/8))[0]

    @property
    def start(self):
        return self.center + self.radius*self.basis[:, 0]

    @property
    def end(self):
        return self.start

    def path(self, begin=0., finish=1., offset=.03, step=.005, roll=0.):
        if (not all(math.isfinite(v) for v in (begin, finish, offset, step, roll))
                or not 0 <= begin < finish <= 1 or offset <= 0 or step <= 0):
            raise ValueError('Invalid circle path parameters')
        count = max(3, math.ceil(self.length*(finish-begin)/step))
        angles = np.linspace(begin*2*math.pi, finish*2*math.pi, count+1)
        seam = self.center + self.radius*(np.cos(angles)[:, None]*self.basis[:, 0]
                                         + np.sin(angles)[:, None]*self.basis[:, 1])
        x = self.basis[:, 0]*math.cos(roll) - self.basis[:, 1]*math.sin(roll)
        z = -self.outward
        return seam, seam+offset*self.outward, quaternion(np.column_stack((x, np.cross(z, x), z)))


def cylinder_geometry(dimensions, center=(0, 0, 0), orientation=(0, 0, 0, 1)):
    size, center = np.asarray(dimensions, dtype=float), np.asarray(center, dtype=float)
    if size.shape != (2,) or not np.all(np.isfinite(size)) or np.any(size <= 0):
        raise ValueError('Cylinder dimensions must be positive [height, radius]')
    if center.shape != (3,) or not np.all(np.isfinite(center)):
        raise ValueError('Invalid cylinder center')
    rot = rotation(orientation)
    contours, edges = {}, []
    for sign, label in ((-1, 'z_minus'), (1, 'z_plus')):
        normal = sign*rot[:, 2]
        basis = np.column_stack((rot[:, 0], sign*rot[:, 1], normal))
        circle = CircleContour(f'contour:{label}', center+normal*size[0]/2,
                               float(size[1]), basis, normal, (label, 'side'))
        contours[label] = circle
        edges.append(CircleContour(f'rim:{label}', circle.center, circle.radius, basis, normal, circle.faces))
    return WorkpieceGeometry('cylinder', edges, contours)


def mesh_geometry(vertices, triangles, center=(0, 0, 0), orientation=(0, 0, 0, 1)):
    """Connected coplanar triangles form a face; triangulation diagonals are removed.

    Each boundary loop is a separate seam, including holes. Boundaries are never
    connected across empty space. Nonmanifold surfaces are rejected. Units are meters.
    """
    vertices = np.asarray(vertices, dtype=float)
    triangles = np.asarray(triangles)
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 3
            or not np.all(np.isfinite(vertices)) or triangles.ndim != 2
            or triangles.shape[1] != 3 or not 0 < len(triangles) <= 20000
            or not np.issubdtype(triangles.dtype, np.integer)
            or np.any(triangles < 0) or np.any(triangles >= len(vertices))):
        raise ValueError('Invalid mesh vertices/triangles (limit 20000 triangles)')
    center = np.asarray(center, dtype=float)
    if center.shape != (3,) or not np.all(np.isfinite(center)):
        raise ValueError('Invalid mesh center')
    # STL-style meshes often repeat vertices for every triangle.
    _, first, inverse = np.unique(np.round(vertices, 8), axis=0, return_index=True, return_inverse=True)
    vertices = vertices[first] @ rotation(orientation).T + center
    triangles = inverse[triangles]
    normals, incident = [], {}
    for index, (a, b, c) in enumerate(triangles):
        normal = np.cross(vertices[b]-vertices[a], vertices[c]-vertices[a])
        scale = np.linalg.norm(vertices[b]-vertices[a])*np.linalg.norm(vertices[c]-vertices[a])
        if np.linalg.norm(normal) < max(1e-18, scale*1e-8):
            raise ValueError('Mesh contains degenerate triangles')
        normals.append(normal/np.linalg.norm(normal))
        for u, v in ((a, b), (b, c), (c, a)):
            incident.setdefault(tuple(sorted((int(u), int(v)))), []).append((index, int(u), int(v)))
    neighbors = {i: [] for i in range(len(triangles))}
    for entries in incident.values():
        if len(entries) > 2:
            raise ValueError('Nonmanifold mesh edge')
        if len(entries) == 2:
            (a, u, v), (b, x, y) = entries
            if (u, v) == (x, y):
                raise ValueError('Mesh triangles have inconsistent winding')
            if np.dot(normals[a], normals[b]) > 1-1e-9:
                neighbors[a].append(b)
                neighbors[b].append(a)
    groups, labels = [], {}
    for seed in range(len(triangles)):
        if seed in labels:
            continue
        group, pending = [], [seed]
        labels[seed] = len(groups)
        while pending:
            index = pending.pop()
            group.append(index)
            for other in neighbors[index]:
                if other not in labels:
                    labels[other] = len(groups)
                    pending.append(other)
        groups.append(group)
    boundaries = [[] for _ in groups]
    edges = []
    for (u, v), entries in sorted(incident.items()):
        faces = sorted({labels[t] for t, _, _ in entries})
        if len(entries) == 2 and len(faces) == 1:
            continue
        outward = np.sum([normals[groups[f][0]] for f in faces], axis=0)
        if np.linalg.norm(outward) < 1e-8:
            raise ValueError('Mesh has an ambiguous edge normal')
        edge = Edge(f'edge_{u}_{v}', tuple(f'face_{f}' for f in faces),
                    vertices[u], vertices[v], outward/np.linalg.norm(outward))
        edges.append(edge)
        for f in faces:
            boundaries[f].append((u, v))
    contours = {}
    for f, boundary in enumerate(boundaries):
        normal = normals[groups[f][0]]
        plane_origin = vertices[triangles[groups[f][0]][0]]
        group_points = vertices[triangles[groups[f]].reshape(-1)]
        if np.max(np.abs((group_points-plane_origin) @ normal)) > 1e-6:
            raise ValueError('Merged mesh patch is not planar; simplify or split the surface')
        adjacent = {}
        for u, v in boundary:
            adjacent.setdefault(u, []).append(v)
            adjacent.setdefault(v, []).append(u)
        if not adjacent or any(len(v) != 2 for v in adjacent.values()):
            raise ValueError('Mesh face has an invalid boundary')
        remaining=set(adjacent);loops=[]
        while remaining:
            start=min(remaining);order=[start];previous,current=None,start
            while True:
                nxt=next(v for v in adjacent[current] if v!=previous)
                if nxt==start:break
                if nxt in order:raise ValueError('Mesh face boundary is not simple')
                order.append(nxt);previous,current=current,nxt
            remaining.difference_update(order)
            points=vertices[order]
            area=float(np.dot(np.sum(np.cross(points,np.roll(points,-1,axis=0)),axis=0),normal))
            if abs(area)<1e-18:raise ValueError('Degenerate mesh boundary loop')
            if area<0:points=points[::-1]
            loops.append((abs(area),np.vstack((points,points[0]))))
        loops.sort(key=lambda item:-item[0])
        face_id=f'face_{f}'
        side='Верхний' if normal[2]>.7 else 'Нижний' if normal[2]<-.7 else f'Боковой {f+1}'
        paths=[item[1] for item in loops]
        for index,points in enumerate(paths):
            key=face_id if index==0 else f'{face_id}_hole_{index}'
            if index==0:
                label=f'{side} внешний контур'
            else:
                spread=np.linalg.svd(points[:-1]-points[:-1].mean(0),compute_uv=False)
                feature='прорези' if spread[0]>1.8*spread[1] else 'отверстия'
                label=f'{side} контур {feature} {index}'
            contours[key]=FaceContour('contour:'+key,points,normal,label,
                'outer' if index==0 else 'hole',face_id,paths if index==0 else None)
    return WorkpieceGeometry('mesh', edges, contours)
