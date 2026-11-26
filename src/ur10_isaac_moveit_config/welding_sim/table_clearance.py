"""Conservative sampled clearance of the moving robot above the tabletop."""
from itertools import product
from pathlib import Path
import math
from xml.etree import ElementTree as ET

import numpy as np
from ament_index_python.packages import get_package_share_directory

from .geometry import rotation

TABLE_CLEARANCE_M = .010
JOINT_SAMPLE_RAD = .025


def _xyz(node, key, default):
    return np.array([float(value) for value in node.get(key, default).split()])


def _origin(node):
    if node is None:
        return np.eye(3), np.zeros(3)
    x, y, z = _xyz(node, 'rpy', '0 0 0') / 2
    cx, cy, cz, sx, sy, sz = math.cos(x), math.cos(y), math.cos(z), math.sin(x), math.sin(y), math.sin(z)
    q = [sx*cy*cz-cx*sy*sz, cx*sy*cz+sx*cy*sz, cx*cy*sz-sx*sy*cz, cx*cy*cz+sx*sy*sz]
    return rotation(q), _xyz(node, 'xyz', '0 0 0')


def _vertices(geometry):
    box = geometry.find('box')
    if box is not None:
        return np.array(list(product((-.5, .5), repeat=3))) * _xyz(box, 'size', '')
    mesh = geometry.find('mesh')
    if mesh is None or not mesh.get('filename', '').startswith('package://'):
        raise ValueError('Unsupported collision shape in robot_description')
    package, _, filename = mesh.get('filename')[len('package://'):].partition('/')
    raw = (Path(get_package_share_directory(package)) / filename).read_bytes()
    if len(raw) < 84 or 84 + int.from_bytes(raw[80:84], 'little') * 50 != len(raw):
        raise ValueError(f'Expected binary collision STL: {filename}')
    records = np.frombuffer(raw, offset=84, dtype=[('face', '<f4', (4, 3)), ('flag', '<u2')])
    vertices = np.unique(records['face'][:, 1:].reshape(-1, 3), axis=0)
    return vertices * _xyz(mesh, 'scale', '1 1 1')


def _turn(axis, angles):
    x, y, z = axis
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    s, c = np.sin(angles)[:, None, None], np.cos(angles)[:, None, None]
    return np.eye(3) + s * cross + (1 - c) * (cross @ cross)


class TableClearance:
    def __init__(self, description, profile, floor):
        robot = ET.fromstring(description)
        self.joints = profile.joints
        links = {link.get('name'): link for link in robot.findall('link')}
        by_child = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
        self.chain = []
        self.shapes = {}
        ordered = {}
        for frame in profile.frames:
            name = frame
            ancestors = []
            while name in by_child and name != profile.frame and name not in ordered:
                ancestors.append(name)
                name = by_child[name].find('parent').get('link')
            ordered.update(dict.fromkeys(reversed(ancestors)))
        moving = set()
        for name in ordered:
            joint = by_child[name]
            parent = joint.find('parent').get('link')
            axis = joint.find('axis')
            number = self.joints.index(joint.get('name')) if joint.get('name') in self.joints else None
            if joint.get('type') not in ('fixed', 'revolute', 'continuous') or (number is not None and axis is None):
                raise ValueError(f'Unsupported joint for clearance: {joint.get("name")}')
            self.chain.append((name, parent, *_origin(joint.find('origin')),
                               number, None if axis is None else _xyz(axis, 'xyz', '')))
            if number is None and parent not in moving:
                continue
            moving.add(name)
            vertices = []
            for collision in links[name].findall('collision'):
                r, p = _origin(collision.find('origin'))
                vertices.append(_vertices(collision.find('geometry')) @ r.T + p)
            if (not vertices and number is not None) or not all(np.isfinite(v).all() for v in vertices):
                raise ValueError(f'No valid collision geometry for {name}')
            if vertices:
                self.shapes[name] = np.ascontiguousarray(np.concatenate(vertices).T)
        if floor.header.frame_id != profile.frame or len(floor.primitives) != 1 or len(floor.primitive_poses) != 1:
            raise ValueError('Expected one tabletop box in the robot world frame')
        box, local = floor.primitives[0], floor.primitive_poses[0]
        if box.type != box.BOX:
            raise ValueError('Expected a tabletop box')
        def transform(pose):
            p, q = pose.position, pose.orientation
            return rotation([q.x, q.y, q.z, q.w]), np.array([p.x, p.y, p.z])
        r, p = transform(floor.pose)
        local_r, local_p = transform(local)
        if not np.allclose(r @ local_r, np.eye(3), atol=1e-7):
            raise ValueError('Tabletop clearance needs a level, axis-aligned collision table')
        self.tabletop = float((p + r @ local_p)[2] + box.dimensions[2] / 2)

    def minimum(self, stages, initial):
        values = dict(zip(initial.name, initial.position))
        previous = np.array([values[j] for j in self.joints])
        result = None
        for stage, trajectory in stages:
            path = trajectory.joint_trajectory
            indices = [path.joint_names.index(j) for j in self.joints]
            samples = [previous]
            for point in path.points:
                target = np.asarray(point.positions)[indices]
                delta = target - previous
                n = max(1, math.ceil(float(np.max(np.abs(delta))) / JOINT_SAMPLE_RAD))
                samples.extend(previous + np.arange(1, n + 1)[:, None] * (delta / n))
                previous = target
            positions = np.asarray(samples)
            for offset in range(0, len(positions), 128):
                distances = self._lowest(positions[offset:offset + 128])
                for link, heights in distances.items():
                    index = int(np.argmin(heights))
                    clearance = float(heights[index] - self.tabletop)
                    if result is None or clearance < result['clearance_m']:
                        result = dict(clearance_m=clearance, link=link, obstacle='floor',
                                      stage=stage, sample=offset + index)
        if result is None:
            raise ValueError('No route to check for tabletop clearance')
        return result

    def _lowest(self, positions):
        if not np.isfinite(positions).all():
            raise ValueError('Nonfinite joint values in tabletop clearance check')
        first_parent = self.chain[0][1]
        transforms = {first_parent: (np.broadcast_to(np.eye(3), (len(positions), 3, 3)),
                                     np.zeros((len(positions), 3)))}
        result = {}
        for name, parent, origin_r, origin_p, number, axis in self.chain:
            r, p = transforms[parent]
            p = p + r @ origin_p
            r = r @ origin_r
            if number is not None:
                r = r @ _turn(axis, positions[:, number])
            transforms[name] = (r, p)
            if name in self.shapes:
                result[name] = p[:, 2] + (r[:, 2] @ self.shapes[name]).min(axis=1)
        return result
