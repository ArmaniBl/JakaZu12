"""Offline RangeVision registration in metres; no robot commands."""
import hashlib
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def points_array(points, minimum=30):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < minimum:
        raise ValueError(f'Expected at least {minimum} XYZ points')
    if not np.isfinite(points).all():
        raise ValueError('Nonfinite scan coordinates')
    return points


def read_cloud(path, units):
    """ASCII PLY or whitespace XYZ/ASC. Explicit units prevent mm/m mistakes."""
    path = Path(path)
    if units not in ('mm', 'm'):
        raise ValueError('Specify scan units: mm or m')
    with path.open(encoding='ascii') as stream:
        if path.suffix.lower() == '.ply':
            if stream.readline().strip() != 'ply':
                raise ValueError('Invalid PLY header')
            properties, count, vertex, ascii_format = [], None, False, False
            for line in stream:
                fields = line.split()
                if fields[:2] == ['format', 'ascii']:
                    ascii_format = True
                if fields[:1] == ['element']:
                    vertex = fields[1] == 'vertex'
                    if vertex:
                        count = int(fields[2])
                if fields[:1] == ['property'] and vertex:
                    if fields[1] == 'list':
                        raise ValueError('List properties on PLY vertices are unsupported')
                    properties.append(fields[-1])
                if line.strip() == 'end_header':
                    break
            if not ascii_format or count is None or not 30 <= count <= 5_000_000:
                raise ValueError('Export ASCII PLY with 30..5000000 vertices')
            indices = [properties.index(axis) for axis in ('x', 'y', 'z')]
            points = [[float(row[i]) for i in indices]
                      for row in (stream.readline().split() for _ in range(count))]
        elif path.suffix.lower() in ('.xyz', '.asc', '.txt'):
            points = np.loadtxt(stream, comments='#', usecols=(0, 1, 2))
        else:
            raise ValueError('Supported exports: ASCII PLY, XYZ, ASC (no header)')
    return points_array(points) * (0.001 if units == 'mm' else 1.0)


def rigid_fit(source, target):
    source, target = points_array(source, 4), points_array(target, 4)
    if source.shape != target.shape:
        raise ValueError('Calibration landmark counts differ')
    a, b = source.mean(0), target.mean(0)
    if np.linalg.svd(source-a, compute_uv=False)[1] < 0.01:
        raise ValueError('Calibration landmarks are too close or collinear')
    u, _, vt = np.linalg.svd((source-a).T @ (target-b))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, b-rotation@a
    return transform


def validate_transform(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('Expected a finite 4x4 scanner-to-world transform')
    r = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1])
            or not np.allclose(r.T@r, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(r), 1)):
        raise ValueError('Calibration must be a rigid transform without scaling/reflection')
    return matrix


def transform_points(points, matrix):
    matrix = validate_transform(matrix)
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def align_translation(scan, model, normals, max_shift=.05, tolerance=.002):
    """Constrained point-to-plane ICP. Rotation is deliberately not inferred.

    Three independent visible surface normals are required, so a top plane
    alone cannot produce a plausible but unconstrained XY correction.
    """
    scan, model = points_array(scan), points_array(model)
    normals = np.asarray(normals, dtype=float)
    if normals.shape != model.shape:
        raise ValueError('Model normals mismatch')
    tree = cKDTree(model)
    delta = np.zeros(3)
    for _ in range(60):
        distances, index = tree.query(scan-delta)
        mask = distances < max_shift
        if mask.sum() < 100:
            raise ValueError('Insufficient overlap near the expected workpiece')
        n = normals[index[mask]]
        if np.linalg.matrix_rank(n, tol=.05) < 3 or np.linalg.cond(n.T@n) > 30:
            raise ValueError('Ambiguous alignment: scan at least three independent surfaces')
        error = np.sum((scan[mask]-delta-model[index[mask]])*n, axis=1)
        step = np.linalg.lstsq(n, error, rcond=None)[0]
        delta += step
        if np.linalg.norm(delta) > max_shift:
            raise ValueError('Correction exceeds maximum allowed translation')
        if np.linalg.norm(step) < 1e-6:
            break
    else:
        raise ValueError('Alignment did not converge')
    distances, index = tree.query(scan-delta)
    # Evaluate every ROI point, not only the points used by ICP.
    rmse = float(np.sqrt(np.mean(distances**2)))
    p95 = float(np.quantile(distances, .95))
    if p95 > tolerance or rmse > tolerance:
        raise ValueError(f'Scan/model mismatch: RMSE={rmse:.6f} m, P95={p95:.6f} m')
    return {'translation_m': delta.tolist(), 'rmse_m': rmse, 'p95_m': p95,
            'point_count': len(scan), 'rotation_changed': False}


def align_pose(scan, model, normals, max_shift=.05, max_angle_deg=15., tolerance=.002):
    """Local six-DOF refinement; reject unobservable or out-of-range solutions."""
    scan, original = points_array(scan), points_array(model)
    normals = np.asarray(normals, dtype=float).copy()
    model = original.copy()
    center = model.mean(0)
    radius = max(float(np.sqrt(np.mean(np.sum((model-center)**2, axis=1)))), .01)
    transform = np.eye(4)
    for iteration in range(60):
        distances, indices = cKDTree(model).query(scan)
        keep = distances < max_shift
        if keep.sum() < 100:
            raise ValueError('Insufficient scan/CAD overlap')
        matched, n = model[indices[keep]], normals[indices[keep]]
        origin = model.mean(0)
        jacobian = np.column_stack((n, np.cross(matched-origin, n)))
        observable = jacobian.copy();observable[:, 3:] /= radius
        singular = np.linalg.svd(observable, compute_uv=False)
        if singular[-1] < .02*singular[0]:
            raise ValueError('Pose is ambiguous: more independent measured surfaces are required')
        errors = np.sum((scan[keep]-matched)*n, axis=1)
        step = np.linalg.lstsq(jacobian, errors, rcond=None)[0]
        rotation = Rotation.from_rotvec(step[3:]).as_matrix()
        update = np.eye(4);update[:3, :3] = rotation
        update[:3, 3] = origin+step[:3]-rotation@origin
        transform = update@transform
        model = transform_points(original, transform)
        normals = normals@rotation.T
        shift = np.linalg.norm(transform_points(center[None], transform)[0]-center)
        angle = np.degrees(Rotation.from_matrix(transform[:3, :3]).magnitude())
        if shift > max_shift or angle > max_angle_deg:
            raise ValueError('Pose correction exceeds 50 mm / 15 degrees; correct the initial placement')
        if np.linalg.norm(step[:3]) < 1e-6 and np.linalg.norm(step[3:]) < 1e-5:
            break
    else:
        raise ValueError('Pose registration did not converge')
    distances, _ = cKDTree(model).query(scan)
    rmse, p95 = float(np.sqrt(np.mean(distances**2))), float(np.quantile(distances, .95))
    if max(rmse, p95) > tolerance:
        raise ValueError(f'Scan/CAD mismatch: RMSE={rmse:.6f} m; P95={p95:.6f} m')
    return transform, dict(rmse_m=rmse, p95_m=p95, point_count=len(scan),
                           center_shift_m=float(shift), rotation_deg=float(angle),
                           iterations=iteration+1)
