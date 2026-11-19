"""Rigid, operator-directed leveling for viewing only; not robot calibration."""
import math
import numpy as np


def scaled_preview_points(points, anchor, factor=1.):
    """Apply one measured unit correction about the saved camera anchor."""
    points,anchor=np.asarray(points,dtype=float),np.asarray(anchor,dtype=float)
    if (points.ndim!=2 or points.shape[1]!=3 or anchor.shape!=(3,)
            or not np.isfinite(points).all() or not np.isfinite(anchor).all()
            or not np.isfinite(factor) or not .01<=factor<=100):
        raise ValueError('Invalid preview metric scale or camera points')
    return (points-anchor)*factor


def yaw_about_point(rotation, translation, angle_deg, pivot):
    """Compose a rigid world-Z rotation without moving its fixed reference point."""
    rotation,translation,pivot = (np.asarray(v,dtype=float) for v in (rotation,translation,pivot))
    if (rotation.shape!=(3,3) or translation.shape!=(3,) or pivot.shape!=(3,)
            or not np.isfinite(rotation).all() or not np.isfinite(translation).all()
            or not np.isfinite(pivot).all() or not math.isfinite(angle_deg)):
        raise ValueError('Invalid preview pivot rotation')
    angle = math.radians(angle_deg)
    yaw = np.array([[math.cos(angle),-math.sin(angle),0.],
                    [math.sin(angle),math.cos(angle),0.],[0.,0.,1.]])
    return yaw@rotation,pivot+yaw@(translation-pivot)


def table_height_anchor(points, anchor, rotation, target_height=0.):
    """One-time visual height reference from the robust lower cloud boundary."""
    points, anchor, rotation = (np.asarray(v,dtype=float) for v in (points,anchor,rotation))
    if (points.ndim!=2 or points.shape[1]!=3 or len(points)<100 or anchor.shape!=(3,)
            or rotation.shape!=(3,3) or not np.isfinite(points).all() or not np.isfinite(anchor).all()
            or not np.isfinite(rotation).all() or not np.isfinite(target_height)
            or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(rotation),1.)):
        raise ValueError('Invalid cloud or rigid transform for visual table height')
    heights = ((points-anchor)@rotation.T)[:,2]
    bottom = float(np.percentile(heights,2.))
    correction = float(target_height-bottom)
    updated = anchor-correction*rotation[2]
    return updated,dict(source='operator: workpiece moved from stand onto table',
                        method='observed cloud lower 2nd percentile; visual estimate',
                        world_z_correction_m=correction,previous_lower_height_m=bottom,
                        target_height_m=target_height,registered=False)


def level_rotation(points, table_yaw_deg, nominal_down_deg=30.):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 100 or not np.isfinite(points).all():
        raise ValueError('A finite measured cloud is required for preview leveling')
    sample = points[::max(1, len(points)//10000)]
    rng = np.random.default_rng(12)
    best = np.zeros(len(sample), dtype=bool)
    for _ in range(180):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-7:
            continue
        normal /= length
        mask = np.abs((sample-a)@normal) < .0015
        if mask.sum() > best.sum():
            best = mask
    if best.mean() < .55:
        raise ValueError('No dominant planar reference for leveling; keep the reference surface visible')
    plane = sample[best]
    _, singular, vectors = np.linalg.svd(plane-plane.mean(0), full_matrices=False)
    if singular[1]/math.sqrt(len(plane)) < .01:
        raise ValueError('Reference plane is too narrow to determine orientation')
    up = vectors[-1]
    # Camera optical +Z points downward toward the table.
    if up[2] > 0:
        up = -up
    down_angle = math.degrees(math.asin(float(np.clip(-up[2], -1, 1))))
    if abs(down_angle-nominal_down_deg) > 25:
        raise ValueError('Dominant plane disagrees with the operator camera angle; inspect the scan')
    # Project optical forward onto the plane. The operator assigned this to
    # increasing row numbers. No PCA-based in-plane yaw guess is used.
    forward = np.array([0., 0., 1.]) - up*up[2]
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    camera_to_table = np.vstack([right, forward, up])
    yaw = math.radians(table_yaw_deg)
    table_to_world = np.array([[math.cos(yaw), -math.sin(yaw), 0.],
                               [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
    return dict(camera_to_world_rotation=(table_to_world@camera_to_table).tolist(),
                reference_normal_camera=up.tolist(), plane_inliers=int(best.sum()),
                reference_point_count=len(sample), estimated_down_angle_deg=down_angle,
                table_yaw_deg=table_yaw_deg, heading='increasing_rows',
                source='dominant plane leveling with operator-assigned heading', registered=False)
