"""Bounded candidate tilts; the seam and TCP positions are never changed."""
import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .geometry import quaternion, rotation


def tilted_orientation(q, tilt=0., direction=0., roll=0.):
    if not all(math.isfinite(v) for v in (tilt,direction,roll)) or not 0<=tilt<=60:
        raise ValueError('Tilt must be finite and within 0..60 degrees')
    def rz(angle):
        angle=math.radians(angle);c,s=math.cos(angle),math.sin(angle)
        return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
    angle=math.radians(tilt);c,s=math.cos(angle),math.sin(angle)
    ry=np.array([[c,0,s],[0,1.,0],[-s,0,c]])
    return quaternion(rotation(q)@rz(roll)@rz(direction)@ry@rz(-direction))


def candidates(tilt, direction, maximum):
    yield tilt,direction,0.
    seen={(round(tilt,8),round(direction%360,8),0.)}
    for inclination in dict.fromkeys((min(30.,maximum),maximum,min(15.,maximum),0.)):
        for heading in (0.,90.,180.,270.):
            for roll in (90.,-90.,180.,0.):
                if inclination==0 and heading!=0:continue
                key=(inclination,heading,roll)
                if key not in seen:
                    seen.add(key);yield key


def radial_orientations(q, seam, center, up, tilt, roll):
    """Lean a downward-facing TCP toward the circular joint at each sample."""
    radial = seam - center
    radial -= (radial @ up)[:, None] * up
    radial /= np.linalg.norm(radial, axis=1, keepdims=True)
    axes = np.cross(up, radial)
    return (Rotation.from_rotvec(axes * math.radians(tilt))
            * Rotation.from_quat(tilted_orientation(q, roll=roll))).as_quat()


def orbit_orientations(q, seam, center, up, tilt, roll):
    """Keep the same torch posture relative to the tube around the full joint."""
    radial = seam - center
    radial -= (radial @ up)[:, None] * up
    radial /= np.linalg.norm(radial, axis=1, keepdims=True)
    basis = rotation(q)
    reference = basis[:, 0] - (basis[:, 0] @ up) * up
    reference /= np.linalg.norm(reference)
    phase = np.arctan2(radial @ np.cross(up, reference), radial @ reference)
    lean = Rotation.from_rotvec(np.cross(up, radial) * math.radians(tilt))
    orbit = Rotation.from_rotvec(phase[:, None] * up)
    return (lean * orbit * Rotation.from_quat(tilted_orientation(q, roll=roll))).as_quat()


# Angle travelled around the tube, tool roll and heading correction (degrees).
# Sampled for the standard pipe plate at S8. MoveIt validates each placement.
PIPE_PLATE_WRIST_PROFILE = (
    (252., -90., 0.),
    (256.5, -78., 0.),
    (265.5, -78., 0.),
    (272.25, -66., 0.),
    (297., -66., 0.),
    (301.5, -66., -12.),
    (306., -66., -12.),
    (310.5, -66., -24.),
    (315.71424, -66., -24.),
    (321.42862, -66., -36.),
    (330.75, -66., -36.),
    (335.25, -66., -48.),
    (339.75, -66., -48.),
    (344.28571, -66., -60.),
    (350.00002, -66., -60.),
    (355.5, -66., -72.),
    (360., -66., -72.),
)


def pipe_plate_orientations(q, seam, center, up):
    """Apply the sampled wrist profile while leaving each pipe TCP position unchanged."""
    radial = seam[0] - center
    radial -= (radial @ up) * up
    radii = seam - center
    angles = np.unwrap(np.arctan2(radii @ np.cross(up, radial), radii @ radial))
    angles = np.clip(np.degrees(angles), 0., 360.)
    angles[-1] = 360.
    orientations = orbit_orientations(q, seam, center, up, 35., -90.)
    anchor_angles, anchor_roll, anchor_heading = np.asarray(PIPE_PLATE_WRIST_PROFILE).T
    anchors = []
    for angle, roll, heading in zip(anchor_angles, anchor_roll, anchor_heading):
        point = center + Rotation.from_rotvec(math.radians(angle) * up).apply(radial)
        orbital = orbit_orientations(q, point[None, :], center, up, 35., roll)[0]
        rotation_at_angle = Rotation.from_rotvec(math.radians(heading) * up) * Rotation.from_quat(orbital)
        anchors.append(rotation_at_angle.as_quat())
    uses_profile = angles >= anchor_angles[0]
    orientations[uses_profile] = Slerp(anchor_angles, Rotation.from_quat(anchors))(angles[uses_profile]).as_quat()
    return orientations


def swept_orientations(q, seam, tilt, heading, roll):
    """Vary the lean along a straight weld while retaining its TCP positions."""
    distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(seam, axis=0), axis=1))]
    fraction = distances / distances[-1]
    signed_tilts = tilt * np.cos(math.pi * fraction)
    return np.array([tilted_orientation(q, abs(angle), (heading - roll) + (180. if angle < 0 else 0.), roll)
                     for angle in signed_tilts])


def aimed_orientations(q, seam, points, max_tilt, bias, roll):
    """Aim at the actual joint from each TCP position, with a bounded lean."""
    nominal = rotation(q)[:, 2]
    direction = seam - points
    distances = np.linalg.norm(direction, axis=1, keepdims=True)
    direction = np.divide(direction, distances, out=np.broadcast_to(nominal, direction.shape).copy(),
                          where=distances > 1e-10)
    if bias:
        tangent = seam[-1] - seam[0]
        tangent /= np.linalg.norm(tangent)
        direction += math.tan(math.radians(bias)) * tangent
        direction /= np.linalg.norm(direction, axis=1, keepdims=True)
    axes = np.cross(nominal, direction)
    magnitudes = np.linalg.norm(axes, axis=1, keepdims=True)
    axes = np.divide(axes, magnitudes, out=np.zeros_like(axes), where=magnitudes > 1e-10)
    angles = np.arctan2(magnitudes[:, 0], direction @ nominal)
    rotation_vectors = axes * np.minimum(angles, math.radians(max_tilt))[:, None]
    return (Rotation.from_rotvec(rotation_vectors)
            * Rotation.from_quat(tilted_orientation(q, roll=roll))).as_quat()


def standoff_orientations(normal, max_tilt):
    """Sample postures for reaching a face with a non-contact probe."""
    down = np.array([0., 0., -1.])
    heading = math.atan2(-normal[1], -normal[0])
    x = np.array([math.cos(heading), math.sin(heading), 0.])
    q = quaternion(np.column_stack((x, np.cross(down, x), down)))
    for tilt in dict.fromkeys((max_tilt, min(30., max_tilt), 0.)):
        for roll in (0., 180., 90., -90., 45., -45., 135., -135.):
            yield tilted_orientation(q, tilt, -roll, roll)


def route_candidates(paths, options, max_tilt, *, allow_reordering, near_joint=False):
    """Order the sampled postures and traversals without changing the selected points."""
    nominal = [('fixed', 'forward', paths, *angles) for angles in options]
    if not allow_reordering or len(paths) != 1:
        if near_joint and not allow_reordering and len(set(options)) > 1:
            preferred = (min(30., max_tilt), 90., 90.)
            options = list(dict.fromkeys([options[0], preferred, *options]))
            return [('fixed', 'forward', paths, *angles) for angles in options]
        return nominal
    edge, seam, points, q = paths[0]
    if getattr(edge, 'kind', None) != 'weld' or getattr(edge, 'level_tcp', False):
        return nominal

    traversals = [('forward', paths)]
    closed = np.allclose(seam[0], seam[-1])
    if closed:
        for quarter in (2, 1, 3):
            index = (len(seam) - 1) * quarter // 4
            if index:
                order = np.r_[index:len(seam) - 1, :index + 1]
                traversals.append((f'loop_quarter_{quarter}', [(edge, seam[order], points[order], q)]))
    else:
        traversals.append(('reverse', [(edge, seam[::-1], points[::-1], q)]))
    if len(traversals) == 1:
        return nominal

    priority = [(max_tilt, heading, roll) for heading, roll in
                ((0., 180.), (270., 90.), (270., 0.), (90., 90.), (180., 180.))]
    ordered = list(dict.fromkeys([options[0], *priority, *options]))
    # Visit distinct tool postures early, then try the remaining entry points.
    search = [('fixed', *traversals[(index + shift) % len(traversals)], *angles)
              for shift in range(len(traversals)) for index, angles in enumerate(ordered)]
    if max_tilt <= 0:
        return search

    circular = getattr(edge, 'radial_offset_center', None) is not None
    if circular:
        variable = [
            ('radial', *traversals[index % len(traversals)], min(max_tilt, tilt), 0., roll)
            for index, (tilt, roll) in enumerate(((30., 0.), (15., 90.), (45., -90.), (30., 180.)))
        ]
    else:
        variable = [
            ('sweep', *traversals[index % len(traversals)], max_tilt, heading, roll)
            for index, (heading, roll) in enumerate(((180., 0.), (0., 180.), (180., 90.), (0., -90.),
                                                    (180., 180.), (0., 0.), (90., 90.), (270., -90.)))
        ]
    for index, candidate in enumerate(variable):
        search.insert(2 + index * 2, candidate)
    if near_joint:
        rolls = (0., 90., -90., 180., 45., -45., 135., -135.)
        aimed = [
            ('aimed', *traversals[index % len(traversals)], max_tilt, bias, roll)
            for bias in ((0.,) if circular else (0., -15., 15.))
            for index, roll in enumerate(rolls)
        ]
        if circular and closed:
            # Keep the complete ring while sampling different starts and wrist postures.
            orbits = [
                ('orbit', *traversal, min(max_tilt, tilt), 0., roll)
                for tilt, roll in ((35., -90.), (60., -67.5), (45., -90.), (55., -90.))
                for traversal in traversals
            ]
            aimed = orbits + aimed
            quarter = next((item for item in traversals if item[0] == 'loop_quarter_1'), None)
            if quarter is not None and max_tilt >= 35. and getattr(edge, 'id', '') == 'seam_pipe_base':
                kind, [(edge, ring, tcp, q)] = quarter
                aimed[:0] = [
                    ('pipe_plate', *quarter, 35., 0., -90.),
                    ('pipe_plate', f'{kind}_reverse', [(edge, ring[::-1], tcp[::-1], q)], 35., 0., -90.),
                ]
        for index, candidate in enumerate(aimed):
            search.insert(1 + index * 2, candidate)
    return search
