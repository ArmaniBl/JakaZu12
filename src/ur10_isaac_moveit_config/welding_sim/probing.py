"""Nominal CAD contacts for previewing a two-sided seam probe."""
import numpy as np


def probe_contacts(geometry, selected, seam):
    """Return two ordered A/B pairs, or none if either surface is ambiguous."""
    from .geometry import Edge
    is_edge = isinstance(selected, Edge)
    if not is_edge and getattr(selected, 'kind', '') != 'weld':
        return []
    seam = np.asarray(seam)
    lengths = np.linalg.norm(np.diff(seam, axis=0), axis=1)
    distance = np.r_[0., np.cumsum(lengths)]
    if distance[-1] < .010:
        return []
    up = selected.outward
    planes = [(name, face) for name, face in geometry.contours.items()
              if face.kind == 'outer' and face.outward @ up > .5
              and (not is_edge or name in selected.faces)]
    contacts = []
    surface_pair = None
    for station, fraction in enumerate((.25, .75), start=1):
        along = fraction * distance[-1]
        index = np.searchsorted(distance, along, side='right') - 1
        tangent = (seam[index + 1] - seam[index]) / lengths[index]
        origin = seam[index] + tangent * (along - distance[index])
        across = np.cross(up, tangent)
        width = np.linalg.norm(across)
        if width < 1e-8:
            return []
        across /= width
        axes = np.column_stack((np.cross(across, up), across))
        pair = []
        for sign, side in ((-1, 'A'), (1, 'B')):
            offset = origin + sign * .005 * across
            hits = []
            for name, face in planes:
                loops = face.loops if face.loops is not None else [face.vertices]
                inside = False
                for loop in loops:
                    xy = (loop - offset) @ axes
                    x, y = xy[:-1].T
                    next_x, next_y = xy[1:].T
                    crossing = (y > 0) != (next_y > 0)
                    intersections = x[crossing] - y[crossing] * (
                        next_x[crossing] - x[crossing]) / (next_y[crossing] - y[crossing])
                    inside ^= bool(np.count_nonzero(intersections > 0) % 2)
                if inside:
                    height = ((face.vertices[0] - offset) @ face.outward) / (up @ face.outward)
                    if abs(height) <= .010:
                        hits.append((name, offset + height * up, face.outward))
            if len(hits) != 1:
                return []
            name, point, normal = hits[0]
            pair.append(name)
            contacts.append(dict(station=station, side=side, surface=name,
                                 position_m=point.tolist(), normal=normal.tolist()))
        if pair[0] == pair[1] or (surface_pair is not None and pair != surface_pair):
            return []
        surface_pair = pair
    return contacts
