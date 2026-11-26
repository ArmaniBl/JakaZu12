"""Joint-axis mapping shared by JAKA visualization and Isaac replay."""
from copy import deepcopy
import math

from .profiles import JAKA, UR_JOINTS

AXIS_SIGNS = (1., -1., -1., -1., 1., 1.)
# The JAKA Xacro mounts the torch at 225° about Link_6. Isaac's tool0
# has zero mount yaw; a constant wrist offset preserves angular motion.
AXIS_OFFSETS = (0., 0., 0., 0., -math.pi, -3 * math.pi / 4)


def ur10_to_jaka(positions):
    angles = [sign * (angle - offset) for sign, offset, angle in
              zip(AXIS_SIGNS, AXIS_OFFSETS, positions, strict=True)]
    for i in (4, 5):
        angles[i] = math.remainder(angles[i], math.tau)
    return angles


def replay_branch(jaka_start, ur_start, all_positions, limits):
    offsets = list(AXIS_OFFSETS)
    # Choose a single equivalent branch for the entire trajectory. Wrapping
    # points individually would introduce jumps through an extra revolution.
    for i, (sign, start, actual) in enumerate(zip(AXIS_SIGNS, jaka_start, ur_start, strict=True)):
        points = [sign * position[i] + offsets[i] for position in all_positions]
        lower, upper = limits[i]
        first = math.ceil((lower - min(points) - 1e-8) / math.tau)
        last = math.floor((upper - max(points) + 1e-8) / math.tau)
        if first > last:
            raise RuntimeError(f'JAKA replay exceeds the UR10 range of {UR_JOINTS[i]}')
        nearest = round((actual - (sign * start + offsets[i])) / math.tau)
        offsets[i] += math.tau * min(last, max(first, nearest))
    return offsets


def convert_trajectory(trajectory, ur_names, offsets, scale):
    result = deepcopy(trajectory)
    path = result.joint_trajectory
    indices = [path.joint_names.index(name) for name in JAKA.joints]
    path.joint_names = list(ur_names)
    # JAKA allows 1.57 rad/s while the sim controller is configured for 1 rad/s.
    # Uniform time scaling preserves the exact motion and velocity continuity.
    for point in path.points:
        point.positions = [s * point.positions[j] + o for j, s, o in
                           zip(indices, AXIS_SIGNS, offsets, strict=True)]
        for field, divisor in (('velocities', scale), ('accelerations', scale**2)):
            values = getattr(point, field)
            if values:
                setattr(point, field, [s * values[j] / divisor for j, s in zip(indices, AXIS_SIGNS, strict=True)])
        point.effort = []
        duration = point.time_from_start
        ns = round((duration.sec * 1_000_000_000 + duration.nanosec) * scale)
        duration.sec, duration.nanosec = divmod(ns, 1_000_000_000)
    return result
