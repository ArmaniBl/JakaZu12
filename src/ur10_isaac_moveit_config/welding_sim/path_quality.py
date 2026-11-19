"""Compare actual joint travel; bounded joints must never be wrapped modulo 2*pi."""
import math
import numpy as np


def trajectory_quality(stages, start, joints):
    initial = np.array([start[j] for j in joints], dtype=float)
    previous = initial.copy()
    travel = np.zeros(len(joints))
    excursion = np.zeros(len(joints))
    duration = 0.
    weights = np.array([1.2, 1.2, 1.2, 1.5, 2., 2.5])
    if len(joints) != 6 or not np.isfinite(initial).all() or not stages:
        raise ValueError('Six finite start joints and a complete route are required')
    for name, trajectory in stages:
        path = trajectory.joint_trajectory
        if len(set(path.joint_names)) != len(path.joint_names) or not set(joints).issubset(path.joint_names) or not path.points:
            raise ValueError('Incomplete joint trajectory: '+name)
        indices = [list(path.joint_names).index(j) for j in joints]
        previous_time = -1.
        for index, point in enumerate(path.points):
            if len(point.positions) != len(path.joint_names):
                raise ValueError('Joint trajectory position count mismatch')
            values = np.array([point.positions[i] for i in indices])
            stamp = point.time_from_start.sec+point.time_from_start.nanosec/1e9
            if not np.isfinite(values).all() or not math.isfinite(stamp) or stamp < 0 or stamp <= previous_time:
                raise ValueError('Nonfinite joints or non-increasing trajectory time')
            if index == 0 and np.max(np.abs(values-previous)) > .005:
                raise ValueError('Discontinuous route at '+name)
            travel += np.abs(values-previous)
            excursion = np.maximum(excursion, np.abs(values-initial))
            previous, previous_time = values, stamp
        duration += previous_time
    cost = float(travel@weights+.5*max(excursion)+.01*duration)
    return dict(cost=cost, weighted_joint_travel_rad=float(travel@weights),
                joint_travel_deg=dict(zip(joints,np.degrees(travel).tolist())),
                max_joint_excursion_deg=float(np.degrees(max(excursion))), duration_sec=duration,
                weights=weights.tolist(), objective='weighted joint travel + 0.5*max excursion + 0.01*duration')
