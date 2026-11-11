"""Persistent, explicitly captured joint configuration for the end of welding."""
import json
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


def load_return_pose(path=None, joints=JOINTS):
    path = Path(path) if path is not None else Path(get_package_share_directory(
        'ur10_isaac_moveit_config')) / 'config' / 'return_pose.json'
    data = json.loads(path.read_text(encoding='utf-8'))
    positions = data.get('joint_positions', {})
    if (data.get('units') != 'rad' or set(positions) != set(joints)
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                   for v in positions.values())):
        raise ValueError('Saved return pose must contain exactly the configured joint names and finite angles in radians')
    return data
