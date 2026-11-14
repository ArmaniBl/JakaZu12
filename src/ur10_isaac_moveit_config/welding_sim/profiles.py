"""Robot-specific names; joint states and trajectories keep the model's names."""
from dataclasses import asdict, dataclass, replace
import json
from pathlib import Path

UR_JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


@dataclass(frozen=True)
class RobotProfile:
    name: str = 'isaac_ur10'
    real: bool = False
    model: str = 'ur10'
    group: str = 'ur_manipulator'
    tip: str = 'welding_tcp'
    frame: str = 'world'
    joint_states: str = '/ur10/joint_states'
    source_topic: str = '/joint_states'
    publisher: str = '_Graph*'
    trajectory_action: str = '/ur10_controller/follow_joint_trajectory'
    joints: tuple = UR_JOINTS
    frames: tuple = ('base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link',
                     'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'tool0', 'welding_torch_link', 'welding_tcp')

    def accepts(self, message):
        return (len(message.name) == len(set(message.name)) == len(message.position)
                and set(self.joints).issubset(message.name))

    def publisher_matches(self, name):
        return name.startswith(self.publisher[:-1]) if self.publisher.endswith('*') else name == self.publisher

    def payload(self):
        return asdict(self)


ISAAC = RobotProfile()
JAKA = RobotProfile(name='jaka_real', real=True, model='jaka_zu12', group='jaka_zu12',
                   tip='welding_tcp', joint_states='/joint_states', publisher='moveit_server',
                   trajectory_action='/jaka_zu12_controller/follow_joint_trajectory',
                   joints=tuple(f'joint_{i}' for i in range(1, 7)),
                   frames=tuple(f'Link_{i}' for i in range(7))+('welding_torch_link','welding_tcp'))


def load_profile(name='isaac_ur10', path=None):
    if name not in ('isaac_ur10', 'jaka_real'):
        raise ValueError(f'Unknown robot profile: {name}')
    profile = JAKA if name == 'jaka_real' else ISAAC
    if path:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        allowed = {'group', 'tip', 'frame', 'joint_states', 'source_topic', 'publisher',
                   'trajectory_action', 'joints', 'frames'}
        if not isinstance(data, dict) or set(data)-allowed:
            raise ValueError('Unknown robot profile fields')
        for key, value in data.items():
            if key in ('joints', 'frames'):
                if (not isinstance(value, list) or len(value) < 2
                        or any(not isinstance(v, str) or not v for v in value)
                        or len(set(value)) != len(value)):
                    raise ValueError(f'Invalid {key}')
                data[key] = tuple(value)
            elif not isinstance(value, str) or not value:
                raise ValueError(f'Invalid {key}')
        profile = replace(profile, **data)
        if len(profile.joints) != 6:
            raise ValueError('The supported robot profiles require six actuated joints')
        if profile.frames[-1] != profile.tip:
            raise ValueError('Last frame must be the configured tool tip')
    return profile
