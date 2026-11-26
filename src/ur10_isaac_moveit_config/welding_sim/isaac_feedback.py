"""Record the simulator's ROS joint feedback during each executed stage."""
import time


class IsaacFeedbackCapture:
    def __init__(self, topic, joints):
        self.joints = joints
        self.report = {'source': topic, 'position_units': 'rad', 'stages': []}
        self.stage = None

    def begin(self, name, trajectory):
        point = trajectory.points[-1]
        self.stage = {
            'name': name,
            'started_unix': time.time(),
            'planned_duration_sec': point.time_from_start.sec + point.time_from_start.nanosec / 1e9,
            'target_joints_rad': dict(zip(trajectory.joint_names, point.positions)),
            'samples': [],
        }
        self.last_sample = float('-inf')
        self.report['stages'].append(self.stage)

    def record(self, state):
        now = time.monotonic()
        if self.stage is None or now - self.last_sample < .1:
            return
        self.last_sample = now
        positions = dict(zip(state.name, state.position))
        self.stage['samples'].append({
            'unix': time.time(),
            'actual_joints_rad': {joint: positions[joint] for joint in self.joints},
        })

    def end(self, outcome):
        if self.stage is not None:
            self.stage.update(finished_unix=time.time(), **outcome)
            self.stage = None

    def close(self):
        self.end({'status': 'interrupted'})
