"""Read a coherent UR10 pose from ROS TF for the browser's schematic view."""
from bisect import bisect_right
import math
import time
from xml.etree import ElementTree

from scipy.spatial.transform import Rotation

from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetPositionFK
from rclpy.time import Time
from tf2_ros import TransformException

from .joint_mapping import ur10_to_jaka

FRAMES = ("base_link", "shoulder_link", "upper_arm_link", "forearm_link",
          "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0")


def robot_snapshot(buffer, feedback_fresh, now_sec=None, frames=FRAMES, world='world', *, optional_frames=()):
    unavailable = {"available": False, "frame": world, "points": []}
    if not feedback_fresh:
        return {**unavailable, "reason": "Нет актуальной обратной связи суставов"}
    try:
        # Use the tip's latest common time for every link, rather than mixing
        # independently sampled transforms while the arm is moving.
        tip = buffer.lookup_transform(world, frames[-1], Time())
        stamp = Time.from_msg(tip.header.stamp)
        if now_sec is not None and abs(now_sec-stamp.nanoseconds/1e9) > 1.5:
            return {**unavailable, "reason": "Данные TF робота устарели"}
        points = []
        for frame in (*frames, *optional_frames):
            try:
                transform = tip if frame == frames[-1] else buffer.lookup_transform(world, frame, stamp)
            except TransformException:
                if frame in optional_frames:
                    continue
                raise
            t, q = transform.transform.translation, transform.transform.rotation
            position = [t.x, t.y, t.z]
            orientation = [q.x, q.y, q.z, q.w]
            if (not all(math.isfinite(v) for v in position + orientation)
                    or sum(v*v for v in orientation) < 1e-12):
                raise ValueError("Invalid robot transform")
            points.append({"frame": frame, "position": position, "orientation": orientation})
        return {"available": True, "frame": world, "stamp": stamp.nanoseconds / 1e9,
                "points": points, "source": "ROS TF / joint feedback"}
    except (TransformException, ValueError):
        return {**unavailable, "reason": "Ожидаем TF робота в системе world"}

class JakaWorkcellModel:
    """Visualize the UR10 feedback using JAKA geometry and its own link lengths."""

    def __init__(self, urdf):
        joints = {joint.attrib['name']: joint for joint in ElementTree.parse(urdf).getroot().findall('joint')}
        self.origins = []
        for name in ('fixed', *(f'joint_{i}' for i in range(1, 7))):
            joint = joints[name]
            origin = joint.find('origin')
            self.origins.append((
                joint.find('child').attrib['link'],
                tuple(map(float, origin.attrib['xyz'].split())),
                Rotation.from_euler('xyz', tuple(map(float, origin.attrib['rpy'].split()))),
            ))

    def points(self, ur10_positions, base_position, base_orientation, torch_orientation=None):
        angles = (0., *ur10_to_jaka(ur10_positions))
        position = base_position
        orientation = Rotation.from_quat(base_orientation)
        points = []
        for (frame, xyz, rpy), angle in zip(self.origins, angles):
            position = position + orientation.apply(xyz)
            orientation = orientation * rpy * Rotation.from_euler('z', angle)
            points.append(dict(frame=frame, position=position.tolist(),
                               orientation=orientation.as_quat().tolist()))
        if torch_orientation is not None:
            # Keep the tool on the JAKA flange and use the orientation of the UR10 tool model.
            points.append(dict(frame='welding_torch_link', position=position.tolist(),
                               orientation=torch_orientation))
        return points


EXECUTION_DISPLAY_TOPIC = "/welding/display_execution"


class ExecutionAnimation:
    """Sample the accepted stage for display using MoveIt's configured robot model."""

    def __init__(self, node, profile):
        self.profile = profile
        self.client = node.create_client(GetPositionFK, "/compute_fk")
        self.subscription = node.create_subscription(
            DisplayTrajectory, EXECUTION_DISPLAY_TOPIC, self.receive, 10)
        self.started = 0.
        self.names, self.times, self.positions = [], [], []
        self.pending = None
        self.pose = None

    def receive(self, message):
        if self.pending is not None:
            self.pending.cancel()
            self.pending = None
        self.pose = None
        self.times = []
        if message.model_id != self.profile.model or len(message.trajectory) != 1:
            return
        path = message.trajectory[0].joint_trajectory
        if len(path.joint_names) != len(self.profile.joints) or set(path.joint_names) != set(self.profile.joints):
            return
        times = []
        for point in path.points:
            seconds = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            if (seconds < 0 or (times and seconds <= times[-1])
                    or len(point.positions) != len(path.joint_names)
                    or not all(map(math.isfinite, point.positions))):
                return
            times.append(seconds)
        if not times or times[-1] <= 0:
            return
        self.started = time.monotonic()
        self.names = list(path.joint_names)
        self.times = times
        self.positions = [list(point.positions) for point in path.points]

    def update(self, now):
        elapsed = now - self.started
        if not self.times or elapsed >= self.times[-1]:
            if self.pending is not None:
                self.pending.cancel()
                self.pending = None
            self.pose = None
            return None
        if self.pending is not None:
            if self.pending.done():
                completed, self.pending = self.pending, None
                self.pose = self.from_fk(completed.result())
            elif now - self.requested > .5:
                self.pending.cancel()
                self.pending = None
                self.pose = None
        if self.pending is None and self.client.service_is_ready():
            upper = bisect_right(self.times, elapsed)
            positions = self.positions[0]
            if upper:
                fraction = (elapsed - self.times[upper - 1]) / (self.times[upper] - self.times[upper - 1])
                positions = [a + fraction * (b - a)
                             for a, b in zip(self.positions[upper - 1], self.positions[upper])]
            request = GetPositionFK.Request()
            request.header.frame_id = self.profile.frame
            request.fk_link_names = list(self.profile.frames)
            request.robot_state.joint_state.name = self.names
            request.robot_state.joint_state.position = positions
            self.pending = self.client.call_async(request)
            self.requested = now
        return self.pose

    def from_fk(self, result):
        expected = set(self.profile.frames)
        if result is None or result.error_code.val != 1 or set(result.fk_link_names) != expected:
            return None
        points = []
        for name, stamped in zip(result.fk_link_names, result.pose_stamped):
            if stamped.header.frame_id != self.profile.frame:
                return None
            t, q = stamped.pose.position, stamped.pose.orientation
            position, orientation = [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]
            if not all(map(math.isfinite, position + orientation)) or sum(v*v for v in orientation) < 1e-12:
                return None
            points.append(dict(frame=name, position=position, orientation=orientation))
        return points if {point["frame"] for point in points} == expected else None
