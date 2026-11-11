"""Read a coherent UR10 pose from ROS TF for the browser's schematic view."""
import math

from rclpy.time import Time
from tf2_ros import TransformException

FRAMES = ("base_link", "shoulder_link", "upper_arm_link", "forearm_link",
          "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0")


def robot_snapshot(buffer, feedback_fresh, now_sec=None, frames=FRAMES, world='world'):
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
        for frame in frames:
            transform = tip if frame == frames[-1] else buffer.lookup_transform(world, frame, stamp)
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
