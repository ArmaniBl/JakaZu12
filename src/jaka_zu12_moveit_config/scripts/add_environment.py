#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class EnvironmentLoader(Node):
    def __init__(self):
        super().__init__("jaka_zu12_environment_loader")
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.timer = self.create_timer(0.5, self.try_apply_scene)
        self.request_sent = False

    @staticmethod
    def make_box(object_id, dimensions, x, y, z=None):
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = object_id

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = dimensions[2] / 2.0 if z is None else z
        pose.orientation.w = 1.0

        collision_object.primitives = [box]
        collision_object.primitive_poses = [pose]
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def try_apply_scene(self):
        if self.request_sent or not self.client.service_is_ready():
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [
            # The floor occupies Z=[-0.10, 0.0], below the support surface.
            self.make_box("floor", [6.0, 6.0, 0.10], 0.0, 0.0, z=-0.05),
            # X points forward and Y points left from the robot base.
            self.make_box(
                "front_track", [0.30, 1.60, 0.15], 0.65, -0.50
            ),
            self.make_box(
                "rear_track", [0.30, 1.60, 0.15], -0.47, -0.50
            ),
            self.make_box("power_supply", [0.40, 0.26, 0.13], 0.10, -1.17),
        ]

        request = ApplyPlanningScene.Request()
        request.scene = scene
        self.request_sent = True
        future = self.client.call_async(request)
        future.add_done_callback(self.scene_applied)

    def scene_applied(self, future):
        try:
            success = future.result().success
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Failed to add environment: {error}")
            rclpy.shutdown()
            return

        if success:
            self.get_logger().info(
                "Added floor, front_track, rear_track and power_supply to the MoveIt "
                "planning scene"
            )
        else:
            self.get_logger().error("MoveIt rejected the environment planning scene")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentLoader()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
