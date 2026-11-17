#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class EnvironmentLoader(Node):
    def __init__(self):
        super().__init__("jaka_zu12_environment_loader")
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.timer = self.create_timer(0.5, self.try_apply_scene)
        self.request_sent = False

    @staticmethod
    def make_box(object_id, dimensions, x, y, z=None):
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = object_id
        collision_object.pose.orientation.w = 1.0

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
        if (self.request_sent or not self.client.service_is_ready()
                or not self.scene_client.service_is_ready()):
            return
        self.request_sent = True
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        self.scene_client.call_async(request).add_done_callback(self.replace_environment)

    def replace_environment(self, future):
        try:
            objects = future.result().scene.world.collision_objects
        except Exception as error:
            self.get_logger().error(f"Failed to read environment: {error}")
            rclpy.shutdown()
            return
        scene = PlanningScene()
        scene.is_diff = True
        for existing in objects:
            removal = CollisionObject()
            removal.header.frame_id = "world"
            removal.id = existing.id
            removal.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(removal)
        # Collision obstacle only; the welding UI/CLI reserve the ID floor.
        # Rotated 90 degrees right around the base centre.
        # Metres: X=[-0.49, 0.31], Y=[-1.065, 0.135]; top Z=0.
        scene.world.collision_objects.append(
            self.make_box("floor", [0.80, 1.20, 0.10], -0.09, -0.465, z=-0.05)
        )
        # A 15 cm cube, rotated 90 degrees right around the base, resting on Z=0.
        scene.world.collision_objects.append(
            self.make_box("welding_workpiece", [0.15, 0.15, 0.15], 0.0, -0.60)
        )

        # Explicit per-object colors persist in MoveIt's monitored scene.
        # Match add_workpiece.py so replacements keep the workpiece color.
        for object_id, rgb in (("floor", (0.45, 0.48, 0.52)),
                               ("welding_workpiece", (0.9, 0.5, 0.1))):
            color = ObjectColor(id=object_id)
            color.color.r, color.color.g, color.color.b = rgb
            color.color.a = 1.0
            scene.object_colors.append(color)

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
                "Added 1.20 x 0.80 m collision floor (top Z=0) and a 0.15 m welding cube at (0, -0.60, 0.075)"
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
