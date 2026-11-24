#!/usr/bin/env python3

import rclpy
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from welding_sim.workpiece_model import preview_object
from welding_sim.table_limits import JAKA_TABLE_TOP_M, wall_specs
from welding_sim.environment import allow_base_support


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
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES
                                         | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        self.scene_client.call_async(request).add_done_callback(self.replace_environment)

    def replace_environment(self, future):
        try:
            current = future.result().scene
            objects = current.world.collision_objects
        except Exception as error:
            self.get_logger().error(f"Failed to read environment: {error}")
            rclpy.shutdown()
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = allow_base_support(current.allowed_collision_matrix, base_link='Link_0')
        for existing in objects:
            removal = CollisionObject()
            removal.header.frame_id = "world"
            removal.id = existing.id
            removal.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(removal)
        # Collision obstacle only; the welding UI/CLI reserve the ID floor.
        # Rotated 90 degrees right around the base centre.
        # One-row operator correction: O8 now occupies the previous O9 position.
        # Metres: X=[-0.44, 0.36], Y=[-1.065, 0.135]; top Z=JAKA_TABLE_TOP_M.
        scene.world.collision_objects.append(
            self.make_box("floor", [0.80, 1.20, 0.10], -0.04, -0.465, z=JAKA_TABLE_TOP_M - .05)
        )
        model_path=Path(get_package_share_directory('jaka_zu12_moveit_config'))/'config/workpiece_model.json'
        for name,dimensions,center in wall_specs():
            scene.world.collision_objects.append(self.make_box(name,dimensions,*center[:2],z=center[2]))
            color=ObjectColor(id=name);color.color.a=0.
            scene.object_colors.append(color)
        model=json.loads(model_path.read_text())
        workpiece = preview_object(model)
        workpiece.pose.position.z += JAKA_TABLE_TOP_M
        scene.world.collision_objects.append(workpiece)

        # Keep supporting surface and workpiece visually distinct.
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
                "Added collision table and startup CAD; placement is preliminary until scanner-to-robot validation"
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
