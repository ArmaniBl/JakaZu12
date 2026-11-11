#!/usr/bin/env python3
"""Add the welding workpiece to MoveIt's planning scene once it is ready."""
import sys
import argparse
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import CollisionObject, ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from welding_sim.runner import object_geometry
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import DurabilityPolicy, QoSProfile


def main():
    parser = argparse.ArgumentParser(description='Add a workpiece to MoveIt; does not edit the Isaac USD scene')
    parser.add_argument('--shape', choices=['box', 'cylinder', 'mesh'], default='box')
    parser.add_argument('--dimensions', type=float, nargs='+', default=[.3, .3, .2],
                        help='Box: x y z; cylinder: height radius (meters)')
    parser.add_argument('--mesh-json', help='JSON with vertices [[x,y,z],...] and triangles [[i,j,k],...]')
    parser.add_argument('--center', type=float, nargs=3, default=[-1., 0., .1])
    parser.add_argument('--quaternion', type=float, nargs=4, default=[0., 0., 0., 1.])
    parser.add_argument('--frame', default='world')
    parser.add_argument('--id', default='welding_workpiece')
    parser.add_argument('--clear-markers', action='store_true', help='Clear obsolete welding preview in RViz')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if not all(math.isfinite(v) for v in args.center+args.quaternion):
        parser.error('Pose must be finite')
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = args.center
    norm = math.sqrt(sum(v*v for v in args.quaternion))
    if norm < 1e-12:
        parser.error('Quaternion must be nonzero')
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = [v/norm for v in args.quaternion]
    workpiece = CollisionObject(id=args.id, operation=CollisionObject.ADD)
    workpiece.pose.orientation.w = 1.
    workpiece.header.frame_id = args.frame
    try:
        if args.shape == 'mesh':
            if not args.mesh_json:
                parser.error('--shape mesh requires --mesh-json')
            data = json.loads(Path(args.mesh_json).read_text(encoding='utf-8'))
            from welding_sim.geometry import mesh_geometry
            mesh_geometry(data['vertices'], data['triangles'])
            workpiece.meshes = [Mesh(vertices=[Point(x=float(x), y=float(y), z=float(z)) for x, y, z in data['vertices']],
                                    triangles=[MeshTriangle(vertex_indices=t) for t in data['triangles']])]
            workpiece.mesh_poses = [pose]
        else:
            kind = SolidPrimitive.BOX if args.shape == 'box' else SolidPrimitive.CYLINDER
            workpiece.primitives = [SolidPrimitive(type=kind, dimensions=args.dimensions)]
            workpiece.primitive_poses = [pose]
        object_geometry(workpiece, args.frame)
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.error(str(error))
    rclpy.init()
    node = Node("welding_workpiece_loader")
    try:
        marker_publisher = (node.create_publisher(MarkerArray, '/welding/markers',
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                            if args.clear_markers else None)
        client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not client.wait_for_service(timeout_sec=60.0):
            node.get_logger().error("MoveIt /apply_planning_scene unavailable after 60 seconds")
            return 1

        color = ObjectColor(id=workpiece.id)
        color.color.r = 0.9
        color.color.g = 0.5
        color.color.b = 0.1
        color.color.a = 1.0

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = [workpiece]
        request.scene.object_colors = [color]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
        if not future.done() or not future.result().success:
            node.get_logger().error("MoveIt did not accept the welding workpiece")
            return 1
        node.get_logger().info(
            f"Added {args.id}: {args.shape}, center {args.center} m in {args.frame}"
        )
        if marker_publisher:
            marker_publisher.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
            for _ in range(3):
                rclpy.spin_once(node, timeout_sec=.1)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
