#!/usr/bin/env python3
"""Add the welding workpiece to MoveIt's planning scene once it is ready."""
import sys
import argparse
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import CollisionObject, ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from welding_sim.geometry import rotation
from welding_sim.runner import object_geometry
from welding_sim.table_limits import DEFAULT_WORKPIECE_CELL, default_workpiece_base
from welding_sim.workpiece_model import load_catalog_model, preview_object, WORKPIECE_CATALOG, CATALOG_PREFIX
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import DurabilityPolicy, QoSProfile


def main():
    parser = argparse.ArgumentParser(description='Add a workpiece to MoveIt; does not edit the Isaac USD scene')
    parser.add_argument('--shape', choices=['box', 'cylinder', 'mesh'], default='box')
    parser.add_argument('--dimensions', type=float, nargs='+', default=[.3, .3, .2],
                        help='Box: x y z; cylinder: height radius (meters)')
    parser.add_argument('--mesh-json', help='JSON with vertices [[x,y,z],...] and triangles [[i,j,k],...]')
    parser.add_argument('--catalog-model', choices=[entry[0] for entry in WORKPIECE_CATALOG])
    parser.add_argument('--require-jaka-idle', action='store_true')
    parser.add_argument('--center', type=float, nargs=3,
                        help=f'Explicit position in meters (default: on {DEFAULT_WORKPIECE_CELL})')
    parser.add_argument('--table-top', type=float, default=0.,
                        help='Table surface height in meters for default placement')
    parser.add_argument('--quaternion', type=float, nargs=4, default=[0., 0., 0., 1.])
    parser.add_argument('--frame', default='world')
    parser.add_argument('--id', default='welding_workpiece')
    parser.add_argument('--clear-markers', action='store_true', help='Clear obsolete welding preview in RViz')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if args.id == "floor" or args.id.startswith('workspace_'):
        parser.error("Object ID is reserved for a collision obstacle")
    if not all(math.isfinite(v) for v in [*(args.center or ()), *args.quaternion, args.table_top]):
        parser.error('Pose must be finite')
    pose = Pose()
    norm = math.sqrt(sum(v*v for v in args.quaternion))
    if norm < 1e-12:
        parser.error('Quaternion must be nonzero')
    orientation = [v / norm for v in args.quaternion]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
    workpiece = CollisionObject(id=args.id, operation=CollisionObject.ADD)
    workpiece.pose.orientation.w = 1.
    workpiece.header.frame_id = args.frame
    try:
        position = list(args.center or default_workpiece_base(args.table_top))
        pose.position.x, pose.position.y, pose.position.z = position
        if args.catalog_model:
            model = load_catalog_model(args.catalog_model)
            # CAD coordinates are in mm, with the base of the part at local Z=0.
            if args.center is None:
                position[2] -= float(np.min(np.asarray(model['vertices_mm']) @ rotation(orientation)[2])) / 1000
            model['initial_pose'] = dict(position_m=position, quaternion_xyzw=orientation)
            workpiece = preview_object(model)
            workpiece.id = args.id
            workpiece.header.frame_id = args.frame
            workpiece.type.key = CATALOG_PREFIX + args.catalog_model
        elif args.shape == 'mesh':
            if not args.mesh_json:
                parser.error('--shape mesh requires --mesh-json')
            data = json.loads(Path(args.mesh_json).read_text(encoding='utf-8'))
            from welding_sim.geometry import mesh_geometry
            mesh_geometry(data['vertices'], data['triangles'])
            if args.center is None:
                pose.position.z -= float(np.min(np.asarray(data['vertices']) @ rotation(orientation)[2]))
            workpiece.meshes = [Mesh(vertices=[Point(x=float(x), y=float(y), z=float(z)) for x, y, z in data['vertices']],
                                    triangles=[MeshTriangle(vertex_indices=t) for t in data['triangles']])]
            workpiece.mesh_poses = [pose]
        else:
            kind = SolidPrimitive.BOX if args.shape == 'box' else SolidPrimitive.CYLINDER
            if args.center is None:
                if kind == SolidPrimitive.BOX and len(args.dimensions) == 3:
                    pose.position.z += float(np.abs(rotation(orientation)[2]) @ args.dimensions) / 2
                elif kind == SolidPrimitive.CYLINDER and len(args.dimensions) == 2:
                    height, radius = args.dimensions
                    axis_z = abs(rotation(orientation)[2, 2])
                    pose.position.z += axis_z * height / 2 + math.sqrt(max(0., 1 - axis_z**2)) * radius
            workpiece.primitives = [SolidPrimitive(type=kind, dimensions=args.dimensions)]
            workpiece.primitive_poses = [pose]
        object_geometry(workpiece, args.frame)
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.error(str(error))
    rclpy.init()
    if args.require_jaka_idle:
        from rangevision import Bridge
        node = Bridge('welding_workpiece_loader')
    else:
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
        if args.require_jaka_idle:
            try:
                node.require_idle()
            except ValueError as error:
                node.get_logger().error(str(error))
                return 1
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
        if not future.done() or not future.result().success:
            node.get_logger().error("MoveIt did not accept the welding workpiece")
            return 1
        location = workpiece.pose.position if args.catalog_model else pose.position
        node.get_logger().info(
            f"Added {args.id}: {args.catalog_model or args.shape}, "
            f"position ({location.x:g}, {location.y:g}, {location.z:g}) m in {args.frame}"
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
