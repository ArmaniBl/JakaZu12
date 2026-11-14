#!/usr/bin/env python3
"""Install the simulation floor in MoveIt without replacing other world objects."""
import argparse
import math
import sys
import rclpy
from moveit_msgs.msg import PlanningSceneComponents,ObjectColor
from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene
from welding_sim.environment import simulation_floor,allow_base_support


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--top',type=float,default=0.)
    args=parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if not math.isfinite(args.top):parser.error('Floor height must be finite')
    rclpy.init();node=rclpy.create_node('simulation_floor_loader')
    def call(kind,name,request):
        client=node.create_client(kind,name)
        if not client.wait_for_service(timeout_sec=60.):raise RuntimeError(name+' unavailable')
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=15.)
        if not future.done():raise RuntimeError(name+' timeout')
        return future.result()
    try:
        query=GetPlanningScene.Request()
        query.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix=call(GetPlanningScene,'/get_planning_scene',query).scene.allowed_collision_matrix
        request=ApplyPlanningScene.Request()
        request.scene.is_diff=True;request.scene.robot_state.is_diff=True
        request.scene.world.collision_objects=[simulation_floor(args.top)]
        request.scene.allowed_collision_matrix=allow_base_support(matrix)
        color=ObjectColor(id='floor');color.color.r=.45;color.color.g=.48;color.color.b=.52;color.color.a=1.
        request.scene.object_colors=[color]
        if not call(ApplyPlanningScene,'/apply_planning_scene',request).success:
            raise RuntimeError('MoveIt rejected the floor')
        node.get_logger().info(f'Added collision floor: 10 x 10 m, top Z={args.top:g}; excluded from welding selection')
        return 0
    except Exception as error:
        node.get_logger().error(str(error));return 1
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':sys.exit(main())
