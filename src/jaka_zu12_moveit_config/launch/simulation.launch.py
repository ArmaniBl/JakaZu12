from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_static_virtual_joint_tfs_launch,
)
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("jaka_zu12", package_name="jaka_zu12_moveit_config")
        .robot_description(mappings={"use_rviz_sim": "true"})
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            generate_static_virtual_joint_tfs_launch(moveit_config),
            generate_rsp_launch(moveit_config),
            generate_move_group_launch(moveit_config),
            generate_moveit_rviz_launch(moveit_config),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    str(
                        moveit_config.package_path
                        / "config"
                        / "ros2_controllers.yaml"
                    ),
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["jaka_zu12_controller"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="jaka_zu12_moveit_config",
                executable="add_environment.py",
                output="screen",
            ),
        ]
    )
