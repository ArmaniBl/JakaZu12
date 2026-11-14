from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = (
        MoveItConfigsBuilder("ur10", package_name="ur10_isaac_moveit_config")
        .robot_description(file_path="config/ur10.urdf.xacro", mappings={"name": "ur10", "ur_type": "ur10", "torch": LaunchConfiguration('torch')})
        .robot_description_semantic(file_path="config/ur10.srdf.xacro", mappings={'torch': LaunchConfiguration('torch')})
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    # Support scenes without /clock. Restamp only received states for MoveIt/TF.
    states = [("/joint_states", "/ur10/joint_states")]
    return LaunchDescription([
        DeclareLaunchArgument('floor_z',default_value='0.0',description='Collision floor top in world, meters'),
        Node(package='ur10_isaac_moveit_config',executable='add_floor.py',output='screen',
             arguments=['--top',LaunchConfiguration('floor_z')]),
        DeclareLaunchArgument('torch', default_value='true', choices=['true', 'false'],
                              description='Attach welding torch and use welding_tcp as the chain tip'),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("workpiece", default_value="false", choices=["true", "false"],
                              description="Add the 30 x 30 x 20 cm welding workpiece to MoveIt"),
        Node(package="ur10_isaac_moveit_config", executable="add_workpiece.py", output="screen",
             condition=IfCondition(LaunchConfiguration("workpiece"))),
        Node(package="ur10_isaac_moveit_config", executable="joint_state_clock.py", output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[config.robot_description, {"use_sim_time": False}], remappings=states),
        Node(package="controller_manager", executable="ros2_control_node", output="screen",
             parameters=[str(config.package_path / "config/ros2_controllers.yaml"), {"use_sim_time": False}]),
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=["ur10_controller", "--controller-manager-timeout", "60"]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[config.to_dict(), {"use_sim_time": False}], remappings=states),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", str(config.package_path / "config/moveit.rviz")],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.planning_pipelines, {"use_sim_time": False}],
             remappings=states,
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
