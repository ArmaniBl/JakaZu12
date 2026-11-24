from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_static_virtual_joint_tfs_launch,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ip = LaunchConfiguration("ip")
    auto_enable = LaunchConfiguration("auto_enable")
    servo_lpf_cutoff_hz = LaunchConfiguration("servo_lpf_cutoff_hz")
    payload_mass_kg = LaunchConfiguration("payload_mass_kg")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    trajectory_action = LaunchConfiguration("trajectory_action")

    moveit_config = MoveItConfigsBuilder(
        "jaka_zu12", package_name="jaka_zu12_moveit_config"
    ).to_moveit_configs()

    return LaunchDescription(
        [
            DeclareLaunchArgument("scan_preview", default_value="false", choices=["true", "false"],
                                  description="Show scanner cloud and automatically fit preliminary CAD while idle"),
            DeclareLaunchArgument("scan_preview_script", default_value="",
                                  description="Path to workspace run_scan_preview.sh"),
            ExecuteProcess(
                cmd=["bash", LaunchConfiguration("scan_preview_script")],
                condition=IfCondition(LaunchConfiguration("scan_preview")),
                output="screen",
            ),
            DeclareLaunchArgument(
                "ip",
                default_value="10.5.5.100",
                description="IPv4 address of the physical JAKA controller",
            ),
            DeclareLaunchArgument(
                "auto_enable",
                default_value="false",
                choices=["true", "false"],
                description="Allow automatic power-on and enable",
            ),
            DeclareLaunchArgument(
                "servo_lpf_cutoff_hz",
                default_value="0.5",
                description="JAKA joint-servo LPF cutoff in Hz",
            ),
            DeclareLaunchArgument(
                "configure_payload",
                default_value="true",
                choices=["true", "false"],
                description="Write the supplied payload; false preserves the controller payload",
            ),
            DeclareLaunchArgument(
                "payload_mass_kg",
                default_value="3.0",
                description="External tool payload mass in kg",
            ),
            *[
                DeclareLaunchArgument(
                    f"payload_centroid_{axis}_mm",
                    default_value="200.0" if axis == "z" else "0.0",
                    description=f"Tool centre of mass, flange {axis.upper()} coordinate in mm",
                )
                for axis in "xyz"
            ],
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/joint_states",
                description="Joint-state topic used by JAKA and MoveIt",
            ),
            DeclareLaunchArgument(
                "trajectory_action",
                default_value="/jaka_zu12_controller/follow_joint_trajectory",
                description="FollowJointTrajectory action used by JAKA and MoveIt",
            ),
            LogInfo(
                msg=[
                    "Starting physical JAKA ZU12 at ",
                    ip,
                    ". Automatic enable: ",
                    auto_enable,
                ]
            ),
            GroupAction(
                [
                    Node(package="jaka_zu12_moveit_config", executable="table_grid.py", output="screen"),
                    SetRemap(src="/joint_states", dst=joint_states_topic),
                    SetRemap(
                        src="/jaka_zu12_controller/follow_joint_trajectory",
                        dst=trajectory_action,
                    ),
                    Node(
                        package="jaka_planner",
                        executable="moveit_server",
                        name="moveit_server",
                        output="screen",
                        parameters=[
                            {
                                "ip": ip,
                                "model": "zu12",
                                "auto_enable": ParameterValue(
                                    auto_enable, value_type=bool
                                ),
                                "servo_lpf_cutoff_hz": ParameterValue(
                                    servo_lpf_cutoff_hz, value_type=float
                                ),
                                "configure_payload": ParameterValue(LaunchConfiguration("configure_payload"), value_type=bool),
                                "payload_mass_kg": ParameterValue(
                                    payload_mass_kg, value_type=float
                                ),
                                **{
                                    f"payload_centroid_{axis}_mm": ParameterValue(
                                        LaunchConfiguration(f"payload_centroid_{axis}_mm"), value_type=float
                                    )
                                    for axis in "xyz"
                                },
                            }
                        ],
                    ),
                    generate_static_virtual_joint_tfs_launch(moveit_config),
                    generate_rsp_launch(moveit_config),
                    generate_move_group_launch(moveit_config),
                    generate_moveit_rviz_launch(moveit_config),
                    Node(
                        package="jaka_zu12_moveit_config",
                        executable="add_environment.py",
                        output="screen",
                    ),
                ]
            ),
        ]
    )
