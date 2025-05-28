import os
import sys
from typing import List
CURPATH = os.path.dirname(os.path.abspath(__file__))
sys.path.extend([CURPATH])
sys.path = list(set(sys.path))
from launch import LaunchDescription, Action, LaunchContext
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution
from launch.conditions import IfCondition, UnlessCondition
from common_definition import *


def declare_launch_arguments(launch_description: LaunchDescription):
    declare_common_launch_arguments(launch_description)


def create_ros2_control(context: LaunchContext, *args, **kwargs) -> List[Action]:
    pkg_share_dir = get_package_share_directory(PACKAGE_NAME)
    moveit_config_builder = get_moveit_config_builder()
    moveit_config = moveit_config_builder.to_moveit_configs()

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        arguments=[],
        parameters=[
            moveit_config.robot_description,
            os.path.join(pkg_share_dir, "config", "ros2_controllers.yaml"),
        ],
        namespace=LaunchConfiguration("namespace"),
        condition=IfCondition(LaunchConfiguration("real_hw")),
    )
    
    return [ros2_control_node]


def create_gz_sim_create_action(context: LaunchContext, *args, **kwargs) -> List[Action]:
    ros_gz_sim_node = Node(
        package="ros_gz_sim", 
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
        ],
        parameters=[
            {"use_sim_time": NotSubstitution(LaunchConfiguration("real_hw"))},
        ],
        namespace=LaunchConfiguration("namespace"),
        condition=IfCondition(AndSubstitution(
            NotSubstitution(LaunchConfiguration("real_hw")),
            LaunchConfiguration("launch_gazebo"))
        ),
    )
    
    return [ros_gz_sim_node]


def create_controller_spawner_action(context: LaunchContext, *args, **kwargs) -> List[Action]:
    real_hw = context.perform_substitution(LaunchConfiguration("real_hw")).lower()

    active_controllers = [
        "joint_state_broadcaster",
    ]
    
    if real_hw == "true":
        active_controllers.extend([
            "scaled_joint_trajectory_controller",
            "io_and_status_controller",
            "speed_scaling_state_broadcaster",
            "force_torque_sensor_broadcaster",
            "tcp_pose_broadcaster",
            "ur_configuration_controller"
        ])
    else:
        active_controllers.extend([
            "joint_trajectory_controller",
        ])

    spawn_active_controllers_node = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=active_controllers,
        parameters=[
            {"use_sim_time": NotSubstitution(LaunchConfiguration("real_hw"))},
        ],
        namespace=LaunchConfiguration("namespace"),
    )

    inactive_controllers = [
        "forward_velocity_controller",
        "forward_position_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
        "tool_contact_controller",
    ]
    spawn_inactive_controllers_node = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=inactive_controllers + ["--inactive"],
        parameters=[
            {"use_sim_time": NotSubstitution(LaunchConfiguration("real_hw"))},
        ],
        namespace=LaunchConfiguration("namespace"),
    )

    return [spawn_active_controllers_node, spawn_inactive_controllers_node]


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    declare_launch_arguments(launch_description)

    launch_description.add_action(OpaqueFunction(
        function=create_ros2_control))
    launch_description.add_action(OpaqueFunction(
        function=create_gz_sim_create_action))
    launch_description.add_action(OpaqueFunction(
        function=create_controller_spawner_action))
    
    return launch_description
