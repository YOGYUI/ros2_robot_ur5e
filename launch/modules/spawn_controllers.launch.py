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
    spawn_controllers_node = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "joint_trajectory_controller"
        ],
        parameters=[
            {"use_sim_time": NotSubstitution(LaunchConfiguration("real_hw"))},
        ],
        namespace=LaunchConfiguration("namespace"),
    )

    return [spawn_controllers_node]


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    declare_launch_arguments(launch_description)

    launch_description.add_action(OpaqueFunction(
        function=create_gz_sim_create_action))
    launch_description.add_action(OpaqueFunction(
        function=create_controller_spawner_action))
    
    return launch_description
