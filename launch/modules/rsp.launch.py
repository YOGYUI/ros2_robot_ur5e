import os
import sys
from typing import List
CURPATH = os.path.dirname(os.path.abspath(__file__))
sys.path.extend([CURPATH])
sys.path = list(set(sys.path))
from launch import LaunchDescription, Action, LaunchContext
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, NotSubstitution
from common_definition import *


def declare_launch_arguments(launch_description: LaunchDescription):
    declare_common_launch_arguments(launch_description)
    launch_description.add_action(DeclareLaunchArgument(
        "robot_state_publish_frequency",
        default_value="50.0",
        description="Robot state publisher node's topic publish rate (Hz)"))


def create_rsp_action(context: LaunchContext, *args, **kwargs) -> List[Action]:
    moveit_config_builder = get_moveit_config_builder()
    moveit_config = moveit_config_builder.to_moveit_configs()

    rsp_launch_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        arguments=[],
        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": LaunchConfiguration("robot_state_publish_frequency"),
                "use_sim_time": NotSubstitution(LaunchConfiguration("real_hw"))
            }
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        namespace=LaunchConfiguration("namespace"),
    )
    return [rsp_launch_node]


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    declare_launch_arguments(launch_description)
    
    launch_description.add_action(OpaqueFunction(
        function=create_rsp_action))

    return launch_description
