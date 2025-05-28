import os
import sys
from typing import List
CURPATH = os.path.dirname(os.path.abspath(__file__))
sys.path.extend([CURPATH])
sys.path = list(set(sys.path))
from launch import LaunchDescription, Action, LaunchContext
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution, AndSubstitution
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.conditions import IfCondition
from common_definition import *


def declare_launch_arguments(launch_description: LaunchDescription):
    declare_common_launch_arguments(launch_description)
    launch_description.add_action(DeclareLaunchArgument(
        "launch_gazebo", 
        default_value="true",
        choices=["true", "false", "True", "False"],
        description="Launch Gazebo."))


def create_gz_sim_action(context: LaunchContext, *args, **kwargs) -> List[Action]:
    pkg_share_dir = get_package_share_directory(PACKAGE_NAME)
    default_world_path = os.path.join(pkg_share_dir, "worlds", "default.world")
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')

    # -r: start running simulation immediately
    # -s: only the Gazebo server to run without the GUI client
    launch_gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s ', default_world_path], 
            'on_exit_shutdown': 'true',
            'namespace': LaunchConfiguration("namespace"),
            'use_namespace': 'true'
        }.items(),
        condition=IfCondition(AndSubstitution(
            NotSubstitution(LaunchConfiguration("real_hw")),
            LaunchConfiguration("launch_gazebo"))
        ),
    )

    # -g: run just the GUI client.
    launch_gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-g '],
        }.items(),
        condition=IfCondition(AndSubstitution(
            NotSubstitution(LaunchConfiguration("real_hw")),
            LaunchConfiguration("launch_gazebo"))
        ),
    )

    # kill ruby process
    script_path = os.path.join(pkg_share_dir, "scripts", "python", "kill_process_ruby.py")
    process = ExecuteProcess(
        cmd=['python3', script_path],
        output='screen',
        name='kill-proc-ruby',
        condition=IfCondition(AndSubstitution(
            NotSubstitution(LaunchConfiguration("real_hw")),
            LaunchConfiguration("launch_gazebo"))
        ),
    )

    ev_hander1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=process,
            on_exit=[launch_gz_server],
        ),
        condition=IfCondition(AndSubstitution(
            NotSubstitution(LaunchConfiguration("real_hw")),
            LaunchConfiguration("launch_gazebo"))
        ),
    )

    ev_hander2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=process,
            on_exit=[launch_gz_client],
        ),
        condition=IfCondition(AndSubstitution(
            NotSubstitution(LaunchConfiguration("real_hw")),
            LaunchConfiguration("launch_gazebo"))
        ),
    )

    return [process, ev_hander1, ev_hander2]


def create_ros_gz_bridge_action(context: LaunchContext, *args, **kwargs) -> List[Action]:
    # bridge gzsim topic <-> ros2 topic
    pkg_share_dir = get_package_share_directory(PACKAGE_NAME)
    param_path = os.path.join(pkg_share_dir, "config", "ros_gz_bridge.yaml")
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={param_path}',
        ],
        respawn=True,
        namespace=LaunchConfiguration("namespace"),
        condition=IfCondition(AndSubstitution(
            NotSubstitution(LaunchConfiguration("real_hw")),
            LaunchConfiguration("launch_gazebo"))
        ),
    )

    return [gz_bridge_node]


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    declare_launch_arguments(launch_description)
    
    launch_description.add_action(OpaqueFunction(
        function=create_gz_sim_action))
    launch_description.add_action(OpaqueFunction(
        function=create_ros_gz_bridge_action))
    
    return launch_description
