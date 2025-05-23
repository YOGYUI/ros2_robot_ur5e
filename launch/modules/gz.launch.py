import os
import sys
CURPATH = os.path.dirname(os.path.abspath(__file__))
sys.path.extend([CURPATH])
sys.path = list(set(sys.path))
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from common_definition import PACKAGE_NAME


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()

    pkg_share_dir = get_package_share_directory(PACKAGE_NAME)
    default_world_path = os.path.join(pkg_share_dir, "worlds", "default.world")

    # -r: start running simulation immediately
    # -s: only the Gazebo server to run without the GUI client
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')
    launch_gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s ', default_world_path], 
            'on_exit_shutdown': 'true',
        }.items(),
    )
    launch_description.add_action(launch_gz_server)

    # -g: run just the GUI client.
    launch_gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-g '],
        }.items(),
    )
    launch_description.add_action(launch_gz_client)

    # bridge gzsim topic <-> ros2 topic
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
    )
    launch_description.add_action(gz_bridge_node)
    
    return launch_description
