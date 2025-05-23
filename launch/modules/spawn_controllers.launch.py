import os
import sys
CURPATH = os.path.dirname(os.path.abspath(__file__))
sys.path.extend([CURPATH])
sys.path = list(set(sys.path))
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()

    ros_gz_sim_node = Node(
        package="ros_gz_sim", 
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
        ],
        parameters=[]
    )
    launch_description.add_action(ros_gz_sim_node)

    spawn_controllers_node = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "joint_trajectory_controller"
        ],
        parameters=[]
    )
    launch_description.add_action(spawn_controllers_node)    
    
    return launch_description
