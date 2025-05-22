import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = "ros2_robot_ur5e"

def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()

    pkg_share_dir = get_package_share_directory(PACKAGE_NAME)
    urdf_path = os.path.join(pkg_share_dir, "description/robot.urdf.xacro")
    robot_description = xacro.process_file(urdf_path).toprettyxml()

    rsp_launch_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "publish_frequency": 50.0,
                "use_sim_time": True
            }
        ],
        remappings=[]
    )
    
    launch_description.add_action(rsp_launch_node)

    return launch_description
