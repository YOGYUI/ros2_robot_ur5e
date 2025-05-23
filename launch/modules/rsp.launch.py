import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from common_definition import PACKAGE_NAME


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()

    pkg_share_dir = get_package_share_directory(PACKAGE_NAME)
    urdf_path = os.path.join(pkg_share_dir, "description", "robot.urdf.xacro")
    mappings = {
        "joint_limit_file": os.path.join(pkg_share_dir, "config", "joint_limits.yaml"),
        "initial_positions_file": os.path.join(pkg_share_dir, "config", "initial_positions.yaml"), 
        "ur_calibration_file": os.path.join(pkg_share_dir, "config", "ur_calibration.yaml"),
    }
    robot_description = xacro.process_file(urdf_path, mappings=mappings).toprettyxml()

    rsp_launch_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        arguments=[],
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
