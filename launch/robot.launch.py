import os
import sys
CURPATH = os.path.dirname(os.path.abspath(__file__))
sys.path.extend([CURPATH])
sys.path = list(set(sys.path))
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from modules import *


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    pkg_share_dir = get_package_share_directory(PACKAGE_NAME)

    declare_common_launch_arguments(launch_description)

    # launch robot state publisher
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share_dir, "launch/modules/rsp.launch.py"),
            ),
        )
    )

    # launch gazebo simulator
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share_dir, "launch/modules/gz.launch.py"),
            ),
        )
    )

    # spawn controllers
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share_dir, "launch/modules/spawn_controllers.launch.py"),
            ),
        )
    )

    # launch moveit motion planning move group
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share_dir, "launch/modules/move_group.launch.py"),
            ),
        )
    )

    # launch rviz
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share_dir, "launch/modules/rviz.launch.py"),
            ),
        )
    )

    # launch real-hardware related packages
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share_dir, "launch/modules/hardware.launch.py"),
            ),
        )
    )

    return launch_description
