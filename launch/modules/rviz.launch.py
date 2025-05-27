import os
import sys
from typing import List
CURPATH = os.path.dirname(os.path.abspath(__file__))
sys.path.extend([CURPATH])
sys.path = list(set(sys.path))
from launch import LaunchDescription, Action, LaunchContext
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, RegisterEventHandler, ExecuteProcess
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from common_definition import *


def declare_launch_arguments(launch_description: LaunchDescription):
    declare_common_launch_arguments(launch_description)
    launch_description.add_action(DeclareLaunchArgument(
        "launch_rviz", 
        default_value="true",
        choices=["true", "false", "True", "False"],
        description="Launch RViz2."))


def create_rviz_launch_action(context: LaunchContext, *args, **kwargs) -> List[Action]:
    pkg_share_dir = get_package_share_directory(PACKAGE_NAME)
    moveit_config_builder = get_moveit_config_builder()
    moveit_config = moveit_config_builder.to_moveit_configs()
    
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.join(os.path.expanduser('~'), 'ros_warehouse_db.sqlite')
    }
    rviz_cfg_path = os.path.join(pkg_share_dir, "rviz", "default.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        respawn=False,
        output="screen",
        arguments=["-d", rviz_cfg_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.sensors_3d,
            warehouse_ros_config,
            {"use_sim_time": NotSubstitution(LaunchConfiguration("real_hw"))}
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        namespace=LaunchConfiguration("namespace"),
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    # change namespace
    script_path = os.path.join(pkg_share_dir, "scripts", "python", "rviz_change_namespace.py")
    process = ExecuteProcess(
        cmd=['python3', script_path, 
             '-c', rviz_cfg_path, 
             '-n', LaunchConfiguration("namespace"),
        ],
        output='screen',
        name='rviz-change-ns'
    )

    ev_hander = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=process,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )
    
    return [process, ev_hander]


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    declare_launch_arguments(launch_description)

    launch_description.add_action(OpaqueFunction(
        function=create_rviz_launch_action))
    
    return launch_description
