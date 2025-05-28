import os
import sys
import yaml
from typing import List
CURPATH = os.path.dirname(os.path.abspath(__file__))
sys.path.extend([CURPATH])
sys.path = list(set(sys.path))
from launch import LaunchDescription, Action, LaunchContext
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, NotSubstitution, AndSubstitution
from launch.conditions import IfCondition
from common_definition import *


def declare_launch_arguments(launch_description: LaunchDescription):
    declare_common_launch_arguments(launch_description)
    launch_description.add_action(DeclareLaunchArgument(
        "launch_ur_dashboard_client", 
        default_value="false",
        choices=["true", "false",],
        description="Launch UR Dashboard Client"))


def create_ur_dashboard_client(context: LaunchContext, *args, **kwargs) -> List[Action]:
    pkg_share_dir = get_package_share_directory(PACKAGE_NAME)
    robot_cfg_file = os.path.join(pkg_share_dir, "config", "ur_control.yaml")
    with open(robot_cfg_file, 'r') as fp:
        cfg = yaml.load(fp, Loader=yaml.FullLoader)

    ur_dashboard_client_node = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        output="screen",
        name="ur_dashboard_client",
        arguments=[],
        parameters=[
            {"robot_ip": cfg["robot_ip"]}
        ],
        namespace=LaunchConfiguration("namespace"),
        condition=IfCondition(AndSubstitution(
            LaunchConfiguration("real_hw"),
            LaunchConfiguration("launch_ur_dashboard_client"))
        ),
        emulate_tty=True,
    )

    return [ur_dashboard_client_node]


def create_robot_state_helper(context: LaunchContext, *args, **kwargs) -> List[Action]:
    pkg_share_dir = get_package_share_directory(PACKAGE_NAME)
    robot_cfg_file = os.path.join(pkg_share_dir, "config", "ur_control.yaml")
    with open(robot_cfg_file, 'r') as fp:
        cfg = yaml.load(fp, Loader=yaml.FullLoader)
    
    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        output="screen",
        name="ur_robot_state_helper",
        arguments=[],
        parameters=[
            {"robot_ip": cfg["robot_ip"]},
            {"headless_mode": cfg["headless_mode"]}
        ],
        namespace=LaunchConfiguration("namespace"),
        condition=IfCondition(LaunchConfiguration("real_hw"))
    )

    return [robot_state_helper_node]


def create_ur_tool_communication(context: LaunchContext, *args, **kwargs) -> List[Action]:
    pkg_share_dir = get_package_share_directory(PACKAGE_NAME)
    robot_cfg_file = os.path.join(pkg_share_dir, "config", "ur_control.yaml")
    with open(robot_cfg_file, 'r') as fp:
        cfg = yaml.load(fp, Loader=yaml.FullLoader)

    action = list()
    if cfg['use_tool_communication']:
        tool_communication_node = Node(
            package="ur_robot_driver",
            executable="tool_communication.py",
            output="screen",
            name="ur_tool_comm",
            arguments=[],
            parameters=[
                {
                    "robot_ip": cfg["robot_ip"],
                    "tcp_port": cfg["tool_tcp_port"],
                    "device_name": cfg["tool_device_name"],
                }
            ],
            namespace=LaunchConfiguration("namespace"),
            condition=IfCondition(LaunchConfiguration("real_hw"))
        )
        action.append(tool_communication_node)

    return action


def create_ur_script_interface(context: LaunchContext, *args, **kwargs) -> List[Action]:
    pkg_share_dir = get_package_share_directory(PACKAGE_NAME)
    robot_cfg_file = os.path.join(pkg_share_dir, "config", "ur_control.yaml")
    with open(robot_cfg_file, 'r') as fp:
        cfg = yaml.load(fp, Loader=yaml.FullLoader)

    urscript_interface_node = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        output="screen",
        name="ur_script_interface",
        arguments=[],
        parameters=[
            {"robot_ip": cfg["robot_ip"]}
        ],
        namespace=LaunchConfiguration("namespace"),
        condition=IfCondition(LaunchConfiguration("real_hw"))
    )

    return [urscript_interface_node]


def create_ur_controller_stopper(context: LaunchContext, *args, **kwargs) -> List[Action]:
    pkg_share_dir = get_package_share_directory(PACKAGE_NAME)
    robot_cfg_file = os.path.join(pkg_share_dir, "config", "ur_control.yaml")
    with open(robot_cfg_file, 'r') as fp:
        cfg = yaml.load(fp, Loader=yaml.FullLoader)
    
    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        output="screen",
        name="ur_controller_stopper",
        arguments=[],
        parameters=[
            {
                "headless_mode": cfg["headless_mode"],
                "joint_controller_active": cfg["joint_controller_active"],
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                    "tcp_pose_broadcaster",
                    "ur_configuration_controller",
                ]
            },
        ],
        namespace=LaunchConfiguration("namespace"),
        condition=IfCondition(LaunchConfiguration("real_hw")),
        emulate_tty=True,
    )

    return [controller_stopper_node]


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    declare_launch_arguments(launch_description)

    launch_description.add_action(OpaqueFunction(
        function=create_ur_dashboard_client))
    launch_description.add_action(OpaqueFunction(
        function=create_robot_state_helper))
    launch_description.add_action(OpaqueFunction(
        function=create_ur_tool_communication))
    launch_description.add_action(OpaqueFunction(
        function=create_ur_script_interface))
    launch_description.add_action(OpaqueFunction(
        function=create_ur_controller_stopper))

    return launch_description
