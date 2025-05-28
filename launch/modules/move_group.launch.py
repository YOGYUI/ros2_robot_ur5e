import os
import sys
from typing import List
CURPATH = os.path.dirname(os.path.abspath(__file__))
sys.path.extend([CURPATH])
sys.path = list(set(sys.path))
from launch import LaunchDescription, Action, LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue 
from moveit_configs_utils.substitutions.xacro import Xacro
from srdfdom.srdf import SRDF
from launch.substitutions import LaunchConfiguration, NotSubstitution
from common_definition import *


def declare_launch_arguments(launch_description: LaunchDescription):
    declare_common_launch_arguments(launch_description)


def create_static_transform_publisher_action(context: LaunchContext, *args, **kwargs) -> List[Action]:
    moveit_config_builder = get_moveit_config_builder()
    moveit_config = moveit_config_builder.to_moveit_configs()

    nodes: List[Action] = list()

    for _, item in moveit_config.robot_description_semantic.items():
        if isinstance(item, ParameterValue):
            xacro: Xacro = item.value[0]
            xml_contents = xacro.perform(context)
            srdf = SRDF.from_xml_string(xml_contents)
        else:
            # maybe string instance
            srdf = SRDF.from_xml_string(item)

        for i, vj in enumerate(srdf.virtual_joints):
            nodes.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_transform_publisher_{i}",
                    output="screen",
                    arguments=[
                        "--frame-id", vj.parent_frame,
                        "--child-frame-id", vj.child_link,
                    ],
                    parameters=[
                        {"use_sim_time": NotSubstitution(LaunchConfiguration("real_hw"))}
                    ],
                    remappings=[
                        ("/tf", "tf"),
                        ("/tf_static", "tf_static"),
                    ],
                    namespace=LaunchConfiguration("namespace"),
                )
            )

    return nodes


def create_move_group_action(context: LaunchContext, *args, **kwargs) -> List[Action]:
    moveit_config_builder = get_moveit_config_builder()
    moveit_config = moveit_config_builder.to_moveit_configs()

    nodes: List[Action] = list()

    use_sim_time = NotSubstitution(LaunchConfiguration("real_hw"))
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": "",
        "disable_capabilities": "",
        "monitor_dynamics": True,
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.join(os.path.expanduser('~'), 'ros_warehouse_db.sqlite')
    }

    moveit_config_dict = moveit_config.to_dict()
    robot_description = {"robot_description": moveit_config_dict["robot_description"]}
    robot_description_semantic = {"robot_description_semantic": moveit_config_dict["robot_description_semantic"]}
    robot_description_kinematics = {"robot_description_kinematics": moveit_config_dict["robot_description_kinematics"]}
    robot_description_planning = {"robot_description_planning": moveit_config_dict["robot_description_planning"]}

    moveit_simple_controller_manager = moveit_config_dict["moveit_simple_controller_manager"]
    real_hw = context.perform_substitution(LaunchConfiguration("real_hw")).lower()
    if real_hw != "true":
        moveit_simple_controller_manager["scaled_joint_trajectory_controller"]["default"] = False
        moveit_simple_controller_manager["joint_trajectory_controller"]["default"] = True
        trajectory_execution["trajectory_execution.execution_duration_monitoring"] = True
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controller_manager,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    ompl_config = moveit_config_dict["ompl"]
    ompl_config["start_state_max_bounds_error"] = 0.1
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"], 
	    "default_planning_pipeline": "ompl", 
        "ompl": ompl_config
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        respawn=True,
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            move_group_configuration,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            warehouse_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        namespace=LaunchConfiguration("namespace"),
    )
    nodes.append(move_group_node)

    return nodes


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    declare_launch_arguments(launch_description)

    launch_description.add_action(OpaqueFunction(
        function=create_static_transform_publisher_action))
    launch_description.add_action(OpaqueFunction(
        function=create_move_group_action))
    
    return launch_description
