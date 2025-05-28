import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = "ros2_robot_ur5e"
builder_: MoveItConfigsBuilder = None


def declare_common_launch_arguments(launch_description: LaunchDescription):
    launch_description.add_action(DeclareLaunchArgument(
        "namespace", 
        default_value="",
        description="Configure robot namespace."))
    launch_description.add_action(DeclareLaunchArgument(
        "tf_prefix", 
        default_value="",
        description="Configure tf prefix."))
    launch_description.add_action(DeclareLaunchArgument(
        "real_hw", 
        default_value="false",
        choices=["true", "false", "True", "False"],
        description="If false, simulation environment will be set."))


def get_moveit_config_builder(force_init: bool = True) -> MoveItConfigsBuilder:
    global builder_

    if builder_ is None or force_init:
        builder_ = MoveItConfigsBuilder(robot_name='ur5e', package_name=PACKAGE_NAME)
        pkg_share_dir = get_package_share_directory(PACKAGE_NAME)

        builder_.robot_description(
            file_path=os.path.join(pkg_share_dir, "description", "robot.urdf.xacro"),
            mappings={
                "namespace": LaunchConfiguration("namespace"),
                "tf_prefix": LaunchConfiguration("tf_prefix"),
                "real_hw": LaunchConfiguration("real_hw"),
                "joint_limit_file": os.path.join(pkg_share_dir, "config", "joint_limits.yaml"),
                "initial_positions_file": os.path.join(pkg_share_dir, "config", "initial_positions.yaml"), 
                "ur_calibration_file": os.path.join(pkg_share_dir, "config", "ur_calibration.yaml"),
                "ur_control_file": os.path.join(pkg_share_dir, "config", "ur_control.yaml"),
            }
        )

        builder_.robot_description_semantic(
            file_path=os.path.join(pkg_share_dir, "description", "robot.srdf.xacro"),
            mappings={
                "tf_prefix": LaunchConfiguration("tf_prefix"),
            }
        )

        builder_.robot_description_kinematics(
            file_path=os.path.join(pkg_share_dir, "config/kinematics.yaml")
        )

        builder_.joint_limits(
            file_path=os.path.join(pkg_share_dir, "config/joint_limits.yaml")
        )

        builder_.planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )

    return builder_
