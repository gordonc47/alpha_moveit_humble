import os
from sys import prefix
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    # Command-line arguments
    tutorial_arg = DeclareLaunchArgument(
        "rviz_tutorial", default_value="False", description="Tutorial flag"
    )

    kinematics_yaml = load_yaml(
        "moveit_resources_alpha_arm_moveit_config", "config/kinematics.yaml"
    )
    
    robot_description_planning_joint_limits = load_yaml(
        "moveit_resources_alpha_arm_moveit_config", "config/joint_limits.yaml"
    )
    
    robot_description_planning_cartesian_limits = load_yaml(
        "moveit_resources_alpha_arm_moveit_config", "config/pilz_cartesian_limits.yaml"
    )
    
    planning_pipelines_config = load_yaml(
        "moveit_resources_alpha_arm_moveit_config", "config/planning_pipelines.yaml"
    )
    
    ompl_planning_config = load_yaml(
        "moveit_resources_alpha_arm_moveit_config", "config/ompl_planning.yaml"
    )

    # Configure MoveIt2 settings
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    # RViz
    print("RVIZ Launched used reached")
    tutorial_mode = LaunchConfiguration("rviz_tutorial")
    rviz_base = os.path.join(get_package_share_directory("moveit2_tutorials"), "launch")
    rviz_full_config = os.path.join(rviz_base, "view_alpha.rviz")
    rviz_empty_config = os.path.join(rviz_base, "view_alpha.rviz")
    rviz_node_tutorial = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_empty_config],
        parameters=[
            kinematics_yaml,
            planning_pipelines_config,
            trajectory_execution,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            ompl_planning_config,
            planning_scene_monitor_parameters,
        ],
        condition=IfCondition(tutorial_mode),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            kinematics_yaml,
            planning_pipelines_config,
            trajectory_execution,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            ompl_planning_config,
            planning_scene_monitor_parameters,
        ],
        condition=UnlessCondition(tutorial_mode),
    )

    return LaunchDescription(
        [
            tutorial_arg,
            rviz_node,
            rviz_node_tutorial,
        ]
    )
