import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_file(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r") as f:
        return f.read()


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_pkg = "bcr_ur5e_moveit_config"

    xacro_file = os.path.join(
        get_package_share_directory(moveit_pkg),
        "config",
        "ur5e_with_tool.urdf.xacro"
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]),
            value_type=str
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": load_file(
            moveit_pkg,
            "config/bcr_ur5e.srdf"
        )
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            moveit_pkg,
            "config/kinematics.yaml"
        )
    }

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            moveit_pkg,
            "config/joint_limits.yaml"
        )
    }

    ompl_planning_pipeline_config = load_yaml(
        moveit_pkg,
        "config/ompl_planning.yaml"
    )

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            planning_scene_monitor_parameters,
            {
                "use_sim_time": True,
                "allow_trajectory_execution": False,
            },
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz = ExecuteProcess(
        cmd=["rviz2"],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        move_group,
        rviz,
    ])
