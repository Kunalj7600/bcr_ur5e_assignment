import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
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

    ros2_controllers_file = os.path.join(
        get_package_share_directory(moveit_pkg),
        "config",
        "ros2_controllers.yaml"
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

    moveit_controllers = load_yaml(
        moveit_pkg,
        "config/moveit_controllers.yaml"
    )

    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.5,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_file,
            {"use_sim_time": False}
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "controller_manager", "spawner",
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager"
                ],
                output="screen"
            )
        ]
    )

    trajectory_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "controller_manager", "spawner",
                    "ur_manipulator_controller",
                    "--controller-manager", "/controller_manager"
                ],
                output="screen"
            )
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": False}
        ],
    )

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
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {"use_sim_time": False},
        ],
    )

    rviz = ExecuteProcess(
        cmd=["rviz2"],
        output="screen",
    )

    return LaunchDescription([
        ros2_control_node,
        joint_state_broadcaster_spawner,
        trajectory_controller_spawner,
        robot_state_publisher,
        move_group,
        rviz,
    ])
