import os
import re
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

from launch_ros.actions import Node


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


def make_mujoco_robot_description():
    moveit_pkg = "bcr_ur5e_moveit_config"
    desc_pkg = "bcr_ur5e_description"

    xacro_file = os.path.join(
        get_package_share_directory(moveit_pkg),
        "config",
        "ur5e_with_tool.urdf.xacro"
    )

    scene_file = os.path.join(
        get_package_share_directory(desc_pkg),
        "mujoco",
        "scene_table.xml"
    )

    urdf = subprocess.check_output(
        ["xacro", xacro_file],
        text=True
    )

    # Replace the mock hardware plugin with the MuJoCo ros2_control plugin.
    mujoco_hardware = f'''<plugin>mujoco_ros2_control/MujocoSystem</plugin>
        <param name="mujoco_model">{scene_file}</param>'''

    urdf = urdf.replace(
        "<plugin>mock_components/GenericSystem</plugin>",
        mujoco_hardware,
        1
    )

    # Remove unsupported sensors
    # Remove unsupported sensors (tcp_fts_sensor, tcp_pose, and any future ones)
    urdf = re.sub(
    	r'<sensor name="[^"]+">.*?</sensor>',
    	'',
    	urdf,
    	flags=re.DOTALL
    )

# Remove all gpio blocks
    urdf = re.sub(
    	r'<gpio name="[^"]+">.*?</gpio>',
    	'',
    	urdf,
    	flags=re.DOTALL
    )

    if "mujoco_ros2_control/MujocoSystem" not in urdf:
        raise RuntimeError("Failed to inject MuJoCo ros2_control plugin into robot_description")

    


    return urdf


def generate_launch_description():
    moveit_pkg = "bcr_ur5e_moveit_config"
    desc_pkg = "bcr_ur5e_description"

    robot_description = {
        "robot_description": make_mujoco_robot_description()
    }
    
    scene_file = os.path.join(
        get_package_share_directory(desc_pkg),
        "mujoco",
        "scene_table.xml"
    )

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

    ros2_controllers_file = os.path.join(
        get_package_share_directory(moveit_pkg),
        "config",
        "ros2_controllers.yaml"
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ],
    )

    # MuJoCo requires the modified ros2_control_node from mujoco_ros2_control.
    mujoco_control_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="mujoco_ros2_control",
                executable="mujoco_ros2_control",
                output="screen",
                parameters=[
                    robot_description,
                    {"use_sim_time": True},
                    {"mujoco_model_path": scene_file}, 
                    ros2_controllers_file,
                ],
            )
        ]
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
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
        period=6.0,
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

    move_group = TimerAction(
        period=7.0,
        actions=[
            Node(
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
                    {"use_sim_time": True},
                ],
            )
        ]
    )

    cycle_controller = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="bcr_cycle_controller",
                executable="cycle_controller",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    ompl_planning_pipeline_config,
                    {"use_sim_time": True},
                ],
            )
        ]
    )

    rviz = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=["rviz2"],
                output="screen",
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        mujoco_control_node,
        joint_state_broadcaster_spawner,
        trajectory_controller_spawner,
        move_group,
        cycle_controller,
        rviz,
    ])
