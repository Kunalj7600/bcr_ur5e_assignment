from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("bcr_ur5e_description")

    xacro_file = PathJoinSubstitution([
        pkg_share,
        "urdf",
        "ur5e_with_tool.urdf.xacro"
    ])

    rviz_config = PathJoinSubstitution([
        pkg_share,
        "rviz",
        "ur5e_display.rviz"
    ])

    robot_description_content = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str
    )

    robot_description = {
        "robot_description": robot_description_content
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ],
        output="screen"
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )

    rviz = ExecuteProcess(
        cmd=["rviz2", "-d", rviz_config],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
