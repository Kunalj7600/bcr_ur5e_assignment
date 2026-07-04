from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    scene_path = PathJoinSubstitution([
        FindPackageShare("bcr_ur5e_description"),
        "mujoco",
        "scene_table.xml"
    ])

    mujoco_viewer = ExecuteProcess(
        cmd=[
            "python3",
            "-m",
            "mujoco.viewer",
            "--mjcf",
            scene_path
        ],
        output="screen"
    )

    return LaunchDescription([
        mujoco_viewer
    ])
