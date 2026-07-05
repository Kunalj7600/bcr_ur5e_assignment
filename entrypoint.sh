#!/bin/bash
# Entrypoint for `docker compose up`.
# Builds the workspace (if needed) and launches the full MoveIt2 + MuJoCo
# demo: MuJoCo viewer, ros2_control bridge, move_group, RViz, and the
# cycle controller. No manual steps required beyond `docker compose up`.

set -e

source /opt/ros/humble/setup.bash
cd /ws

echo "[entrypoint] Building workspace..."
colcon build --symlink-install

colcon build --symlink-install --packages-select bcr_ur5e_moveit_config

colcon build --symlink-install --packages-select bcr_ur5e_description

colcon build --symlink-install --packages-select bcr_cycle_controller

echo "[entrypoint] Sourcing workspace overlay..."
source /ws/install/setup.bash

echo "[entrypoint] Launching moveit_mujoco_demo..."
exec ros2 launch bcr_ur5e_moveit_config moveit_mujoco_demo.launch.py
