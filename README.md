# BCR UR5e MuJoCo Cyclic Motion Assignment

This is my submission for the UR5e + MuJoCo + ROS2 take-home. Basically it's a
UR5e arm bolted to a table, simulated in MuJoCo, controlled through
ros2_control + MoveIt2.

## Build & run

```bash
docker compose build
docker compose up
```

That's it — no manual steps. `docker compose up` builds the workspace,
launches MuJoCo (viewer window), the `mujoco_ros2_control` bridge,
`robot_state_publisher`, `move_group`, RViz (robot model + TF pre-loaded),
and the cycle controller.

Give it a few seconds after the containers report "You can start planning
now!" before triggering a cycle — MoveIt2 and the controllers need a moment
to come up.

## Home configuration

The arm returns to this configuration at the end of every cycle (and before
the first move), defined as the `home` group state in
`src/bcr_ur5e_moveit_config/config/bcr_ur5e.srdf`:

| Joint | Value (rad) | Value (deg) |

 shoulder_pan_joint: 0
 shoulder_lift_joint: -90 deg
 elbow_joint: -90 deg
 wrist_1_joint: -90 deg
 wrist_2_joint: 90 deg
 wrist_3_joint: 0

This lifts the tool clear of the table before and after every cycle.

## Triggering a cycle

Publish a pose-set JSON (schema: see `test_case_1.json`) as the `data` field
of a `std_msgs/String` message on `/start`. The topic is QoS Reliable +
Transient Local (depth 1), so match that QoS when publishing:

```bash
JSON=$(python3 -c "import json; print(json.dumps(json.load(open('test_case_1.json'))))")
ros2 topic pub /start std_msgs/msg/String "{data: '$JSON'}" --once \
  --qos-reliability reliable --qos-durability transient_local
```

Watch progress:

```bash
ros2 topic echo /cycle_status       # ready | running | completed
ros2 topic echo /motion_state       # idle | approach | linear | retract
ros2 topic echo /attempted_count
ros2 topic echo /completed_count
ros2 topic echo /tcp_pose           # ee_link pose in base_link
```

A cycle is idempotent — publishing `/start` again re-runs it cleanly from
home.

## Reporting topics

| Topic | Type | QoS |

 /joint_states - straight from the mujoco bridge
 /tcp_pose - pose of ee_link (tip of the tool, not tool0) in base_link, published continuously at like 20Hz
 /cycle_status - ready / running / completed
 /motion_state - idle / approach / linear / retract, also continuous
 /attempted_count / /completed_count - reset to 0 every new cycle

## Networking

`ROS_DOMAIN_ID=42` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` are set in
`docker-compose.yml`, and the container runs with `network_mode: host`, so
topics are reachable from the host without extra configuration.

## Repo layout

- `src/bcr_ur5e_description` — URDF/xacro (UR5e + table + tool + `ee_link`),
  MuJoCo scene (`mujoco/scene_table.xml`) and assets.
- `src/bcr_ur5e_moveit_config` — MoveIt2 config (SRDF, kinematics,
  controllers), the bring-up launch file (`moveit_mujoco_demo.launch.py`),
  and the RViz config.
- `src/bcr_cycle_controller` — the `/start` → MoveIt2 trigger/state-machine
  node that runs the approach/linear/retract cycle and publishes the
  reporting topics.

## Notes To Self :)

- Planning and /tcp_pose are both based on ee_link (the tip of the tool),
  not tool0, since that's what the assignment wants as the actual TCP.
- The table is just baked into the URDF as a fixed link with its own
 collision box, so MoveIt already knows about it for collision checking - I didn't need to add it separately as a planning scene object.
- If a move fails for any reason (bad IK, no plan, would hit something) it just gets skipped and the arm moves on to the next pose's approach instead of getting stuck.
