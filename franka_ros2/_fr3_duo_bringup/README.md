# fr3_duo_bringup

FR3 duo + Robotiq 2F-85: robot model, ros2_control, MoveIt and launch files for the `moveit_master` / `articulation_orchestrator` pipeline.

## RViz test on mock hardware
Build and source the workspace, then use three terminals:

```
# 1. robot, controllers, move_group, RViz, moveit_master
ros2 launch fr3_duo_bringup rviz_mock.launch.py

# 2. smoke test: both arms to {arm}_home, each gripper opens, closes, opens
ros2 run articulation_orchestrator articulation_orchestrator

# 3. the packets -> tray task (default scene: 10 packets)
ros2 run articulation_orchestrator run_placements --keep-going
ros2 run articulation_orchestrator run_placements --count 2      # first two only
```

In RViz (MotionPlanning display) you see:
- the table, the tray and the packets (green collision objects) as soon as `run_placements` starts
- the arms moving
- a grasped packet turning purple (attached) and moving with the gripper, then staying in the tray after release

Results: `/tmp/packet_runs/<timestamp>/results.csv`.

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `rviz` | `true` | start RViz |
| `moveit_master` | `true` | start moveit_master (5 s after the rest) |
| `spine` | `0.15` | spine height (m); the default scene is laid out for it |
| `tcp_offset` | `0.13` | TCP above `robotiq_85_base_link` (m) |

## What it starts
- **robot_state_publisher:** the robot from `urdf/fr3_duo_robotiq.urdf.xacro`.
- **ros2_control:** `mock_components/GenericSystem` for every movable joint. The launch adds the hardware block. Arms start at `{arm}_home`, grippers open.
- **Controllers:** `joint_state_broadcaster`, `{left,right}_arm_controller` and `{left,right}_gripper_controller` (joint trajectory controllers).
- **move_group:** with `config/` (kinematics, joint limits incl. Robotiq knuckles, OMPL, controllers) and `srdf/fr3_duo_robotiq.srdf.xacro`.
- **moveit_master:** with `moveit_master/config/moveit_master.yaml` and the joint limits (needed to re-time Cartesian moves).
- **Static TF** `world → base` (the SRDF virtual joint).

## Tested (headless, `rosj` image, Jazzy)
- The smoke test succeeds.
- Three runs of the 10-packet scene put 10/10 packets in the tray (`--keep-going`, default retries). About one packet per run needs its retry: the straight descent from a random pre-grasp IK solution occasionally stops at a joint limit.
