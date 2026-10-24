# moveit_master

The only node that commands the robot through `MoveGroupInterface`. It runs the `SubMovement` lists that `articulation_orchestrator` sends, on whichever FR3 duo arm each step names. It makes no task decisions.

## Interface
- Action `/moveit_master/execute_sequence` ([ExecuteSequence.action](action/ExecuteSequence.action)): `steps`, `velocity_scaling` → `success`, `failed_step`, `message`; feedback `current_step`, `current_label`.
- [SubMovement.msg](msg/SubMovement.msg): `type` plus `arm` (`left` / `right`, `""` = `default_arm`) and `label` (shown in feedback and errors).

## Parameters ([config/moveit_master.yaml](config/moveit_master.yaml))
| Parameter | Default | Meaning |
|---|---|---|
| `arms` | `[left, right]` | arms to drive |
| `default_arm` | first arm | used when a step has `arm: ""` |
| `arm_group_format` | `{arm}_arm` | SRDF arm group |
| `gripper_group_format` | `{arm}_gripper` | SRDF group with the Robotiq knuckle joint (`""` = none) |
| `tcp_link_format` | `{arm}_robotiq_85_tcp` | TCP between the fingertip pads; `""` = SRDF end effector |
| `gripper_root_link_format` | `{arm}_robotiq_85_base_link` | this link and every link below it may touch a grasped object |

You can override one arm with `<arm>.arm_group`, `<arm>.tcp_link` and so on.

## Run
```
ros2 launch moveit_master moveit_sequence.launch.py [urdf:=... srdf:=... xacro_args:="ee_id:=..."]
```
move_group must already be running with the same URDF/SRDF and with controllers for both arms and both grippers.

## What a goal does
- **Startup:** for each arm, it opens `MoveGroupInterface` for the arm group and the gripper group, checks the TCP link and the gripper root link, and collects the gripper links.
- **Goal:** one goal at a time; others are rejected while busy. It sets the velocity and acceleration scaling (default 0.2) on every arm, then runs the steps in order and publishes feedback before each one.
- **Steps** (each on its arm's groups):
  - `ARM_NAMED`: an SRDF state, e.g. `left_home`.
  - `ARM_POSE`: a TCP pose target, planned and executed.
  - `ARM_CARTESIAN`: a straight TCP line by an offset in the world frame or the TCP frame. It must be 100% feasible and is re-timed to the scaling.
  - `GRIPPER_JOINT`: a Robotiq knuckle angle, from 0 (open) to 0.8 rad (closed).
  - `GRIPPER_NAMED`: an SRDF gripper state.
  - `ATTACH` / `DETACH`: attaches an object to the arm's TCP link or puts it back in the world.
  - `ALLOW_COLLISION`: changes the allowed-collision entries between an object and the arm's gripper links plus `touch_links`.
  - `WAIT`: sleeps.
- **Failure or cancel:** cancel stops every arm and gripper. In both cases the scene changes the sequence made are rolled back: attached objects are detached and allowed collisions removed.

One arm moves at a time. The arm standing still is part of the planning scene, so every plan avoids it.
