# PLAN
Tasks completing and collecting data in Isaac Sim with a fr3 bimanual setup with Roboiq 2F-85 grippers

1. set up a scene with fr3 bimanual platform and robotiq 2f
2. to integrate articulation_orchestrator- moveit_master pipeline with bimanual fr3: how to connect Moveit output from moveit_master pkg to isaacsim.ros2.bridge endpoint provided by Franka? DIVIDE ACTIONS BETWEEN ARMS in articulation orchestrator or on package higher than it ❗️
3. move arms and grippers in Isaac sim
4. add objects (any packets that can be found in Isaac assets)
5. store objects positions in yaml in articulation_orchestrator pkg (positions are known because we set them as placements)
6. Use articulation_orchestrator directly from a manually run Python file (integrated in ros with a client node sending goals to MoveitMaster) where all parameters will come together
7. Test this pipeline with 10 placements
8. Record Isaac sim scene: use ready Isaac sim internal tools for it

# Architecture

```
assets/scenes/packets_tray.yaml   table, tray (slots), packets: one file for MoveIt and Isaac
        │
run_placements.py  (manual run, plan steps 6-7)
   ├─ scene.load_scene ─► PlanningSceneUpdater ─► /apply_planning_scene
   ├─ ArmAllocator: which arm (side of the table, /compute_ik check, balance)
   ├─ ArticulationOrchestrator.pick_place_steps(arm, packet, scene, slot) ─► SubMovement[]
   └─ action moveit_master/execute_sequence
        ▼
moveit_master (C++): per-arm MoveGroupInterface ─► move_group ─► controllers ─► Isaac bridge
```

| Module | Job |
|---|---|
| `steps.py` | SubMovement builders (each step names its `arm`), pose math |
| `scene.py` | scene YAML → table / tray / packets; tray slots and stacking; collision objects |
| `arm_allocator.py` | `ArmAllocator` (which arm does a packet) + `IkChecker` (/compute_ik) |
| `articulation_orchestrator.py` | grasp geometry and the pick-and-place sequence; `main` = smoke test (home, open/close both grippers) |
| `run_placements.py` | runs the tasks one packet at a time, recovery after a failure, `results.csv` |
| `planning_scene_updater.py` | adds, moves and removes collision objects |
| `execution_logger.py` | per-goal log: joints, link poses, steps.csv |
| `assets/objects_dictionary.yaml` | per packet type: size, grasp depth, heights, gripper angles |

## Splitting the work between the arms
One packet at a time; the other arm waits at `{arm}_home`. `ArmAllocator`:
1. Packet at world y > +margin → left arm, y < −margin → right arm.
2. Near the middle, or if the side arm fails the IK check, the other arm is tried. Ties go to the arm that has done fewer packets.
3. IK check: the pre-grasp and place-hover TCP poses must have a collision-free IK solution.

The tray slot is the lowest stack nearest the packet, so each arm stays on its own side of the tray.

## One packet (18 steps)
1. **Pick:** open → pre-grasp above the packet (TCP down, fingers closing across its width) → allow packet ↔ gripper and table → straight down → close (squeezes past contact) → settle → attach → lift → forbid table.
2. **Place:** hover over the slot → allow the tray → straight down → release → settle → detach → straight up → forbid.
3. **Finish:** go to `{arm}_home`.

If it fails, moveit_master rolls back the scene. If the packet had already been grasped, it is removed from the planning scene. The arm then opens its gripper and goes home.

## Run
### RViz test (mock hardware, no Isaac)
```
ros2 launch fr3_duo_bringup rviz_mock.launch.py                  # robot, controllers, move_group, RViz, moveit_master
ros2 run articulation_orchestrator articulation_orchestrator     # smoke test: arms home, grippers open/close
ros2 run articulation_orchestrator run_placements --keep-going   # the 10 packets -> tray
```
Details in [fr3_duo_bringup/README.md](../_fr3_duo_bringup/README.md).

### Other commands
```
ros2 run articulation_orchestrator run_placements --dry-run      # print the sequences, move nothing
ros2 run articulation_orchestrator execution_logger              # optional, data collection
```
`run_placements` options: `--scene`, `--count`, `--velocity`, `--arm left|right`, `--retries` (default 2: a packet that failed before the grasp is retried), `--keep-going`, `--no-ik`, `--split-y`, `--margin`, `--out-dir`.

## Robot model
[fr3_duo_bringup](../_fr3_duo_bringup) holds the model used with MoveIt:
- **URDF:** the duo with arms named `fr3v2` (joints `left_fr3v2_joint1` ...), the MoveIt `base` link with planar joints, a Robotiq 2F-85 per arm, and `{arm}_robotiq_85_tcp` 0.13 m above `robotiq_85_base_link` (between the pads, z = approach, y = closing).
- **SRDF:** the duo SRDF from franka_description, plus `{arm}_gripper` groups, `{arm}_home` (FR3 "ready"), end effectors, and gripper collision exclusions.

## Still needed for Isaac
- **Joint names:** the Isaac USD uses the default description's `left_fr3v2_1_joint1`; MoveIt uses `left_fr3v2_joint1`. Rename in one place (USD, or the MoveIt model with `arm_type:=fr3v2_1` plus SRDF and controller names).
- **Bringup:** an Isaac variant of the launch file, with ros2_control on `topic_based_ros2_control` (`/fr3_duo/{left,right}_arm/joint_commands` and `joint_states`), `use_sim_time` and `/clock` from Isaac. Not installed in the `rosj` image yet. The Robotiq macro already supports `sim_isaac`.
- **Gripper on contact:** in Isaac the fingers stall on the packet; the gripper controller must accept that (a GripperCommand controller with `allow_failure`, or goal tolerances on the trajectory controller).
- **Scene:** the spine height (0.15 m), the table, tray and packet poses in the scene YAML, and the packet sizes in objects_dictionary.yaml must match the Isaac stage.
