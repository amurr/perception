#!/bin/bash
# RViz pipeline test: FR3 duo + Robotiq on mock hardware, driven by moveit_master / articulation_orchestrator.
#
# Run ON THE HOST. tmux runs here; every pane runs its ROS 2 process inside the franka container
# (image franka_ros2-franka_ros2_jazzy, started with franka_ros2/docker-compose.yml) via docker exec.
#
#   xhost +local:docker                          # once per login, so RViz can open a window
#   bash _tmux/rviz_pipeline_test.sh             # build + start
#   bash _tmux/rviz_pipeline_test.sh --no-build  # start only
#   bash _tmux/rviz_pipeline_test.sh --clean     # delete build/ + install/ of the 5 packages, rebuild, start
#   bash _tmux/rviz_pipeline_test.sh --stop      # stop everything (tmux session + processes in the container)
#   CONTAINER=my_container bash _tmux/rviz_pipeline_test.sh
#
# Panes:
#   0  launch      robot, controllers, move_group, RViz, moveit_master
#   1  smoke test  both arms home, grippers open/close (runs by itself once moveit_master is up)
#   2  placements  run_placements typed in, press Enter to start
#   3  logger      execution_logger (data per packet in /tmp/packet_exec_logs inside the container)
#
# tmux: Ctrl-b + arrow keys = switch pane, Ctrl-b d = detach.
# Closing tmux does NOT stop processes started with docker exec: use --stop (a new start also stops
# the previous run first, otherwise two controller_managers / moveit_masters fight over the arms).

CONTAINER=${CONTAINER:-franka_bimanual_jazzy}
SESSION=${SESSION:-rviz_test}

# inside the container: franka_ros2/ is mounted at /ros2_ws/src
SETUP="cd /ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash"
WAIT_MASTER="until ros2 action list | grep -q moveit_master/execute_sequence; do echo waiting for moveit_master; sleep 2; done"

# the command a pane runs: a bash in the container, with the workspace sourced
in_container() {
  echo "docker exec -it $CONTAINER bash -c \"$SETUP && $1\""
}

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "container '$CONTAINER' is not running (start it with franka_ros2/docker-compose.yml)"
  exit 1
fi

# every process of a previous run, inside the container
PIPELINE="rviz_mock.launch.py|ros2_control_node|move_group|moveit_master|rviz2|robot_state_publisher|static_transform_publisher|spawner|execution_logger|articulation_orchestrator|run_placements"

stop_pipeline() {
  tmux kill-session -t $SESSION 2>/dev/null
  docker exec $CONTAINER pkill -INT -f "rviz_mock.launch.py"     # lets the launch shut its nodes down
  sleep 3
  docker exec $CONTAINER pkill -KILL -f "$PIPELINE"              # whatever is left
}

if [ "$1" == "--stop" ]; then
  stop_pipeline
  echo "stopped"
  exit 0
fi

PACKAGES="franka_description robotiq_description moveit_master articulation_orchestrator fr3_duo_bringup"

# 0. --clean: fresh build of the pipeline packages (e.g. after a build with other colcon options)
if [ "$1" == "--clean" ]; then
  docker exec $CONTAINER bash -c "cd /ros2_ws && for p in $PACKAGES; do rm -rf build/\$p install/\$p; done"
fi

# 1. build the packages of the pipeline (skip with --no-build).
#    --base-paths: only these five packages; everything else comes from /opt/ros and install/
#    No --symlink-install: the container's workspace is built without it, and colcon cannot
#    switch an existing build to symlinks ("existing path cannot be removed: Is a directory").
if [ "$1" != "--no-build" ]; then
  docker exec $CONTAINER bash -c "cd /ros2_ws && source /opt/ros/jazzy/setup.bash && \
    if [ -f install/setup.bash ]; then source install/setup.bash; fi && \
    colcon build --base-paths \
      src/franka_description \
      src/ros2_robotiq_gripper/robotiq_description \
      src/_moveit_master \
      src/_articulation_orchestrator \
      src/_fr3_duo_bringup" || exit 1
fi

# 2. stop a previous run, then one tmux session with four panes
stop_pipeline
tmux new-session -d -s $SESSION -n pipeline
tmux split-window -h -t $SESSION
tmux split-window -v -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.2

# 3. what runs in each pane
tmux send-keys -t $SESSION:0.0 "$(in_container 'ros2 launch fr3_duo_bringup rviz_mock.launch.py')" Enter
tmux send-keys -t $SESSION:0.1 "$(in_container "$WAIT_MASTER && ros2 run articulation_orchestrator articulation_orchestrator")" Enter
tmux send-keys -t $SESSION:0.2 "$(in_container "$WAIT_MASTER && exec bash")" Enter
tmux send-keys -t $SESSION:0.2 "$SETUP && clear" Enter
tmux send-keys -t $SESSION:0.2 "ros2 run articulation_orchestrator run_placements --keep-going"    # no Enter: you start it
tmux send-keys -t $SESSION:0.3 "$(in_container 'ros2 run articulation_orchestrator execution_logger')" Enter

# 4. show it
tmux select-pane -t $SESSION:0.2
tmux attach -t $SESSION
