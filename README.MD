# local development

xhost +local:docker
docker build -t rosj /home/artem/r/franka_bimanual
docker run --rm -it \
  --name rosj \
  --net=host \
  --ipc=host \
  -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=/tmp/.Xauthority \
  -v "${XAUTHORITY:-$HOME/.Xauthority}":/tmp/.Xauthority:ro \
  -e QT_QPA_PLATFORM=xcb \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  --device /dev/dri \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/artem/r/franka_bimanual:/ros2_ws/src/franka_bimanual:rw \
  rosj


docker exec -it -e DISPLAY="$DISPLAY" rosj bash

## grasp poses generating in RViz

run on host:
xhost +SI:localuser:artem

run in franka container:
```
cd /ros2_ws && source install/setup.bash
ros2 launch franka_description visualize_franka.launch.py robot_type:=mobile_fr3_duo_v0_2 load_gripper:=false ee_id:=none
```


# in Pod
bash franka_bimanual/franka_isaacsim/pod_scripts/start_xrdp.sh


## Isaac Sim
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH}

cd /workspace/perception_learning/franka_isaacsim/mobile_fr3_duo_usd
../isaacsim_launch/isaacsim_launch.sh \
  /workspace/perception_learning/franka_isaacsim/mobile_fr3_duo_usd/worlds/sensorized_world.usda

#### ROS 2 bridge

export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH}

cd /workspace/perception_learning/franka_isaacsim/mobile_fr3_duo_usd
/workspace/isaacsim/python.sh tools/scripts/launch_ros2_bridge.py --usd worlds/sensorized_world.usda



## foxglove

sudo ufw allow 8765/tcp

source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:=0.0.0.0 port:=8765


on my pc: in foxglove web dash:
ws://157.157.221.29:PORT_MAPPED_TO_CONTAINERSPORT8765_BY_RUNPOD
  (157.157.221.29:32695 - :8765)