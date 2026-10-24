# installing dependencies
bash ./perception_learning/ros2h_installing.sh
bash ./perception_learning/franka_isaacsim/xrdp/start_xrdp.sh


# Isaac Sim
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH}

cd /workspace/perception_learning/franka_isaacsim/mobile_fr3_duo_usd
../isaacsim_launch/isaacsim_launch.sh \
  /workspace/perception_learning/franka_isaacsim/mobile_fr3_duo_usd/worlds/sensorized_world.usda

## ROS 2 bridge

export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH}

cd /workspace/perception_learning/franka_isaacsim/mobile_fr3_duo_usd
/workspace/isaacsim/python.sh tools/scripts/launch_ros2_bridge.py --usd worlds/sensorized_world.usda



# foxglove

sudo ufw allow 8765/tcp

source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:=0.0.0.0 port:=8765


on my pc: in foxglove web dash:
ws://157.157.221.29:PORT_MAPPED_TO_CONTAINERSPORT8765_BY_RUNPOD
  (157.157.221.29:32695 - :8765)