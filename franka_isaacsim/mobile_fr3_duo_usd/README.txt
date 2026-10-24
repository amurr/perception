Isaac Sim USD package: mobile_fr3_duo_with_vision_and_manipulation_kit
Isaac Sim version: 6.0.0-rc.59
Main file: mobile_fr3_duo_with_vision_and_manipulation_kit.usda

Open mobile_fr3_duo_with_vision_and_manipulation_kit.usda in Isaac Sim 6.0.0-rc.59.
All layer/payload references are relative under this directory.

Mobile sensor-test world:
  Open worlds/sensor_test_world.usda for the floor, lighting, and
  colored camera/lidar targets with a relative robot reference.
  Target poses/colors/purposes: worlds/sensor_test_targets.json
  The world supplies visual/geometric targets only; it does not prove
  camera/lidar plugin or sensor-output correctness.

Sensorized world entrypoint (native Isaac sensors):
  GUI: File → Open → worlds/sensorized_world.usda
  Bootstrap config: worlds/native_sensor_bootstrap.json
  Sensor profile(s): sensors/
  Archive-local tooling: tools/
    (bin/run_isaac_script.sh, bootstrap/capture/merge scripts, libs, schemas)

ROS 2 bridge (depth + arm/gripper control only):
  Contract: ros2_bridge_config.json
  Start GUI:
    export ISAACSIM_PATH=/workspace/isaacsim
    ${ISAACSIM_PATH}/python.sh tools/scripts/launch_ros2_bridge.py \
      --usd worlds/sensorized_world.usda
  Start headless test:
    ${ISAACSIM_PATH}/python.sh tools/scripts/launch_ros2_bridge.py \
      --usd worlds/sensorized_world.usda --headless --test
  This creates depth image and camera_info topics for the head and both wrists,
  plus JointState state/command topics for each arm and its gripper. RGB, lidar,
  IMU, mobile-base, and head-joint ROS interfaces are intentionally absent.

Headless bootstrap (activate Camera/OmniLidar/IsaacImuSensor children):
  export ISAACSIM_PATH=/path/to/isaac-sim-6.0.0-rc.59
  export ISAACSIM6_TOOLING_ROOT=<extract>/mobile_fr3_duo_with_vision_and_manipulation_kit/tools
  "${ISAACSIM6_TOOLING_ROOT}/bin/run_isaac_script.sh" bootstrap_native_sensors.py \
    --model mobile_fr3_duo_with_vision_and_manipulation_kit --usd <extract>/mobile_fr3_duo_with_vision_and_manipulation_kit/worlds/sensorized_world.usda \
    --report /tmp/${model}_bootstrap.json

Headless semantic capture (full profile):
  "${ISAACSIM6_TOOLING_ROOT}/bin/run_isaac_script.sh" capture_native_sensors.py \
    --model mobile_fr3_duo_with_vision_and_manipulation_kit --usd <extract>/mobile_fr3_duo_with_vision_and_manipulation_kit/worlds/sensorized_world.usda \
    --evidence-dir /tmp/${model}_native_capture \
    --report /tmp/${model}_native_capture/capture_report.json
  Geometry-only frames are never counted as active sensors.
  The ROS 2 bridge is configured separately by tools/scripts/launch_ros2_bridge.py.

Union all-sensor acceptance (exact-disjoint 7+11 batch-and-merge):
  Do NOT attach all 18 devices in one session (reproducible device-lost abort).
  1) Capture vision batch (7 logical names) from the vision-kit archive extract.
  2) Capture mobile batch (11 logical names) from the mobile-sensors archive extract.
  3) Merge under Isaac python.sh (numpy required; bare host python3 is not enough):
     "${ISAACSIM_PATH}/python.sh" \
       <extract>/mobile_fr3_duo_with_vision_and_manipulation_kit/tools/scripts/merge_native_sensor_captures.py \
       --model mobile_fr3_duo_with_vision_and_manipulation_kit \
       --batch-summary <vision_batch>/capture_summary.json \
       --batch-summary <mobile_batch>/capture_summary.json \
       --report <union_dir>/capture_summary.json
  Pass requires exact_disjoint_coverage=true and merged_device_count=18.
