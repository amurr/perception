Archive-local Isaac sensor workflow tooling for mobile_fr3_duo_with_vision_and_manipulation_kit.
Set ISAACSIM_PATH to Isaac Sim 6.0.0-rc.59, then from this tools/ directory:
  export ISAACSIM6_TOOLING_ROOT="$PWD"
  ./bin/run_isaac_script.sh capture_native_sensors.py \
    --model mobile_fr3_duo_with_vision_and_manipulation_kit --usd ../worlds/sensorized_world.usda \
    --evidence-dir /tmp/native_capture --report /tmp/native_capture/capture_report.json
Merge requires Isaac python.sh (numpy). See the package README.txt.
