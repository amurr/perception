#!/usr/bin/env python3
"""Run the minimal ROS 2 bridge for the dual FR3 arm scene.

Publishes depth images/camera info for the head and two wrists. Publishes and
accepts sensor_msgs/JointState for each arm and its Robotiq joints. No RGB,
lidar, IMU, mobile-base, or head-joint bridge nodes are created.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any


def load_config(path: Path) -> dict[str, Any]:
    config = json.loads(path.read_text(encoding="utf-8"))
    if config.get("schema_version") != 1:
        raise ValueError("Unsupported ros2_bridge_config schema_version")
    if len(config.get("depth_cameras", [])) != 3:
        raise ValueError("The bridge contract must contain exactly three depth cameras")
    if len(config.get("joint_groups", [])) != 2:
        raise ValueError("The bridge contract must contain exactly two joint groups")
    for camera in config["depth_cameras"]:
        for key in ("name", "frame_prim_suffix", "frame_id", "image_topic", "camera_info_topic"):
            if not camera.get(key):
                raise ValueError(f"Depth camera is missing {key}: {camera}")
    for group in config["joint_groups"]:
        for key in ("name", "state_topic", "command_topic", "joint_name_prefixes"):
            if not group.get(key):
                raise ValueError(f"Joint group is missing {key}: {group}")
    return config


def find_unique_frame(stage: Any, suffix: str) -> str:
    matches = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.IsValid() and (prim.GetName() == suffix or prim.GetName().endswith(suffix))
    ]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one frame ending in {suffix!r}, found {matches}")
    return matches[0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", type=Path, default=Path(__file__).resolve().parents[2] / "worlds/sensorized_world.usda")
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[2] / "ros2_bridge_config.json")
    parser.add_argument("--test", action="store_true", help="Run 120 frames and exit")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    config = load_config(args.config.resolve())
    if not args.usd.resolve().is_file():
        raise FileNotFoundError(args.usd)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})
    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    import omni.graph.core as og
    import omni.usd
    import usdrt.Sdf
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import Gf, UsdGeom

    app_utils.enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()
    stage_utils.open_stage(str(args.usd.resolve()))
    while stage_utils.is_stage_loading():
        simulation_app.update()
    for _ in range(10):
        simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No stage after opening USD")

    keys = og.Controller.Keys
    nodes = [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick")]
    connections = []
    values = []
    for camera in config["depth_cameras"]:
        frame_path = find_unique_frame(stage, camera["frame_prim_suffix"])
        camera_path = f"{frame_path}/ROS2DepthCamera__{camera['name']}"
        camera_prim = UsdGeom.Camera.Define(stage, camera_path)
        camera_prim.AddRotateXOp().Set(180.0)
        focal = camera["focal_length_px"][0]
        width = camera["resolution_hw"][1]
        camera_prim.GetFocalLengthAttr().Set(20.955 * focal / width)
        camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(camera["min_range_m"], camera["max_range_m"]))
        prefix = camera["name"]
        nodes.extend([
            (f"CreateRenderProduct__{prefix}", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            (f"DepthHelper__{prefix}", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            (f"CameraInfo__{prefix}", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ])
        connections.extend([
            (f"OnPlaybackTick.outputs:tick", f"CreateRenderProduct__{prefix}.inputs:execIn"),
            (f"CreateRenderProduct__{prefix}.outputs:execOut", f"DepthHelper__{prefix}.inputs:execIn"),
            (f"CreateRenderProduct__{prefix}.outputs:execOut", f"CameraInfo__{prefix}.inputs:execIn"),
            (f"CreateRenderProduct__{prefix}.outputs:renderProductPath", f"DepthHelper__{prefix}.inputs:renderProductPath"),
            (f"CreateRenderProduct__{prefix}.outputs:renderProductPath", f"CameraInfo__{prefix}.inputs:renderProductPath"),
        ])
        values.extend([
            (f"CreateRenderProduct__{prefix}.inputs:cameraPrim", [usdrt.Sdf.Path(camera_path)]),
            (f"CreateRenderProduct__{prefix}.inputs:width", width),
            (f"CreateRenderProduct__{prefix}.inputs:height", camera["resolution_hw"][0]),
            (f"DepthHelper__{prefix}.inputs:frameId", camera["frame_id"]),
            (f"DepthHelper__{prefix}.inputs:topicName", camera["image_topic"]),
            (f"DepthHelper__{prefix}.inputs:type", "depth"),
            (f"CameraInfo__{prefix}.inputs:frameId", camera["frame_id"]),
            (f"CameraInfo__{prefix}.inputs:topicName", camera["camera_info_topic"]),
        ])

    for group in config["joint_groups"]:
        prefix = group["name"]
        nodes.extend([
            (f"ReadJointState__{prefix}", "isaacsim.sensors.physics.IsaacReadJointState"),
            (f"PublishJointState__{prefix}", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            (f"SubscribeJointState__{prefix}", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            (f"ArticulationController__{prefix}", "isaacsim.core.nodes.IsaacArticulationController"),
        ])
        connections.extend([
            ("OnPlaybackTick.outputs:tick", f"ReadJointState__{prefix}.inputs:execIn"),
            (f"ReadJointState__{prefix}.outputs:execOut", f"PublishJointState__{prefix}.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", f"SubscribeJointState__{prefix}.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", f"ArticulationController__{prefix}.inputs:execIn"),
        ])
        for output in ("jointNames", "jointPositions", "jointVelocities", "jointEfforts", "jointDofTypes", "stageMetersPerUnit", "sensorTime"):
            connections.append((f"ReadJointState__{prefix}.outputs:{output}", f"PublishJointState__{prefix}.inputs:{output}"))
        for output in ("jointNames", "positionCommand", "velocityCommand", "effortCommand"):
            connections.append((f"SubscribeJointState__{prefix}.outputs:{output}", f"ArticulationController__{prefix}.inputs:{output}"))
        values.extend([
            (f"ReadJointState__{prefix}.inputs:prim", [usdrt.Sdf.Path(config["robot_prim"])]),
            (f"PublishJointState__{prefix}.inputs:topicName", group["state_topic"]),
            (f"SubscribeJointState__{prefix}.inputs:topicName", group["command_topic"]),
            (f"ArticulationController__{prefix}.inputs:robotPath", config["robot_prim"]),
        ])

    graph, _, _, _ = og.Controller.edit(
        {"graph_path": "/ROS2_FR3_Duo_Bridge", "evaluator_name": "execution"},
        {keys.CREATE_NODES: nodes, keys.CONNECT: connections, keys.SET_VALUES: values},
    )
    og.Controller.evaluate_sync(graph)
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    app_utils.play()
    frames = 0
    while simulation_app.is_running() and (not args.test or frames < 120):
        simulation_app.update()
        frames += 1
    app_utils.stop()
    simulation_app.close()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ROS 2 bridge failed: {error}", file=sys.stderr)
        sys.exit(1)