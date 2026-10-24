"""Reusable Isaac-native sensor capture adapter (offline-safe core).

Runtime Isaac imports stay in ``scripts/capture_native_sensors.py``. This module
owns per-device oracle expectations, view-class visibility classification,
timeout deadlines, and finite/semantic metric evaluation for RGB, depth, lidar,
and IMU across all sensor-bearing profiles.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from .process_failure_capsule import build_process_failure_capsule
from .sensor_bootstrap import (
    CAMERA_DEVICE_CLASSES,
    IMU_DEVICE_CLASSES,
    LIDAR_DEVICE_CLASSES,
)
from .sensor_profile import load_sensor_profile
from .sensor_test_world import load_sensor_test_world_config
from .strict_json import (
    dumps_strict_json,
    finite_or_none,
    json_safe,
    write_strict_json,
)


class SensorCaptureError(RuntimeError):
    """Raised when native sensor capture configuration or evaluation fails."""


# Isaac 6.0 experimental runtime contracts used by the capture script.
SUPPORTED_RUNTIME_CONTRACT = {
    "camera": {
        "authoring": "isaacsim.sensors.experimental.rtx.RtxCamera",
        "runtime": "isaacsim.sensors.experimental.rtx.CameraSensor",
        "annotators": {
            "rgb": "rgb",
            "depth": "distance_to_image_plane",
        },
        "frame_convention": (
            "ROS optical (+Z forward, +Y down) requires a local 180° X rotation "
            "so USD Camera (-Z forward, +Y up) looks along the optical axis"
        ),
        "step": "omni.replicator.core.orchestrator.step",
    },
    "lidar_2d": {
        "authoring": "isaacsim.sensors.experimental.rtx.Lidar",
        "runtime": "isaacsim.sensors.experimental.rtx.LidarSensor",
        "annotators": {
            "range_scan": "generic-model-output",
            "point_cloud": "generic-model-output",
        },
        "config": "Example_Rotary_2D",
        "gmo_fields": {
            "azimuth_deg": "x",
            "elevation_deg": "y",
            "range_m": "z",
        },
        "capture_path": "replicator_writer",
        "step": "omni.timeline.play",
    },
    "imu": {
        "authoring": "isaacsim.sensors.experimental.physics.IMU",
        "runtime": "isaacsim.sensors.experimental.physics.IMUSensor",
        "frame": {
            "linear_acceleration": "sensor body frame [x, y, z] m/s^2",
            "angular_velocity": "sensor body frame [x, y, z] rad/s",
            "orientation": "quaternion [w, x, y, z] sensor-to-world",
            "gravity_convention": (
                "get_data(read_gravity=True) on a settled upright Isaac-native "
                "IMU reports approximately -g along the sensor axis that aligns "
                "with world +Z (observed linear_acceleration ≈ [0, 0, -9.81]); "
                "angular velocity remains near zero"
            ),
            "gravity_vector_sign": "negative_world_up",
        },
        "step": "omni.timeline.play",
    },
}

# Physically motivated acceptance bands from the packaged world geometry.
# These are not free parameters tuned against a failing capture.
FRONT_WALL_NAME = "FrontWallWhite"
LEFT_PILLAR_NAME = "LeftPillarRed"
RIGHT_PILLAR_NAME = "RightPillarGreen"
SIDE_SPHERE_NAME = "SideBlackSphere"
DEPTH_RANGE_TOLERANCE_M = 1.5
LIDAR_BEARING_WINDOW_DEG = 15.0
LIDAR_RANGE_TOLERANCE_M = 1.5
IMU_GRAVITY_NOMINAL_M_S2 = 9.81
IMU_GRAVITY_TOLERANCE_M_S2 = 0.8
IMU_GRAVITY_LATERAL_TOLERANCE_M_S2 = 0.8
IMU_ANGULAR_VELOCITY_MAX_RAD_S = 0.5
CAPTURE_EXIT_FULL_PROFILE_OK = 0
CAPTURE_EXIT_SELECTION_OK_INCOMPLETE = 2
CAPTURE_EXIT_SELECTION_FAILED = 3
RGB_CENTER_MIN_MEAN_LUMINANCE = 80.0
RGB_MIN_STD_LUMINANCE = 1.0
# Structured non-wall views: quarter of the wall-bright floor (material/lighting scale).
RGB_STRUCTURED_MIN_MEAN_LUMINANCE = RGB_CENTER_MIN_MEAN_LUMINANCE / 4.0
# Horizon / floor band separation for open lateral and rear optical frames.
RGB_HORIZON_MIN_BAND_DELTA = RGB_STRUCTURED_MIN_MEAN_LUMINANCE / 4.0
# Half the mean-L1 gap between packaged red and green materials at 0–255.
RGB_TARGET_COLOR_MAX_L1 = 80.0
# Allow occlusion / off-center placement of a geometrically projected target.
ORACLE_PROJECTION_SAFETY_DIVISOR = 16.0
DEPTH_FINITE_FRACTION_MIN = 0.50
DEPTH_INVALID_MAX_M = 1.0e4
# Lower-half floor visibility reduced by the same projection safety divisor.
DEPTH_OPEN_VIEW_FINITE_FRACTION_MIN = 0.5 / ORACLE_PROJECTION_SAFETY_DIVISOR

VIEW_FORWARD_WALL = "forward_wall"
VIEW_LATERAL_LEFT = "lateral_left"
VIEW_LATERAL_RIGHT = "lateral_right"
VIEW_REAR_OPEN = "rear_open"
VIEW_NEAR_WORKSPACE = "near_workspace"
VIEW_EE_WORKSPACE = "ee_workspace"
VIEW_REAR_LIDAR = "rear_lidar"
VIEW_IMU_BODY = "imu_body"

REPRESENTATIVE_DEVICES_BY_MODEL: dict[str, dict[str, str]] = {
    "franka_mobile_sensors": {
        "rgb": "camera_front_d455_rgb",
        "depth": "camera_front_d455_depth",
        "lidar": "lidar_front_sick",
        "imu": "olv_imu",
    },
    "franka_vision_and_manipulation_kit": {
        "rgb": "head_zed_left_rgb",
        "depth": "head_zed_depth",
    },
    "mobile_fr3_duo_with_vision_and_manipulation_kit": {
        "rgb": "camera_front_d455_rgb",
        "depth": "camera_front_d455_depth",
        "lidar": "lidar_front_sick",
        "imu": "olv_imu",
    },
}


class CaptureDeadline:
    """Monotonic deadline used to abort hung capture loops."""

    def __init__(self, timeout_s: float) -> None:
        if timeout_s <= 0.0:
            raise SensorCaptureError(f"timeout_s must be positive, got {timeout_s}")
        self.timeout_s = float(timeout_s)
        self.started_at = time.monotonic()
        self.deadline_at = self.started_at + self.timeout_s

    def remaining_s(self) -> float:
        return self.deadline_at - time.monotonic()

    def expired(self) -> bool:
        return self.remaining_s() <= 0.0

    def raise_if_expired(self, where: str) -> None:
        if self.expired():
            raise SensorCaptureError(
                f"Capture timed out after {self.timeout_s:.1f}s at {where}"
            )


def representative_devices_for_model(model: str) -> dict[str, str]:
    devices = REPRESENTATIVE_DEVICES_BY_MODEL.get(model)
    if devices is None:
        raise SensorCaptureError(
            f"No representative capture set for model {model!r}; "
            f"supported: {sorted(REPRESENTATIVE_DEVICES_BY_MODEL)}"
        )
    return dict(devices)


def logical_names_for_representatives(model: str) -> list[str]:
    return sorted(set(representative_devices_for_model(model).values()))


def logical_names_for_model(model: str) -> list[str]:
    profile = load_sensor_profile(model)
    return [device["logical_name"] for device in profile["devices"]]


def output_class_for_device(device: Mapping[str, Any]) -> str:
    device_class = device["device_class"]
    outputs = set(device.get("outputs") or [])
    if device_class in CAMERA_DEVICE_CLASSES:
        if outputs == {"depth"} or ("depth" in outputs and "rgb" not in outputs):
            return "depth"
        if "rgb" in outputs:
            return "rgb"
        return "camera"
    if device_class in LIDAR_DEVICE_CLASSES:
        return "lidar"
    if device_class in IMU_DEVICE_CLASSES:
        return "imu"
    raise SensorCaptureError(f"Unsupported device_class: {device_class!r}")


def view_class_for_device(device: Mapping[str, Any]) -> str:
    """Map a profile device to an oracle view class (not free threshold tuning)."""
    logical_name = str(device["logical_name"])
    role = str(device.get("role") or "")
    output_class = output_class_for_device(device)
    if output_class == "imu":
        return VIEW_IMU_BODY
    if output_class == "lidar":
        if "rear" in logical_name or "rear" in role:
            return VIEW_REAR_LIDAR
        return VIEW_FORWARD_WALL
    if "wrist" in logical_name or "wrist" in role:
        return VIEW_EE_WORKSPACE
    if "rear" in logical_name or "rear" in role:
        return VIEW_REAR_OPEN
    if "head" in logical_name or role.startswith("head"):
        return VIEW_NEAR_WORKSPACE
    if logical_name.startswith("camera_left") or role.startswith("left_"):
        return VIEW_LATERAL_LEFT
    if logical_name.startswith("camera_right") or role.startswith("right_"):
        return VIEW_LATERAL_RIGHT
    return VIEW_FORWARD_WALL


def _target_by_name(manifest: Mapping[str, Any], name: str) -> dict[str, Any]:
    for target in manifest.get("targets") or []:
        if target.get("name") == name:
            return dict(target)
    raise SensorCaptureError(f"Oracle target {name!r} missing from target manifest")


def look_direction_xy(view_class: str) -> tuple[float, float] | None:
    """World-XY unit look direction for corridor-oriented optical frames."""
    if view_class == VIEW_FORWARD_WALL:
        return (1.0, 0.0)
    if view_class == VIEW_REAR_OPEN:
        return (-1.0, 0.0)
    if view_class == VIEW_LATERAL_LEFT:
        return (0.0, 1.0)
    if view_class == VIEW_LATERAL_RIGHT:
        return (0.0, -1.0)
    return None


def primary_target_name_for_view(view_class: str) -> str | None:
    if view_class == VIEW_FORWARD_WALL:
        return FRONT_WALL_NAME
    if view_class == VIEW_LATERAL_LEFT:
        return LEFT_PILLAR_NAME
    if view_class == VIEW_LATERAL_RIGHT:
        return RIGHT_PILLAR_NAME
    return None


def range_along_look_m(
    translation_m: Sequence[float], look_xy: tuple[float, float]
) -> float:
    return abs(
        float(translation_m[0]) * float(look_xy[0])
        + float(translation_m[1]) * float(look_xy[1])
    )


def fov_deg_from_calibration(
    calibration: Mapping[str, Any],
) -> tuple[float, float]:
    resolution = calibration.get("resolution_hw") or [720, 1280]
    focal = calibration.get("focal_length_px") or [650.0, 650.0]
    height = float(resolution[0])
    width = float(resolution[1])
    fx = float(focal[0])
    fy = float(focal[1])
    if fx <= 0.0 or fy <= 0.0:
        raise SensorCaptureError("focal_length_px must be positive for FOV derivation")
    fov_h = math.degrees(2.0 * math.atan((width / 2.0) / fx))
    fov_v = math.degrees(2.0 * math.atan((height / 2.0) / fy))
    return fov_h, fov_v


def projected_target_image_fraction(
    target: Mapping[str, Any],
    *,
    range_m: float,
    fov_h_deg: float,
    fov_v_deg: float,
) -> float:
    """Axis-aligned angular footprint of a packaged target over camera FOV."""
    if range_m <= 0.0 or fov_h_deg <= 0.0 or fov_v_deg <= 0.0:
        return 0.0
    dims = target["dimensions_m"]
    width_m = max(float(dims["x"]), float(dims["y"]))
    height_m = float(dims["z"])
    angular_w = math.degrees(2.0 * math.atan((width_m / 2.0) / range_m))
    angular_h = math.degrees(2.0 * math.atan((height_m / 2.0) / range_m))
    return max(0.0, (angular_w / fov_h_deg) * (angular_h / fov_v_deg))


def visibility_for_device(
    device: Mapping[str, Any],
    *,
    view_class: str,
) -> dict[str, Any]:
    output_class = output_class_for_device(device)
    if view_class == VIEW_IMU_BODY:
        return {
            "note": "IMU has no optical FOV; gravity/angular-velocity checks apply",
        }
    if view_class == VIEW_REAR_LIDAR:
        return {
            FRONT_WALL_NAME: "may_be_visible_at_wide_fov_edge",
            "reason": (
                "rear lidar faces opposite yaw, but a ~275° scan can still "
                "graze the +X corridor wall at FOV edges; do not assert absence"
            ),
        }
    if view_class == VIEW_EE_WORKSPACE:
        return {
            FRONT_WALL_NAME: "expected_not_visible",
            LEFT_PILLAR_NAME: "expected_not_visible",
            RIGHT_PILLAR_NAME: "expected_not_visible",
            "reason": (
                "wrist optical frame targets the end-effector workspace; "
                "corridor-colored targets are out of the near D405 range"
            ),
        }
    if view_class == VIEW_NEAR_WORKSPACE:
        return {
            FRONT_WALL_NAME: "expected_not_visible",
            LEFT_PILLAR_NAME: "expected_not_visible",
            RIGHT_PILLAR_NAME: "expected_not_visible",
            "reason": (
                "head optical frames look into the near manipulation workspace "
                "on the packaged clear-corridor scale, not the +X corridor wall"
            ),
        }
    if view_class == VIEW_REAR_OPEN:
        return {
            FRONT_WALL_NAME: "expected_not_visible",
            LEFT_PILLAR_NAME: "expected_not_visible",
            RIGHT_PILLAR_NAME: "expected_not_visible",
            SIDE_SPHERE_NAME: "expected_not_visible",
            "reason": (
                "rear optical frame looks −X; packaged colored targets lie on "
                "+X/+Y/−Y and must not dominate this view"
            ),
        }
    if view_class == VIEW_LATERAL_LEFT:
        return {
            FRONT_WALL_NAME: "expected_not_visible",
            LEFT_PILLAR_NAME: "expected_not_visible",
            SIDE_SPHERE_NAME: "expected_not_visible",
            RIGHT_PILLAR_NAME: "expected_not_visible",
            "reason": (
                "left optical frame faces +Y but the D455 mount looks into the "
                "near floor/horizon band; packaged lateral pillars lie outside "
                "that optical FOV"
            ),
        }
    if view_class == VIEW_LATERAL_RIGHT:
        return {
            FRONT_WALL_NAME: "expected_not_visible",
            RIGHT_PILLAR_NAME: "expected_not_visible",
            LEFT_PILLAR_NAME: "expected_not_visible",
            "reason": (
                "right optical frame faces −Y but the D455 mount looks into the "
                "near floor/horizon band; packaged lateral pillars lie outside "
                "that optical FOV"
            ),
        }
    # forward_wall
    if output_class == "rgb":
        return {
            FRONT_WALL_NAME: "expected_visible",
            LEFT_PILLAR_NAME: "may_be_visible",
            RIGHT_PILLAR_NAME: "may_be_visible",
            SIDE_SPHERE_NAME: "likely_occluded_or_out_of_fov",
        }
    if output_class == "depth":
        return {
            FRONT_WALL_NAME: "expected_in_range",
            "FarMagentaCylinder": "may_be_occluded_by_front_wall",
        }
    return {
        FRONT_WALL_NAME: "expected_hit_near_bearing_0",
        "FarMagentaCylinder": "may_be_occluded_by_front_wall",
    }


def _device_check_payload(
    device: Mapping[str, Any],
    *,
    view_class: str,
    front_wall: Mapping[str, Any],
    primary_target: Mapping[str, Any] | None,
    clear_corridor: Mapping[str, Any],
    gravity_magnitude: float,
) -> dict[str, Any]:
    logical_name = device["logical_name"]
    output_class = output_class_for_device(device)
    visibility = visibility_for_device(device, view_class=view_class)
    calibration = dict(device.get("simulation_calibration") or {})
    resolution = calibration.get("resolution_hw")
    front_range = float(front_wall["range_from_origin_m"])
    front_bearing = float(front_wall["bearing_from_origin_deg"])
    front_color = [float(item) for item in front_wall["color_rgb"]]
    look_xy = look_direction_xy(view_class)
    base = {
        "logical_name": logical_name,
        "output_class": output_class,
        "view_class": view_class,
        "role": device.get("role"),
        "frame_name_suffix": device.get("frame_name_suffix"),
        "visibility": visibility,
        "resolution_hw": list(resolution) if resolution else None,
        "look_direction_xy": list(look_xy) if look_xy is not None else None,
    }

    primary_range = None
    primary_color = list(front_color)
    projected_fraction = 0.0
    if primary_target is not None and look_xy is not None:
        primary_range = range_along_look_m(primary_target["translation_m"], look_xy)
        primary_color = [float(item) for item in primary_target["color_rgb"]]
        fov_h, fov_v = fov_deg_from_calibration(calibration)
        projected_fraction = projected_target_image_fraction(
            primary_target,
            range_m=primary_range,
            fov_h_deg=fov_h,
            fov_v_deg=fov_v,
        )
        base["primary_target"] = {
            "name": primary_target["name"],
            "color": primary_target.get("color"),
            "color_rgb": primary_color,
            "translation_m": list(primary_target["translation_m"]),
            "range_along_look_m": primary_range,
            "projected_image_fraction": projected_fraction,
        }

    if output_class == "rgb":
        min_color_fraction = projected_fraction / ORACLE_PROJECTION_SAFETY_DIVISOR
        max_absent_fraction = max(
            min_color_fraction,
            1.0 / ORACLE_PROJECTION_SAFETY_DIVISOR,
        )
        if view_class == VIEW_FORWARD_WALL:
            base.update(
                {
                    "require_center_bright": True,
                    "require_target_color_presence": False,
                    "require_structured_scene": False,
                    "require_wall_color_absent": False,
                    "require_horizon_structure": False,
                    "min_std_luminance": RGB_MIN_STD_LUMINANCE,
                    "center_min_mean_luminance": RGB_CENTER_MIN_MEAN_LUMINANCE,
                    "structured_min_mean_luminance": RGB_STRUCTURED_MIN_MEAN_LUMINANCE,
                    "horizon_min_band_delta": RGB_HORIZON_MIN_BAND_DELTA,
                    "target_color_rgb": primary_color,
                    "target_color_max_l1": RGB_TARGET_COLOR_MAX_L1,
                    "target_color_min_fraction": min_color_fraction,
                    "wall_color_rgb": front_color,
                    "wall_color_max_fraction": max_absent_fraction,
                }
            )
            return base
        if view_class in (VIEW_LATERAL_LEFT, VIEW_LATERAL_RIGHT, VIEW_REAR_OPEN):
            base.update(
                {
                    "require_center_bright": False,
                    "require_target_color_presence": False,
                    "require_structured_scene": True,
                    "require_wall_color_absent": True,
                    "require_horizon_structure": True,
                    "min_std_luminance": RGB_MIN_STD_LUMINANCE,
                    "center_min_mean_luminance": RGB_CENTER_MIN_MEAN_LUMINANCE,
                    "structured_min_mean_luminance": RGB_STRUCTURED_MIN_MEAN_LUMINANCE,
                    "horizon_min_band_delta": RGB_HORIZON_MIN_BAND_DELTA,
                    "target_color_rgb": front_color,
                    "target_color_max_l1": RGB_TARGET_COLOR_MAX_L1,
                    "target_color_min_fraction": 0.0,
                    "wall_color_rgb": front_color,
                    "wall_color_max_fraction": max_absent_fraction,
                }
            )
            return base
        # near_workspace / ee_workspace: depth owns geometry; RGB asserts a lit
        # non-blank frame. Neutral/bright EE surfaces may match white chroma, so
        # wall-color absence is not required here.
        base.update(
            {
                "require_center_bright": False,
                "require_target_color_presence": False,
                "require_structured_scene": True,
                "require_wall_color_absent": False,
                "require_horizon_structure": False,
                "min_std_luminance": 0.0,
                "center_min_mean_luminance": RGB_CENTER_MIN_MEAN_LUMINANCE,
                "structured_min_mean_luminance": RGB_STRUCTURED_MIN_MEAN_LUMINANCE,
                "horizon_min_band_delta": RGB_HORIZON_MIN_BAND_DELTA,
                "target_color_rgb": front_color,
                "target_color_max_l1": RGB_TARGET_COLOR_MAX_L1,
                "target_color_min_fraction": 0.0,
                "wall_color_rgb": front_color,
                "wall_color_max_fraction": max_absent_fraction,
            }
        )
        return base

    if output_class == "depth":
        if view_class == VIEW_FORWARD_WALL:
            base.update(
                {
                    "require_center_range": True,
                    "require_dense_finite_fraction": True,
                    "require_range_band_hits": False,
                    "require_open_finite_fraction": False,
                    "forbid_center_front_wall_range": False,
                    "finite_fraction_min": DEPTH_FINITE_FRACTION_MIN,
                    "expected_range_m": front_range,
                    "range_tolerance_m": DEPTH_RANGE_TOLERANCE_M,
                    "range_band_min_hits": 0,
                    "forbidden_range_m": front_range,
                    "forbidden_range_tolerance_m": DEPTH_RANGE_TOLERANCE_M,
                }
            )
            return base
        if view_class in (VIEW_LATERAL_LEFT, VIEW_LATERAL_RIGHT, VIEW_REAR_OPEN):
            base.update(
                {
                    "require_center_range": False,
                    "require_dense_finite_fraction": False,
                    "require_range_band_hits": False,
                    "require_open_finite_fraction": True,
                    "forbid_center_front_wall_range": True,
                    "finite_fraction_min": DEPTH_OPEN_VIEW_FINITE_FRACTION_MIN,
                    "expected_range_m": front_range,
                    "range_tolerance_m": DEPTH_RANGE_TOLERANCE_M,
                    "range_band_min_hits": 0,
                    "forbidden_range_m": front_range,
                    "forbidden_range_tolerance_m": DEPTH_RANGE_TOLERANCE_M,
                }
            )
            return base
        if view_class == VIEW_NEAR_WORKSPACE:
            corridor_scale = float(clear_corridor["x_max_m"])
            base.update(
                {
                    "require_center_range": True,
                    "require_dense_finite_fraction": True,
                    "require_range_band_hits": False,
                    "require_open_finite_fraction": False,
                    "forbid_center_front_wall_range": True,
                    "finite_fraction_min": DEPTH_FINITE_FRACTION_MIN,
                    "expected_range_m": corridor_scale,
                    "range_tolerance_m": corridor_scale,
                    "range_band_min_hits": 0,
                    "forbidden_range_m": front_range,
                    "forbidden_range_tolerance_m": DEPTH_RANGE_TOLERANCE_M,
                }
            )
            return base
        if view_class == VIEW_EE_WORKSPACE:
            min_range = float(calibration.get("min_range_m", 0.07))
            max_range = float(calibration.get("max_range_m", 0.5))
            mid_range = 0.5 * (min_range + max_range)
            half_span = 0.5 * (max_range - min_range)
            base.update(
                {
                    "require_center_range": True,
                    "require_dense_finite_fraction": True,
                    "require_range_band_hits": False,
                    "require_open_finite_fraction": False,
                    "forbid_center_front_wall_range": True,
                    "finite_fraction_min": DEPTH_FINITE_FRACTION_MIN,
                    "expected_range_m": mid_range,
                    "range_tolerance_m": half_span,
                    "range_band_min_hits": 0,
                    "forbidden_range_m": front_range,
                    "forbidden_range_tolerance_m": DEPTH_RANGE_TOLERANCE_M,
                    "calibration_min_range_m": min_range,
                    "calibration_max_range_m": max_range,
                }
            )
            return base
        raise SensorCaptureError(f"No depth oracle for view_class {view_class!r}")

    if output_class == "lidar":
        base.update(
            {
                "require_finite_points": True,
                "require_forward_wall_hit": view_class == VIEW_FORWARD_WALL,
                "obstacle_bearing_deg": front_bearing,
                "bearing_window_deg": LIDAR_BEARING_WINDOW_DEG,
                "expected_range_m": front_range,
                "range_tolerance_m": LIDAR_RANGE_TOLERANCE_M,
            }
        )
        return base
    if output_class == "imu":
        magnitude = float(gravity_magnitude or IMU_GRAVITY_NOMINAL_M_S2)
        base.update(
            {
                "frame_convention": SUPPORTED_RUNTIME_CONTRACT["imu"]["frame"],
                "expected_gravity_magnitude_m_s2": magnitude,
                "expected_gravity_vector_m_s2": [0.0, 0.0, -magnitude],
                "gravity_tolerance_m_s2": IMU_GRAVITY_TOLERANCE_M_S2,
                "gravity_lateral_tolerance_m_s2": IMU_GRAVITY_LATERAL_TOLERANCE_M_S2,
                "angular_velocity_max_rad_s": IMU_ANGULAR_VELOCITY_MAX_RAD_S,
                "read_gravity": True,
            }
        )
        return base
    raise SensorCaptureError(f"No oracle check for output_class {output_class!r}")


def build_capture_expectations(
    model: str,
    *,
    target_manifest: Mapping[str, Any] | None = None,
    world_config: Mapping[str, Any] | None = None,
    profile: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Build packed-world oracle expectations for every profile device."""
    from .sensor_test_world import build_target_manifest

    resolved_world = (
        dict(world_config)
        if world_config is not None
        else load_sensor_test_world_config()
    )
    resolved_profile = dict(profile) if profile is not None else load_sensor_profile(model)
    manifest = (
        dict(target_manifest)
        if target_manifest is not None
        else build_target_manifest(model, resolved_world)
    )
    front_wall = _target_by_name(manifest, FRONT_WALL_NAME)
    left_pillar = _target_by_name(manifest, LEFT_PILLAR_NAME)
    right_pillar = _target_by_name(manifest, RIGHT_PILLAR_NAME)
    representatives = representative_devices_for_model(model)
    gravity = [float(item) for item in manifest["gravity_m_s2"]]
    gravity_magnitude = abs(gravity[2])
    clear_corridor = dict(manifest.get("clear_corridor") or resolved_world["clear_corridor"])

    front_range = float(front_wall["range_from_origin_m"])
    front_bearing = float(front_wall["bearing_from_origin_deg"])
    color_rgb = [float(item) for item in front_wall["color_rgb"]]
    primary_by_view = {
        VIEW_FORWARD_WALL: front_wall,
        VIEW_LATERAL_LEFT: left_pillar,
        VIEW_LATERAL_RIGHT: right_pillar,
    }

    devices: dict[str, Any] = {}
    visibility: dict[str, Any] = {}
    checks: dict[str, Any] = {}
    for device in resolved_profile["devices"]:
        logical_name = device["logical_name"]
        view_class = view_class_for_device(device)
        payload = _device_check_payload(
            device,
            view_class=view_class,
            front_wall=front_wall,
            primary_target=primary_by_view.get(view_class),
            clear_corridor=clear_corridor,
            gravity_magnitude=gravity_magnitude,
        )
        devices[logical_name] = {
            "logical_name": logical_name,
            "output_class": payload["output_class"],
            "view_class": view_class,
            "role": device.get("role"),
            "frame_name_suffix": device.get("frame_name_suffix"),
        }
        visibility[logical_name] = dict(payload["visibility"])
        checks[logical_name] = payload

    return {
        "model": model,
        "expected_device_count": len(resolved_profile["devices"]),
        "runtime_contract": SUPPORTED_RUNTIME_CONTRACT,
        "representatives": representatives,
        "gravity_m_s2": gravity,
        "gravity_magnitude_m_s2": gravity_magnitude,
        "front_wall": {
            "name": front_wall["name"],
            "color": front_wall["color"],
            "color_rgb": color_rgb,
            "translation_m": list(front_wall["translation_m"]),
            "range_from_origin_m": front_range,
            "bearing_from_origin_deg": front_bearing,
        },
        "lateral_targets": {
            "left": {
                "name": left_pillar["name"],
                "color_rgb": list(left_pillar["color_rgb"]),
                "translation_m": list(left_pillar["translation_m"]),
                "range_along_look_m": range_along_look_m(
                    left_pillar["translation_m"], (0.0, 1.0)
                ),
            },
            "right": {
                "name": right_pillar["name"],
                "color_rgb": list(right_pillar["color_rgb"]),
                "translation_m": list(right_pillar["translation_m"]),
                "range_along_look_m": range_along_look_m(
                    right_pillar["translation_m"], (0.0, -1.0)
                ),
            },
        },
        "devices": devices,
        "visibility": visibility,
        "checks": checks,
        "checks_by_class": {
            key: checks[name]
            for key, name in representatives.items()
            if name in checks
        },
    }


def _as_numpy(array_like: Any) -> np.ndarray:
    if array_like is None:
        raise SensorCaptureError("Sensor returned no data array")
    if hasattr(array_like, "numpy"):
        array_like = array_like.numpy()
    array = np.asarray(array_like)
    if array.size == 0:
        raise SensorCaptureError("Sensor returned an empty array")
    return array


def _center_roi(height: int, width: int, fraction: float = 0.25) -> tuple[slice, slice]:
    half_h = max(1, int(height * fraction / 2.0))
    half_w = max(1, int(width * fraction / 2.0))
    center_h = height // 2
    center_w = width // 2
    return (
        slice(max(0, center_h - half_h), min(height, center_h + half_h)),
        slice(max(0, center_w - half_w), min(width, center_w + half_w)),
    )


def _color_match_fraction(
    rgb_f: np.ndarray,
    target_rgb_0_1: Sequence[float],
    *,
    max_l1: float,
) -> float:
    target = np.asarray(target_rgb_0_1, dtype=np.float32) * 255.0
    channel_l1 = np.mean(np.abs(rgb_f - target.reshape(1, 1, 3)), axis=2)
    if channel_l1.size == 0:
        return 0.0
    return float(np.mean(channel_l1 <= float(max_l1)))


def evaluate_rgb_array(
    rgb: Any,
    *,
    expected_resolution_hw: Sequence[int],
    expectations: Mapping[str, Any],
) -> dict[str, Any]:
    array = _as_numpy(rgb)
    if array.ndim != 3 or array.shape[2] < 3:
        raise SensorCaptureError(f"RGB array must be HxWxC, got shape {array.shape}")
    height, width = int(array.shape[0]), int(array.shape[1])
    expected_h, expected_w = int(expected_resolution_hw[0]), int(expected_resolution_hw[1])
    shape_ok = height == expected_h and width == expected_w
    finite = bool(np.isfinite(array[..., :3].astype(np.float32)).all())
    rgb_f = array[..., :3].astype(np.float32)
    luminance = rgb_f.mean(axis=2)
    std_luminance = float(luminance.std())
    mean_luminance = float(luminance.mean())
    row_slice, col_slice = _center_roi(height, width)
    center = luminance[row_slice, col_slice]
    center_mean = float(center.mean()) if center.size else None
    if center_mean is not None and not math.isfinite(center_mean):
        center_mean = None
    center_rgb_mean = rgb_f[row_slice, col_slice].reshape(-1, 3).mean(axis=0)
    target_color = list(expectations["target_color_rgb"])
    target = np.asarray(target_color, dtype=np.float32) * 255.0
    color_l1 = float(np.mean(np.abs(center_rgb_mean - target)))
    color_max_l1 = float(
        expectations.get("target_color_max_l1", RGB_TARGET_COLOR_MAX_L1)
    )
    target_fraction = _color_match_fraction(
        rgb_f, target_color, max_l1=color_max_l1
    )
    wall_color = list(expectations.get("wall_color_rgb") or target_color)
    # Wall absence is an orientation check on the optical center, not the full frame
    # (bright floor/horizon patches can approach white under the key light).
    wall_fraction = _color_match_fraction(
        rgb_f[row_slice, col_slice], wall_color, max_l1=color_max_l1
    )
    bright_min = float(expectations["center_min_mean_luminance"])
    min_std = float(expectations["min_std_luminance"])
    structured_min = float(
        expectations.get(
            "structured_min_mean_luminance", RGB_STRUCTURED_MIN_MEAN_LUMINANCE
        )
    )
    require_bright = bool(expectations.get("require_center_bright", False))
    require_color = bool(expectations.get("require_target_color_presence", False))
    require_structured = bool(expectations.get("require_structured_scene", False))
    require_wall_absent = bool(expectations.get("require_wall_color_absent", False))
    require_horizon = bool(expectations.get("require_horizon_structure", False))
    min_color_fraction = float(expectations.get("target_color_min_fraction", 0.0))
    max_wall_fraction = float(
        expectations.get(
            "wall_color_max_fraction", 1.0 / ORACLE_PROJECTION_SAFETY_DIVISOR
        )
    )
    horizon_delta_min = float(
        expectations.get("horizon_min_band_delta", RGB_HORIZON_MIN_BAND_DELTA)
    )
    upper_band = float(luminance[: max(1, height // 3)].mean())
    lower_band = float(luminance[min(height - 1, (2 * height) // 3) :].mean())
    horizon_delta = abs(lower_band - upper_band)
    # A wall-filling bright target can be nearly uniform (low std). Reject only
    # dark near-blank frames, not bright low-contrast oracle walls.
    center_for_blank = center_mean if center_mean is not None else 0.0
    blank_dark = center_for_blank < bright_min and std_luminance < min_std
    checks = {
        "shape_ok": shape_ok,
        "finite": finite,
    }
    if require_bright:
        checks["center_bright_ok"] = (
            center_mean is not None and center_mean >= bright_min
        )
        checks["not_blank_dark_ok"] = not blank_dark
    if require_structured:
        checks["structured_scene_ok"] = (
            mean_luminance >= structured_min and std_luminance >= min_std
        )
    if require_color:
        checks["target_color_presence_ok"] = target_fraction >= min_color_fraction
    if require_wall_absent:
        checks["wall_color_absent_ok"] = wall_fraction <= max_wall_fraction
    if require_horizon:
        checks["horizon_structure_ok"] = horizon_delta >= horizon_delta_min
    ok = all(checks.values())
    return {
        "ok": ok,
        "checks": checks,
        "view_class": expectations.get("view_class"),
        "shape": [height, width, int(array.shape[2])],
        "expected_resolution_hw": [expected_h, expected_w],
        "mean_luminance": mean_luminance,
        "std_luminance": std_luminance,
        "center_mean_luminance": center_mean,
        "center_rgb_mean": center_rgb_mean.tolist(),
        "target_color_rgb_0_1": target_color,
        "center_to_target_l1": color_l1,
        "target_color_match_fraction": target_fraction,
        "wall_color_match_fraction": wall_fraction,
        "target_color_min_fraction": min_color_fraction,
        "wall_color_max_fraction": max_wall_fraction,
        "horizon_band_delta": horizon_delta,
        "horizon_min_band_delta": horizon_delta_min,
        "visibility": dict(expectations.get("visibility") or {}),
        "failure_reasons": [name for name, passed in checks.items() if not passed],
    }


def evaluate_depth_array(
    depth: Any,
    *,
    expected_resolution_hw: Sequence[int],
    expectations: Mapping[str, Any],
) -> dict[str, Any]:
    array = _as_numpy(depth)
    if array.ndim == 3 and array.shape[2] == 1:
        array = array[:, :, 0]
    if array.ndim != 2:
        raise SensorCaptureError(f"Depth array must be HxW, got shape {array.shape}")
    height, width = int(array.shape[0]), int(array.shape[1])
    expected_h, expected_w = int(expected_resolution_hw[0]), int(expected_resolution_hw[1])
    depth_f = array.astype(np.float32)
    finite_mask = np.isfinite(depth_f) & (depth_f > 0.0) & (depth_f < DEPTH_INVALID_MAX_M)
    finite_fraction = float(finite_mask.mean()) if depth_f.size else 0.0
    finite_count = int(finite_mask.sum())
    row_slice, col_slice = _center_roi(height, width)
    center = depth_f[row_slice, col_slice]
    center_mask = finite_mask[row_slice, col_slice]
    center_values = center[center_mask]
    median_center = float(np.median(center_values)) if center_values.size else None
    if median_center is not None and not math.isfinite(median_center):
        median_center = None
    expected_range = float(expectations["expected_range_m"])
    tolerance = float(expectations["range_tolerance_m"])
    range_ok = (
        median_center is not None
        and abs(median_center - expected_range) <= tolerance
    )
    finite_values = depth_f[finite_mask]
    band_mask = (
        (finite_values >= expected_range - tolerance)
        & (finite_values <= expected_range + tolerance)
        if finite_values.size
        else np.zeros(0, dtype=bool)
    )
    range_band_hits = int(band_mask.sum()) if band_mask.size else 0
    forbidden_range = float(
        expectations.get("forbidden_range_m", expected_range)
    )
    forbidden_tol = float(
        expectations.get("forbidden_range_tolerance_m", tolerance)
    )
    center_hits_forbidden = (
        median_center is not None
        and abs(median_center - forbidden_range) <= forbidden_tol
    )
    require_range = bool(expectations.get("require_center_range", False))
    require_dense = bool(expectations.get("require_dense_finite_fraction", False))
    require_band = bool(expectations.get("require_range_band_hits", False))
    require_open = bool(expectations.get("require_open_finite_fraction", False))
    forbid_wall = bool(expectations.get("forbid_center_front_wall_range", False))
    finite_fraction_min = float(
        expectations.get("finite_fraction_min", DEPTH_FINITE_FRACTION_MIN)
    )
    band_min_hits = int(expectations.get("range_band_min_hits", 0))
    checks = {
        "shape_ok": height == expected_h and width == expected_w,
    }
    if require_dense or require_open:
        checks["finite_fraction_ok"] = finite_fraction >= finite_fraction_min
    if require_range:
        checks["center_range_ok"] = range_ok
    if require_band:
        checks["range_band_hits_ok"] = range_band_hits >= band_min_hits
    if forbid_wall:
        # Null center is acceptable for open/lateral views; a wall-like center is not.
        checks["center_not_front_wall_ok"] = not center_hits_forbidden
    return {
        "ok": all(checks.values()),
        "checks": checks,
        "view_class": expectations.get("view_class"),
        "shape": [height, width],
        "expected_resolution_hw": [expected_h, expected_w],
        "finite_fraction": finite_fraction,
        "finite_count": finite_count,
        "median_center_range_m": median_center,
        "expected_range_m": expected_range,
        "range_tolerance_m": tolerance,
        "range_band_hit_count": range_band_hits,
        "range_band_min_hits": band_min_hits,
        "forbidden_range_m": forbidden_range,
        "visibility": dict(expectations.get("visibility") or {}),
        "failure_reasons": [name for name, passed in checks.items() if not passed],
    }


def evaluate_lidar_scan(
    azimuth_deg: Any,
    elevation_deg: Any,
    range_m: Any,
    *,
    expectations: Mapping[str, Any],
) -> dict[str, Any]:
    azimuth = _as_numpy(azimuth_deg).astype(np.float32).reshape(-1)
    elevation = _as_numpy(elevation_deg).astype(np.float32).reshape(-1)
    ranges = _as_numpy(range_m).astype(np.float32).reshape(-1)
    if not (azimuth.shape == elevation.shape == ranges.shape):
        raise SensorCaptureError(
            f"Lidar channel shape mismatch: az={azimuth.shape}, el={elevation.shape}, "
            f"range={ranges.shape}"
        )
    finite_mask = (
        np.isfinite(azimuth) & np.isfinite(elevation) & np.isfinite(ranges) & (ranges > 0.0)
    )
    point_count = int(azimuth.size)
    finite_count = int(finite_mask.sum())
    bearing = float(expectations["obstacle_bearing_deg"])
    window = float(expectations["bearing_window_deg"])
    delta = np.abs((azimuth - bearing + 180.0) % 360.0 - 180.0)
    in_window = finite_mask & (delta <= window)
    window_ranges = ranges[in_window]
    nearest = float(np.min(window_ranges)) if window_ranges.size else None
    if nearest is not None and not math.isfinite(nearest):
        nearest = None
    expected_range = float(expectations["expected_range_m"])
    tolerance = float(expectations["range_tolerance_m"])
    range_match = (
        nearest is not None and abs(nearest - expected_range) <= tolerance
    )
    require_wall_hit = bool(expectations.get("require_forward_wall_hit", True))
    checks = {
        "point_count_ok": point_count > 0,
        "finite_points_ok": finite_count > 0,
    }
    if require_wall_hit:
        checks["obstacle_direction_hits_ok"] = int(window_ranges.size) > 0
        checks["obstacle_range_ok"] = range_match
    return {
        "ok": all(checks.values()),
        "checks": checks,
        "view_class": expectations.get("view_class"),
        "point_count": point_count,
        "finite_count": finite_count,
        "bearing_window_hit_count": int(window_ranges.size),
        "nearest_range_in_bearing_window_m": nearest,
        "obstacle_bearing_deg": bearing,
        "bearing_window_deg": window,
        "expected_range_m": expected_range,
        "range_tolerance_m": tolerance,
        "require_forward_wall_hit": require_wall_hit,
        "azimuth_deg_min_max": (
            [float(np.min(azimuth)), float(np.max(azimuth))] if point_count else None
        ),
        "visibility": dict(expectations.get("visibility") or {}),
        "failure_reasons": [name for name, passed in checks.items() if not passed],
    }


def evaluate_imu_sample(
    linear_acceleration: Any,
    angular_velocity: Any,
    *,
    expectations: Mapping[str, Any],
    orientation: Any | None = None,
) -> dict[str, Any]:
    accel = _as_numpy(linear_acceleration).astype(np.float32).reshape(-1)
    gyro = _as_numpy(angular_velocity).astype(np.float32).reshape(-1)
    if accel.size != 3 or gyro.size != 3:
        raise SensorCaptureError(
            f"IMU vectors must be length 3, got accel={accel.size}, gyro={gyro.size}"
        )
    finite = bool(np.isfinite(accel).all() and np.isfinite(gyro).all())
    gravity_mag = float(np.linalg.norm(accel))
    expected_g = float(expectations["expected_gravity_magnitude_m_s2"])
    g_tol = float(expectations["gravity_tolerance_m_s2"])
    lateral_tol = float(
        expectations.get(
            "gravity_lateral_tolerance_m_s2", IMU_GRAVITY_LATERAL_TOLERANCE_M_S2
        )
    )
    expected_vector = expectations.get("expected_gravity_vector_m_s2")
    if expected_vector is None:
        expected_vector = [0.0, 0.0, -expected_g]
    expected_vector_arr = np.asarray(expected_vector, dtype=np.float32).reshape(3)
    gyro_max = float(np.max(np.abs(gyro)))
    gravity_vector_ok = bool(
        finite
        and abs(float(accel[0]) - float(expected_vector_arr[0])) <= lateral_tol
        and abs(float(accel[1]) - float(expected_vector_arr[1])) <= lateral_tol
        and abs(float(accel[2]) - float(expected_vector_arr[2])) <= g_tol
    )
    gravity_sign_ok = bool(finite and float(accel[2]) < 0.0)
    checks = {
        "finite": finite,
        "gravity_magnitude_ok": abs(gravity_mag - expected_g) <= g_tol,
        "gravity_vector_sign_ok": gravity_sign_ok,
        "gravity_vector_ok": gravity_vector_ok,
        "angular_velocity_near_zero_ok": gyro_max
        <= float(expectations["angular_velocity_max_rad_s"]),
    }
    result = {
        "ok": all(checks.values()),
        "checks": checks,
        "view_class": expectations.get("view_class", VIEW_IMU_BODY),
        "linear_acceleration_m_s2": accel.tolist(),
        "angular_velocity_rad_s": gyro.tolist(),
        "gravity_magnitude_m_s2": gravity_mag,
        "expected_gravity_magnitude_m_s2": expected_g,
        "expected_gravity_vector_m_s2": expected_vector_arr.tolist(),
        "gravity_tolerance_m_s2": g_tol,
        "gravity_lateral_tolerance_m_s2": lateral_tol,
        "angular_velocity_max_abs_rad_s": gyro_max,
        "frame_convention": dict(expectations.get("frame_convention") or {}),
        "read_gravity": bool(expectations.get("read_gravity", True)),
        "visibility": dict(expectations.get("visibility") or {}),
        "failure_reasons": [name for name, passed in checks.items() if not passed],
    }
    if orientation is not None:
        quat = _as_numpy(orientation).astype(np.float32).reshape(-1)
        result["orientation_wxyz"] = quat.tolist()
        result["checks"]["orientation_finite"] = bool(
            quat.size == 4 and np.isfinite(quat).all()
        )
        result["ok"] = all(result["checks"].values())
        result["failure_reasons"] = [
            name for name, passed in result["checks"].items() if not passed
        ]
    return result


def compact_sensor_metrics(result: Mapping[str, Any]) -> dict[str, Any]:
    """Extract a compact per-sensor summary for INDEX / rollup reports."""
    output_class = result.get("output_class") or result.get("annotator")
    metrics: dict[str, Any] = {
        "logical_name": result.get("logical_name"),
        "output_class": result.get("output_class"),
        "view_class": result.get("view_class"),
        "ok": bool(result.get("ok")),
        "visibility": dict(result.get("visibility") or {}),
        "failure_reasons": list(result.get("failure_reasons") or []),
    }
    if "shape" in result:
        metrics["shape"] = result["shape"]
    if "center_mean_luminance" in result:
        metrics["center_mean_luminance"] = result["center_mean_luminance"]
        metrics["mean_luminance"] = result.get("mean_luminance")
        metrics["std_luminance"] = result.get("std_luminance")
        if "target_color_match_fraction" in result:
            metrics["target_color_match_fraction"] = result["target_color_match_fraction"]
        if "wall_color_match_fraction" in result:
            metrics["wall_color_match_fraction"] = result["wall_color_match_fraction"]
        if "horizon_band_delta" in result:
            metrics["horizon_band_delta"] = result["horizon_band_delta"]
    if "median_center_range_m" in result:
        metrics["median_center_range_m"] = result["median_center_range_m"]
        metrics["finite_fraction"] = result.get("finite_fraction")
        if "range_band_hit_count" in result:
            metrics["range_band_hit_count"] = result["range_band_hit_count"]
            metrics["range_band_min_hits"] = result.get("range_band_min_hits")
        if "expected_range_m" in result:
            metrics["expected_range_m"] = result["expected_range_m"]
    if "point_count" in result:
        metrics["point_count"] = result["point_count"]
        metrics["nearest_range_in_bearing_window_m"] = result.get(
            "nearest_range_in_bearing_window_m"
        )
        metrics["finite_count"] = result.get("finite_count")
    if "gravity_magnitude_m_s2" in result:
        metrics["gravity_magnitude_m_s2"] = result["gravity_magnitude_m_s2"]
        metrics["angular_velocity_max_abs_rad_s"] = result.get(
            "angular_velocity_max_abs_rad_s"
        )
        metrics["linear_acceleration_m_s2"] = result.get("linear_acceleration_m_s2")
        metrics["expected_gravity_vector_m_s2"] = result.get(
            "expected_gravity_vector_m_s2"
        )
        checks = result.get("checks") or {}
        if "gravity_vector_sign_ok" in checks:
            metrics["gravity_vector_sign_ok"] = bool(checks["gravity_vector_sign_ok"])
        if "gravity_vector_ok" in checks:
            metrics["gravity_vector_ok"] = bool(checks["gravity_vector_ok"])
    if output_class:
        metrics["annotator"] = result.get("annotator")
    return metrics


def summarize_capture_results(
    device_results: Mapping[str, Mapping[str, Any]],
    *,
    declared_logical_names: Sequence[str],
    selected_logical_names: Sequence[str],
) -> dict[str, Any]:
    """Summarize a capture run with explicit selection vs declared coverage.

    Top-level ``ok`` is true only for a complete declared-profile pass. A selected
    batch that succeeds on its filter sets ``selection_ok`` without implying
    full-profile completeness.
    """
    declared = [str(name) for name in declared_logical_names]
    selected = [str(name) for name in selected_logical_names]
    declared_set = set(declared)
    selected_set = set(selected)
    captured_set = {str(name) for name in device_results}

    selection_invalid = sorted(selected_set - declared_set)
    duplicate_selected = sorted(
        {name for name in selected if selected.count(name) > 1}
    )
    missing_selected = sorted(selected_set - captured_set)
    unexpected_captured = sorted(captured_set - selected_set)
    unselected_declared = sorted(declared_set - selected_set)

    devices_semantically_ok = bool(device_results) and all(
        bool(result.get("ok")) for result in device_results.values()
    )
    selection_ok = (
        not selection_invalid
        and not duplicate_selected
        and not missing_selected
        and not unexpected_captured
        and len(selected_set) > 0
        and devices_semantically_ok
    )
    full_profile_ok = selection_ok and selected_set == declared_set and not unselected_declared
    return {
        # Never treat a partial selection as a full-profile green.
        "ok": full_profile_ok,
        "selection_ok": selection_ok,
        "full_profile_ok": full_profile_ok,
        "declared_device_count": len(declared_set),
        "selected_device_count": len(selected_set),
        "captured_device_count": len(captured_set),
        "declared_logical_names": sorted(declared_set),
        "selected_logical_names": sorted(selected_set),
        "captured_logical_names": sorted(captured_set),
        "unselected_declared_logical_names": unselected_declared,
        "missing_selected": missing_selected,
        "unexpected_captured": unexpected_captured,
        "invalid_selected": selection_invalid,
        "duplicate_selected": duplicate_selected,
        "device_count": len(device_results),
        "device_ok": {
            name: bool(result.get("ok")) for name, result in device_results.items()
        },
        "failure_reasons": {
            name: list(result.get("failure_reasons") or [])
            for name, result in device_results.items()
            if not result.get("ok")
        },
        "per_sensor": {
            name: compact_sensor_metrics(result) for name, result in device_results.items()
        },
    }


def merge_capture_batch_summaries(
    batches: Sequence[Mapping[str, Any]],
    *,
    declared_logical_names: Sequence[str],
    model: str | None = None,
) -> dict[str, Any]:
    """Authoritative merge: exact disjoint coverage of every declared sensor."""
    declared = [str(name) for name in declared_logical_names]
    declared_set = set(declared)
    if not declared_set:
        raise SensorCaptureError("declared_logical_names must be non-empty")
    if not batches:
        raise SensorCaptureError("merge requires at least one batch summary")

    covered: dict[str, str] = {}
    overlaps: list[dict[str, Any]] = []
    batch_failures: list[dict[str, Any]] = []
    per_sensor: dict[str, Any] = {}
    device_ok: dict[str, bool] = {}
    batch_reports: list[dict[str, Any]] = []

    for index, batch in enumerate(batches):
        batch_model = batch.get("model")
        if model is not None and batch_model is not None and batch_model != model:
            batch_failures.append(
                {
                    "batch_index": index,
                    "reason": "model_mismatch",
                    "expected_model": model,
                    "actual_model": batch_model,
                }
            )
        selected = [
            str(name)
            for name in (
                batch.get("selected_logical_names")
                or batch.get("summary", {}).get("selected_logical_names")
                or []
            )
        ]
        if not selected:
            selected = sorted(
                {
                    str(name)
                    for name in (
                        batch.get("captured_logical_names")
                        or batch.get("per_sensor")
                        or batch.get("device_ok")
                        or {}
                    )
                }
            )
        selected_set = set(selected)
        selection_ok = bool(
            batch.get(
                "selection_ok",
                batch.get("summary", {}).get("selection_ok", False),
            )
        )
        batch_device_ok = dict(
            batch.get("device_ok")
            or batch.get("summary", {}).get("device_ok")
            or {
                name: bool((batch.get("per_sensor") or {}).get(name, {}).get("ok"))
                for name in selected
            }
        )
        batch_per_sensor = dict(
            batch.get("per_sensor") or batch.get("summary", {}).get("per_sensor") or {}
        )
        if not selection_ok:
            batch_failures.append(
                {
                    "batch_index": index,
                    "reason": "selection_not_ok",
                    "selected_logical_names": sorted(selected_set),
                }
            )
        invalid = sorted(selected_set - declared_set)
        if invalid:
            batch_failures.append(
                {
                    "batch_index": index,
                    "reason": "selected_outside_declared",
                    "invalid_selected": invalid,
                }
            )
        for name in sorted(selected_set):
            if name in covered:
                overlaps.append(
                    {
                        "logical_name": name,
                        "first_batch_index": covered[name],
                        "second_batch_index": index,
                    }
                )
            else:
                covered[name] = index
            ok = bool(batch_device_ok.get(name, False))
            device_ok[name] = ok
            if name in batch_per_sensor:
                per_sensor[name] = dict(batch_per_sensor[name])
            elif name not in per_sensor:
                per_sensor[name] = {
                    "logical_name": name,
                    "ok": ok,
                    "failure_reasons": [] if ok else ["missing_per_sensor_metrics"],
                }
        batch_reports.append(
            {
                "batch_index": index,
                "model": batch_model,
                "selected_device_count": len(selected_set),
                "selected_logical_names": sorted(selected_set),
                "selection_ok": selection_ok,
            }
        )

    missing = sorted(declared_set - set(covered))
    extra = sorted(set(covered) - declared_set)
    semantic_failures = sorted(
        name for name, ok in device_ok.items() if name in declared_set and not ok
    )
    exact_coverage = not missing and not extra and not overlaps
    ok = (
        exact_coverage
        and not batch_failures
        and not semantic_failures
        and len(device_ok) == len(declared_set)
        and all(device_ok[name] for name in declared_set)
    )
    return {
        "ok": ok,
        "full_profile_ok": ok,
        "selection_ok": ok,
        "model": model,
        "declared_device_count": len(declared_set),
        "merged_device_count": len(covered),
        "declared_logical_names": sorted(declared_set),
        "merged_logical_names": sorted(covered),
        "missing_declared": missing,
        "extra_merged": extra,
        "overlaps": overlaps,
        "batch_failures": batch_failures,
        "semantic_failures": semantic_failures,
        "exact_disjoint_coverage": exact_coverage and not batch_failures,
        "batches": batch_reports,
        "device_ok": {name: bool(device_ok.get(name)) for name in sorted(declared_set)},
        "per_sensor": {
            name: per_sensor.get(name, {"logical_name": name, "ok": False})
            for name in sorted(declared_set)
        },
        "failure_reasons": {
            "missing_declared": missing,
            "extra_merged": extra,
            "overlaps": overlaps,
            "batch_failures": batch_failures,
            "semantic_failures": semantic_failures,
        },
    }


def capture_exit_code(*, selection_ok: bool, full_profile_ok: bool) -> int:
    if full_profile_ok:
        return CAPTURE_EXIT_FULL_PROFILE_OK
    if selection_ok:
        return CAPTURE_EXIT_SELECTION_OK_INCOMPLETE
    return CAPTURE_EXIT_SELECTION_FAILED


