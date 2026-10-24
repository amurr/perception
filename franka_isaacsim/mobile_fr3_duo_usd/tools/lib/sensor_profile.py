"""Load and validate declarative Isaac-native sensor profiles."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

from .paths import SENSOR_PROFILES_DIR, SENSOR_PROFILE_SCHEMA_PATH


SENSOR_BEARING_MODELS = (
    "franka_vision_and_manipulation_kit",
    "franka_mobile_sensors",
    "mobile_fr3_duo_with_vision_and_manipulation_kit",
)


class SensorProfileError(ValueError):
    """Raised when a sensor profile fails schema or layered validation."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise SensorProfileError(message)


def _validate_layered_invariants(profile: Mapping[str, Any]) -> None:
    devices = profile.get("devices") or []
    logical_names = [device.get("logical_name") for device in devices]
    _require(
        len(logical_names) == len(set(logical_names)),
        f"{profile.get('profile_name')}: device logical_name values must be unique",
    )
    suffixes = [device.get("frame_name_suffix") for device in devices]
    _require(
        all(isinstance(suffix, str) and suffix for suffix in suffixes),
        f"{profile.get('profile_name')}: every device needs a frame_name_suffix",
    )
    for device in devices:
        outputs = set(device.get("outputs") or [])
        device_class = device.get("device_class")
        if device_class == "imu":
            _require(
                outputs <= {"linear_acceleration", "angular_velocity"},
                f"{device.get('logical_name')}: IMU outputs must be acceleration/velocity",
            )
        elif device_class == "lidar_2d":
            _require(
                outputs <= {"point_cloud", "range_scan"},
                f"{device.get('logical_name')}: lidar outputs must be point_cloud/range_scan",
            )
        elif device_class in {
            "stereo_rgb_camera",
            "rgb_camera",
            "depth_camera",
            "rgb_depth_camera",
        }:
            _require(
                outputs <= {"rgb", "depth"},
                f"{device.get('logical_name')}: camera outputs must be rgb/depth",
            )
        calibration = device.get("simulation_calibration") or {}
        if profile.get("calibration_policy") == "synthetic_simulation":
            _require(
                calibration.get("source") == "synthetic_simulation",
                f"{device.get('logical_name')}: synthetic profiles require "
                "simulation_calibration.source=synthetic_simulation",
            )


def validate_sensor_profile(
    profile: Mapping[str, Any],
    schema_path: Path | None = None,
) -> dict[str, Any]:
    schema_file = schema_path or SENSOR_PROFILE_SCHEMA_PATH
    if not schema_file.is_file():
        raise SensorProfileError(f"Schema file missing: {schema_file}")
    try:
        import jsonschema
    except ImportError as error:
        raise SensorProfileError(
            "Strict schema validation requires the 'jsonschema' package "
            "(see isaacsim6/requirements-ci.txt)."
        ) from error
    try:
        schema = json.loads(schema_file.read_text(encoding="utf-8"))
        jsonschema.validate(instance=profile, schema=schema)
    except Exception as error:
        raise SensorProfileError(f"Schema validation failed: {error}") from error
    _validate_layered_invariants(profile)
    return dict(profile)


def _load_raw_profile(profile_name: str, profiles_dir: Path) -> dict[str, Any]:
    path = profiles_dir / f"{profile_name}.json"
    if not path.is_file():
        raise SensorProfileError(f"Sensor profile not found: {path}")
    profile = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(profile, dict):
        raise SensorProfileError(f"Sensor profile root must be an object: {path}")
    if profile.get("profile_name") != profile_name:
        raise SensorProfileError(
            f"profile_name {profile.get('profile_name')!r} != {profile_name!r}"
        )
    return profile


def _merge_devices(
    base_devices: list[dict[str, Any]],
    extra_devices: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    merged: dict[str, dict[str, Any]] = {}
    for device in [*base_devices, *extra_devices]:
        name = device["logical_name"]
        if name in merged and merged[name] != device:
            raise SensorProfileError(
                f"Conflicting device definitions for logical_name {name!r}"
            )
        merged[name] = device
    return list(merged.values())


def load_sensor_profile(
    profile_name: str,
    *,
    profiles_dir: Path | None = None,
    schema_path: Path | None = None,
    _stack: tuple[str, ...] = (),
) -> dict[str, Any]:
    root = profiles_dir or SENSOR_PROFILES_DIR
    if profile_name in _stack:
        raise SensorProfileError(
            f"Sensor profile include cycle: {' -> '.join([*_stack, profile_name])}"
        )
    profile = _load_raw_profile(profile_name, root)
    devices = list(profile.get("devices") or [])
    for included in profile.get("includes") or []:
        included_profile = load_sensor_profile(
            included,
            profiles_dir=root,
            schema_path=schema_path,
            _stack=(*_stack, profile_name),
        )
        devices = _merge_devices(devices, list(included_profile.get("devices") or []))
    resolved = dict(profile)
    resolved["devices"] = devices
    if "includes" in resolved:
        # Keep includes for provenance; validation runs on the merged device list.
        pass
    return validate_sensor_profile(resolved, schema_path=schema_path)


def list_sensor_profiles(profiles_dir: Path | None = None) -> list[str]:
    root = profiles_dir or SENSOR_PROFILES_DIR
    return sorted(path.stem for path in root.glob("*.json"))


def match_frame_name(frame_names: list[str], frame_name_suffix: str) -> list[str]:
    """Return frame names whose exact suffix matches the profile suffix."""
    matches = [
        name for name in frame_names if name == frame_name_suffix or name.endswith(frame_name_suffix)
    ]
    # Prefer exact equality when present.
    exact = [name for name in matches if name == frame_name_suffix]
    return exact or matches
