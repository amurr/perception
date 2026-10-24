"""Reusable native Isaac sensor bootstrap from declarative profiles.

Offline helpers resolve frame suffixes, emit deterministic bootstrap configs, and
build sensorized world packaging files. Isaac runtime attachment lives behind
optional imports so ordinary CI stays offline.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping, Sequence

from .paths import SENSOR_PROFILE_SCHEMA_PATH, SENSOR_PROFILES_DIR
from .sensor_profile import (
    SENSOR_BEARING_MODELS,
    SensorProfileError,
    load_sensor_profile,
    match_frame_name,
)
from .sensor_test_world import (
    TARGET_MANIFEST_RELATIVE,
    build_sensor_test_world_usda,
    build_target_manifest,
    load_sensor_test_world_config,
)


NATIVE_SENSOR_CHILD_NAME = "NativeSensor"
SENSORIZED_WORLD_ENTRYPOINT_RELATIVE = "worlds/sensorized_world.usda"
BOOTSTRAP_CONFIG_RELATIVE = "worlds/native_sensor_bootstrap.json"
BOOTSTRAP_SCHEMA_VERSION = 1
ARCHIVE_TOOLS_ROOT_RELATIVE = "tools"
ARCHIVE_PROFILE_RELATIVE_PREFIX = "sensors"
UNION_SENSOR_MODEL = "mobile_fr3_duo_with_vision_and_manipulation_kit"

# Curated archive-local capture/merge tooling (paths relative to isaacsim6/).
_ARCHIVE_TOOLING_RELATIVE_FILES = (
    "bin/run_isaac_script.sh",
    "bin/teardown_isaac.sh",
    "bin/verify_teardown.py",
    "bin/write_process_failure_capsule.py",
    "scripts/bootstrap_native_sensors.py",
    "scripts/capture_native_sensors.py",
    "scripts/merge_native_sensor_captures.py",
    "lib/__init__.py",
    "lib/bootstrap.py",
    "lib/logging_util.py",
    "lib/paths.py",
    "lib/process_failure_capsule.py",
    "lib/sensor_bootstrap.py",
    "lib/sensor_capture.py",
    "lib/sensor_profile.py",
    "lib/sensor_test_world.py",
    "lib/strict_json.py",
    "lib/teardown.py",
    "schema/native_sensor_bootstrap.schema.json",
    "schema/sensor_profile.schema.json",
    "schema/sensor_test_world.schema.json",
    "worlds/sensor_test_world.json",
    "requirements-ci.txt",
)

# ROS optical frames look along +Z (Y down). USD Camera looks along -Z (Y up).
# 180° about X maps optical +Z → USD -Z and optical +Y → USD +Y.
ROS_OPTICAL_TO_USD_CAMERA_QUAT_WXYZ = (0.0, 1.0, 0.0, 0.0)

# Synthetic 2D scan config from the Isaac RTX lidar registry (not a hardware USD).
DEFAULT_LIDAR_2D_CONFIG = "Example_Rotary_2D"

CAMERA_DEVICE_CLASSES = frozenset(
    {
        "stereo_rgb_camera",
        "rgb_camera",
        "depth_camera",
        "rgb_depth_camera",
    }
)
LIDAR_DEVICE_CLASSES = frozenset({"lidar_2d"})
IMU_DEVICE_CLASSES = frozenset({"imu"})

class SensorBootstrapError(RuntimeError):
    """Raised when native sensor bootstrap config or attachment fails."""


def is_sensor_bearing_model(model: str) -> bool:
    return model in SENSOR_BEARING_MODELS


def expected_device_count(profile: Mapping[str, Any]) -> int:
    return len(list(profile.get("devices") or []))


def count_devices_by_class(profile: Mapping[str, Any]) -> dict[str, int]:
    counts = {"camera": 0, "lidar_2d": 0, "imu": 0}
    for device in profile.get("devices") or []:
        device_class = device.get("device_class")
        if device_class in CAMERA_DEVICE_CLASSES:
            counts["camera"] += 1
        elif device_class in LIDAR_DEVICE_CLASSES:
            counts["lidar_2d"] += 1
        elif device_class in IMU_DEVICE_CLASSES:
            counts["imu"] += 1
        else:
            raise SensorBootstrapError(
                f"Unsupported device_class for bootstrap: {device_class!r}"
            )
    return counts


def collect_prim_leaf_names(stage: Any) -> list[str]:
    """Return every prim leaf name on an opened USD stage."""
    names: list[str] = []
    for prim in stage.Traverse():
        if not prim.IsValid():
            continue
        names.append(prim.GetName())
    return names


def collect_prim_paths_by_leaf_name(stage: Any) -> dict[str, list[str]]:
    """Map leaf prim name → full prim path(s)."""
    mapping: dict[str, list[str]] = {}
    for prim in stage.Traverse():
        if not prim.IsValid():
            continue
        mapping.setdefault(prim.GetName(), []).append(str(prim.GetPath()))
    return mapping


def resolve_unique_frame_path(
    frame_name_suffix: str,
    *,
    leaf_to_paths: Mapping[str, Sequence[str]],
) -> str:
    """Resolve a profile frame suffix to exactly one prim path."""
    leaf_names = list(leaf_to_paths.keys())
    matches = match_frame_name(leaf_names, frame_name_suffix)
    if not matches:
        raise SensorBootstrapError(
            f"No prim leaf matches frame_name_suffix {frame_name_suffix!r}"
        )
    paths: list[str] = []
    for leaf in matches:
        paths.extend(leaf_to_paths.get(leaf) or [])
    # Prefer exact leaf equality when present among matched leaves.
    exact_paths = [
        path
        for leaf in matches
        if leaf == frame_name_suffix
        for path in (leaf_to_paths.get(leaf) or [])
    ]
    candidate_paths = exact_paths or paths
    unique = sorted(set(candidate_paths))
    if not unique:
        raise SensorBootstrapError(
            f"No prim path for frame_name_suffix {frame_name_suffix!r}"
        )
    if len(unique) > 1:
        raise SensorBootstrapError(
            f"Ambiguous frame_name_suffix {frame_name_suffix!r}: {unique}"
        )
    return unique[0]


def native_sensor_prim_path(frame_path: str, logical_name: str | None = None) -> str:
    """Deterministic child path for an active native sensor under a robot frame.

    Multiple logical devices may share one frame (e.g. ZED left RGB + depth on the
    left optical frame); the child prim name is then ``NativeSensor__<logical>``.
    """
    if logical_name:
        safe = "".join(
            ch if ch.isalnum() or ch in {"_", "-"} else "_" for ch in logical_name
        )
        return f"{frame_path.rstrip('/')}/{NATIVE_SENSOR_CHILD_NAME}__{safe}"
    return f"{frame_path.rstrip('/')}/{NATIVE_SENSOR_CHILD_NAME}"


def is_native_sensor_prim_name(name: str) -> bool:
    return name == NATIVE_SENSOR_CHILD_NAME or name.startswith(
        f"{NATIVE_SENSOR_CHILD_NAME}__"
    )


def resolve_profile_frame_bindings(
    profile: Mapping[str, Any],
    *,
    leaf_to_paths: Mapping[str, Sequence[str]],
) -> list[dict[str, Any]]:
    """Bind every profile device to a resolved frame path (fail on miss/ambiguity).

    Frame paths may be shared by multiple logical devices; sensor prim paths remain
    unique via ``native_sensor_prim_path(..., logical_name)``.
    """
    bindings: list[dict[str, Any]] = []
    seen_sensor_paths: dict[str, str] = {}
    for device in profile.get("devices") or []:
        suffix = device["frame_name_suffix"]
        frame_path = resolve_unique_frame_path(suffix, leaf_to_paths=leaf_to_paths)
        logical_name = device["logical_name"]
        sensor_prim_path = native_sensor_prim_path(frame_path, logical_name)
        prior = seen_sensor_paths.get(sensor_prim_path)
        if prior is not None:
            raise SensorBootstrapError(
                f"Sensor prim {sensor_prim_path!r} claimed by both {prior!r} "
                f"and {logical_name!r}"
            )
        seen_sensor_paths[sensor_prim_path] = logical_name
        bindings.append(
            {
                "logical_name": logical_name,
                "device_class": device["device_class"],
                "frame_name_suffix": suffix,
                "frame_path": frame_path,
                "sensor_prim_path": sensor_prim_path,
                "outputs": list(device.get("outputs") or []),
                "simulation_calibration": dict(device.get("simulation_calibration") or {}),
            }
        )
    return bindings


def count_active_native_sensors(stage: Any, *, root_path: str | None = None) -> dict[str, int]:
    """Count authored native sensor prims (geometry/Xform frames never count)."""
    counts = {"camera": 0, "lidar_2d": 0, "imu": 0, "total": 0}
    for prim in stage.Traverse():
        if not prim.IsValid():
            continue
        path = str(prim.GetPath())
        if root_path is not None and not (
            path == root_path or path.startswith(root_path.rstrip("/") + "/")
        ):
            continue
        if not is_native_sensor_prim_name(prim.GetName()):
            continue
        type_name = prim.GetTypeName()
        if type_name == "Camera":
            counts["camera"] += 1
        elif type_name == "OmniLidar":
            counts["lidar_2d"] += 1
        elif type_name == "IsaacImuSensor":
            counts["imu"] += 1
        else:
            continue
        counts["total"] += 1
    return counts


def build_native_sensor_bootstrap_config(
    model: str,
    *,
    profiles_dir: Path | None = None,
    schema_path: Path | None = None,
) -> dict[str, Any]:
    """Deterministic bootstrap config packaged beside the sensorized world."""
    if not is_sensor_bearing_model(model):
        raise SensorBootstrapError(
            f"Native sensor bootstrap is only for sensor-bearing models, got {model!r}"
        )
    profile = load_sensor_profile(
        model,
        profiles_dir=profiles_dir or SENSOR_PROFILES_DIR,
        schema_path=schema_path or SENSOR_PROFILE_SCHEMA_PATH,
    )
    by_class = count_devices_by_class(profile)
    return {
        "schema_version": BOOTSTRAP_SCHEMA_VERSION,
        "model_name": model,
        "profile_name": profile["profile_name"],
        "entrypoint_relative": SENSORIZED_WORLD_ENTRYPOINT_RELATIVE,
        "robot_reference_prim": "/World/Robot",
        "native_sensor_child_name": NATIVE_SENSOR_CHILD_NAME,
        "calibration_policy": profile["calibration_policy"],
        "expected_device_count": expected_device_count(profile),
        "expected_by_class": by_class,
        "devices": [
            {
                "logical_name": device["logical_name"],
                "device_class": device["device_class"],
                "frame_name_suffix": device["frame_name_suffix"],
                "outputs": list(device.get("outputs") or []),
            }
            for device in profile["devices"]
        ],
        "invocation": {
            "script": "bootstrap_native_sensors.py",
            "notes": (
                "Open worlds/sensorized_world.usda (or pass --usd), then run "
                "isaacsim6/bin/run_isaac_script.sh bootstrap_native_sensors.py "
                "--model <model> to attach Camera/OmniLidar/IsaacImuSensor children. "
                "Geometry-only frames are never counted as active sensors."
            ),
        },
    }


def validate_native_sensor_bootstrap_config(config: Mapping[str, Any]) -> dict[str, Any]:
    required = (
        "schema_version",
        "model_name",
        "profile_name",
        "entrypoint_relative",
        "expected_device_count",
        "expected_by_class",
        "devices",
        "native_sensor_child_name",
    )
    for key in required:
        if key not in config:
            raise SensorBootstrapError(f"Bootstrap config missing {key!r}")
    if config["schema_version"] != BOOTSTRAP_SCHEMA_VERSION:
        raise SensorBootstrapError(
            f"Unsupported bootstrap schema_version: {config['schema_version']!r}"
        )
    if config["native_sensor_child_name"] != NATIVE_SENSOR_CHILD_NAME:
        raise SensorBootstrapError(
            f"native_sensor_child_name must be {NATIVE_SENSOR_CHILD_NAME!r}"
        )
    if not is_sensor_bearing_model(str(config["model_name"])):
        raise SensorBootstrapError(
            f"Bootstrap model is not sensor-bearing: {config['model_name']!r}"
        )
    if int(config["expected_device_count"]) != len(config["devices"]):
        raise SensorBootstrapError(
            "expected_device_count does not match devices[] length"
        )
    return dict(config)


def _profile_names_for_archive(model: str) -> tuple[str, ...]:
    if model == UNION_SENSOR_MODEL:
        return (
            "franka_vision_and_manipulation_kit",
            "franka_mobile_sensors",
            UNION_SENSOR_MODEL,
        )
    return (model,)


def build_sensor_workflow_files(
    model: str,
    *,
    tooling_root: Path | None = None,
    profiles_dir: Path | None = None,
) -> dict[str, bytes]:
    """Archive-local profile copies + capture/merge tooling tree for one family."""
    if not is_sensor_bearing_model(model):
        raise SensorBootstrapError(
            f"Sensor workflow packaging is only for sensor-bearing models, got {model!r}"
        )
    from .paths import TOOLING_ROOT

    root = Path(tooling_root or TOOLING_ROOT).resolve()
    source_profiles = Path(profiles_dir or SENSOR_PROFILES_DIR).resolve()
    files: dict[str, bytes] = {}

    for profile_name in _profile_names_for_archive(model):
        profile_path = source_profiles / f"{profile_name}.json"
        if not profile_path.is_file():
            raise SensorBootstrapError(f"Missing sensor profile for archive: {profile_path}")
        payload = profile_path.read_bytes()
        files[f"{ARCHIVE_PROFILE_RELATIVE_PREFIX}/{profile_name}.json"] = payload
        files[f"{ARCHIVE_TOOLS_ROOT_RELATIVE}/sensors/{profile_name}.json"] = payload

    for relative in _ARCHIVE_TOOLING_RELATIVE_FILES:
        source = root / relative
        if not source.is_file():
            raise SensorBootstrapError(
                f"Missing archive tooling file for sensor workflow: {source}"
            )
        files[f"{ARCHIVE_TOOLS_ROOT_RELATIVE}/{relative}"] = source.read_bytes()

    tools_readme = "\n".join(
        [
            f"Archive-local Isaac sensor workflow tooling for {model}.",
            "Set ISAACSIM_PATH to Isaac Sim 6.0.0-rc.59, then from this tools/ directory:",
            "  export ISAACSIM6_TOOLING_ROOT=\"$PWD\"",
            "  ./bin/run_isaac_script.sh capture_native_sensors.py \\",
            f"    --model {model} --usd ../{SENSORIZED_WORLD_ENTRYPOINT_RELATIVE} \\",
            "    --evidence-dir /tmp/native_capture --report /tmp/native_capture/capture_report.json",
            "Merge requires Isaac python.sh (numpy). See the package README.txt.",
            "",
        ]
    )
    files[f"{ARCHIVE_TOOLS_ROOT_RELATIVE}/README.txt"] = tools_readme.encode("utf-8")
    return dict(sorted(files.items()))


def build_sensorized_world_files(
    model: str,
    *,
    world_config: Mapping[str, Any] | None = None,
    profiles_dir: Path | None = None,
) -> dict[str, bytes]:
    """Archive-relative sensorized world + bootstrap config for one family."""
    if not is_sensor_bearing_model(model):
        raise SensorBootstrapError(
            f"Sensorized world is only for sensor-bearing models, got {model!r}"
        )
    world = world_config if world_config is not None else load_sensor_test_world_config()
    usda = build_sensor_test_world_usda(model, world)
    # Retarget the doc line so the sensorized entrypoint is distinct in text.
    usda = usda.replace(
        "Mobile sensor-test world with collision floor, neutral lighting,",
        "Sensorized world entrypoint with collision floor, neutral lighting,",
        1,
    )
    usda = usda.replace(
        " Visual/geometric targets only; not a sensor-output proof.\"\"\"",
        " Activate native sensors via bootstrap_native_sensors.py;"
        " visual/geometric targets only until bootstrap runs.\"\"\"",
        1,
    )
    manifest = build_target_manifest(model, world)
    bootstrap = validate_native_sensor_bootstrap_config(
        build_native_sensor_bootstrap_config(model, profiles_dir=profiles_dir)
    )
    return {
        SENSORIZED_WORLD_ENTRYPOINT_RELATIVE: usda.encode("utf-8"),
        TARGET_MANIFEST_RELATIVE: (
            json.dumps(manifest, indent=2, sort_keys=False) + "\n"
        ).encode("utf-8"),
        BOOTSTRAP_CONFIG_RELATIVE: (
            json.dumps(bootstrap, indent=2, sort_keys=False) + "\n"
        ).encode("utf-8"),
    }


def _focal_length_mm_from_calibration(calibration: Mapping[str, Any]) -> float | None:
    resolution = calibration.get("resolution_hw")
    focal_px = calibration.get("focal_length_px")
    if not resolution or not focal_px:
        return None
    _height, width = int(resolution[0]), int(resolution[1])
    fx = float(focal_px[0])
    if width <= 0 or fx <= 0:
        return None
    # Horizontal aperture default used by Isaac Camera (mm).
    horizontal_aperture_mm = 20.955
    return horizontal_aperture_mm * fx / float(width)


def attach_native_sensors_from_profile(
    stage: Any,
    profile: Mapping[str, Any],
    *,
    activate_runtime: bool = True,
    logical_names: Sequence[str] | None = None,
    enforce_full_profile_counts: bool = True,
) -> dict[str, Any]:
    """Create native sensor children for profile devices on an open stage.

    Requires Isaac Sim APIs. Geometry-only frames never count as active sensors;
    only authored Camera / OmniLidar / IsaacImuSensor children do.

    When ``logical_names`` is set, only those devices are authored/activated.
    Full-profile count checks then require ``enforce_full_profile_counts=False``.
    """
    try:
        from isaacsim.sensors.experimental.physics import IMU, IMUSensor
        from isaacsim.sensors.experimental.rtx import (
            CameraSensor,
            Lidar,
            LidarSensor,
            RtxCamera,
        )
    except ImportError as error:
        raise SensorBootstrapError(
            "Isaac sensor APIs unavailable; run under Isaac python.sh"
        ) from error

    leaf_to_paths = collect_prim_paths_by_leaf_name(stage)
    bindings = resolve_profile_frame_bindings(profile, leaf_to_paths=leaf_to_paths)
    if logical_names is not None:
        wanted = set(logical_names)
        missing = sorted(wanted - {binding["logical_name"] for binding in bindings})
        if missing:
            raise SensorBootstrapError(
                f"Unknown logical_names for attach filter: {missing}"
            )
        bindings = [binding for binding in bindings if binding["logical_name"] in wanted]
        if not bindings:
            raise SensorBootstrapError("logical_names filter selected no devices")

    attached: list[dict[str, Any]] = []
    runtime_handles: list[Any] = []
    runtime_by_name: dict[str, Any] = {}

    for binding in bindings:
        device_class = binding["device_class"]
        sensor_path = binding["sensor_prim_path"]
        calibration = binding["simulation_calibration"]
        outputs = set(binding["outputs"])
        logical_name = binding["logical_name"]
        record: dict[str, Any] = {
            "logical_name": logical_name,
            "device_class": device_class,
            "frame_path": binding["frame_path"],
            "sensor_prim_path": sensor_path,
            "outputs": list(binding["outputs"]),
            "annotators": [],
            "active": False,
        }

        if device_class in CAMERA_DEVICE_CLASSES:
            tick_rate = float(calibration.get("sample_rate_hz") or 30.0)
            camera = RtxCamera(
                sensor_path,
                tick_rate=tick_rate,
                translations=[[0.0, 0.0, 0.0]],
                reset_xform_op_properties=True,
            )
            # Local (not world) pose: keep translation at the optical origin and
            # rotate into the USD Camera look axis.
            camera.set_local_poses(
                translations=[[0.0, 0.0, 0.0]],
                orientations=[list(ROS_OPTICAL_TO_USD_CAMERA_QUAT_WXYZ)],
            )
            focal_mm = _focal_length_mm_from_calibration(calibration)
            if focal_mm is not None:
                camera.camera.set_focal_lengths([focal_mm])
            clip_near = float(calibration.get("min_range_m") or 0.05)
            clip_far = float(calibration.get("max_range_m") or 20.0)
            camera.camera.set_clipping_ranges([[clip_near, clip_far]])
            record["camera_local_orientation_wxyz"] = list(
                ROS_OPTICAL_TO_USD_CAMERA_QUAT_WXYZ
            )
            if activate_runtime:
                resolution = calibration.get("resolution_hw") or [480, 640]
                annotators: list[str] = []
                if "rgb" in outputs:
                    annotators.append("rgb")
                if "depth" in outputs:
                    annotators.append("distance_to_image_plane")
                if not annotators:
                    annotators = ["rgb"]
                handle = CameraSensor(
                    camera,
                    resolution=(int(resolution[0]), int(resolution[1])),
                    annotators=annotators,
                )
                runtime_handles.append(handle)
                runtime_by_name[logical_name] = handle
                record["annotators"] = list(annotators)
            record["prim_type"] = "Camera"
            record["active"] = True
        elif device_class in LIDAR_DEVICE_CLASSES:
            tick_rate = float(calibration.get("scan_frequency_hz") or 10.0)
            lidar_config = str(
                calibration.get("isaac_lidar_config") or DEFAULT_LIDAR_2D_CONFIG
            )
            lidar = Lidar.create(
                path=sensor_path,
                config=lidar_config,
                tick_rate=tick_rate,
                accumulate_outputs=True,
                aux_output_level="FULL",
                attributes={
                    "omni:sensor:Core:outputFrameOfReference": "WORLD",
                },
                translations=[[0.0, 0.0, 0.0]],
                reset_xform_op_properties=True,
            )
            record["lidar_config"] = lidar_config
            if activate_runtime:
                handle = LidarSensor(lidar, annotators=["generic-model-output"])
                runtime_handles.append(handle)
                runtime_by_name[logical_name] = handle
                record["annotators"] = ["generic-model-output"]
            record["prim_type"] = "OmniLidar"
            record["active"] = True
        elif device_class in IMU_DEVICE_CLASSES:
            imu = IMU.create(
                sensor_path,
                translations=[[0.0, 0.0, 0.0]],
                reset_xform_op_properties=True,
            )
            if activate_runtime:
                handle = IMUSensor(imu)
                runtime_handles.append(handle)
                runtime_by_name[logical_name] = handle
                record["annotators"] = ["imu_frame"]
            record["prim_type"] = "IsaacImuSensor"
            record["active"] = True
        else:
            raise SensorBootstrapError(f"Unsupported device_class: {device_class!r}")

        attached.append(record)

    counts = count_active_native_sensors(stage)
    filtered_profile = {"devices": [
        device
        for device in (profile.get("devices") or [])
        if logical_names is None or device["logical_name"] in set(logical_names)
    ]}
    expected = expected_device_count(filtered_profile)
    expected_by_class = count_devices_by_class(filtered_profile)
    if enforce_full_profile_counts and logical_names is not None:
        raise SensorBootstrapError(
            "logical_names filter requires enforce_full_profile_counts=False"
        )
    if counts["total"] != expected:
        raise SensorBootstrapError(
            f"Active sensor count mismatch: expected {expected}, actual {counts['total']}"
        )
    if (
        counts["camera"] != expected_by_class["camera"]
        or counts["lidar_2d"] != expected_by_class["lidar_2d"]
        or counts["imu"] != expected_by_class["imu"]
    ):
        raise SensorBootstrapError(
            f"Active sensor class counts mismatch: expected {expected_by_class}, actual "
            f"{{'camera': {counts['camera']}, 'lidar_2d': {counts['lidar_2d']}, "
            f"'imu': {counts['imu']}}}"
        )

    return {
        "ok": True,
        "expected_device_count": expected,
        "actual_device_count": counts["total"],
        "expected_by_class": expected_by_class,
        "actual_by_class": {
            "camera": counts["camera"],
            "lidar_2d": counts["lidar_2d"],
            "imu": counts["imu"],
        },
        "attached": attached,
        "runtime_handle_count": len(runtime_handles),
        "filtered_logical_names": (
            sorted(logical_names) if logical_names is not None else None
        ),
        "_runtime_handles": runtime_handles,
        "_runtime_by_name": runtime_by_name,
    }
