"""Config-driven sensor-test world generation for mobile Isaac archives."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping

from .paths import SENSOR_TEST_WORLD_CONFIG_PATH, SENSOR_TEST_WORLD_SCHEMA_PATH


MOBILE_MODELS = (
    "franka_mobile_sensors",
    "mobile_fr3_duo_v0_2",
    "tmrv0_2",
    "mobile_fr3_duo_with_vision_and_manipulation_kit",
)

WORLD_ENTRYPOINT_RELATIVE = "worlds/sensor_test_world.usda"
TARGET_MANIFEST_RELATIVE = "worlds/sensor_test_targets.json"
REQUIRED_MATERIAL_NAMES = (
    "red",
    "green",
    "blue",
    "yellow",
    "magenta",
    "white",
    "black",
)


class SensorTestWorldError(RuntimeError):
    """Raised when the sensor-test world config or composition is invalid."""


def is_mobile_model(model: str) -> bool:
    return model in MOBILE_MODELS


def robot_reference_for_model(model: str) -> str:
    return f"../{model}.usda"


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise SensorTestWorldError(message)


def _fmt_number(value: float) -> str:
    text = f"{float(value):.6f}".rstrip("0").rstrip(".")
    if text == "-0":
        return "0"
    return text


def _fmt_tuple(values: list[float] | tuple[float, ...]) -> str:
    return "(" + ", ".join(_fmt_number(item) for item in values) + ")"


def _emitted_local_half_extents(
    shape: str,
    dimensions: Mapping[str, float],
) -> tuple[float, float, float]:
    """Half-extents of the geometry actually authored in USDA (pre-rotation)."""
    x = float(dimensions["x"])
    y = float(dimensions["y"])
    z = float(dimensions["z"])
    if shape == "box":
        return (x / 2.0, y / 2.0, z / 2.0)
    if shape == "cylinder":
        radius = max(x, y) / 2.0
        return (radius, radius, z / 2.0)
    if shape == "sphere":
        radius = max(x, y, z) / 2.0
        return (radius, radius, radius)
    raise SensorTestWorldError(f"Unsupported target shape: {shape}")


def _emitted_dimensions_m(
    shape: str,
    dimensions: Mapping[str, float],
) -> dict[str, float]:
    half_x, half_y, half_z = _emitted_local_half_extents(shape, dimensions)
    return {
        "x": half_x * 2.0,
        "y": half_y * 2.0,
        "z": half_z * 2.0,
    }


def _rotate_xyz_matrix(rotate_xyz_deg: list[float] | tuple[float, ...]) -> list[list[float]]:
    """USD xformOp:rotateXYZ matrix R = Rx * Ry * Rz."""
    rx, ry, rz = (math.radians(float(angle)) for angle in rotate_xyz_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # Rz
    r00, r01, r02 = cz, -sz, 0.0
    r10, r11, r12 = sz, cz, 0.0
    r20, r21, r22 = 0.0, 0.0, 1.0
    # Ry * Rz
    m00, m01, m02 = cy * r00 + sy * r20, cy * r01 + sy * r21, cy * r02 + sy * r22
    m10, m11, m12 = r10, r11, r12
    m20, m21, m22 = -sy * r00 + cy * r20, -sy * r01 + cy * r21, -sy * r02 + cy * r22
    # Rx * (Ry * Rz)
    return [
        [m00, m01, m02],
        [
            cx * m10 - sx * m20,
            cx * m11 - sx * m21,
            cx * m12 - sx * m22,
        ],
        [
            sx * m10 + cx * m20,
            sx * m11 + cx * m21,
            sx * m12 + cx * m22,
        ],
    ]


def _world_xy_half_extents(
    half_extents: tuple[float, float, float],
    rotate_xyz_deg: list[float] | tuple[float, ...],
) -> tuple[float, float]:
    rotation = _rotate_xyz_matrix(rotate_xyz_deg)
    hx, hy, hz = half_extents
    world_half_x = (
        abs(rotation[0][0]) * hx + abs(rotation[0][1]) * hy + abs(rotation[0][2]) * hz
    )
    world_half_y = (
        abs(rotation[1][0]) * hx + abs(rotation[1][1]) * hy + abs(rotation[1][2]) * hz
    )
    return (world_half_x, world_half_y)


def _target_xy_aabb(
    translation: list[float],
    shape: str,
    dimensions: Mapping[str, float],
    rotate_xyz_deg: list[float] | tuple[float, ...],
) -> tuple[float, float, float, float]:
    half_extents = _emitted_local_half_extents(shape, dimensions)
    half_x, half_y = _world_xy_half_extents(half_extents, rotate_xyz_deg)
    center_x = float(translation[0])
    center_y = float(translation[1])
    return (
        center_x - half_x,
        center_x + half_x,
        center_y - half_y,
        center_y + half_y,
    )


def _aabb_intersects_corridor(
    translation: list[float],
    shape: str,
    dimensions: Mapping[str, float],
    rotate_xyz_deg: list[float] | tuple[float, ...],
    corridor: Mapping[str, float],
) -> bool:
    target_x_min, target_x_max, target_y_min, target_y_max = _target_xy_aabb(
        translation,
        shape,
        dimensions,
        rotate_xyz_deg,
    )
    return not (
        target_x_max <= float(corridor["x_min_m"])
        or target_x_min >= float(corridor["x_max_m"])
        or target_y_max <= float(corridor["y_min_m"])
        or target_y_min >= float(corridor["y_max_m"])
    )


def _bearing_deg(translation: list[float]) -> float:
    return math.degrees(math.atan2(float(translation[1]), float(translation[0])))


def _range_m(translation: list[float]) -> float:
    return math.hypot(float(translation[0]), float(translation[1]))


def validate_sensor_test_world_config(config: Mapping[str, Any]) -> dict[str, Any]:
    schema_path = SENSOR_TEST_WORLD_SCHEMA_PATH
    if not schema_path.is_file():
        raise SensorTestWorldError(f"Schema file missing: {schema_path}")
    try:
        import jsonschema
    except ImportError as error:
        raise SensorTestWorldError(
            "Strict schema validation requires the 'jsonschema' package "
            "(see isaacsim6/requirements-ci.txt)."
        ) from error
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    try:
        jsonschema.validate(instance=dict(config), schema=schema)
    except Exception as error:
        raise SensorTestWorldError(f"Schema validation failed: {error}") from error

    materials = config["materials"]
    for name in REQUIRED_MATERIAL_NAMES:
        _require(name in materials, f"materials.{name} required")

    names = [target["name"] for target in config["targets"]]
    _require(len(names) == len(set(names)), "target names must be unique")

    colors_used = {target["color"] for target in config["targets"]}
    missing_colors = [name for name in REQUIRED_MATERIAL_NAMES if name not in colors_used]
    _require(
        not missing_colors,
        "targets must exercise every distinct material color; missing: "
        + ", ".join(missing_colors),
    )

    gravity = [float(item) for item in config["gravity_m_s2"]]
    _require(abs(gravity[0]) < 1e-9 and abs(gravity[1]) < 1e-9, "gravity must be Z-aligned")
    _require(gravity[2] < 0.0, "gravity Z component must be finite and downward")

    corridor = config["clear_corridor"]
    _require(
        float(corridor["x_min_m"]) < float(corridor["x_max_m"]),
        "clear_corridor x_min_m must be < x_max_m",
    )
    _require(
        float(corridor["y_min_m"]) < float(corridor["y_max_m"]),
        "clear_corridor y_min_m must be < y_max_m",
    )
    _require(
        float(corridor["x_min_m"]) < 0.0 < float(corridor["x_max_m"]),
        "clear_corridor must contain the spawn origin on X",
    )
    _require(
        float(corridor["y_min_m"]) < 0.0 < float(corridor["y_max_m"]),
        "clear_corridor must contain the spawn origin on Y",
    )

    for target in config["targets"]:
        if not target["collision"]:
            continue
        if _aabb_intersects_corridor(
            list(target["translation_m"]),
            str(target["shape"]),
            target["dimensions_m"],
            list(target["rotate_xyz_deg"]),
            corridor,
        ):
            raise SensorTestWorldError(
                f"target {target['name']} collision AABB intersects the clear corridor"
            )
    return dict(config)


def load_sensor_test_world_config(path: Path | None = None) -> dict[str, Any]:
    config_path = Path(path) if path is not None else SENSOR_TEST_WORLD_CONFIG_PATH
    if not config_path.is_file():
        raise SensorTestWorldError(f"World config not found: {config_path}")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    return validate_sensor_test_world_config(config)


def build_target_manifest(model: str, config: Mapping[str, Any] | None = None) -> dict[str, Any]:
    world = config if config is not None else load_sensor_test_world_config()
    gravity = [float(item) for item in world["gravity_m_s2"]]
    floor = world["floor"]
    spawn = world["robot_spawn"]
    targets = []
    for target in sorted(world["targets"], key=lambda item: item["name"]):
        translation = [float(item) for item in target["translation_m"]]
        dimensions = _emitted_dimensions_m(
            str(target["shape"]),
            target["dimensions_m"],
        )
        targets.append(
            {
                "name": target["name"],
                "prim_path": f"/World/Targets/{target['name']}",
                "shape": target["shape"],
                "purpose": list(target["purpose"]),
                "color": target["color"],
                "color_rgb": [float(item) for item in world["materials"][target["color"]]],
                "translation_m": translation,
                "rotate_xyz_deg": [float(item) for item in target["rotate_xyz_deg"]],
                "dimensions_m": dimensions,
                "collision": bool(target["collision"]),
                "range_from_origin_m": round(_range_m(translation), 6),
                "bearing_from_origin_deg": round(_bearing_deg(translation), 6),
            }
        )
    return {
        "schema_version": 1,
        "world_name": world["world_name"],
        "model": model,
        "entrypoint": WORLD_ENTRYPOINT_RELATIVE,
        "robot_reference": robot_reference_for_model(model),
        "limitation": (
            "This world supplies visual/geometric camera and lidar targets only. "
            "It does not add Isaac sensor plugins, synthesize sensor outputs, "
            "or prove plugin/output correctness; that remains a separate user test round."
        ),
        "gravity_m_s2": gravity,
        "floor": {
            "size_m": float(floor["size_m"]),
            "thickness_m": float(floor["thickness_m"]),
            "top_z_m": 0.0,
            "static_friction": float(floor["static_friction"]),
            "dynamic_friction": float(floor["dynamic_friction"]),
            "restitution": float(floor["restitution"]),
            "color_rgb": [float(item) for item in floor["color_rgb"]],
        },
        "robot_spawn": {
            "translation_m": [float(item) for item in spawn["translation_m"]],
            "rotate_xyz_deg": [float(item) for item in spawn["rotate_xyz_deg"]],
        },
        "clear_corridor": dict(world["clear_corridor"]),
        "materials": {
            name: [float(item) for item in world["materials"][name]]
            for name in REQUIRED_MATERIAL_NAMES
        },
        "targets": targets,
    }


def _material_block(name: str, rgb: list[float]) -> str:
    material_name = "Floor" if name == "Floor" else name.capitalize()
    path = f"/World/Looks/{material_name}"
    shader = f"{path}/PreviewSurface"
    return (
        f'    def Material "{material_name}"\n'
        "    {\n"
        f"        token outputs:surface.connect = <{shader}.outputs:surface>\n"
        '        def Shader "PreviewSurface"\n'
        "        {\n"
        '            uniform token info:id = "UsdPreviewSurface"\n'
        f"            color3f inputs:diffuseColor = {_fmt_tuple(rgb)}\n"
        "            float inputs:roughness = 0.4\n"
        "            float inputs:metallic = 0.0\n"
        "            token outputs:surface\n"
        "        }\n"
        "    }\n"
    )


def _physics_floor_material_block(floor: Mapping[str, Any]) -> str:
    return (
        '    def Material "PhysicsFloor" (\n'
        '        prepend apiSchemas = ["PhysicsMaterialAPI"]\n'
        "    )\n"
        "    {\n"
        f"        float physics:staticFriction = {_fmt_number(float(floor['static_friction']))}\n"
        f"        float physics:dynamicFriction = {_fmt_number(float(floor['dynamic_friction']))}\n"
        f"        float physics:restitution = {_fmt_number(float(floor['restitution']))}\n"
        "    }\n"
    )


def _xform_ops(
    translation: list[float],
    rotate_xyz_deg: list[float],
    scale: list[float] | None = None,
    *,
    indent: str = "        ",
) -> str:
    lines = [
        f"{indent}double3 xformOp:translate = {_fmt_tuple(translation)}",
        f"{indent}float3 xformOp:rotateXYZ = {_fmt_tuple(rotate_xyz_deg)}",
    ]
    order = ["xformOp:translate", "xformOp:rotateXYZ"]
    if scale is not None:
        lines.append(f"{indent}float3 xformOp:scale = {_fmt_tuple(scale)}")
        order.append("xformOp:scale")
    lines.append(
        f"{indent}uniform token[] xformOpOrder = ["
        + ", ".join(f'"{item}"' for item in order)
        + "]"
    )
    return "\n".join(lines) + "\n"


def _target_prim(target: Mapping[str, Any]) -> str:
    name = target["name"]
    color = str(target["color"]).capitalize()
    dims = target["dimensions_m"]
    translation = [float(item) for item in target["translation_m"]]
    rotate = [float(item) for item in target["rotate_xyz_deg"]]
    api = (
        ' (\n            prepend apiSchemas = ["PhysicsCollisionAPI"]\n        )'
        if target["collision"]
        else ""
    )
    shape = str(target["shape"])
    half_x, half_y, half_z = _emitted_local_half_extents(shape, dims)
    if shape == "box":
        scale = [half_x, half_y, half_z]
        geom = (
            f'        def Cube "Geom"{api}\n'
            "        {\n"
            "            double size = 2\n"
            f"            rel material:binding = </World/Looks/{color}>\n"
            "        }\n"
        )
        body = _xform_ops(translation, rotate, scale, indent="        ") + geom
    elif shape == "cylinder":
        geom = (
            f'        def Cylinder "Geom"{api}\n'
            "        {\n"
            f"            double radius = {_fmt_number(half_x)}\n"
            f"            double height = {_fmt_number(half_z * 2.0)}\n"
            '            uniform token axis = "Z"\n'
            f"            rel material:binding = </World/Looks/{color}>\n"
            "        }\n"
        )
        body = _xform_ops(translation, rotate, indent="        ") + geom
    elif shape == "sphere":
        geom = (
            f'        def Sphere "Geom"{api}\n'
            "        {\n"
            f"            double radius = {_fmt_number(half_x)}\n"
            f"            rel material:binding = </World/Looks/{color}>\n"
            "        }\n"
        )
        body = _xform_ops(translation, rotate, indent="        ") + geom
    else:
        raise SensorTestWorldError(f"Unsupported target shape: {shape}")
    return f'    def Xform "{name}"\n    {{\n{body}    }}\n'


def build_sensor_test_world_usda(model: str, config: Mapping[str, Any] | None = None) -> str:
    """Render a dependency-free world USDA that references the archive-local robot."""
    world = validate_sensor_test_world_config(
        config if config is not None else load_sensor_test_world_config()
    )
    gravity = [float(item) for item in world["gravity_m_s2"]]
    magnitude = abs(gravity[2])
    direction = (0.0, 0.0, -1.0 if gravity[2] < 0 else 1.0)
    floor = world["floor"]
    lighting = world["lighting"]
    spawn = world["robot_spawn"]
    robot_ref = robot_reference_for_model(model)

    size_m = float(floor["size_m"])
    thickness_m = float(floor["thickness_m"])
    floor_scale = [size_m / 2.0, size_m / 2.0, thickness_m / 2.0]
    floor_translation = [0.0, 0.0, -thickness_m / 2.0]

    material_blocks = [
        _material_block("Floor", [float(v) for v in floor["color_rgb"]]),
        _physics_floor_material_block(floor),
    ]
    for name in REQUIRED_MATERIAL_NAMES:
        material_blocks.append(
            _material_block(name, [float(v) for v in world["materials"][name]])
        )

    target_blocks = [
        _target_prim(target)
        for target in sorted(world["targets"], key=lambda item: item["name"])
    ]

    robot_ops = _xform_ops(
        [float(item) for item in spawn["translation_m"]],
        [float(item) for item in spawn["rotate_xyz_deg"]],
        indent="        ",
    )
    floor_ops = _xform_ops(
        floor_translation,
        [0.0, 0.0, 0.0],
        floor_scale,
        indent="            ",
    )

    parts = [
        "#usda 1.0",
        "(",
        '    defaultPrim = "World"',
        "    metersPerUnit = 1",
        '    upAxis = "Z"',
        '    doc = """Mobile sensor-test world with collision floor, neutral lighting,',
        " colored camera/lidar targets, and a relative robot reference.",
        ' Visual/geometric targets only; not a sensor-output proof."""',
        ")",
        "",
        'def Xform "World"',
        "{",
        '    def PhysicsScene "physicsScene"',
        "    {",
        f"        vector3f physics:gravityDirection = {_fmt_tuple(direction)}",
        f"        float physics:gravityMagnitude = {_fmt_number(magnitude)}",
        "    }",
        "",
        '    def Xform "Robot" (',
        f"        prepend references = @{robot_ref}@",
        "    )",
        "    {",
        robot_ops.rstrip("\n"),
        "    }",
        "",
        '    def Xform "Environment"',
        "    {",
        '        def Xform "Floor"',
        "        {",
        floor_ops.rstrip("\n"),
        '            def Cube "CollisionPlane" (',
        '                prepend apiSchemas = ["PhysicsCollisionAPI"]',
        "            )",
        "            {",
        "                double size = 2",
        "                rel material:binding = </World/Looks/Floor>",
        "                rel material:binding:physics = </World/Looks/PhysicsFloor>",
        "            }",
        "        }",
        "",
        '        def DistantLight "KeyLight"',
        "        {",
        f"            float inputs:intensity = {_fmt_number(float(lighting['key_intensity']))}",
        f"            color3f inputs:color = {_fmt_tuple([float(v) for v in lighting['key_color_rgb']])}",
        f"            float3 xformOp:rotateXYZ = {_fmt_tuple([float(v) for v in lighting['key_rotate_xyz_deg']])}",
        '            uniform token[] xformOpOrder = ["xformOp:rotateXYZ"]',
        "        }",
        "",
        '        def DomeLight "FillLight"',
        "        {",
        f"            float inputs:intensity = {_fmt_number(float(lighting['dome_intensity']))}",
        f"            color3f inputs:color = {_fmt_tuple([float(v) for v in lighting['dome_color_rgb']])}",
        "        }",
        "    }",
        "",
        '    def Xform "Targets"',
        "    {",
    ]
    for block in target_blocks:
        indented = "\n".join(
            ("    " + line) if line else line for line in block.rstrip("\n").split("\n")
        )
        parts.append(indented)
    parts.extend(
        [
            "    }",
            "",
            '    def Scope "Looks"',
            "    {",
        ]
    )
    for block in material_blocks:
        parts.append(block.rstrip("\n"))
    parts.extend(["    }", "}", ""])
    return "\n".join(parts)


def build_sensor_test_world_files(
    model: str,
    *,
    config: Mapping[str, Any] | None = None,
) -> dict[str, bytes]:
    """Return archive-relative files to embed for one mobile model."""
    if not is_mobile_model(model):
        raise SensorTestWorldError(f"Sensor-test world is only for mobile models, got {model!r}")
    world = config if config is not None else load_sensor_test_world_config()
    usda = build_sensor_test_world_usda(model, world)
    manifest = build_target_manifest(model, world)
    return {
        WORLD_ENTRYPOINT_RELATIVE: usda.encode("utf-8"),
        TARGET_MANIFEST_RELATIVE: (
            json.dumps(manifest, indent=2, sort_keys=False) + "\n"
        ).encode("utf-8"),
    }
