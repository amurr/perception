"""Resolve tooling and artifact roots from environment / repo-relative defaults."""

from __future__ import annotations

import os
from pathlib import Path

TOOLING_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = TOOLING_ROOT.parent
CONFIGS_DIR = TOOLING_ROOT / "configs"
SCHEMA_PATH = TOOLING_ROOT / "schema" / "model_config.schema.json"
SENSOR_TEST_WORLD_SCHEMA_PATH = (
    TOOLING_ROOT / "schema" / "sensor_test_world.schema.json"
)
SENSOR_TEST_WORLD_CONFIG_PATH = TOOLING_ROOT / "worlds" / "sensor_test_world.json"
SENSOR_PROFILES_DIR = TOOLING_ROOT / "sensors"
SENSOR_PROFILE_SCHEMA_PATH = TOOLING_ROOT / "schema" / "sensor_profile.schema.json"
NATIVE_SENSOR_BOOTSTRAP_SCHEMA_PATH = (
    TOOLING_ROOT / "schema" / "native_sensor_bootstrap.schema.json"
)


def _path_from_env(name: str, default: Path | None = None) -> Path | None:
    value = os.environ.get(name)
    if value:
        return Path(value)
    return default


DEFAULT_LOCAL_ARTIFACTS_ROOT = _path_from_env(
    "ISAACSIM6_LOCAL_ARTIFACTS_ROOT",
    REPOSITORY_ROOT / "build" / "isaacsim6",
)
DEFAULT_REMOTE_ARTIFACTS_ROOT = _path_from_env("ISAACSIM6_REMOTE_ARTIFACTS_ROOT")
DEFAULT_MANIFEST_PATH = _path_from_env(
    "ISAACSIM6_ASSET_MANIFEST",
    REPOSITORY_ROOT / "config" / "asset_manifest.json",
)
DEFAULT_ISAAC_ROOT = _path_from_env("ISAACSIM_PATH")


def isaac_root_from_env() -> Path:
    """Isaac Sim installation root; requires ISAACSIM_PATH."""
    if DEFAULT_ISAAC_ROOT is None:
        raise RuntimeError("ISAACSIM_PATH is required (path to Isaac Sim installation)")
    return DEFAULT_ISAAC_ROOT


def artifact_root_from_env() -> Path:
    """Active model artifact root (remote or local run directory)."""
    value = os.environ.get("ISAACSIM6_ARTIFACT_ROOT")
    if not value:
        raise RuntimeError("ISAACSIM6_ARTIFACT_ROOT is required for workflow scripts")
    return Path(value)


def resolve_usd_path(artifact_root: Path, model_name: str) -> Path:
    layout_path = artifact_root / "logs" / "import_layout.json"
    if layout_path.is_file():
        import json

        layout = json.loads(layout_path.read_text(encoding="utf-8"))
        candidate = layout.get("output_usd")
        if candidate and Path(candidate).is_file():
            return Path(candidate)
    usd_dir = artifact_root / "usd"
    preferred = usd_dir / model_name / f"{model_name}.usda"
    if preferred.is_file():
        return preferred
    usd_files = sorted(usd_dir.rglob("*.usd*"))
    for path in usd_files:
        if path.name.startswith(f"{model_name}.") and "payloads" not in str(path):
            return path
    if not usd_files:
        raise FileNotFoundError(f"No USD under {usd_dir}")
    return usd_files[0]


def package_input_paths(artifact_root: Path, package_name: str, urdf_filename: str) -> tuple[Path, Path]:
    package_root = artifact_root / "input" / package_name
    urdf_path = package_root / "urdf" / urdf_filename
    return package_root, urdf_path
