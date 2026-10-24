#!/usr/bin/env python3
"""Attach Isaac-native sensors for a sensor-bearing model from its profile.

Opens the sensorized world (or an explicit USD), resolves every profile
``frame_name_suffix`` to a unique robot prim, and creates a ``NativeSensor``
child (Camera / OmniLidar / IsaacImuSensor). Geometry-only frames never count
as active sensors. Semantic output/range acceptance is out of scope.
"""

from __future__ import annotations

import argparse
import json
import os
import runpy
import sys
import traceback
from pathlib import Path


def finish_with_observable_exit(simulation_app, exit_code: int) -> int:
    if exit_code != 0:
        os._exit(exit_code)
    from lib.teardown import close_simulation_app

    close_simulation_app(simulation_app)
    return exit_code


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True)
    parser.add_argument(
        "--usd",
        type=Path,
        default=None,
        help="USD to open (default: <artifact-root>/usd/<model>/worlds/sensorized_world.usda "
        "or packaged extract layout).",
    )
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument(
        "--artifact-root",
        type=Path,
        default=None,
        help="Model artifact root containing usd/<model>/ (optional with --usd).",
    )
    parser.add_argument(
        "--save-overlay",
        type=Path,
        default=None,
        help="Optional path to export the session stage after attachment "
        "(best-effort; bootstrap remains the supported activation path).",
    )
    parser.add_argument(
        "--no-runtime",
        action="store_true",
        help="Author sensor prims only; skip CameraSensor/LidarSensor/IMUSensor runtime.",
    )
    args = parser.parse_args()

    tooling = Path(
        os.environ.get(
            "ISAACSIM6_TOOLING_ROOT",
            str(Path(__file__).resolve().parents[1]),
        )
    )
    ensure_tooling = runpy.run_path(str(tooling / "lib" / "bootstrap.py"))[
        "ensure_tooling_on_path"
    ]
    ensure_tooling(tooling)

    from lib.paths import artifact_root_from_env
    from lib.sensor_bootstrap import (
        SENSORIZED_WORLD_ENTRYPOINT_RELATIVE,
        SensorBootstrapError,
        attach_native_sensors_from_profile,
        count_active_native_sensors,
        is_sensor_bearing_model,
    )
    from lib.sensor_profile import load_sensor_profile
    from lib.teardown import register_external_teardown

    report_path = args.report.resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    register_external_teardown(
        report_path.parent / f"teardown_bootstrap_{args.model}.json",
        gate="native_sensor_bootstrap",
    )

    report: dict = {
        "ok": False,
        "model": args.model,
        "isaac_version_env": os.environ.get("ISAACSIM_VERSION", ""),
    }
    exit_code = 1

    if not is_sensor_bearing_model(args.model):
        report["error"] = f"Model is not sensor-bearing: {args.model}"
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        return 2

    if args.usd is not None:
        usd_path = args.usd.resolve()
    else:
        try:
            artifact_root = args.artifact_root or artifact_root_from_env()
        except RuntimeError as error:
            report["error"] = str(error)
            report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            return 2
        usd_path = (
            Path(artifact_root)
            / "usd"
            / args.model
            / SENSORIZED_WORLD_ENTRYPOINT_RELATIVE
        ).resolve()
        if not usd_path.is_file():
            # Packaged extract layout: <root>/<model>/worlds/sensorized_world.usda
            alt = (
                Path(artifact_root) / args.model / SENSORIZED_WORLD_ENTRYPOINT_RELATIVE
            ).resolve()
            if alt.is_file():
                usd_path = alt

    report["usd_path"] = str(usd_path)
    if not usd_path.is_file():
        report["error"] = f"USD missing: {usd_path}"
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        return 2

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": True})
    try:
        import omni.usd
        from isaacsim.core.experimental.utils.stage import is_stage_loading, open_stage

        profile = load_sensor_profile(args.model)
        report["expected_device_count"] = len(profile["devices"])
        report["expected_by_class"] = {
            "camera": sum(
                1
                for device in profile["devices"]
                if device["device_class"]
                in {
                    "stereo_rgb_camera",
                    "rgb_camera",
                    "depth_camera",
                    "rgb_depth_camera",
                }
            ),
            "lidar_2d": sum(
                1 for device in profile["devices"] if device["device_class"] == "lidar_2d"
            ),
            "imu": sum(
                1 for device in profile["devices"] if device["device_class"] == "imu"
            ),
        }

        open_stage(usd_path=str(usd_path))
        while is_stage_loading():
            simulation_app.update()
        for _ in range(10):
            simulation_app.update()

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("No stage after open_stage")

        before = count_active_native_sensors(stage)
        report["active_sensors_before"] = before
        if before["total"] != 0:
            raise SensorBootstrapError(
                f"Stage already has active NativeSensor prims before bootstrap: {before}"
            )

        result = attach_native_sensors_from_profile(
            stage,
            profile,
            activate_runtime=not args.no_runtime,
        )
        # Drop non-JSON runtime handles from the persisted report.
        result.pop("_runtime_handles", None)
        report.update(result)
        report["actual_device_count"] = result["actual_device_count"]
        report["actual_by_class"] = result["actual_by_class"]

        for _ in range(5):
            simulation_app.update()

        after = count_active_native_sensors(stage)
        report["active_sensors_after"] = after
        if after["total"] != report["expected_device_count"]:
            raise SensorBootstrapError(
                f"Post-attach recount mismatch: expected "
                f"{report['expected_device_count']}, actual {after['total']}"
            )

        if args.save_overlay is not None:
            overlay = args.save_overlay.resolve()
            overlay.parent.mkdir(parents=True, exist_ok=True)
            stage.GetRootLayer().Export(str(overlay))
            report["overlay_path"] = str(overlay)

        report["ok"] = True
        exit_code = 0
    except Exception as error:
        report["ok"] = False
        report["error"] = str(error)
        report["traceback"] = traceback.format_exc()
        exit_code = 1
    finally:
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    return finish_with_observable_exit(simulation_app, exit_code)


if __name__ == "__main__":
    sys.exit(main())
