#!/usr/bin/env python3
"""Capture and evaluate Isaac-native sensor outputs for a sensor-bearing model.

Opens a sensorized world, attaches NativeSensor children (all profile devices by
default), steps the supported camera/lidar/IMU paths under a capture deadline,
and writes per-sensor metrics against the packaged target oracle. Binary
artifacts are kept only for representative devices.
"""

from __future__ import annotations

import argparse
import json
import os
import runpy
import sys
import traceback
from pathlib import Path
from typing import Any


def finish_with_observable_exit(simulation_app, exit_code: int) -> int:
    if exit_code != 0:
        os._exit(exit_code)
    from lib.teardown import close_simulation_app

    close_simulation_app(simulation_app)
    return exit_code


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    from lib.sensor_capture import write_strict_json

    write_strict_json(path, payload)


def _persist_failure_capsule(path: Path, report: dict[str, Any]) -> None:
    capsule = {
        "ok": False,
        "provenance": "in_process_except",
        "error": report.get("error"),
        "where": report.get("where"),
        "model": report.get("model"),
        "usd_path": report.get("usd_path"),
        "timeout_s": report.get("timeout_s"),
        "traceback": report.get("traceback"),
        "preexisting_kit_note": "Do not signal foreign Kit PIDs; owned teardown only.",
    }
    _write_json(path, capsule)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True)
    parser.add_argument("--usd", type=Path, default=None)
    parser.add_argument("--artifact-root", type=Path, default=None)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=900.0)
    parser.add_argument("--warmup-frames", type=int, default=24)
    parser.add_argument("--settle-frames", type=int, default=120)
    parser.add_argument(
        "--logical-name",
        action="append",
        default=None,
        help="Optional device filter (repeatable). Defaults to all profile devices.",
    )
    parser.add_argument(
        "--representatives-only",
        action="store_true",
        help="Capture only the model representative devices (prior focused path).",
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
        is_sensor_bearing_model,
    )
    from lib.sensor_capture import (
        CaptureDeadline,
        SensorCaptureError,
        build_capture_expectations,
        capture_exit_code,
        compact_sensor_metrics,
        evaluate_depth_array,
        evaluate_imu_sample,
        evaluate_lidar_scan,
        evaluate_rgb_array,
        logical_names_for_model,
        logical_names_for_representatives,
        output_class_for_device,
        representative_devices_for_model,
        summarize_capture_results,
        write_strict_json,
    )
    from lib.sensor_profile import load_sensor_profile
    from lib.sensor_test_world import TARGET_MANIFEST_RELATIVE, build_target_manifest
    from lib.teardown import register_external_teardown

    report_path = args.report.resolve()
    evidence_dir = args.evidence_dir.resolve()
    evidence_dir.mkdir(parents=True, exist_ok=True)
    failure_capsule_path = evidence_dir / "failure_capsule.json"
    register_external_teardown(
        evidence_dir / f"teardown_capture_{args.model}.json",
        gate="native_sensor_capture",
    )

    report: dict[str, Any] = {
        "ok": False,
        "model": args.model,
        "isaac_version_env": os.environ.get("ISAACSIM_VERSION", ""),
        "timeout_s": float(args.timeout_s),
        "where": "startup",
    }
    exit_code = 1
    simulation_app = None

    try:
        if not is_sensor_bearing_model(args.model):
            raise SensorCaptureError(f"Model is not sensor-bearing: {args.model}")

        representatives = representative_devices_for_model(args.model)
        representative_names = set(representatives.values())
        if args.logical_name:
            logical_names = list(args.logical_name)
        elif args.representatives_only:
            logical_names = logical_names_for_representatives(args.model)
        else:
            logical_names = logical_names_for_model(args.model)

        if args.usd is not None:
            usd_path = args.usd.resolve()
        else:
            artifact_root = args.artifact_root or artifact_root_from_env()
            usd_path = (
                Path(artifact_root)
                / "usd"
                / args.model
                / SENSORIZED_WORLD_ENTRYPOINT_RELATIVE
            ).resolve()
            if not usd_path.is_file():
                alt = (
                    Path(artifact_root) / args.model / SENSORIZED_WORLD_ENTRYPOINT_RELATIVE
                ).resolve()
                if alt.is_file():
                    usd_path = alt
        report["usd_path"] = str(usd_path)
        if not usd_path.is_file():
            raise SensorCaptureError(f"USD missing: {usd_path}")

        model_root = usd_path.parent.parent if usd_path.parent.name == "worlds" else usd_path.parent
        target_manifest_path = model_root / TARGET_MANIFEST_RELATIVE
        if target_manifest_path.is_file():
            target_manifest = json.loads(target_manifest_path.read_text(encoding="utf-8"))
        else:
            target_manifest = build_target_manifest(args.model)

        profile = load_sensor_profile(args.model)
        declared_logical_names = logical_names_for_model(args.model)
        expectations = build_capture_expectations(
            args.model,
            target_manifest=target_manifest,
            profile=profile,
        )
        write_strict_json(evidence_dir / "expectations.json", expectations)
        report["representatives"] = representatives
        report["logical_names"] = logical_names
        report["declared_logical_names"] = declared_logical_names
        report["selected_logical_names"] = list(logical_names)
        report["declared_device_count"] = len(declared_logical_names)
        report["selected_device_count"] = len(logical_names)
        # Legacy alias: declared profile size (not the selection size).
        report["expected_device_count"] = len(declared_logical_names)
        report["runtime_contract"] = expectations["runtime_contract"]

        deadline = CaptureDeadline(float(args.timeout_s))
        report["where"] = "simulation_app_start"

        from isaacsim import SimulationApp

        simulation_app = SimulationApp({"headless": True, "width": 1280, "height": 720})

        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaacsim.core.experimental.utils.stage import is_stage_loading, open_stage
        from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data

        report["where"] = "open_stage"
        deadline.raise_if_expired(report["where"])
        open_stage(usd_path=str(usd_path))
        while is_stage_loading():
            deadline.raise_if_expired("stage_loading")
            simulation_app.update()
        for _ in range(10):
            deadline.raise_if_expired("post_open_warmup")
            simulation_app.update()

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("No stage after open_stage")

        report["where"] = "attach_runtime"
        attach = attach_native_sensors_from_profile(
            stage,
            profile,
            activate_runtime=True,
            logical_names=logical_names,
            enforce_full_profile_counts=False,
        )
        runtime_by_name = attach.pop("_runtime_by_name", {})
        attach.pop("_runtime_handles", None)
        report["attach"] = attach
        if not runtime_by_name:
            raise SensorBootstrapError("No runtime handles created for capture devices")
        missing = sorted(set(logical_names) - set(runtime_by_name))
        if missing:
            raise SensorBootstrapError(
                f"Runtime handles missing for logical_names: {missing}"
            )

        import isaacsim.core.experimental.utils.app as app_utils
        import numpy as np

        # Phase A: orchestrator camera capture first (depth stays valid).
        # Phase B: timeline play + Writer for lidar/IMU (GMO needs play).
        rep.orchestrator.set_capture_on_play(False)
        if app_utils.is_playing():
            app_utils.stop(commit=True)

        device_results: dict[str, Any] = {}
        artifacts: dict[str, str] = {}
        profile_by_name = {
            device["logical_name"]: device for device in profile["devices"]
        }
        camera_devices = [
            profile_by_name[name]
            for name in logical_names
            if output_class_for_device(profile_by_name[name]) in {"rgb", "depth"}
        ]
        lidar_devices = [
            profile_by_name[name]
            for name in logical_names
            if output_class_for_device(profile_by_name[name]) == "lidar"
        ]
        imu_devices = [
            profile_by_name[name]
            for name in logical_names
            if output_class_for_device(profile_by_name[name]) == "imu"
        ]

        for device in camera_devices:
            logical_name = device["logical_name"]
            output_class = output_class_for_device(device)
            report["where"] = f"capture_{logical_name}"
            handle = runtime_by_name[logical_name]
            resolution = device.get("simulation_calibration", {}).get(
                "resolution_hw"
            ) or [720, 1280]
            annotator = "rgb" if output_class == "rgb" else "distance_to_image_plane"
            array_np = None
            for frame_index in range(int(args.warmup_frames)):
                deadline.raise_if_expired(f"{logical_name}_orchestrator_{frame_index}")
                rep.orchestrator.step(
                    rt_subframes=8,
                    delta_time=0.0,
                    pause_timeline=True,
                    wait_for_render=True,
                )
                data, _info = handle.get_data(annotator)
                if data is not None:
                    array_np = data.numpy()
            if array_np is None:
                raise SensorCaptureError(
                    f"{logical_name} never produced {annotator} data"
                )
            save_binary = logical_name in representative_names or os.environ.get(
                "ISAACSIM6_SAVE_ALL_BINARIES", ""
            ).strip() in {"1", "true", "yes"}
            if save_binary:
                np_path = evidence_dir / f"{logical_name}_{output_class}.npy"
                np.save(np_path, array_np)
                artifacts[f"{logical_name}_{output_class}_npy"] = str(np_path)
            check = expectations["checks"][logical_name]
            if output_class == "rgb":
                metrics = evaluate_rgb_array(
                    array_np,
                    expected_resolution_hw=resolution,
                    expectations=check,
                )
                if save_binary:
                    try:
                        from PIL import Image

                        image = Image.fromarray(
                            array_np[:, :, :3].astype("uint8"), mode="RGB"
                        )
                        png_path = evidence_dir / f"{logical_name}_rgb.png"
                        image.save(png_path)
                        artifacts[f"{logical_name}_rgb_png"] = str(png_path)
                    except Exception as image_error:  # pragma: no cover - best effort
                        report.setdefault("warnings", []).append(
                            f"rgb png save failed for {logical_name}: {image_error}"
                        )
            else:
                metrics = evaluate_depth_array(
                    array_np,
                    expected_resolution_hw=resolution,
                    expectations=check,
                )
            device_results[logical_name] = {
                "logical_name": logical_name,
                "output_class": output_class,
                "annotator": annotator,
                "runtime": "CameraSensor",
                "capture_path": "orchestrator_step",
                "frame_name_suffix": device.get("frame_name_suffix"),
                **metrics,
            }
            report["devices"] = {
                name: compact_sensor_metrics(result)
                for name, result in device_results.items()
            }
            write_strict_json(report_path, report)

        lidar_writers: dict[str, Any] = {}
        lidar_writer_name = None
        if lidar_devices:

            class _NativeCaptureGmoWriter(rep.Writer):
                def __init__(self) -> None:
                    self.annotators = [rep.annotators.get("GenericModelOutput")]
                    self.data_structure = "renderProduct"
                    self.latest_gmo = None
                    self.valid_frames = 0
                    self.empty_frames = 0

                def write(self, data: dict) -> None:
                    if "renderProducts" not in data:
                        return
                    for _name, rp_data in data["renderProducts"].items():
                        raw = rp_data.get("GenericModelOutput")
                        if isinstance(raw, dict):
                            raw = raw.get("data")
                        parsed = parse_generic_model_output_data(raw)
                        if int(getattr(parsed, "numElements", 0) or 0) <= 0:
                            self.empty_frames += 1
                            return
                        self.latest_gmo = parsed
                        self.valid_frames += 1

            lidar_writer_name = _NativeCaptureGmoWriter.__name__
            if lidar_writer_name not in rep.WriterRegistry.get_writers():
                rep.WriterRegistry.register(_NativeCaptureGmoWriter)
            for device in lidar_devices:
                logical_name = device["logical_name"]
                runtime_by_name[logical_name].attach_writer(lidar_writer_name)
                lidar_writers[logical_name] = runtime_by_name[logical_name]._writers[
                    lidar_writer_name
                ]

        report["where"] = "timeline_play_settle"
        if lidar_devices or imu_devices:
            rep.orchestrator.set_capture_on_play(True)
            timeline = omni.timeline.get_timeline_interface()
            timeline.set_end_time(max(30.0, float(args.settle_frames) / 30.0 + 5.0))
            app_utils.play(commit=True)
            for frame_index in range(int(args.settle_frames)):
                deadline.raise_if_expired(f"settle_frame_{frame_index}")
                app_utils.update_app()

        for device in lidar_devices:
            logical_name = device["logical_name"]
            report["where"] = f"capture_{logical_name}"
            handle = runtime_by_name[logical_name]
            writer = lidar_writers[logical_name]
            collection_frames = max(int(args.warmup_frames), 180)
            for frame_index in range(collection_frames):
                deadline.raise_if_expired(f"{logical_name}_warmup_{frame_index}")
                app_utils.update_app()
                if writer.valid_frames >= 2 and writer.latest_gmo is not None:
                    break
            gmo = writer.latest_gmo
            try:
                if lidar_writer_name is not None:
                    handle.detach_writer(lidar_writer_name)
            except Exception:
                pass
            if gmo is None:
                raise SensorCaptureError(
                    f"{logical_name} never produced generic-model-output points "
                    f"(writer_empty_frames={writer.empty_frames}, "
                    f"writer_valid_frames={writer.valid_frames}); "
                    "supported alternative Writer path also empty"
                )

            azimuth = np.asarray(gmo.x, dtype=np.float32)
            elevation = np.asarray(gmo.y, dtype=np.float32)
            ranges = np.asarray(gmo.z, dtype=np.float32)
            if logical_name in representative_names:
                npz_path = evidence_dir / f"{logical_name}_scan.npz"
                np.savez_compressed(
                    npz_path,
                    azimuth_deg=azimuth,
                    elevation_deg=elevation,
                    range_m=ranges,
                )
                artifacts[f"{logical_name}_npz"] = str(npz_path)
            metrics = evaluate_lidar_scan(
                azimuth,
                elevation,
                ranges,
                expectations=expectations["checks"][logical_name],
            )
            device_results[logical_name] = {
                "logical_name": logical_name,
                "output_class": "lidar",
                "annotator": "generic-model-output",
                "runtime": "LidarSensor",
                "capture_path": "replicator_writer",
                "writer_valid_frames": int(writer.valid_frames),
                "num_elements": int(gmo.numElements),
                "frame_name_suffix": device.get("frame_name_suffix"),
                **metrics,
            }
            report["devices"] = {
                name: compact_sensor_metrics(result)
                for name, result in device_results.items()
            }
            write_strict_json(report_path, report)

        for device in imu_devices:
            logical_name = device["logical_name"]
            report["where"] = f"capture_{logical_name}"
            handle = runtime_by_name[logical_name]
            frame = None
            for frame_index in range(max(int(args.warmup_frames), 60)):
                deadline.raise_if_expired(f"{logical_name}_warmup_{frame_index}")
                app_utils.update_app()
                frame = handle.get_data(read_gravity=True)
            if frame is None:
                raise SensorCaptureError(f"{logical_name} never produced IMU data")
            metrics = evaluate_imu_sample(
                frame["linear_acceleration"],
                frame["angular_velocity"],
                expectations=expectations["checks"][logical_name],
                orientation=frame.get("orientation"),
            )
            device_results[logical_name] = {
                "logical_name": logical_name,
                "output_class": "imu",
                "annotator": "imu_frame",
                "runtime": "IMUSensor",
                "physics_step": frame.get("physics_step"),
                "time": frame.get("time"),
                "frame_name_suffix": device.get("frame_name_suffix"),
                **metrics,
            }
            report["devices"] = {
                name: compact_sensor_metrics(result)
                for name, result in device_results.items()
            }
            write_strict_json(report_path, report)

        if app_utils.is_playing():
            app_utils.stop(commit=True)

        if len(device_results) != len(logical_names):
            missing_results = sorted(set(logical_names) - set(device_results))
            raise SensorCaptureError(
                f"Missing capture results for: {missing_results}"
            )

        summary = summarize_capture_results(
            device_results,
            declared_logical_names=declared_logical_names,
            selected_logical_names=logical_names,
        )
        report["devices"] = {
            name: compact_sensor_metrics(result) for name, result in device_results.items()
        }
        report["device_details"] = device_results
        report["summary"] = summary
        report["artifacts"] = artifacts
        report["selection_ok"] = bool(summary["selection_ok"])
        report["full_profile_ok"] = bool(summary["full_profile_ok"])
        report["ok"] = bool(summary["ok"])
        report["captured_device_count"] = int(summary["captured_device_count"])
        report["where"] = "complete"
        exit_code = capture_exit_code(
            selection_ok=bool(summary["selection_ok"]),
            full_profile_ok=bool(summary["full_profile_ok"]),
        )
        write_strict_json(
            evidence_dir / "capture_summary.json",
            {
                "ok": report["ok"],
                "selection_ok": report["selection_ok"],
                "full_profile_ok": report["full_profile_ok"],
                "model": args.model,
                "declared_device_count": summary["declared_device_count"],
                "selected_device_count": summary["selected_device_count"],
                "captured_device_count": summary["captured_device_count"],
                "declared_logical_names": summary["declared_logical_names"],
                "selected_logical_names": summary["selected_logical_names"],
                "captured_logical_names": summary["captured_logical_names"],
                "unselected_declared_logical_names": summary[
                    "unselected_declared_logical_names"
                ],
                # Legacy alias of declared_device_count for older readers.
                "expected_device_count": summary["declared_device_count"],
                "summary": summary,
                "device_ok": summary["device_ok"],
                "per_sensor": summary["per_sensor"],
                "artifacts": artifacts,
                "head_zed_depth_frame": (
                    profile_by_name.get("head_zed_depth", {}).get("frame_name_suffix")
                ),
            },
        )
        write_strict_json(
            evidence_dir / "per_sensor_summary.json",
            summary["per_sensor"],
        )
    except Exception as error:
        report["ok"] = False
        report["selection_ok"] = False
        report["full_profile_ok"] = False
        report["error"] = str(error)
        report["traceback"] = traceback.format_exc()
        exit_code = 1
        _persist_failure_capsule(failure_capsule_path, report)
    finally:
        write_strict_json(report_path, report)
        if evidence_dir is not None:
            write_strict_json(evidence_dir / "capture_report.json", report)

    if simulation_app is None:
        return exit_code
    return finish_with_observable_exit(simulation_app, exit_code)


if __name__ == "__main__":
    sys.exit(main())
