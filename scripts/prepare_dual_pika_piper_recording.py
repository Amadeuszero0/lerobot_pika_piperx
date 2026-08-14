#!/usr/bin/env python3
"""Bind D435i roles, validate simultaneous RGB streams, and build record config.

This utility never opens Pika/Piper devices and never sends robot commands.
The existing teleoperation config is read as a template and is never modified.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BINDINGS = REPO_ROOT / "config" / "piper" / "d435i_roles_local.yaml"
DEFAULT_BASE_CONFIG = REPO_ROOT / "config" / "piper" / "dual_pika_piper_local.yaml"
DEFAULT_RECORD_CONFIG = (
    REPO_ROOT / "config" / "piper" / "dual_pika_piper_record_local.yaml"
)
ROLES = ("third_view", "left_wrist", "right_wrist")


@dataclass(frozen=True)
class CameraInfo:
    serial: str
    name: str
    usb: str
    physical_port: str


def load_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def write_yaml_atomic(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    os.replace(temporary, path)


def validate_bindings(data: dict[str, Any], *, require_complete: bool) -> dict[str, str | None]:
    roles = data.get("roles")
    if not isinstance(roles, dict):
        raise ValueError("bindings must contain a roles mapping")
    unknown = set(roles) - set(ROLES)
    if unknown:
        raise ValueError(f"unknown camera roles: {sorted(unknown)}")

    normalized: dict[str, str | None] = {}
    for role in ROLES:
        value = roles.get(role)
        if value is None or str(value).strip() == "":
            normalized[role] = None
        else:
            normalized[role] = str(value).strip()

    assigned = [serial for serial in normalized.values() if serial is not None]
    if len(assigned) != len(set(assigned)):
        raise ValueError("each camera role must use a distinct serial number")
    if require_complete:
        missing = [role for role, serial in normalized.items() if serial is None]
        if missing:
            raise ValueError(f"unbound camera roles: {', '.join(missing)}")
    return normalized


def camera_settings(data: dict[str, Any]) -> dict[str, Any]:
    settings = data.get("camera") or {}
    if not isinstance(settings, dict):
        raise ValueError("camera settings must be a mapping")
    result = {
        "width": int(settings.get("width", 640)),
        "height": int(settings.get("height", 480)),
        "fps": int(settings.get("fps", 30)),
        "require_usb3": bool(settings.get("require_usb3", True)),
    }
    if any(result[key] <= 0 for key in ("width", "height", "fps")):
        raise ValueError("camera width, height, and fps must be positive")
    return result


def discover_d435i() -> list[CameraInfo]:
    try:
        import pyrealsense2 as rs
    except Exception as exc:
        raise RuntimeError(f"pyrealsense2 is unavailable: {exc}") from exc

    def get_info(device: Any, field: Any, default: str = "unknown") -> str:
        try:
            return device.get_info(field) if device.supports(field) else default
        except Exception:
            return default

    cameras: list[CameraInfo] = []
    for device in rs.context().query_devices():
        name = get_info(device, rs.camera_info.name)
        if "D435I" not in name.upper():
            continue
        cameras.append(
            CameraInfo(
                serial=get_info(device, rs.camera_info.serial_number),
                name=name,
                usb=get_info(device, rs.camera_info.usb_type_descriptor),
                physical_port=get_info(device, rs.camera_info.physical_port),
            )
        )
    return sorted(cameras, key=lambda camera: camera.serial)


def validate_connected_cameras(
    bindings: dict[str, Any], cameras: list[CameraInfo]
) -> dict[str, CameraInfo]:
    roles = validate_bindings(bindings, require_complete=True)
    settings = camera_settings(bindings)
    by_serial = {camera.serial: camera for camera in cameras}
    expected_serials = {serial for serial in roles.values() if serial is not None}
    missing = sorted(expected_serials - set(by_serial))
    if missing:
        raise RuntimeError(f"bound D435i cameras are not connected: {', '.join(missing)}")
    extras = sorted(set(by_serial) - expected_serials)
    if extras:
        raise RuntimeError(
            "unbound D435i cameras are connected; bind or unplug them first: "
            + ", ".join(extras)
        )

    result: dict[str, CameraInfo] = {}
    for role, serial in roles.items():
        assert serial is not None
        camera = by_serial[serial]
        if settings["require_usb3"] and not camera.usb.startswith("3"):
            raise RuntimeError(
                f"{role} ({serial}) negotiated USB {camera.usb}; USB 3.x is required"
            )
        result[role] = camera
    return result


def make_camera_config(serial: str, settings: dict[str, Any]) -> dict[str, Any]:
    return {
        "type": "intelrealsense",
        "serial_number_or_name": serial,
        "width": settings["width"],
        "height": settings["height"],
        "fps": settings["fps"],
    }


def build_record_config(
    base_config: dict[str, Any],
    bindings: dict[str, Any],
    *,
    dataset_root: str | None = None,
    repo_id: str | None = None,
    num_episodes: int | None = None,
    episode_time_s: float | None = None,
    reset_time_s: float | None = None,
    single_task: str | None = None,
    move_speed_percent: int | None = None,
    no_startup_motion: bool = False,
) -> dict[str, Any]:
    roles = validate_bindings(bindings, require_complete=True)
    settings = camera_settings(bindings)
    result = copy.deepcopy(base_config)
    robots = result.get("robot", {}).get("robots", {})
    if set(robots) != {"left", "right"}:
        raise ValueError("base config must contain exactly left/right Piper robots")

    third_serial = roles["third_view"]
    left_serial = roles["left_wrist"]
    right_serial = roles["right_wrist"]
    assert third_serial and left_serial and right_serial

    # Keep the proven teleop/control settings untouched.  Only the generated
    # recording config receives cameras, physical joint-angle observations,
    # and automatic follower-role recovery.
    robots["left"]["cameras"] = {
        "third_view": make_camera_config(third_serial, settings),
        "wrist": make_camera_config(left_serial, settings),
    }
    robots["right"]["cameras"] = {
        "wrist": make_camera_config(right_serial, settings),
    }
    for robot in robots.values():
        robot["configure_role_on_connect"] = True
        robot["record_joint_angles"] = True
        if move_speed_percent is not None:
            robot["move_speed_percent"] = move_speed_percent

    if no_startup_motion:
        teleops = result.get("teleop", {}).get("teleops", {})
        if set(teleops) != {"left", "right"}:
            raise ValueError("base config must contain exactly left/right Pika teleoperators")
        for teleop in teleops.values():
            teleop["move_to_base_on_start"] = False

    dataset = result.setdefault("dataset", {})
    dataset["video"] = True
    overrides = {
        "root": dataset_root,
        "repo_id": repo_id,
        "num_episodes": num_episodes,
        "episode_time_s": episode_time_s,
        "reset_time_s": reset_time_s,
        "single_task": single_task,
    }
    dataset.update({key: value for key, value in overrides.items() if value is not None})
    return result


def stream_preflight(
    role_cameras: dict[str, CameraInfo], *, width: int, height: int, fps: int, seconds: float
) -> dict[str, dict[str, float | int | str]]:
    try:
        import pyrealsense2 as rs
    except Exception as exc:
        raise RuntimeError(f"pyrealsense2 is unavailable: {exc}") from exc

    pipelines: dict[str, Any] = {}
    counts = {role: 0 for role in role_cameras}
    last_frame: dict[str, float | None] = {role: None for role in role_cameras}
    max_gap = {role: 0.0 for role in role_cameras}
    try:
        for role, camera in role_cameras.items():
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(camera.serial)
            config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
            pipeline.start(config)
            pipelines[role] = pipeline

        warmup_deadline = time.monotonic() + 2.0
        while time.monotonic() < warmup_deadline:
            for pipeline in pipelines.values():
                pipeline.poll_for_frames()
            time.sleep(0.002)

        started = time.monotonic()
        deadline = started + seconds
        while time.monotonic() < deadline:
            received = False
            for role, pipeline in pipelines.items():
                frames = pipeline.poll_for_frames()
                if not frames or not frames.get_color_frame():
                    continue
                received = True
                now = time.monotonic()
                previous = last_frame[role]
                if previous is not None:
                    max_gap[role] = max(max_gap[role], now - previous)
                last_frame[role] = now
                counts[role] += 1
            if not received:
                time.sleep(0.002)

        elapsed = max(time.monotonic() - started, 0.001)
        result: dict[str, dict[str, float | int | str]] = {}
        failures: list[str] = []
        for role, camera in role_cameras.items():
            observed_fps = counts[role] / elapsed
            result[role] = {
                "serial": camera.serial,
                "usb": camera.usb,
                "frames": counts[role],
                "observed_fps": round(observed_fps, 2),
                "max_gap_s": round(max_gap[role], 4),
            }
            if observed_fps < fps * 0.8 or max_gap[role] >= 1.0:
                failures.append(
                    f"{role}: {observed_fps:.1f} fps, max gap {max_gap[role]:.3f}s"
                )
        if failures:
            raise RuntimeError("simultaneous RGB stream preflight failed: " + "; ".join(failures))
        return result
    finally:
        for pipeline in reversed(list(pipelines.values())):
            try:
                pipeline.stop()
            except Exception:
                pass


def save_snapshots(cameras: list[CameraInfo], output_dir: Path) -> None:
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
    except Exception as exc:
        raise RuntimeError(f"snapshot dependencies are unavailable: {exc}") from exc

    output_dir.mkdir(parents=True, exist_ok=True)
    for camera in cameras:
        pipeline = rs.pipeline()
        started = False
        config = rs.config()
        config.enable_device(camera.serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        try:
            pipeline.start(config)
            started = True
            frames = None
            for _ in range(30):
                frames = pipeline.wait_for_frames(2000)
            color = frames.get_color_frame() if frames else None
            if not color:
                raise RuntimeError(f"no color frame from {camera.serial}")
            image = np.asanyarray(color.get_data())
            cv2.putText(
                image,
                f"D435i {camera.serial} | USB {camera.usb}",
                (12, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            destination = output_dir / f"d435i_{camera.serial}.jpg"
            if not cv2.imwrite(str(destination), image):
                raise RuntimeError(f"failed to write {destination}")
            print(destination)
        finally:
            if started:
                pipeline.stop()


def print_discovery(cameras: list[CameraInfo], bindings: dict[str, Any]) -> None:
    roles = validate_bindings(bindings, require_complete=False)
    role_by_serial = {serial: role for role, serial in roles.items() if serial}
    rows = [
        {
            "role": role_by_serial.get(camera.serial, "UNBOUND"),
            "serial": camera.serial,
            "usb": camera.usb,
            "physical_port": camera.physical_port,
        }
        for camera in cameras
    ]
    print(json.dumps(rows, indent=2, ensure_ascii=False))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bindings", type=Path, default=DEFAULT_BINDINGS)
    subparsers = parser.add_subparsers(dest="command", required=True)

    discover = subparsers.add_parser("discover", help="list connected D435i cameras")
    discover.add_argument("--snapshots", type=Path)

    bind = subparsers.add_parser("bind", help="bind one physical role to a serial")
    bind.add_argument("role", choices=ROLES)
    bind.add_argument("serial")
    bind.add_argument("--allow-disconnected", action="store_true")

    generate = subparsers.add_parser("generate", help="build an independent record config")
    generate.add_argument("--base-config", type=Path, default=DEFAULT_BASE_CONFIG)
    generate.add_argument("--output", type=Path, default=DEFAULT_RECORD_CONFIG)
    generate.add_argument("--offline", action="store_true")
    generate.add_argument("--dataset-root")
    generate.add_argument("--repo-id")
    generate.add_argument("--num-episodes", type=int)
    generate.add_argument("--episode-time-s", type=float)
    generate.add_argument("--reset-time-s", type=float)
    generate.add_argument("--single-task")
    generate.add_argument("--move-speed-percent", type=int)
    generate.add_argument("--no-startup-motion", action="store_true")

    preflight = subparsers.add_parser(
        "preflight", help="validate roles, USB 3, and simultaneous RGB streams"
    )
    preflight.add_argument("--seconds", type=float, default=10.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bindings = load_yaml(args.bindings)

    if args.command == "discover":
        cameras = discover_d435i()
        print_discovery(cameras, bindings)
        if args.snapshots:
            save_snapshots(cameras, args.snapshots)
        return 0

    if args.command == "bind":
        if not args.allow_disconnected:
            connected = {camera.serial: camera for camera in discover_d435i()}
            camera = connected.get(args.serial)
            if camera is None:
                raise RuntimeError(f"D435i {args.serial} is not connected")
            settings = camera_settings(bindings)
            if settings["require_usb3"] and not camera.usb.startswith("3"):
                raise RuntimeError(
                    f"D435i {args.serial} is USB {camera.usb}; replace the cable first"
                )
        roles = bindings.setdefault("roles", {})
        roles[args.role] = str(args.serial)
        validate_bindings(bindings, require_complete=False)
        write_yaml_atomic(args.bindings, bindings)
        print(f"bound {args.role} -> {args.serial} in {args.bindings}")
        return 0

    if args.command == "generate":
        for name in ("num_episodes", "episode_time_s", "reset_time_s"):
            value = getattr(args, name)
            if value is not None and value <= 0:
                raise ValueError(f"--{name.replace('_', '-')} must be positive")
        if args.move_speed_percent is not None and not 1 <= args.move_speed_percent <= 100:
            raise ValueError("--move-speed-percent must be between 1 and 100")
        if not args.offline:
            validate_connected_cameras(bindings, discover_d435i())
        record_config = build_record_config(
            load_yaml(args.base_config),
            bindings,
            dataset_root=args.dataset_root,
            repo_id=args.repo_id,
            num_episodes=args.num_episodes,
            episode_time_s=args.episode_time_s,
            reset_time_s=args.reset_time_s,
            single_task=args.single_task,
            move_speed_percent=args.move_speed_percent,
            no_startup_motion=args.no_startup_motion,
        )
        write_yaml_atomic(args.output, record_config)
        print(f"record config written: {args.output}")
        print("base teleop config modified=false")
        print(
            "record_profile="
            + json.dumps(
                {
                    "root": record_config["dataset"].get("root"),
                    "repo_id": record_config["dataset"].get("repo_id"),
                    "num_episodes": record_config["dataset"].get("num_episodes"),
                    "episode_time_s": record_config["dataset"].get("episode_time_s"),
                    "move_speed_percent": {
                        side: robot.get("move_speed_percent")
                        for side, robot in record_config["robot"]["robots"].items()
                    },
                    "automatic_startup_motion": any(
                        teleop.get("move_to_base_on_start", True)
                        for teleop in record_config["teleop"]["teleops"].values()
                    ),
                },
                ensure_ascii=False,
            )
        )
        return 0

    if args.command == "preflight":
        if args.seconds <= 0:
            raise ValueError("--seconds must be positive")
        role_cameras = validate_connected_cameras(bindings, discover_d435i())
        settings = camera_settings(bindings)
        result = stream_preflight(
            role_cameras,
            width=settings["width"],
            height=settings["height"],
            fps=settings["fps"],
            seconds=args.seconds,
        )
        print(json.dumps(result, indent=2, ensure_ascii=False))
        print("camera_preflight=PASS")
        print("robot_commands_sent=false")
        return 0

    raise AssertionError(args.command)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
