"""CLI helpers for deployment camera configuration."""

from __future__ import annotations

import argparse


def add_realsense_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--camera",
        action="append",
        default=[],
        metavar="NAME=SERIAL",
        help="RealSense observation name and serial; repeat for multiple cameras",
    )
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)


def parse_camera_pairs(values: list[str]) -> dict[str, str]:
    pairs: dict[str, str] = {}
    for value in values:
        name, separator, serial = value.partition("=")
        if not separator or not name or not serial:
            raise ValueError(f"Camera must use NAME=SERIAL syntax, got {value!r}")
        if name in pairs:
            raise ValueError(f"Duplicate camera name: {name!r}")
        pairs[name] = serial
    return pairs


def make_realsense_configs(args: argparse.Namespace) -> dict:
    if args.camera_width <= 0 or args.camera_height <= 0 or args.camera_fps <= 0:
        raise ValueError("Camera width, height, and fps must be positive")
    from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig

    return {
        name: RealSenseCameraConfig(
            serial_number_or_name=serial,
            width=args.camera_width,
            height=args.camera_height,
            fps=args.camera_fps,
        )
        for name, serial in parse_camera_pairs(args.camera).items()
    }
