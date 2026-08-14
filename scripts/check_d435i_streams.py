#!/usr/bin/env python3
"""Open every connected D435i color stream together and save labeled snapshots."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pyrealsense2 as rs


@dataclass
class StreamState:
    serial: str
    name: str
    usb: str
    pipeline: Any
    frames: int = 0
    last_frame_at: float | None = None
    max_gap_s: float = 0.0
    snapshot_written: bool = False


def device_info(device: Any, field: Any, default: str = "") -> str:
    try:
        return device.get_info(field) if device.supports(field) else default
    except Exception:
        return default


def find_d435i() -> list[dict[str, str]]:
    found = []
    for device in rs.context().query_devices():
        name = device_info(device, rs.camera_info.name)
        if "D435I" not in name.upper():
            continue
        found.append(
            {
                "name": name,
                "serial": device_info(device, rs.camera_info.serial_number),
                "usb": device_info(device, rs.camera_info.usb_type_descriptor, "unknown"),
            }
        )
    return sorted(found, key=lambda item: item["serial"])


def start_stream(device: dict[str, str], width: int, height: int, fps: int) -> StreamState:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(device["serial"])
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    pipeline.start(config)
    return StreamState(
        serial=device["serial"],
        name=device["name"],
        usb=device["usb"],
        pipeline=pipeline,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--expected", type=int, default=3)
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/d435i_check"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    devices = find_d435i()
    print("Detected D435i devices:")
    print(json.dumps(devices, indent=2, ensure_ascii=False))
    if len(devices) != args.expected:
        print(f"FAIL: expected {args.expected} D435i devices, found {len(devices)}")
        return 2

    args.output_dir.mkdir(parents=True, exist_ok=True)
    states: list[StreamState] = []
    try:
        for device in devices:
            try:
                state = start_stream(device, args.width, args.height, args.fps)
            except Exception as exc:
                print(
                    f"FAIL: could not start {device['serial']} ({device['usb']}): "
                    f"{type(exc).__name__}: {exc}"
                )
                return 2
            states.append(state)
            print(
                f"Started {state.serial}: {args.width}x{args.height}@{args.fps}, USB {state.usb}"
            )

        # Let auto exposure settle while all pipelines are active.
        warmup_deadline = time.monotonic() + 3.0
        while time.monotonic() < warmup_deadline:
            for state in states:
                state.pipeline.poll_for_frames()
            time.sleep(0.002)

        started_at = time.monotonic()
        deadline = started_at + args.seconds
        while time.monotonic() < deadline:
            got_frame = False
            for state in states:
                frameset = state.pipeline.poll_for_frames()
                if not frameset:
                    continue
                color = frameset.get_color_frame()
                if not color:
                    continue
                got_frame = True
                now = time.monotonic()
                if state.last_frame_at is not None:
                    state.max_gap_s = max(state.max_gap_s, now - state.last_frame_at)
                state.last_frame_at = now
                state.frames += 1
                if not state.snapshot_written:
                    image = np.asanyarray(color.get_data())
                    cv2.putText(
                        image,
                        f"D435i {state.serial} | USB {state.usb}",
                        (12, 32),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.72,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    destination = args.output_dir / f"d435i_{state.serial}.jpg"
                    if not cv2.imwrite(str(destination), image):
                        raise RuntimeError(f"failed to write {destination}")
                    state.snapshot_written = True
            if not got_frame:
                time.sleep(0.002)

        elapsed = max(time.monotonic() - started_at, 0.001)
        minimum_frames = args.fps * args.seconds * 0.7
        failed = False
        print("\nStream results:")
        for state in states:
            observed_fps = state.frames / elapsed
            ok = state.frames >= minimum_frames and state.max_gap_s < 1.0
            failed = failed or not ok
            print(
                f"[{'OK' if ok else 'FAIL'}] {state.serial}: frames={state.frames}, "
                f"observed_fps={observed_fps:.1f}, max_gap={state.max_gap_s:.3f}s, "
                f"USB={state.usb}"
            )
        print(f"Snapshots: {args.output_dir}")
        print("robot_commands_sent=false")
        return 1 if failed else 0
    finally:
        for state in reversed(states):
            try:
                state.pipeline.stop()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
