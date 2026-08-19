#!/usr/bin/env python3
"""Read-only live check for the site dual-Pika/Piper configuration."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import yaml

from lerobot_real.devices.pika import PikaDevice
from lerobot_real.devices.piper.piper_motors_bus import (
    PiperFeedbackError,
    PiperMotorsBus,
)

SIDES = ("left", "right")
DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "piper"
    / "dual_pika_piper_local.yaml"
)


def _load_config(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    robots = data.get("robot", {}).get("robots", {})
    teleops = data.get("teleop", {}).get("teleops", {})
    if set(robots) != set(SIDES) or set(teleops) != set(SIDES):
        raise ValueError("configuration must contain left and right robots/teleops")
    return data


def _wait_for_piper_feedback(bus: PiperMotorsBus) -> tuple[Any, Any, tuple[float, ...]]:
    deadline = time.monotonic() + 5.0
    last_error: PiperFeedbackError | None = None
    while time.monotonic() < deadline:
        try:
            return (
                bus.get_arm_status(),
                bus.get_motor_status(require_enabled=False),
                bus.get_end_pose(),
            )
        except PiperFeedbackError as exc:
            last_error = exc
            time.sleep(0.05)
    raise TimeoutError(f"No fresh feedback from {bus.id} on {bus.port}") from last_error


def _read_gripper_max_range_mm(bus: PiperMotorsBus) -> int | None:
    """Query firmware jaw type when supported; this never sends a motion command."""
    query = getattr(bus.piper, "ArmParamEnquiryAndConfig", None)
    read = getattr(bus.piper, "GetGripperTeachingPendantParamFeedback", None)
    if query is None or read is None:
        return None
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        query(param_enquiry=0x04)
        time.sleep(0.1)
        message = read()
        feedback = message.arm_gripper_teaching_param_feedback
        value = int(feedback.max_range_config)
        if value in (70, 100):
            return value
    raise TimeoutError(f"No valid gripper range feedback from {bus.id} on {bus.port}")


def _read_pipers(config: dict[str, Any]) -> dict[str, Any]:
    buses: dict[str, PiperMotorsBus] = {}
    try:
        for side in SIDES:
            port = config["robot"]["robots"][side]["port"]
            bus = PiperMotorsBus(
                id=f"{side}_preflight",
                port=port,
                motors={},
                calibration={},
                feedback_timeout_s=0.5,
            )
            bus.connect(handshake=False)
            buses[side] = bus

        results: dict[str, Any] = {}
        for side, bus in buses.items():
            status, motors, end_pose = _wait_for_piper_feedback(bus)
            firmware_gripper_max_mm = _read_gripper_max_range_mm(bus)
            configured_gripper_max_mm = round(
                float(config["robot"]["robots"][side].get("gripper_max_width_m", 0.068))
                * 1000.0,
                3,
            )
            if (
                firmware_gripper_max_mm is not None
                and configured_gripper_max_mm > firmware_gripper_max_mm
            ):
                raise RuntimeError(
                    f"{side} configured gripper range {configured_gripper_max_mm} mm "
                    f"exceeds firmware jaw range {firmware_gripper_max_mm} mm"
                )
            enabled = [
                bool(
                    getattr(motors, f"motor_{joint_index}")
                    .foc_status.driver_enable_status
                )
                for joint_index in range(1, 7)
            ]
            results[side] = {
                "port": bus.port,
                "arm_status": int(status.arm_status),
                "error_code": int(status.err_code),
                "motors_enabled": enabled,
                "configured_gripper_max_mm": configured_gripper_max_mm,
                "firmware_gripper_max_mm": firmware_gripper_max_mm,
                "end_pose_xyz_mm_rpy_deg": [
                    round(value, 3) for value in end_pose
                ],
            }
        return results
    finally:
        for bus in reversed(list(buses.values())):
            if bus.is_connected:
                bus.piper.DisconnectPort()


def _read_pikas(config: dict[str, Any]) -> dict[str, Any]:
    devices: dict[str, PikaDevice] = {}
    senses: dict[str, Any] = {}
    try:
        for side in SIDES:
            teleop = config["teleop"]["teleops"][side]
            device = PikaDevice(
                1,
                pika_sense_port=teleop["port"],
                pika_tracker_device=teleop["tracker_device_id"],
            )
            devices[side] = device
            senses[side] = device.pika_sense

        results: dict[str, Any] = {}
        for side in SIDES:
            device = devices[side]
            sense = senses[side]
            pose = sense.get_pose(device.pika_tracker_device)
            if pose is None:
                raise RuntimeError(
                    f"{side} tracker {device.pika_tracker_device!r} has no fresh pose"
                )
            distance = sense.get_gripper_distance()
            command_state = sense.get_command_state()
            results[side] = {
                "port": config["teleop"]["teleops"][side]["port"],
                "tracker_device_id": device.pika_tracker_device,
                "position_m": [round(float(value), 6) for value in pose.position],
                "quaternion_xyzw": [
                    round(float(value), 6) for value in pose.rotation
                ],
                "gripper_distance_mm": (
                    None if distance is None else round(float(distance), 3)
                ),
                "command_state": (
                    None if command_state is None else int(command_state)
                ),
            }
        return results
    finally:
        for device in reversed(list(devices.values())):
            device.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", nargs="?", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()

    config = _load_config(args.config)
    result = {
        "piper": _read_pipers(config),
        "pika": _read_pikas(config),
        "robot_commands_sent": False,
        "checked_at_unix_s": round(time.time(), 3),
    }
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
