import argparse
import math
from pathlib import Path
from typing import Any

import yaml

MULTI_ROBOT_TYPES = {
    "lerobot_real::dual_piper",
    "lerobot_real::multiple_mock_robot",
}
MULTI_TELEOP_TYPES = {
    "lerobot_real::dual_pika_teleop",
    "lerobot_real::dual_piper_leader",
}
SINGLE_ROBOT_PORT_TYPES = {"lerobot_real::piper"}
SINGLE_TELEOP_PORT_TYPES = {
    "lerobot_real::pika_teleop",
    "lerobot_real::piper_leader",
}


def _is_multi(config: dict[str, Any], child_key: str, multi_types: set[str]) -> bool:
    return config.get("type") in multi_types or child_key in config


def _contains_placeholder(value: Any) -> bool:
    if isinstance(value, str):
        return "REPLACE_" in value
    if isinstance(value, dict):
        return any(_contains_placeholder(item) for item in value.values())
    if isinstance(value, list):
        return any(_contains_placeholder(item) for item in value)
    return False


def check(path: Path) -> list[str]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    errors: list[str] = []
    robot = data.get("robot", {})
    teleop = data.get("teleop", {})
    dataset = data.get("dataset", {})
    robots = robot.get("robots", {})
    teleops = teleop.get("teleops", {})
    robot_is_multi = _is_multi(robot, "robots", MULTI_ROBOT_TYPES)
    teleop_is_multi = _is_multi(teleop, "teleops", MULTI_TELEOP_TYPES)

    if not robot.get("type") or not teleop.get("type"):
        errors.append("robot.type and teleop.type are required")
    if robot_is_multi and set(robots) != {"left", "right"}:
        errors.append("robot.robots must contain exactly left and right")
    if teleop_is_multi and set(teleops) != {"left", "right"}:
        errors.append("teleop.teleops must contain exactly left and right")
    if robot_is_multi != teleop_is_multi:
        errors.append("robot and teleop must both be single-arm or multi-arm")
    if robot_is_multi and teleop_is_multi and set(robots) != set(teleops):
        errors.append("robot and teleop side names must match")
    if teleop.get("type") == "lerobot_real::dual_pika_teleop" and set(teleops) == {
        "left",
        "right",
    }:
        tracker_ids = [teleops[side].get("tracker_device_id") for side in ("left", "right")]
        if not all(tracker_ids):
            errors.append("both dual-Pika sides must define tracker_device_id")
        elif len(set(tracker_ids)) != len(tracker_ids):
            errors.append("dual-Pika tracker_device_id values must be distinct")
        elif any(
            not tracker_id.startswith("LHR-") and not _contains_placeholder(tracker_id)
            for tracker_id in tracker_ids
        ):
            errors.append("dual-Pika tracker_device_id values must use persistent LHR-* serials")
    if (
        not robot_is_multi
        and robot.get("type") in SINGLE_ROBOT_PORT_TYPES
        and not robot.get("port")
    ):
        errors.append("single-arm robot.port is required")
    if (
        not teleop_is_multi
        and teleop.get("type") in SINGLE_TELEOP_PORT_TYPES
        and not teleop.get("port")
    ):
        errors.append("single-arm teleop.port is required")
    if not dataset.get("repo_id") or not dataset.get("single_task"):
        errors.append("dataset.repo_id and dataset.single_task are required")
    if _contains_placeholder(data):
        errors.append("replace every REPLACE_* placeholder before hardware use")

    config_pairs = (
        [(side, robots[side], teleops[side]) for side in robots]
        if robot_is_multi and teleop_is_multi and set(robots) == set(teleops)
        else [("", robot, teleop)]
    )
    for side, robot_config, teleop_config in config_pairs:
        if (
            robot_config.get("type") != "lerobot_real::piper"
            or teleop_config.get("type") != "lerobot_real::pika_teleop"
        ):
            continue
        base_pose = teleop_config.get("robot_base_pose")
        label = f"{side} " if side else ""
        command_mode = robot_config.get("cartesian_command_mode", "step")
        if command_mode not in {"step", "direct", "official_ik"}:
            errors.append(
                f"{label}robot.cartesian_command_mode must be 'step', 'direct', "
                "or 'official_ik'"
            )
        if command_mode == "official_ik":
            for name in ("ik_urdf_path", "ik_package_dir"):
                if not robot_config.get(name):
                    errors.append(f"{label}robot.{name} is required for official_ik")
        for name in (
            "max_cartesian_following_error_mm",
            "max_rotation_following_error_rad",
        ):
            value = robot_config.get(name, 1.0)
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value <= 0
            ):
                errors.append(f"{label}robot.{name} must be finite and positive")
        hold_on_disconnect = robot_config.get("hold_position_on_disconnect", False)
        if not isinstance(hold_on_disconnect, bool):
            errors.append(f"{label}robot.hold_position_on_disconnect must be boolean")
        gripper_max_width_m = robot_config.get("gripper_max_width_m", 0.068)
        if (
            not isinstance(gripper_max_width_m, (int, float))
            or isinstance(gripper_max_width_m, bool)
            or not math.isfinite(gripper_max_width_m)
            or not 0 < gripper_max_width_m <= 0.1
        ):
            errors.append(f"{label}robot.gripper_max_width_m must be in (0, 0.1]")
        gripper_input_min_mm = teleop_config.get("gripper_input_min_mm", 0.0)
        gripper_input_max_mm = teleop_config.get("gripper_input_max_mm", 100.0)
        if (
            not isinstance(gripper_input_min_mm, (int, float))
            or isinstance(gripper_input_min_mm, bool)
            or not isinstance(gripper_input_max_mm, (int, float))
            or isinstance(gripper_input_max_mm, bool)
            or not math.isfinite(gripper_input_min_mm)
            or not math.isfinite(gripper_input_max_mm)
            or gripper_input_min_mm < 0
            or gripper_input_max_mm > 100
            or gripper_input_min_mm >= gripper_input_max_mm
        ):
            errors.append(
                f"{label}teleop gripper input endpoints must satisfy "
                "0 <= min < max <= 100 mm"
            )
        control_frame = teleop_config.get("control_frame", "official")
        if control_frame not in {"official", "robot_base"}:
            errors.append(
                f"{label}teleop.control_frame must be 'official' or 'robot_base'"
            )
        world_to_base_rpy = teleop_config.get(
            "tracker_world_to_robot_base_rpy", [0, 0, 0]
        )
        if (
            not isinstance(world_to_base_rpy, (list, tuple))
            or len(world_to_base_rpy) != 3
            or not all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in world_to_base_rpy
            )
        ):
            errors.append(
                f"{label}teleop.tracker_world_to_robot_base_rpy must contain "
                "three finite values"
            )
        if base_pose is None:
            continue
        if (
            not isinstance(base_pose, (list, tuple))
            or len(base_pose) != 6
            or not all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in base_pose
            )
        ):
            errors.append(f"{label}teleop.robot_base_pose must contain six finite values")
            continue
        for axis, value in zip("xyz", base_pose[:3], strict=True):
            bounds = robot_config.get(f"workspace_{axis}")
            if bounds is not None and len(bounds) == 2 and not bounds[0] <= value <= bounds[1]:
                errors.append(
                    f"{label}teleop.robot_base_pose {axis}={value} is outside "
                    f"robot.workspace_{axis} {bounds}"
                )

    robot_configs = list(robots.values()) if robot_is_multi else [robot]
    teleop_configs = list(teleops.values()) if teleop_is_multi else [teleop]
    ports = [
        config.get("port")
        for config in [*robot_configs, *teleop_configs]
        if isinstance(config, dict) and config.get("port")
    ]
    if len(ports) != len(set(ports)):
        errors.append("CAN/serial port names must not be reused in one configuration")
    return errors


def main() -> None:
    parser = argparse.ArgumentParser(description="Static check for Piper YAML files")
    parser.add_argument("paths", nargs="+", type=Path)
    args = parser.parse_args()
    failed = False
    for path in args.paths:
        errors = check(path)
        if errors:
            failed = True
            print(f"[FAIL] {path}")
            for error in errors:
                print(f"  - {error}")
        else:
            print(f"[OK]   {path}")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
