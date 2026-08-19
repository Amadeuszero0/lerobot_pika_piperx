import sys
from pathlib import Path

import pytest

from lerobot_real.scripts.lerobot_record import get_cfg
from lerobot_real.scripts.piper_check_config import check
from lerobot_real.scripts.robot_teleop import get_cfg as get_teleop_cfg

TEMPLATE_NAMES = [
    "pika_piper.yaml",
    "piper_leader_follower.yaml",
    "dual_pika_direct.yaml",
    "dual_pika_piper.yaml",
    "dual_piper_leader_follower.yaml",
]
PLACEHOLDER_ERROR = "replace every REPLACE_* placeholder before hardware use"
PARSED_CONFIG_TYPES = {
    "pika_piper.yaml": ("PiperFollowerConfig", "PikaTeleopConfig"),
    "piper_leader_follower.yaml": ("PiperFollowerConfig", "PiperLeaderConfig"),
    "dual_pika_direct.yaml": ("MultipleMockRobotConfig", "DualPikaTeleopConfig"),
    "dual_pika_piper.yaml": ("DualPiperFollowerConfig", "DualPikaTeleopConfig"),
    "dual_piper_leader_follower.yaml": (
        "DualPiperFollowerConfig",
        "DualPiperLeaderConfig",
    ),
}
PIPER_X_TRACKER_TO_EEF = (-190, 0, 0, -90, 0, -90)


@pytest.mark.parametrize("template_name", TEMPLATE_NAMES)
def test_templates_have_valid_structure_but_require_site_values(template_name: str) -> None:
    repository_root = Path(__file__).resolve().parents[1]
    errors = check(repository_root / "config" / "piper" / template_name)
    expected = [] if template_name == "pika_piper.yaml" else [PLACEHOLDER_ERROR]
    assert errors == expected


@pytest.mark.parametrize("template_name", TEMPLATE_NAMES)
def test_templates_decode_into_runtime_configs(
    template_name: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    repository_root = Path(__file__).resolve().parents[1]
    config_path = repository_root / "config" / "piper" / template_name
    monkeypatch.setattr(sys, "argv", ["config-parse-check", "--config_path", str(config_path)])

    cfg = get_cfg()
    monkeypatch.setattr(
        sys,
        "argv",
        ["teleop-config-parse-check", "--config_path", str(config_path)],
    )
    teleop_cfg = get_teleop_cfg()

    expected_robot, expected_teleop = PARSED_CONFIG_TYPES[template_name]
    assert type(cfg.robot).__name__ == expected_robot
    assert type(cfg.teleop).__name__ == expected_teleop
    assert type(teleop_cfg.robot).__name__ == expected_robot
    assert type(teleop_cfg.teleop).__name__ == expected_teleop
    if template_name == "pika_piper.yaml":
        assert tuple(cfg.teleop.tracker_to_robot_eef) == PIPER_X_TRACKER_TO_EEF
        assert cfg.teleop.control_frame == "robot_base"
        assert tuple(cfg.teleop.tracker_world_to_robot_base_rpy) == (0, 0, 0)
        assert cfg.robot.cartesian_command_mode == "direct"
        assert cfg.robot.max_cartesian_following_error_mm == 600.0
        assert cfg.robot.max_rotation_following_error_rad == 3.2
        assert cfg.robot.max_cartesian_step_mm == 3.0
        assert cfg.robot.max_rotation_step_rad == 0.02
        assert cfg.robot.move_speed_percent == 30
        assert cfg.robot.hold_position_on_disconnect is True
        assert teleop_cfg.fps == 50
    elif template_name == "dual_pika_piper.yaml":
        assert all(
            tuple(child.tracker_to_robot_eef) == PIPER_X_TRACKER_TO_EEF
            for child in cfg.teleop.teleops.values()
        )
        assert teleop_cfg.fps == 50


def test_checker_accepts_complete_dual_arm_config(tmp_path: Path) -> None:
    config_path = tmp_path / "valid.yaml"
    config_path.write_text(
        """
robot:
  type: lerobot_real::dual_piper
  robots:
    left: {type: "lerobot_real::piper", port: can_left}
    right: {type: "lerobot_real::piper", port: can_right}
teleop:
  type: lerobot_real::dual_piper_leader
  teleops:
    left: {type: "lerobot_real::piper_leader", port: can_leader_left}
    right: {type: "lerobot_real::piper_leader", port: can_leader_right}
dataset:
  repo_id: local/test
  single_task: move the object
""".strip(),
        encoding="utf-8",
    )
    assert check(config_path) == []


def test_checker_accepts_complete_single_arm_config(tmp_path: Path) -> None:
    config_path = tmp_path / "single.yaml"
    config_path.write_text(
        """
robot:
  type: lerobot_real::piper
  port: can_follower
teleop:
  type: lerobot_real::piper_leader
  port: can_leader
dataset:
  repo_id: local/test
  single_task: move the object
""".strip(),
        encoding="utf-8",
    )
    assert check(config_path) == []


def test_checker_requires_single_arm_ports(tmp_path: Path) -> None:
    config_path = tmp_path / "missing_ports.yaml"
    config_path.write_text(
        """
robot:
  type: lerobot_real::piper
teleop:
  type: lerobot_real::pika_teleop
dataset:
  repo_id: local/test
  single_task: move the object
""".strip(),
        encoding="utf-8",
    )
    assert check(config_path) == [
        "single-arm robot.port is required",
        "single-arm teleop.port is required",
    ]


def test_checker_rejects_pika_base_pose_outside_piper_workspace(tmp_path: Path) -> None:
    config_path = tmp_path / "unsafe_base.yaml"
    config_path.write_text(
        """
robot:
  type: lerobot_real::piper
  port: can_follower
  workspace_x: [100, 600]
  workspace_y: [-500, 500]
  workspace_z: [50, 600]
teleop:
  type: lerobot_real::pika_teleop
  port: /dev/pika
  robot_base_pose: [50, 0, 250, 180, 0, 0]
dataset:
  repo_id: local/test
  single_task: move the object
""".strip(),
        encoding="utf-8",
    )

    assert check(config_path) == [
        "teleop.robot_base_pose x=50 is outside robot.workspace_x [100, 600]"
    ]


def test_checker_rejects_reused_ports(tmp_path: Path) -> None:
    config_path = tmp_path / "duplicate.yaml"
    config_path.write_text(
        """
robot:
  type: lerobot_real::dual_piper
  robots:
    left: {type: "lerobot_real::piper", port: can_shared}
    right: {type: "lerobot_real::piper", port: can_right}
teleop:
  type: lerobot_real::dual_piper_leader
  teleops:
    left: {type: "lerobot_real::piper_leader", port: can_shared}
    right: {type: "lerobot_real::piper_leader", port: can_leader_right}
dataset:
  repo_id: local/test
  single_task: move the object
""".strip(),
        encoding="utf-8",
    )
    assert check(config_path) == ["CAN/serial port names must not be reused in one configuration"]


def test_checker_rejects_invalid_gripper_ranges(tmp_path: Path) -> None:
    config_path = tmp_path / "bad_gripper.yaml"
    config_path.write_text(
        """
robot:
  type: lerobot_real::piper
  port: can_follower
  gripper_max_width_m: 0.12
teleop:
  type: lerobot_real::pika_teleop
  port: /dev/pika
  gripper_input_min_mm: 99
  gripper_input_max_mm: 98
dataset:
  repo_id: local/test
  single_task: move the object
""".strip(),
        encoding="utf-8",
    )

    assert check(config_path) == [
        "robot.gripper_max_width_m must be in (0, 0.1]",
        "teleop gripper input endpoints must satisfy 0 <= min < max <= 100 mm",
    ]


def test_checker_requires_persistent_distinct_dual_pika_trackers(tmp_path: Path) -> None:
    config_path = tmp_path / "dual_pika.yaml"
    config_path.write_text(
        """
robot:
  type: lerobot_real::multiple_mock_robot
  robots:
    left: {type: "lerobot_real::mock_robot", teleop_id: left_pika}
    right: {type: "lerobot_real::mock_robot", teleop_id: right_pika}
teleop:
  type: lerobot_real::dual_pika_teleop
  teleops:
    left: {type: "lerobot_real::pika_teleop", port: /dev/pika_left}
    right: {type: "lerobot_real::pika_teleop", port: /dev/pika_right}
dataset:
  repo_id: local/test
  single_task: move the object
""".strip(),
        encoding="utf-8",
    )
    assert check(config_path) == ["both dual-Pika sides must define tracker_device_id"]

    text = config_path.read_text(encoding="utf-8").replace(
        "port: /dev/pika_left}",
        "port: /dev/pika_left, tracker_device_id: T20}",
    ).replace(
        "port: /dev/pika_right}",
        "port: /dev/pika_right, tracker_device_id: T20}",
    )
    config_path.write_text(text, encoding="utf-8")
    assert check(config_path) == ["dual-Pika tracker_device_id values must be distinct"]

    config_path.write_text(text.replace("T20}", "T21}", 1), encoding="utf-8")
    assert check(config_path) == [
        "dual-Pika tracker_device_id values must use persistent LHR-* serials"
    ]
