import copy
import importlib.util
import sys
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "prepare_dual_pika_piper_recording.py"
SPEC = importlib.util.spec_from_file_location("prepare_dual_pika_piper_recording", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


def bindings() -> dict:
    return {
        "camera": {"width": 640, "height": 480, "fps": 30, "require_usb3": True},
        "roles": {
            "third_view": "third",
            "left_wrist": "left",
            "right_wrist": "right",
        },
    }


def base_config() -> dict:
    return {
        "fps": 50,
        "robot": {
            "type": "lerobot_real::dual_piper",
            "robots": {
                "left": {
                    "type": "lerobot_real::piper",
                    "port": "can_left",
                    "cameras": {},
                    "move_speed_percent": 40,
                },
                "right": {
                    "type": "lerobot_real::piper",
                    "port": "can_right",
                    "cameras": {},
                    "move_speed_percent": 40,
                },
            },
        },
        "teleop": {
            "type": "lerobot_real::dual_pika_teleop",
            "marker": "unchanged",
            "teleops": {
                "left": {"robot_base_pose": [100, 0, 200, 0, 0, 0]},
                "right": {"robot_base_pose": [100, 0, 200, 0, 0, 0]},
            },
        },
        "dataset": {"repo_id": "local/test", "fps": 30},
    }


def test_record_config_keeps_base_teleop_and_control_settings_unchanged() -> None:
    base = base_config()
    original = copy.deepcopy(base)

    result = module.build_record_config(base, bindings())

    assert base == original
    assert result["teleop"] == original["teleop"]
    assert result["robot"]["robots"]["left"]["move_speed_percent"] == 40
    assert result["robot"]["robots"]["right"]["move_speed_percent"] == 40
    assert set(result["robot"]["robots"]["left"]["cameras"]) == {
        "third_view",
        "wrist",
    }
    assert set(result["robot"]["robots"]["right"]["cameras"]) == {"wrist"}
    assert (
        result["robot"]["robots"]["left"]["cameras"]["third_view"][
            "serial_number_or_name"
        ]
        == "third"
    )
    assert result["dataset"]["video"] is True
    for robot in result["robot"]["robots"].values():
        assert robot["record_joint_angles"] is True


def test_smoke_overrides_are_written_without_modifying_base() -> None:
    base = base_config()
    original = copy.deepcopy(base)

    result = module.build_record_config(
        base,
        bindings(),
        dataset_root="/tmp/smoke",
        repo_id="local/smoke",
        num_episodes=1,
        episode_time_s=10,
        reset_time_s=3,
        single_task="Pick and place test",
        move_speed_percent=10,
        no_startup_motion=True,
    )

    assert base == original
    assert result["dataset"]["root"] == "/tmp/smoke"
    assert result["dataset"]["repo_id"] == "local/smoke"
    assert result["dataset"]["num_episodes"] == 1
    assert result["dataset"]["episode_time_s"] == 10
    assert result["dataset"]["reset_time_s"] == 3
    assert result["dataset"]["single_task"] == "Pick and place test"
    for robot in result["robot"]["robots"].values():
        assert robot["move_speed_percent"] == 10
    for teleop in result["teleop"]["teleops"].values():
        assert teleop["move_to_base_on_start"] is False
        assert teleop["robot_base_pose"] is not None


def test_incomplete_or_duplicate_bindings_are_rejected() -> None:
    incomplete = bindings()
    incomplete["roles"]["right_wrist"] = None
    with pytest.raises(ValueError, match="unbound camera roles"):
        module.build_record_config(base_config(), incomplete)

    duplicate = bindings()
    duplicate["roles"]["right_wrist"] = "left"
    with pytest.raises(ValueError, match="distinct serial"):
        module.build_record_config(base_config(), duplicate)


def test_connected_camera_validation_requires_usb3_and_no_extras() -> None:
    CameraInfo = module.CameraInfo
    cameras = [
        CameraInfo("third", "D435I", "3.2", "1-1"),
        CameraInfo("left", "D435I", "3.2", "1-2"),
        CameraInfo("right", "D435I", "3.2", "1-3"),
    ]
    validated = module.validate_connected_cameras(bindings(), cameras)
    assert set(validated) == set(module.ROLES)

    usb2 = list(cameras)
    usb2[1] = CameraInfo("left", "D435I", "2.1", "2-1")
    with pytest.raises(RuntimeError, match="USB 2.1"):
        module.validate_connected_cameras(bindings(), usb2)

    extra = cameras + [CameraInfo("extra", "D435I", "3.2", "1-4")]
    with pytest.raises(RuntimeError, match="unbound D435i"):
        module.validate_connected_cameras(bindings(), extra)
