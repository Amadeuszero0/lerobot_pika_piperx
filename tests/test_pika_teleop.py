import math
from collections import deque
from importlib import import_module
from threading import Lock
from types import SimpleNamespace

import numpy as np
import pytest

from lerobot_real.configs.piper import DualPikaTeleopConfig
from lerobot_real.devices.pika import PikaDevice
from lerobot_real.devices.umi.vive_tracker.transformations import Transformations
from lerobot_real.scripts.robot_teleop import _activate_real_teleop
from lerobot_real.teleoperators.pika_teleop.dual_pika_teleop import DualPikaTeleop
from lerobot_real.teleoperators.pika_teleop.pika_teleop import POSE_KEYS, PikaTeleop
from lerobot_real.teleoperators.pika_teleop.pika_teleop_config import PikaTeleopConfig


class FakePikaSense:
    def __init__(self, pose_callback=None, gripper_sample=None) -> None:
        self.pose_callback = pose_callback
        self.gripper_sample = gripper_sample

    def get_pose(self, tracker):
        if self.pose_callback is not None:
            self.pose_callback()
        return None

    def get_gripper_distance(self):
        return self.gripper_sample


def make_teleop(
    sense: FakePikaSense,
    *,
    activation_mode: str = "gripper_gesture",
    prefix: str = "",
) -> PikaTeleop:
    teleop = object.__new__(PikaTeleop)
    teleop.config = SimpleNamespace(
        use_gripper=True,
        scale_xyz=1.0,
        activation_mode=activation_mode,
        activation_close_threshold_mm=15.0,
        activation_open_threshold_mm=70.0,
        gripper_input_min_mm=0.0,
        gripper_input_max_mm=100.0,
        control_frame="official",
        tracker_world_to_robot_base_rpy=(0.0, 0.0, 0.0),
    )
    teleop.prefix = "" if not prefix else f"{prefix}."
    teleop._is_connected = True
    teleop._data_lock = Lock()
    teleop._teleop_enabled = True
    teleop._last_robot_pose = [300.0, 0.0, 250.0, 0.0, 0.0, 0.0]
    teleop._last_gripper_pos = 0.25
    teleop._last_action = teleop._hold_action_locked()
    teleop._session_generation = 1
    teleop._thread_error = None
    teleop._activation_ready = False
    teleop._pending_robot_sync = False
    teleop._gesture_closed = False
    teleop._gesture_opened = False
    teleop.begin_tracker_matrix = None
    teleop.begin_tracker_robot_matrix = None
    teleop._tracker_matrix_window = deque(maxlen=3)
    teleop.robot_base_matrix = np.eye(4)
    teleop.tracker_to_robot_matrix = np.eye(4)
    teleop.robot_eef_to_tracker_matrix = np.eye(4)
    teleop.tracker_world_to_robot_base_rotation = np.eye(3)
    teleop.pika_device = SimpleNamespace(pika_tracker_device="tracker")
    teleop.pika_sense = sense
    return teleop


def piper_x_official_tool_matrix() -> np.ndarray:
    gripper_mount = Transformations.xyzrpy_to_rotation_matrix(
        0, 0, 0, 0, 0, math.pi / 2
    )
    pika_tool = (
        Transformations.xyzrpy_to_rotation_matrix(0, 0, 0, 0, -math.pi / 2, 0)
        @ Transformations.xyzrpy_to_rotation_matrix(190, 0, 0, 0, 0, 0)
    )
    return gripper_mount @ pika_tool


def test_none_gripper_sample_holds_previous_value() -> None:
    teleop = make_teleop(FakePikaSense(gripper_sample=None))

    action = teleop.get_action()

    assert action["gripper.pos"] == 0.25


def test_gripper_distance_maps_closed_to_zero_and_open_to_one() -> None:
    teleop = make_teleop(FakePikaSense(gripper_sample=80.0))

    action = teleop.get_action()

    assert action["gripper.pos"] == pytest.approx(0.8)


def test_gripper_distance_uses_calibrated_input_endpoints() -> None:
    teleop = make_teleop(FakePikaSense(gripper_sample=98.0))
    teleop.config.gripper_input_min_mm = 2.0
    teleop.config.gripper_input_max_mm = 98.0

    assert teleop.get_action()["gripper.pos"] == pytest.approx(1.0)

    teleop.pika_sense.gripper_sample = 2.0
    assert teleop.get_action()["gripper.pos"] == pytest.approx(0.0)

    teleop.pika_sense.gripper_sample = 50.0
    assert teleop.get_action()["gripper.pos"] == pytest.approx(0.5)


def test_pika_config_rejects_invalid_gripper_input_endpoints() -> None:
    with pytest.raises(ValueError, match="gripper input endpoints"):
        PikaTeleopConfig(
            port="/dev/pika-test",
            gripper_input_min_mm=98.0,
            gripper_input_max_mm=98.0,
        )


def test_button_state_change_during_sensor_read_discards_stale_sample() -> None:
    teleop = make_teleop(FakePikaSense())
    teleop.pika_sense.pose_callback = lambda: teleop.set_teleop_enabled(False)

    action = teleop.get_action()

    assert teleop._teleop_enabled is False
    assert action is None


def test_gripper_gesture_requires_full_close_open_close_while_waiting() -> None:
    teleop = make_teleop(FakePikaSense())
    teleop._teleop_enabled = False
    teleop._activation_ready = True

    assert teleop._update_gripper_gesture_locked(98.0) is False
    assert teleop._update_gripper_gesture_locked(10.0) is False
    assert teleop._update_gripper_gesture_locked(40.0) is False
    assert teleop._update_gripper_gesture_locked(75.0) is False
    assert teleop._update_gripper_gesture_locked(40.0) is False
    assert teleop._update_gripper_gesture_locked(10.0) is True
    assert teleop._teleop_enabled is False
    assert teleop._activation_ready is False
    assert teleop.has_pending_robot_sync() is True
    assert teleop._update_gripper_gesture_locked(98.0) is False


def test_gesture_activation_recaptures_current_robot_reference() -> None:
    teleop = make_teleop(FakePikaSense())
    stale_obs = {
        "pose.x": 300.0,
        "pose.y": 0.0,
        "pose.z": 250.0,
        "pose.rx": 0.0,
        "pose.ry": 0.0,
        "pose.rz": 0.0,
        "gripper.pos": 0.2,
    }
    current_obs = {
        "pose.x": 315.0,
        "pose.y": -8.0,
        "pose.z": 275.0,
        "pose.rx": 0.1,
        "pose.ry": -0.2,
        "pose.rz": 0.3,
        "gripper.pos": 0.7,
    }
    teleop.set_teleop_enabled(True, stale_obs)
    for distance in (10.0, 75.0, 10.0):
        teleop._update_gripper_gesture_locked(distance)

    assert teleop.apply_pending_robot_sync(current_obs) is True
    assert teleop.is_teleop_enabled is True
    assert teleop.has_pending_robot_sync() is False
    assert teleop._last_robot_pose == [315.0, -8.0, 275.0, 0.1, -0.2, 0.3]
    assert teleop._last_gripper_pos == 0.7


def test_dual_gesture_activation_synchronizes_both_robot_references_together() -> None:
    left = make_teleop(FakePikaSense(), prefix="left")
    right = make_teleop(FakePikaSense(), prefix="right")
    for teleop in (left, right):
        teleop._teleop_enabled = False
        teleop._activation_ready = True

    dual = object.__new__(DualPikaTeleop)
    dual.teleops = {"left": left, "right": right}

    for distance in (10.0, 75.0, 10.0):
        left._update_gripper_gesture_locked(distance)
    assert dual.has_pending_robot_sync() is False

    for distance in (10.0, 75.0, 10.0):
        right._update_gripper_gesture_locked(distance)
    assert dual.has_pending_robot_sync() is True

    obs = {
        **{
            f"left.{key}": value
            for key, value in zip(
                (*POSE_KEYS, "gripper.pos"),
                (310.0, 25.0, 270.0, 0.1, 0.2, 0.3, 0.4),
                strict=True,
            )
        },
        **{
            f"right.{key}": value
            for key, value in zip(
                (*POSE_KEYS, "gripper.pos"),
                (320.0, -25.0, 280.0, -0.1, -0.2, -0.3, 0.6),
                strict=True,
            )
        },
    }
    assert dual.apply_pending_robot_sync(obs) is True
    assert left.is_teleop_enabled is True
    assert right.is_teleop_enabled is True
    assert left._last_robot_pose[:3] == [310.0, 25.0, 270.0]
    assert right._last_robot_pose[:3] == [320.0, -25.0, 280.0]


def test_gesture_activation_primes_robot_reference_without_starting() -> None:
    teleop = make_teleop(FakePikaSense())
    obs = {
        "pose.x": 410.0,
        "pose.y": 20.0,
        "pose.z": 330.0,
        "pose.rx": 0.1,
        "pose.ry": 0.2,
        "pose.rz": 0.3,
        "gripper.pos": 0.6,
    }

    teleop.set_teleop_enabled(True, obs)

    assert teleop._teleop_enabled is False
    assert teleop._activation_ready is True
    assert teleop.has_pending_robot_sync() is False
    assert teleop._last_robot_pose == [410.0, 20.0, 330.0, 0.1, 0.2, 0.3]
    assert teleop._last_action["gripper.pos"] == 0.6
    assert teleop.get_action() is None


def test_pausing_cancels_a_pending_gesture() -> None:
    teleop = make_teleop(FakePikaSense())
    teleop._teleop_enabled = False
    teleop._activation_ready = True

    teleop.set_teleop_enabled(False)

    assert teleop._activation_ready is False
    assert teleop.has_pending_robot_sync() is False
    assert teleop._update_gripper_gesture_locked(10.0) is False
    assert teleop._update_gripper_gesture_locked(75.0) is False


def test_command_activation_starts_immediately() -> None:
    teleop = make_teleop(FakePikaSense(), activation_mode="command")
    teleop._teleop_enabled = False

    teleop.set_teleop_enabled(True)

    assert teleop._teleop_enabled is True


def test_piper_x_mapping_matches_official_gripper_center_formula() -> None:
    official_tool = piper_x_official_tool_matrix()
    tracker_to_sdk_end = Transformations.xyzrpy_to_rotation_matrix(
        -190, 0, 0, -math.pi / 2, 0, -math.pi / 2
    )
    assert np.allclose(tracker_to_sdk_end, np.linalg.inv(official_tool))

    sdk_base = Transformations.xyzrpy_to_rotation_matrix(300, 0, 250, math.pi, 0, 0)
    pika_start = Transformations.xyzrpy_to_rotation_matrix(100, 20, 300, 0.1, -0.2, 0.3)
    pika_current = Transformations.xyzrpy_to_rotation_matrix(115, 12, 325, -0.2, 0.4, 0.1)

    configured_target = (
        sdk_base
        @ np.linalg.inv(pika_start @ tracker_to_sdk_end)
        @ (pika_current @ tracker_to_sdk_end)
    )
    official_gripper_target = (
        (sdk_base @ official_tool) @ np.linalg.inv(pika_start) @ pika_current
    )
    official_sdk_target = official_gripper_target @ np.linalg.inv(official_tool)

    startup_target = (
        sdk_base
        @ np.linalg.inv(pika_start @ tracker_to_sdk_end)
        @ (pika_start @ tracker_to_sdk_end)
    )
    assert np.allclose(startup_target, sdk_base)
    assert np.allclose(configured_target, official_sdk_target)


def test_official_control_target_matches_unmodified_official_formula() -> None:
    teleop = make_teleop(FakePikaSense())
    sdk_base = Transformations.xyzrpy_to_rotation_matrix(
        300, -20, 250, 0.4, -0.2, 0.1
    )
    tracker_start = Transformations.xyzrpy_to_rotation_matrix(
        100, 20, 300, 0.1, -0.2, 0.3
    )
    tracker_current = Transformations.xyzrpy_to_rotation_matrix(
        115, 12, 325, -0.2, 0.4, 0.1
    )
    tracker_to_robot = Transformations.xyzrpy_to_rotation_matrix(
        -190, 0, 0, -math.pi / 2, 0, -math.pi / 2
    )
    teleop.robot_base_matrix = sdk_base
    teleop.begin_tracker_robot_matrix = tracker_start @ tracker_to_robot

    target = teleop._official_control_target(tracker_current @ tracker_to_robot)
    expected = (
        sdk_base
        @ np.linalg.inv(tracker_start @ tracker_to_robot)
        @ (tracker_current @ tracker_to_robot)
    )

    assert np.allclose(target, expected)


def test_official_control_target_ignores_robot_base_axis_correction() -> None:
    teleop = make_teleop(FakePikaSense())
    sdk_base = Transformations.xyzrpy_to_rotation_matrix(
        300, -20, 250, 0.4, -0.2, 0.1
    )
    tracker_start = Transformations.xyzrpy_to_rotation_matrix(
        100, 20, 300, 0.1, -0.2, 0.3
    )
    tracker_current = Transformations.xyzrpy_to_rotation_matrix(
        115, 12, 325, -0.2, 0.4, 0.1
    )
    tracker_to_robot = Transformations.xyzrpy_to_rotation_matrix(
        -190, 0, 0, -math.pi / 2, 0, -math.pi / 2
    )
    mapping = Transformations.rpy_to_rotation_matrix(0, math.pi, 0)
    teleop.robot_base_matrix = sdk_base
    teleop.begin_tracker_robot_matrix = tracker_start @ tracker_to_robot
    teleop.tracker_world_to_robot_base_rotation = mapping

    target = teleop._official_control_target(tracker_current @ tracker_to_robot)
    expected = (
        sdk_base
        @ np.linalg.inv(tracker_start @ tracker_to_robot)
        @ (tracker_current @ tracker_to_robot)
    )

    assert np.allclose(target, expected)


def test_tracker_filter_rejects_single_frame_translation_and_rotation_spike() -> None:
    teleop = make_teleop(FakePikaSense())
    steady = Transformations.xyzrpy_to_rotation_matrix(100, 20, 300, 0.1, -0.2, 0.3)
    spike = Transformations.xyzrpy_to_rotation_matrix(108, 14, 305, 0.5, 0.2, -0.4)

    first = teleop._filter_tracker_matrix(steady)
    during_spike = teleop._filter_tracker_matrix(spike)
    recovered = teleop._filter_tracker_matrix(steady)

    assert np.allclose(first, steady)
    assert np.allclose(during_spike, steady)
    assert np.allclose(recovered, steady)


def test_tracker_filter_has_one_frame_delay_during_continuous_motion() -> None:
    teleop = make_teleop(FakePikaSense())
    samples = [
        Transformations.xyzrpy_to_rotation_matrix(
            100 + index, 20, 300, 0, 0, math.radians(index)
        )
        for index in range(3)
    ]

    filtered = [teleop._filter_tracker_matrix(sample) for sample in samples]

    assert np.allclose(filtered[0], samples[0])
    assert np.allclose(filtered[1], samples[0])
    assert np.allclose(filtered[2], samples[1])


def test_activation_frame_mapping_uses_official_gripper_center() -> None:
    teleop = make_teleop(FakePikaSense())
    official_tool = piper_x_official_tool_matrix()
    teleop.tracker_to_robot_matrix = np.linalg.inv(official_tool)
    teleop.robot_eef_to_tracker_matrix = official_tool
    sdk_base = Transformations.xyzrpy_to_rotation_matrix(300, 0, 250, math.pi, 0, 0)
    tracker_start = Transformations.xyzrpy_to_rotation_matrix(100, 20, 300, 0.1, -0.2, 0.3)
    teleop.robot_base_matrix = sdk_base

    gripper_base, axis_mapping = teleop._activation_frame_mapping(tracker_start)

    expected_gripper_base = sdk_base @ official_tool
    assert np.allclose(gripper_base, expected_gripper_base)
    assert np.allclose(
        axis_mapping, expected_gripper_base[:3, :3] @ tracker_start[:3, :3].T
    )


def test_robot_base_control_maps_tracking_world_without_changing_tool_offset() -> None:
    teleop = make_teleop(FakePikaSense())
    teleop.config.control_frame = "robot_base"
    official_tool = piper_x_official_tool_matrix()
    teleop.tracker_to_robot_matrix = np.linalg.inv(official_tool)
    teleop.robot_eef_to_tracker_matrix = official_tool
    teleop.robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(
        300, 10, 250, 0.4, -0.2, 0.1
    )
    tracker_start = Transformations.xyzrpy_to_rotation_matrix(
        100, 20, 300, -0.1, 0.3, -0.2
    )
    teleop.begin_tracker_matrix = tracker_start
    mapping = Transformations.rpy_to_rotation_matrix(math.pi / 2, 0, 0)
    teleop.tracker_world_to_robot_base_rotation = mapping

    startup_target = teleop._robot_base_control_target(tracker_start)

    assert np.allclose(startup_target, teleop.robot_base_matrix)

    tracker_current = tracker_start.copy()
    tracker_delta_rotation = Transformations.rpy_to_rotation_matrix(0, 0, 0.2)
    tracker_current[:3, :3] = tracker_delta_rotation @ tracker_start[:3, :3]
    tracker_delta_position = np.array([10.0, -20.0, 30.0])
    tracker_current[:3, 3] += tracker_delta_position

    sdk_target = teleop._robot_base_control_target(tracker_current)
    gripper_start = teleop.robot_base_matrix @ official_tool
    gripper_target = sdk_target @ official_tool

    assert np.allclose(
        gripper_target[:3, 3],
        gripper_start[:3, 3] + mapping @ tracker_delta_position,
    )
    assert np.allclose(
        gripper_target[:3, :3],
        mapping @ tracker_delta_rotation @ mapping.T @ gripper_start[:3, :3],
    )

def test_activation_moves_to_configured_base_before_capturing_live_pose() -> None:
    events: list[object] = []
    obs = {
        "pose.x": 301.0,
        "pose.y": 1.0,
        "pose.z": 249.0,
        "pose.rx": 3.14,
        "pose.ry": 0.0,
        "pose.rz": 0.0,
        "gripper.pos": 0.5,
    }

    class Robot:
        def move_to_tcp_pose(self, target) -> None:
            events.append(("move", tuple(target)))

        def get_observation(self):
            events.append("observe")
            return obs

    class Teleop:
        config = SimpleNamespace(robot_base_pose=(300, 0, 250, 180, 0, 0))

        def set_teleop_enabled(self, enabled, observation) -> None:
            events.append(("enable", enabled, observation))

    _activate_real_teleop(Robot(), Teleop(), reset_to_base=True)

    assert events == [
        ("move", (300, 0, 250, 180, 0, 0)),
        "observe",
        ("enable", True, obs),
    ]


def test_dual_activation_moves_both_robots_to_their_configured_bases() -> None:
    events: list[object] = []
    obs = {"left.pose.x": 1.0, "right.pose.x": 2.0}

    class ChildRobot:
        def __init__(self, side: str) -> None:
            self.side = side

        def move_to_tcp_pose(self, target) -> None:
            events.append(("move", self.side, tuple(target)))

    class Robot:
        robots = {
            "left": ChildRobot("left"),
            "right": ChildRobot("right"),
        }

        def get_observation(self):
            events.append("observe")
            return obs

    class ChildTeleop:
        def __init__(self, base_pose) -> None:
            self.config = SimpleNamespace(robot_base_pose=base_pose)

    class Teleop:
        teleops = {
            "left": ChildTeleop((300, 150, 250, 180, 0, 0)),
            "right": ChildTeleop((300, -150, 250, 180, 0, 0)),
        }

        def set_teleop_enabled(self, enabled, observation) -> None:
            events.append(("enable", enabled, observation))

    _activate_real_teleop(Robot(), Teleop(), reset_to_base=True)

    assert events == [
        ("move", "left", (300, 150, 250, 180, 0, 0)),
        ("move", "right", (300, -150, 250, 180, 0, 0)),
        "observe",
        ("enable", True, obs),
    ]


def test_pika_device_disconnect_releases_all_devices_and_cache(monkeypatch) -> None:
    calls: list[str] = []

    class Device:
        def __init__(self, name: str, fail: bool = False) -> None:
            self.name = name
            self.fail = fail

        def disconnect(self) -> None:
            calls.append(self.name)
            if self.fail:
                raise RuntimeError(self.name)

    sense = Device("sense", fail=True)
    gripper = Device("gripper")
    monkeypatch.setattr(PikaDevice, "PIKA_DEVICE_MAP", {"sense-port": sense, "grip-port": gripper})
    pika = object.__new__(PikaDevice)
    pika._pika_sense_port = "sense-port"
    pika._pika_gripper_port = "grip-port"
    pika._pika_sense = sense
    pika._pika_gripper = gripper

    with pytest.raises(RuntimeError, match="sense"):
        pika.disconnect()

    assert calls == ["sense", "gripper"]
    assert PikaDevice.PIKA_DEVICE_MAP == {}


def test_pika_device_waits_for_configured_tracker_without_auto_discovery(monkeypatch) -> None:
    pika_sense_module = import_module("pika.sense")
    pika_device_module = import_module("lerobot_real.devices.pika.pika_device")

    tracker_reads: list[str] = []

    class Sense:
        def __init__(self, port: str) -> None:
            self.port = port

        def connect(self) -> bool:
            return True

        def disconnect(self) -> None:
            return None

        def get_vive_tracker(self) -> object:
            return object()

        def get_pose(self, tracker: str) -> object:
            tracker_reads.append(tracker)
            return object()

        def get_tracker_devices(self) -> list[str]:
            raise AssertionError("configured tracker must not use transient discovery")

    monkeypatch.setattr(pika_sense_module, "Sense", Sense)
    monkeypatch.setattr(pika_device_module.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(PikaDevice, "PIKA_DEVICE_MAP", {})
    device = PikaDevice(
        1,
        pika_sense_port="/dev/pika_left",
        pika_tracker_device="LHR-818D4A5D",
    )

    assert device.pika_sense.port == "/dev/pika_left"
    assert device.pika_tracker_device == "LHR-818D4A5D"
    assert tracker_reads == ["LHR-818D4A5D"]
    device.disconnect()


def test_pika_teleop_uses_configured_tracker(monkeypatch) -> None:
    import lerobot_real.teleoperators.pika_teleop.pika_teleop as pika_teleop_module

    class Device:
        def __init__(self, *_args, **kwargs) -> None:
            self.pika_tracker_device = kwargs.get("pika_tracker_device", "T20")
            self.pika_sense = object()

    monkeypatch.setattr(pika_teleop_module, "PikaDevice", Device)
    teleop = PikaTeleop(
        PikaTeleopConfig(
            port="/dev/pika_left",
            tracker_device_id="LHR-818D4A5D",
        )
    )

    assert teleop.pika_device.pika_tracker_device == "LHR-818D4A5D"


def test_dual_pika_config_requires_distinct_ports_and_trackers() -> None:
    left = PikaTeleopConfig(
        port="/dev/pika_left",
        tracker_device_id="LHR-LEFT",
    )
    right = PikaTeleopConfig(
        port="/dev/pika_right",
        tracker_device_id="LHR-RIGHT",
    )

    config = DualPikaTeleopConfig(teleops={"left": left, "right": right})
    assert config.teleops == {"left": left, "right": right}

    right.tracker_device_id = left.tracker_device_id
    with pytest.raises(ValueError, match="distinct tracker_device_id"):
        DualPikaTeleopConfig(teleops={"left": left, "right": right})

    right.tracker_device_id = "LHR-RIGHT"
    right.port = left.port
    with pytest.raises(ValueError, match="distinct serial ports"):
        DualPikaTeleopConfig(teleops={"left": left, "right": right})

    right.port = "/dev/pika_right"
    right.tracker_device_id = None
    with pytest.raises(ValueError, match="must define tracker_device_id"):
        DualPikaTeleopConfig(teleops={"left": left, "right": right})


@pytest.mark.parametrize(
    "kwargs",
    [
        {"frequency": 0},
        {"scale_xyz": float("nan")},
        {"tracker_device_id": ""},
        {"tracker_to_robot_eef": (0, 0, 0)},
        {"activation_mode": "invalid"},
        {"control_frame": "invalid"},
        {"tracker_world_to_robot_base_rpy": (0, 0)},
        {"activation_close_threshold_mm": 70, "activation_open_threshold_mm": 15},
    ],
)
def test_invalid_pika_runtime_config_is_rejected(kwargs) -> None:
    with pytest.raises(ValueError):
        PikaTeleopConfig(**kwargs)
