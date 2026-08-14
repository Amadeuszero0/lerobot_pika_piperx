from types import SimpleNamespace

import pytest

from lerobot_real.robots.mock_robot import multiple_mock_robot as module
from lerobot_real.robots.mock_robot.mock_robot import MockRobot


def test_dual_camera_registry_preserves_side_prefixes(monkeypatch, tmp_path) -> None:
    class FakeMockRobot:
        def __init__(self, config, prefix: str) -> None:
            self.cameras = {"wrist": object()}

    monkeypatch.setattr(module, "MockRobot", FakeMockRobot)
    config = SimpleNamespace(
        id="dual-mock",
        calibration_dir=tmp_path,
        robots={"left": object(), "right": object()},
    )

    robot = module.MultipleMockRobot(config)

    assert set(robot.cameras) == {"left.wrist", "right.wrist"}


def test_partial_connect_rolls_back_already_connected_robot() -> None:
    class FakeRobot:
        def __init__(self, fail: bool = False) -> None:
            self.fail = fail
            self.connected = False
            self.disconnect_calls = 0

        @property
        def is_connected(self) -> bool:
            return self.connected

        def connect(self, calibrate: bool = True) -> None:
            if self.fail:
                raise RuntimeError("connect failed")
            self.connected = True

        def disconnect(self) -> None:
            self.disconnect_calls += 1
            self.connected = False

    left = FakeRobot()
    right = FakeRobot(fail=True)
    robot = object.__new__(module.MultipleMockRobot)
    robot.robots = {"left": left, "right": right}

    with pytest.raises(RuntimeError, match="connect failed"):
        robot.connect()

    assert left.disconnect_calls == 1


def test_mock_robot_connection_flags_are_properties() -> None:
    assert isinstance(MockRobot.is_connected, property)
    assert isinstance(MockRobot.is_calibrated, property)


def test_send_action_returns_each_childs_executed_action() -> None:
    class FakeRobot:
        def send_action(self, action):
            key = next(iter(action))
            return {key: action[key] / 2}

    robot = object.__new__(module.MultipleMockRobot)
    robot.robots = {"left": FakeRobot(), "right": FakeRobot()}

    sent = robot.send_action({"left.pose.x": 10.0, "right.pose.x": 20.0})

    assert sent == {"left.pose.x": 5.0, "right.pose.x": 10.0}


def test_mock_robot_camera_connect_failure_rolls_back_open_cameras() -> None:
    class Camera:
        def __init__(self, fail: bool = False) -> None:
            self.fail = fail
            self.is_connected = False
            self.disconnect_calls = 0

        def connect(self) -> None:
            if self.fail:
                raise RuntimeError("camera failed")
            self.is_connected = True

        def disconnect(self) -> None:
            self.disconnect_calls += 1
            self.is_connected = False

    first = Camera()
    second = Camera(fail=True)
    robot = object.__new__(MockRobot)
    robot.cameras = {"first": first, "second": second}
    robot._is_connected = False
    robot.config = SimpleNamespace(id="test")

    with pytest.raises(RuntimeError, match="camera failed"):
        robot.connect(calibrate=False)

    assert first.disconnect_calls == 1
    assert not robot.is_connected


def test_mock_robot_disconnect_attempts_every_camera() -> None:
    calls: list[str] = []

    class Camera:
        is_connected = True

        def __init__(self, name: str, fail: bool = False) -> None:
            self.name = name
            self.fail = fail

        def disconnect(self) -> None:
            calls.append(self.name)
            if self.fail:
                raise RuntimeError(self.name)
            self.is_connected = False

    robot = object.__new__(MockRobot)
    robot.cameras = {"first": Camera("first", fail=True), "second": Camera("second")}
    robot._is_connected = True

    with pytest.raises(RuntimeError, match="first"):
        robot.disconnect()

    assert calls == ["first", "second"]
    assert not robot.is_connected
