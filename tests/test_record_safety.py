import pytest

from lerobot_real.scripts import lerobot_record
from lerobot_real.scripts._device_cleanup import disconnect_devices


def test_record_loop_persists_action_returned_by_robot(monkeypatch) -> None:
    events = {"exit_early": False}
    built_frames: list[tuple[str, dict]] = []

    class FakeTeleoperator:
        def get_action(self):
            return {"joint.pos": 100.0}

    class FakeRobot:
        name = "fake"
        robot_type = "fake"

        def get_observation(self):
            return {"joint.pos": 0.0}

        def send_action(self, action):
            return {"joint.pos": 5.0}

    class FakeDataset:
        fps = 30
        features = {}

        def add_frame(self, frame):
            events["exit_early"] = True

    def fake_build_dataset_frame(features, values, prefix):
        built_frames.append((prefix, dict(values)))
        return {}

    monkeypatch.setattr(lerobot_record, "Teleoperator", FakeTeleoperator)
    monkeypatch.setattr(lerobot_record, "build_dataset_frame", fake_build_dataset_frame)
    monkeypatch.setattr(lerobot_record, "precise_sleep", lambda duration: None)

    lerobot_record.record_loop(
        robot=FakeRobot(),
        events=events,
        fps=30,
        teleop_action_processor=lambda value: value[0],
        robot_action_processor=lambda value: value[0],
        robot_observation_processor=lambda value: value,
        dataset=FakeDataset(),
        teleop=FakeTeleoperator(),
        control_time_s=1,
        single_task="test",
    )

    assert (lerobot_record.ACTION, {"joint.pos": 5.0}) in built_frames
    assert (lerobot_record.ACTION, {"joint.pos": 100.0}) not in built_frames


def test_disconnect_devices_attempts_every_cleanup() -> None:
    calls: list[str] = []

    class Device:
        def __init__(self, name: str, fail: bool = False) -> None:
            self.name = name
            self.fail = fail

        def disconnect(self) -> None:
            calls.append(self.name)
            if self.fail:
                raise RuntimeError(self.name)

    devices = [Device("robot", fail=True), Device("teleop")]

    disconnect_devices(devices, suppress_errors=True)
    assert calls == ["robot", "teleop"]

    with pytest.raises(RuntimeError, match="robot"):
        disconnect_devices(devices, suppress_errors=False)


def test_headless_record_commands_update_only_recording_events() -> None:
    assert lerobot_record._parse_headless_record_command("\n") == "finish"
    assert lerobot_record._parse_headless_record_command("R\n") == "rerecord"
    assert lerobot_record._parse_headless_record_command("quit\n") == "quit"

    events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}

    lerobot_record._apply_record_command(events, "finish")
    assert events == {
        "exit_early": True,
        "rerecord_episode": False,
        "stop_recording": False,
    }

    events["exit_early"] = False
    lerobot_record._apply_record_command(events, "rerecord")
    assert events["exit_early"] is True
    assert events["rerecord_episode"] is True
    assert events["stop_recording"] is False

    events["exit_early"] = False
    events["rerecord_episode"] = False
    lerobot_record._apply_record_command(events, "quit")
    assert events["exit_early"] is True
    assert events["rerecord_episode"] is False
    assert events["stop_recording"] is True


def test_headless_episode_decision_requires_explicit_valid_choice() -> None:
    answers = iter(["", "invalid", "r"])

    assert lerobot_record._prompt_headless_episode_decision(
        lambda prompt: next(answers)
    ) == "rerecord"


def test_discard_current_episode_keeps_episode_index() -> None:
    class FakeDataset:
        def __init__(self) -> None:
            self.episode_buffer = {
                "size": 2,
                "episode_index": 3,
                "observation.state": [[1.0], [2.0]],
            }

        def clear_episode_buffer(self) -> None:
            self.episode_buffer = {
                "size": 0,
                "episode_index": 3,
                "observation.state": [],
            }

    dataset = FakeDataset()
    lerobot_record._discard_current_episode(dataset)

    assert dataset.episode_buffer["size"] == 0
    assert dataset.episode_buffer["episode_index"] == 3

    dataset = FakeDataset()
    lerobot_record._discard_current_episode(dataset, async_episode_saver=object())

    assert dataset.episode_buffer["size"] == 0
    assert dataset.episode_buffer["episode_index"] == 3
