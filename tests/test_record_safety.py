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


def test_record_loop_uses_robot_specific_dataset_frame_builders(monkeypatch) -> None:
    events = {"exit_early": False}
    saved_frames: list[dict] = []

    class FakeTeleoperator:
        def get_action(self):
            return {"pose.x": 100.0}

    class FakeRobot:
        name = "fake"
        robot_type = "fake"

        def get_observation(self):
            return {"feedback": 1.0}

        def send_action(self, action):
            return {"effective_target": 2.0}

        def build_dataset_observation_frame(self, observation, features):
            return {"observation.custom": [observation["feedback"]]}

        def build_dataset_action_frame(self, sent_action, features):
            return {"action.custom": [sent_action["effective_target"]]}

    class FakeDataset:
        fps = 30
        features = {}

        def add_frame(self, frame):
            saved_frames.append(frame)
            events["exit_early"] = True

    monkeypatch.setattr(lerobot_record, "Teleoperator", FakeTeleoperator)
    monkeypatch.setattr(
        lerobot_record,
        "build_dataset_frame",
        lambda *args, **kwargs: pytest.fail("standard frame builder must not be used"),
    )
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

    assert saved_frames == [
        {
            "observation.custom": [1.0],
            "action.custom": [2.0],
            "task": "test",
        }
    ]


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


def test_dataset_finalization_manager_flushes_on_exception() -> None:
    calls: list[str] = []

    class Dataset:
        def finalize(self) -> None:
            calls.append("finalize")

    class Saver:
        def close(self) -> None:
            calls.append("close")

    with pytest.raises(RuntimeError, match="recording failed"):
        with (
            lerobot_record.DatasetFinalizationManager(Dataset()),
            lerobot_record.AsyncEpisodeSaverManager(Saver()),
        ):
            raise RuntimeError("recording failed")

    assert calls == ["close", "finalize"]


def test_headless_record_commands_update_only_recording_events() -> None:
    assert lerobot_record._parse_headless_record_command("s\n") == "finish"
    assert lerobot_record._parse_headless_record_command("\n") is None
    assert (
        lerobot_record._parse_headless_record_command(
            "\n", allow_blank_finish=True
        )
        == "finish"
    )
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


def test_headless_prepare_command_requires_active_teleoperation() -> None:
    assert (
        lerobot_record._parse_headless_prepare_command(
            "\n", teleop_ready=False
        )
        is None
    )
    assert (
        lerobot_record._parse_headless_prepare_command(
            "\n", teleop_ready=True
        )
        == "finish"
    )
    assert (
        lerobot_record._parse_headless_prepare_command(
            "q\n", teleop_ready=False
        )
        == "quit"
    )


def test_headless_episode_review_requires_explicit_choice() -> None:
    answers = iter(["", "invalid", "r"])

    assert lerobot_record._prompt_headless_episode_decision(
        lambda prompt: next(answers)
    ) == "rerecord"


def test_record_loop_supports_manual_episode_duration(monkeypatch) -> None:
    events = {
        "exit_early": False,
        "rerecord_episode": False,
        "stop_recording": False,
    }

    class FakeRobot:
        def get_observation(self):
            return {"joint.pos": 0.0}

    monkeypatch.setattr(lerobot_record, "precise_sleep", lambda duration: None)

    lerobot_record.record_loop(
        robot=FakeRobot(),
        events=events,
        fps=30,
        teleop_action_processor=lambda value: value[0],
        robot_action_processor=lambda value: value[0],
        robot_observation_processor=lambda value: value,
        control_time_s=None,
        command_poller=lambda: "finish",
    )

    assert events["exit_early"] is False


def test_prepare_loop_can_teleoperate_without_a_dataset(monkeypatch) -> None:
    events = {
        "exit_early": False,
        "rerecord_episode": False,
        "stop_recording": False,
    }
    sent_actions: list[dict] = []
    commands = iter([None, "finish"])

    class FakeTeleoperator:
        def get_action(self):
            return {"joint.pos": 1.0}

    class FakeRobot:
        name = "fake"

        def get_observation(self):
            return {"joint.pos": 0.0}

        def send_action(self, action):
            sent_actions.append(dict(action))
            return action

    monkeypatch.setattr(lerobot_record, "Teleoperator", FakeTeleoperator)
    monkeypatch.setattr(lerobot_record, "precise_sleep", lambda duration: None)

    lerobot_record.record_loop(
        robot=FakeRobot(),
        events=events,
        fps=30,
        teleop_action_processor=lambda value: value[0],
        robot_action_processor=lambda value: value[0],
        robot_observation_processor=lambda value: value,
        dataset=None,
        teleop=FakeTeleoperator(),
        control_time_s=None,
        command_poller=lambda: next(commands),
    )

    assert sent_actions == [{"joint.pos": 1.0}]


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
