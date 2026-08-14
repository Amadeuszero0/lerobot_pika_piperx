import time
from types import SimpleNamespace

import pytest

import lerobot_real.robots.piper.piper_follower as piper_follower_module
from lerobot_real.configs.piper import PiperFollowerConfig
from lerobot_real.devices.piper.piper_motors_bus import (
    PiperFeedbackError,
    PiperFeedbackStaleError,
)
from lerobot_real.robots.piper.piper_follower import (
    JOINT_ANGLE_KEYS,
    JOINT_KEYS,
    POSE_KEYS,
    PiperFollower,
)
from lerobot_real.teleoperators.piper_leader.piper_leader import PiperLeader


class FakeLeaderBus:
    def __init__(self, *, fail_feedback: bool = False) -> None:
        self.calls: list[object] = []
        self.fail_feedback = fail_feedback

    def connect(self) -> None:
        self.calls.append("connect")

    def set_leader(self) -> None:
        self.calls.append("set_leader")

    def wait_for_leader_feedback(self, timeout_s: float) -> None:
        self.calls.append(("wait", timeout_s))
        if self.fail_feedback:
            raise TimeoutError("no leader feedback")

    def disconnect(self, *, disable_torque: bool, park: bool) -> None:
        self.calls.append(("disconnect", disable_torque, park))


def make_leader(bus: FakeLeaderBus) -> PiperLeader:
    leader = object.__new__(PiperLeader)
    leader.config = SimpleNamespace(
        configure_role_on_connect=True,
        feedback_startup_timeout_s=2.0,
    )
    leader.bus = bus
    return leader


def test_leader_connect_stays_backdrivable_and_waits_for_control_feedback() -> None:
    bus = FakeLeaderBus()
    make_leader(bus).connect()

    assert bus.calls == ["connect", "set_leader", ("wait", 2.0)]


def test_leader_connect_cleans_up_when_feedback_never_arrives() -> None:
    bus = FakeLeaderBus(fail_feedback=True)

    with pytest.raises(TimeoutError, match="no leader feedback"):
        make_leader(bus).connect()

    assert bus.calls[-1] == ("disconnect", True, False)


class FakeCartesianBus:
    is_connected = True

    def __init__(self, pose: tuple[float, ...]) -> None:
        self.pose = pose
        self.end_pose_commands: list[tuple] = []
        self.end_pose_error: Exception | None = None

    def get_end_pose(self) -> tuple[float, ...]:
        if self.end_pose_error is not None:
            raise self.end_pose_error
        return self.pose

    def set_end_pose(self, pose: tuple, *, move_mode: str, speed_percent: int) -> None:
        self.end_pose_commands.append((pose, move_mode, speed_percent))

    def set_gripper_percent(self, value: float, effort: int) -> None:
        raise AssertionError("gripper command must not be sent for an invalid current pose")


class MovingFakeCartesianBus(FakeCartesianBus):
    def __init__(self, pose: tuple[float, ...]) -> None:
        super().__init__(pose)
        self.gripper_commands: list[tuple[float, int]] = []
        self.joint_state_commands: list[tuple[tuple[float, ...], int, int]] = []
        self.ready_checks = 0
        self.ready_error: Exception | None = None

    def assert_follower_ready(self) -> None:
        self.ready_checks += 1
        if self.ready_error is not None:
            raise self.ready_error

    def get_joint_position(self) -> dict[str, float]:
        return {"gripper": 25.0}

    def get_joint_radians(self) -> tuple[float, ...]:
        return (0.0, 0.5, -1.0, 0.1, 0.2, -0.1)

    def set_end_pose(self, pose: tuple, *, move_mode: str, speed_percent: int) -> None:
        super().set_end_pose(pose, move_mode=move_mode, speed_percent=speed_percent)
        self.pose = pose

    def set_gripper_percent(self, value: float, effort: int) -> None:
        self.gripper_commands.append((value, effort))

    def set_joint_state(
        self,
        target_joint: tuple[float, ...],
        *,
        speed_percent: int,
        gripper_effort: int,
    ) -> tuple[float, ...]:
        command = (tuple(target_joint), speed_percent, gripper_effort)
        self.joint_state_commands.append(command)
        return tuple(target_joint)


def make_cartesian_follower(bus: FakeCartesianBus) -> PiperFollower:
    follower = object.__new__(PiperFollower)
    follower.id = "test-piper"
    follower.prefix = ""
    follower.bus = bus
    follower.cameras = {}
    follower._camera_executor = None
    follower._official_ik = None
    follower._last_ik_joint_command = None
    follower._last_ik_action = None
    follower._ik_over_limit = False
    follower._feedback_stale_since = None
    follower._last_observation = None
    follower.config = SimpleNamespace(
        control_space="cartesian",
        cartesian_command_mode="step",
        feedback_startup_timeout_s=5.0,
        workspace_x=(100.0, 600.0),
        workspace_y=(-400.0, 400.0),
        workspace_z=(50.0, 600.0),
        max_cartesian_step_mm=5.0,
        max_rotation_step_rad=0.05,
        max_cartesian_following_error_mm=100.0,
        max_rotation_following_error_rad=0.5,
        move_mode="move_p",
        move_speed_percent=5,
        gripper_effort=500,
    )
    return follower


class FakeOfficialIK:
    def __init__(self, joints_rad: tuple[float, ...] | None) -> None:
        self.joints_rad = joints_rad

    def solve_native_pose(self, pose, current_joints, *, gripper_width_m):
        if self.joints_rad is None:
            return None
        return SimpleNamespace(
            joints_rad=self.joints_rad,
            position_error_m=0.0,
            rotation_error_rad=0.0,
        )

    def native_pose_from_joints(self, joints_rad: tuple[float, ...]) -> tuple[float, ...]:
        return (310.0, 0.0, 250.0, 0.0, 0.0, 0.1)


class FakeAsyncOfficialIK:
    def __init__(self, update=None) -> None:
        self.update = update
        self.calls: list[tuple] = []

    def update_target(self, pose, current_joints, *, gripper_width_m):
        self.calls.append((pose, current_joints, gripper_width_m))
        return self.update


def test_cartesian_control_rejects_invalid_current_pose_before_sending() -> None:
    bus = FakeCartesianBus((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    action = {key: value for key, value in zip(POSE_KEYS, (300, 0, 250, 0, 0, 0), strict=True)}
    action["gripper.pos"] = 0.5

    with pytest.raises(PiperFeedbackError, match="outside workspace"):
        follower._send_cartesian_action(action)

    assert bus.end_pose_commands == []


def test_cartesian_control_rejects_non_finite_current_orientation() -> None:
    bus = FakeCartesianBus((300.0, 0.0, 250.0, float("nan"), 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.cartesian_command_mode = "direct"
    action = {
        key: value
        for key, value in zip(POSE_KEYS, (300, 0, 250, 0, 0, 0), strict=True)
    }
    action["gripper.pos"] = 0.5

    with pytest.raises(PiperFeedbackError, match="non-finite Cartesian pose"):
        follower._send_cartesian_action(action)

    assert bus.end_pose_commands == []


def test_cartesian_direct_mode_sends_the_full_target_pose() -> None:
    bus = MovingFakeCartesianBus((300.0, 0.0, 250.0, 0.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.cartesian_command_mode = "direct"
    action = {
        key: value
        for key, value in zip(
            POSE_KEYS,
            (330.0, 0.0, 250.0, 0.0, 0.0, 0.0),
            strict=True,
        )
    }
    action["gripper.pos"] = 0.5

    sent = follower._send_cartesian_action(action)

    commanded_pose, move_mode, speed_percent = bus.end_pose_commands[0]
    assert commanded_pose == pytest.approx((330.0, 0.0, 250.0, 0.0, 0.0, 0.0))
    assert move_mode == "move_p"
    assert speed_percent == 5
    assert sent["pose.x"] == pytest.approx(330.0)
    assert bus.gripper_commands == [(50.0, 500)]


def test_cartesian_direct_mode_rejects_excessive_following_error() -> None:
    bus = MovingFakeCartesianBus((300.0, 0.0, 250.0, 0.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.cartesian_command_mode = "direct"
    action = {
        key: value
        for key, value in zip(
            POSE_KEYS,
            (450.0, 0.0, 250.0, 0.0, 0.0, 0.0),
            strict=True,
        )
    }
    action["gripper.pos"] = 0.5

    with pytest.raises(PiperFeedbackError, match="direct target is 150.000 mm away"):
        follower._send_cartesian_action(action)

    assert bus.end_pose_commands == []
    assert bus.gripper_commands == []


def test_cartesian_direct_mode_rejects_excessive_rotation_error() -> None:
    bus = MovingFakeCartesianBus((300.0, 0.0, 250.0, 0.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.cartesian_command_mode = "direct"
    action = {
        key: value
        for key, value in zip(
            POSE_KEYS,
            (300.0, 0.0, 250.0, 0.0, 0.0, 0.6),
            strict=True,
        )
    }
    action["gripper.pos"] = 0.5

    with pytest.raises(PiperFeedbackError, match="direct target is 0.6000 rad away"):
        follower._send_cartesian_action(action)

    assert bus.end_pose_commands == []
    assert bus.gripper_commands == []


def test_cartesian_official_ik_sends_joint_command_instead_of_end_pose() -> None:
    bus = MovingFakeCartesianBus((300.0, 0.0, 250.0, 0.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.cartesian_command_mode = "official_ik"
    solution = (0.01, 0.51, -1.01, 0.11, 0.19, -0.09)
    follower._official_ik = FakeOfficialIK(solution)
    action = {
        key: value
        for key, value in zip(
            POSE_KEYS,
            (310.0, 0.0, 250.0, 0.0, 0.0, 0.1),
            strict=True,
        )
    }
    action["gripper.pos"] = 0.5

    sent = follower._send_cartesian_action(action)

    assert bus.end_pose_commands == []
    assert bus.joint_state_commands == [((*solution, 0.034), 5, 500)]
    assert sent["pose.x"] == pytest.approx(310.0)
    assert sent["gripper.pos"] == pytest.approx(0.5)


def test_cartesian_official_ik_holds_last_action_when_solver_fails() -> None:
    bus = MovingFakeCartesianBus((300.0, 0.0, 250.0, 0.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.cartesian_command_mode = "official_ik"
    follower._official_ik = FakeOfficialIK(None)
    follower._last_ik_action = {
        **dict(zip(POSE_KEYS, (305.0, 0.0, 250.0, 0.0, 0.0, 0.0), strict=True)),
        "gripper.pos": 0.4,
    }
    action = {
        **dict(zip(POSE_KEYS, (450.0, 0.0, 250.0, 0.0, 0.0, 0.0), strict=True)),
        "gripper.pos": 0.5,
    }

    sent = follower._send_cartesian_action(action)

    assert bus.end_pose_commands == []
    assert bus.joint_state_commands == []
    assert sent == {**follower._last_ik_action, "gripper.pos": 0.5}
    assert follower._ik_over_limit is True


def test_cartesian_official_ik_streams_last_joint_target_while_worker_solves() -> None:
    bus = MovingFakeCartesianBus((300.0, 0.0, 250.0, 0.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.cartesian_command_mode = "official_ik"
    worker = FakeAsyncOfficialIK()
    follower._official_ik = worker
    follower._last_ik_joint_command = (0.0, 0.5, -1.0, 0.1, 0.2, -0.1)
    follower._last_ik_action = {
        **dict(zip(POSE_KEYS, (305.0, 0.0, 250.0, 0.0, 0.0, 0.0), strict=True)),
        "gripper.pos": 0.4,
    }
    action = {
        **dict(zip(POSE_KEYS, (310.0, 0.0, 250.0, 0.0, 0.0, 0.0), strict=True)),
        "gripper.pos": 0.5,
    }

    sent = follower._send_cartesian_action(action)

    assert len(worker.calls) == 1
    assert bus.joint_state_commands == [
        ((*follower._last_ik_joint_command, 0.034), 5, 500)
    ]
    assert sent == {**follower._last_ik_action, "gripper.pos": 0.5}


def test_cartesian_official_ik_sends_latest_completed_worker_solution() -> None:
    bus = MovingFakeCartesianBus((300.0, 0.0, 250.0, 0.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.cartesian_command_mode = "official_ik"
    solution = (0.01, 0.51, -1.01, 0.11, 0.19, -0.09)
    solved_target = (307.0, 1.0, 249.0, 0.0, 0.0, 0.02)
    result = SimpleNamespace(
        joints_rad=solution,
        position_error_m=0.0,
        rotation_error_rad=0.0,
    )
    follower._official_ik = FakeAsyncOfficialIK(
        SimpleNamespace(target_pose=solved_target, result=result)
    )
    action = {
        **dict(zip(POSE_KEYS, (310.0, 0.0, 250.0, 0.0, 0.0, 0.0), strict=True)),
        "gripper.pos": 0.5,
    }

    sent = follower._send_cartesian_action(action)

    assert bus.joint_state_commands == [((*solution, 0.034), 5, 500)]
    assert tuple(sent[key] for key in POSE_KEYS) == solved_target


def test_cartesian_official_ik_holds_during_transient_stale_feedback() -> None:
    bus = MovingFakeCartesianBus((300.0, 0.0, 250.0, 0.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.cartesian_command_mode = "official_ik"
    follower._official_ik = FakeOfficialIK((0.0, 0.5, -1.0, 0.1, 0.2, -0.1))
    follower._last_ik_action = {
        **dict(zip(POSE_KEYS, (305.0, 0.0, 250.0, 0.0, 0.0, 0.0), strict=True)),
        "gripper.pos": 0.4,
    }
    bus.ready_error = PiperFeedbackStaleError("briefly stale")
    action = {
        **dict(zip(POSE_KEYS, (310.0, 0.0, 250.0, 0.0, 0.0, 0.0), strict=True)),
        "gripper.pos": 0.5,
    }

    sent = follower.send_action(action)

    assert sent == follower._last_ik_action
    assert follower._feedback_stale_since is not None
    assert bus.joint_state_commands == []


def test_cartesian_official_ik_resumes_after_stale_feedback_recovers() -> None:
    bus = MovingFakeCartesianBus((300.0, 0.0, 250.0, 0.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.cartesian_command_mode = "official_ik"
    solution = (0.0, 0.5, -1.0, 0.1, 0.2, -0.1)
    follower._official_ik = FakeOfficialIK(solution)
    follower._feedback_stale_since = time.monotonic()
    action = {
        **dict(zip(POSE_KEYS, (310.0, 0.0, 250.0, 0.0, 0.0, 0.0), strict=True)),
        "gripper.pos": 0.5,
    }

    follower.send_action(action)

    assert follower._feedback_stale_since is None
    assert bus.joint_state_commands == [((*solution, 0.034), 5, 500)]


def test_cartesian_official_ik_aborts_after_continuous_stale_feedback() -> None:
    bus = MovingFakeCartesianBus((300.0, 0.0, 250.0, 0.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.cartesian_command_mode = "official_ik"
    follower._official_ik = FakeOfficialIK((0.0, 0.5, -1.0, 0.1, 0.2, -0.1))
    follower._feedback_stale_since = time.monotonic() - 5.1
    bus.ready_error = PiperFeedbackStaleError("still stale")
    action = {
        **dict(zip(POSE_KEYS, (310.0, 0.0, 250.0, 0.0, 0.0, 0.0), strict=True)),
        "gripper.pos": 0.5,
    }

    with pytest.raises(PiperFeedbackStaleError, match="remained stale for 5.0s"):
        follower.send_action(action)

    assert bus.joint_state_commands == []


def test_cartesian_official_ik_reuses_observation_during_stale_feedback() -> None:
    bus = MovingFakeCartesianBus((300.0, 0.0, 250.0, 0.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.cartesian_command_mode = "official_ik"
    expected = {
        **dict(zip(POSE_KEYS, (300.0, 0.0, 250.0, 0.0, 0.0, 0.0), strict=True)),
        "gripper.pos": 0.25,
    }
    follower._last_observation = expected
    bus.end_pose_error = PiperFeedbackStaleError("briefly stale")

    observation = follower.get_observation()

    assert observation == expected
    assert observation is not follower._last_observation
    assert follower._feedback_stale_since is not None


def test_cartesian_official_ik_observation_recovers_after_stale_feedback() -> None:
    bus = MovingFakeCartesianBus((300.0, 0.0, 250.0, 0.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.cartesian_command_mode = "official_ik"
    follower._feedback_stale_since = time.monotonic()

    observation = follower.get_observation()

    assert observation["pose.x"] == pytest.approx(300.0)
    assert follower._last_observation == observation
    assert follower._feedback_stale_since is None


def test_cartesian_observation_can_include_physical_joint_angles_without_changing_action() -> None:
    bus = MovingFakeCartesianBus((300.0, 0.0, 250.0, 0.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.record_joint_angles = True

    observation = follower.get_observation()

    expected_joints = bus.get_joint_radians()
    for key, value in zip(JOINT_ANGLE_KEYS, expected_joints, strict=True):
        assert observation[key] == pytest.approx(value)
        assert follower.observation_features[key] is float
    assert observation["pose.x"] == pytest.approx(300.0)
    assert observation["gripper.pos"] == pytest.approx(0.25)
    assert set(follower.action_features) == {*POSE_KEYS, "gripper.pos"}


def test_cartesian_control_rejects_derived_non_finite_pose_before_sending(
    monkeypatch,
) -> None:
    bus = FakeCartesianBus((300.0, 0.0, 250.0, 180.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    action = {
        key: value
        for key, value in zip(POSE_KEYS, (300, 0, 250, 0, 0, 0), strict=True)
    }
    action["gripper.pos"] = 0.5
    monkeypatch.setattr(
        piper_follower_module,
        "rotation_step_towards",
        lambda current, target, max_step: (float("nan"), 0.0, 0.0),
    )

    with pytest.raises(PiperFeedbackError, match="computed a non-finite Cartesian command"):
        follower._send_cartesian_action(action)

    assert bus.end_pose_commands == []


def test_move_to_tcp_pose_uses_bounded_commands_and_waits_for_feedback() -> None:
    bus = MovingFakeCartesianBus((290.0, 0.0, 250.0, 180.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.config.cartesian_command_mode = "direct"

    follower.move_to_tcp_pose(
        [300.0, 0.0, 250.0, 180.0, 0.0, 0.0],
        timeout_s=1.0,
    )

    assert bus.ready_checks >= 1
    assert len(bus.end_pose_commands) == 2
    first_pose, move_mode, speed_percent = bus.end_pose_commands[0]
    second_pose, _, _ = bus.end_pose_commands[1]
    assert first_pose[:3] == pytest.approx((295.0, 0.0, 250.0))
    assert second_pose[:3] == pytest.approx((300.0, 0.0, 250.0))
    assert move_mode == "move_p"
    assert speed_percent == 5
    assert bus.gripper_commands == [(25.0, 500), (25.0, 500)]


def test_move_to_tcp_pose_supports_a_dual_arm_prefix() -> None:
    bus = MovingFakeCartesianBus((295.0, 150.0, 250.0, 180.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)
    follower.prefix = "left."

    follower.move_to_tcp_pose(
        [300.0, 150.0, 250.0, 180.0, 0.0, 0.0],
        timeout_s=1.0,
    )

    assert len(bus.end_pose_commands) == 1
    assert bus.end_pose_commands[0][0][:3] == pytest.approx((300.0, 150.0, 250.0))


def test_move_to_tcp_pose_rejects_target_outside_workspace_before_command() -> None:
    bus = MovingFakeCartesianBus((300.0, 0.0, 250.0, 180.0, 0.0, 0.0))
    follower = make_cartesian_follower(bus)

    with pytest.raises(ValueError, match="startup x=.*outside workspace"):
        follower.move_to_tcp_pose([50.0, 0.0, 250.0, 180.0, 0.0, 0.0])

    assert bus.end_pose_commands == []
    assert bus.gripper_commands == []


class FakeFollowerConnectBus:
    is_calibrated = True

    def __init__(self, fail_feedback: bool = False) -> None:
        self.is_connected = False
        self.fail_feedback = fail_feedback
        self.pose = (300.0, 0.0, 250.0, 0.0, 0.0, 0.0)
        self.calls: list[object] = []

    def connect(self) -> None:
        self.is_connected = True
        self.calls.append("connect")

    def set_follower(self) -> None:
        self.calls.append("set_follower")

    def wait_for_follower_feedback(self, control_space: str, timeout_s: float) -> None:
        self.calls.append(("wait", control_space, timeout_s))
        if self.fail_feedback:
            raise TimeoutError("no follower feedback")

    def enable_torque(self) -> None:
        self.calls.append("enable")

    def get_end_pose(self) -> tuple[float, ...]:
        self.calls.append("get_end_pose")
        return self.pose

    def set_end_pose(
        self,
        pose: tuple[float, ...],
        *,
        move_mode: str,
        speed_percent: int,
    ) -> None:
        self.calls.append(("hold", pose, move_mode, speed_percent))

    def assert_follower_ready(self) -> None:
        self.calls.append("ready")

    def disconnect(self, *, disable_torque: bool, park: bool) -> None:
        self.calls.append(("disconnect", disable_torque, park))
        self.is_connected = False


def make_connecting_follower(
    bus: FakeFollowerConnectBus,
    *,
    disable_torque_on_disconnect: bool = False,
    hold_position_on_disconnect: bool = False,
) -> PiperFollower:
    follower = object.__new__(PiperFollower)
    follower.id = "test-piper"
    follower.bus = bus
    follower.cameras = {}
    follower._camera_executor = None
    follower.config = SimpleNamespace(
        port="can-test",
        configure_role_on_connect=True,
        control_space="cartesian",
        feedback_startup_timeout_s=5.0,
        park_on_connect=False,
        park_on_disconnect=False,
        disable_torque_on_disconnect=disable_torque_on_disconnect,
        hold_position_on_disconnect=hold_position_on_disconnect,
        move_mode="move_p",
        move_speed_percent=5,
    )
    return follower


def test_follower_waits_for_valid_feedback_before_enable() -> None:
    bus = FakeFollowerConnectBus()
    follower = make_connecting_follower(bus)

    follower.connect(calibrate=False)

    assert bus.calls == [
        "connect",
        "set_follower",
        ("wait", "cartesian", 5.0),
        "enable",
        "ready",
    ]


def test_follower_does_not_enable_when_startup_feedback_is_missing() -> None:
    bus = FakeFollowerConnectBus(fail_feedback=True)

    with pytest.raises(TimeoutError, match="no follower feedback"):
        make_connecting_follower(bus).connect(calibrate=False)

    assert "enable" not in bus.calls
    assert bus.calls[-1] == ("disconnect", False, False)


def test_follower_connect_failure_can_explicitly_disable_torque() -> None:
    bus = FakeFollowerConnectBus(fail_feedback=True)
    follower = make_connecting_follower(bus, disable_torque_on_disconnect=True)

    with pytest.raises(TimeoutError, match="no follower feedback"):
        follower.connect(calibrate=False)

    assert bus.calls[-1] == ("disconnect", True, False)


def test_follower_disconnect_can_leave_torque_enabled() -> None:
    bus = FakeFollowerConnectBus()
    follower = make_connecting_follower(bus, disable_torque_on_disconnect=False)
    bus.is_connected = True

    follower.disconnect()

    assert bus.calls[-1] == ("disconnect", False, False)


def test_follower_holds_current_pose_before_disconnect(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    bus = FakeFollowerConnectBus()
    follower = make_connecting_follower(
        bus,
        disable_torque_on_disconnect=False,
        hold_position_on_disconnect=True,
    )
    bus.is_connected = True
    monkeypatch.setattr(piper_follower_module.time, "sleep", lambda _: None)

    follower.disconnect()

    assert bus.calls[-3:] == [
        "get_end_pose",
        ("hold", bus.pose, "move_p", 5),
        ("disconnect", False, False),
    ]


def test_follower_disconnect_can_explicitly_disable_torque() -> None:
    bus = FakeFollowerConnectBus()
    follower = make_connecting_follower(bus, disable_torque_on_disconnect=True)
    bus.is_connected = True

    follower.disconnect()

    assert bus.calls[-1] == ("disconnect", True, False)


def test_cartesian_config_requires_explicit_workspace() -> None:
    with pytest.raises(ValueError, match="requires workspace_x/y/z"):
        PiperFollowerConfig(port="can-test", control_space="cartesian")


def test_non_finite_action_is_rejected() -> None:
    follower = object.__new__(PiperFollower)
    follower.id = "test-piper"
    follower.prefix = ""
    follower.config = SimpleNamespace(control_space="cartesian")
    action = {key: 0.0 for key in POSE_KEYS}
    action["pose.x"] = float("nan")
    action["gripper.pos"] = 0.5

    with pytest.raises(ValueError, match="non-finite pose.x"):
        follower._strip_and_validate(action)


def test_joint_action_returns_the_bus_clamped_command() -> None:
    class FakeJointBus:
        def set_joint_position(self, goal, *, speed_percent):
            assert speed_percent == 5
            return {key: min(100.0, value) for key, value in goal.items()}

    follower = object.__new__(PiperFollower)
    follower.bus = FakeJointBus()
    follower.config = SimpleNamespace(max_relative_target=None, move_speed_percent=5)
    local = {key: 150.0 for key in JOINT_KEYS}

    sent = follower._send_joint_action(local)

    assert set(sent) == set(JOINT_KEYS)
    assert set(sent.values()) == {100.0}
