import importlib
import math
import sys
import time
from types import ModuleType, SimpleNamespace

import pytest
from lerobot.motors import Motor, MotorCalibration, MotorNormMode


def make_motor_status_message(
    *, enabled: tuple[bool, ...] = (True, True, True, True, True, True), fault=None
):
    motors = {}
    for joint_index, is_enabled in enumerate(enabled, start=1):
        status = SimpleNamespace(
            voltage_too_low=False,
            motor_overheating=False,
            driver_overcurrent=False,
            driver_overheating=False,
            collision_status=False,
            driver_error_status=False,
            driver_enable_status=is_enabled,
            stall_status=False,
        )
        if fault is not None and fault[0] == joint_index:
            setattr(status, fault[1], True)
        motors[f"motor_{joint_index}"] = SimpleNamespace(foc_status=status)
    return SimpleNamespace(Hz=100, time_stamp=time.time(), **motors)


class FakePiperInterface:
    def __init__(self, port: str) -> None:
        self.port = port
        self.connect_calls: list[bool] = []
        self.disconnect_calls = 0
        self.disable_calls = 0
        self.gripper_calls: list[tuple[int, int, int, int]] = []
        self.mode_calls: list[tuple[int, int, int, int]] = []
        self.joint_calls: list[tuple[int, ...]] = []
        self.end_pose_calls: list[tuple[int, ...]] = []
        self.connected = False

    def ConnectPort(self, can_init: bool = False) -> None:
        self.connect_calls.append(can_init)
        self.connected = True

    def get_connect_status(self) -> bool:
        return self.connected

    def DisconnectPort(self) -> None:
        self.disconnect_calls += 1
        self.connected = False

    def DisablePiper(self) -> bool:
        self.disable_calls += 1
        return False

    def EnablePiper(self) -> bool:
        return True

    def GripperCtrl(self, angle: int, effort: int, status: int, zero: int) -> None:
        self.gripper_calls.append((angle, effort, status, zero))

    def ModeCtrl(
        self, ctrl_mode: int, move_mode: int, speed_percent: int, mit_mode: int
    ) -> None:
        self.mode_calls.append((ctrl_mode, move_mode, speed_percent, mit_mode))

    def EndPoseCtrl(self, *pose: int) -> None:
        self.end_pose_calls.append(pose)

    def JointCtrl(self, *joints: int) -> None:
        self.joint_calls.append(joints)


@pytest.fixture
def bus(monkeypatch: pytest.MonkeyPatch):
    fake_sdk = ModuleType("piper_sdk")
    fake_sdk.C_PiperInterface_V2 = FakePiperInterface
    monkeypatch.setitem(sys.modules, "piper_sdk", fake_sdk)

    module_name = "lerobot_real.devices.piper.piper_motors_bus"
    sys.modules.pop(module_name, None)
    module = importlib.import_module(module_name)

    motors = {
        **{
            f"joint{index}": Motor(
                index, "AGILEX-M", MotorNormMode.RANGE_M100_100
            )
            for index in range(1, 7)
        },
        "gripper": Motor(7, "AGILEX-S", MotorNormMode.RANGE_0_100),
    }
    calibration = {
        **{
            f"joint{index}": MotorCalibration(index, 0, 0, -1000, 1000)
            for index in range(1, 7)
        },
        "gripper": MotorCalibration(7, 0, 0, 0, 1000),
    }
    yield module.PiperMotorsBus(
        id="test-piper",
        port="can-test",
        motors=motors,
        calibration=calibration,
    )
    sys.modules.pop(module_name, None)


def test_connect_disconnect_and_reconnect_reinitializes_can(bus) -> None:
    bus.connect()
    assert bus.is_connected
    assert bus.piper.connect_calls == [False]

    bus.disconnect()
    assert not bus.is_connected
    assert bus.piper.disable_calls == 1
    assert bus.piper.disconnect_calls == 1

    bus.connect()
    assert bus.is_connected
    assert bus.piper.connect_calls == [False, True]


def test_disconnect_can_leave_torque_enabled(bus) -> None:
    bus.connect()

    bus.disconnect(disable_torque=False)

    assert not bus.is_connected
    assert bus.piper.disable_calls == 0
    assert bus.piper.disconnect_calls == 1


def test_enable_torque_supports_modern_sdk(bus, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delattr(FakePiperInterface, "EnablePiper")
    enable_calls = []
    bus.piper.EnableArm = lambda: enable_calls.append(None)
    messages = iter(
        [
            make_motor_status_message(enabled=(False,) * 6),
            make_motor_status_message(),
        ]
    )
    bus.piper.GetArmLowSpdInfoMsgs = lambda: next(messages)

    bus.enable_torque(num_retry=2)

    assert len(enable_calls) == 2


def test_disable_torque_supports_modern_sdk(bus, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delattr(FakePiperInterface, "DisablePiper")
    disable_calls = []
    bus.piper.DisableArm = lambda: disable_calls.append(None)
    messages = iter(
        [
            make_motor_status_message(),
            make_motor_status_message(enabled=(False,) * 6),
        ]
    )
    bus.piper.GetArmLowSpdInfoMsgs = lambda: next(messages)

    bus.disable_torque(num_retry=2)

    assert len(disable_calls) == 2


def test_disconnect_still_disables_and_closes_can_when_parking_fails(bus) -> None:
    bus.connect()
    bus.parking = lambda: (_ for _ in ()).throw(RuntimeError("parking failed"))

    with pytest.raises(RuntimeError, match="parking failed"):
        bus.disconnect(park=True)

    assert bus.piper.disable_calls == 1
    assert bus.piper.disconnect_calls == 1
    assert not bus.is_connected


def test_normalization_clamps_values_to_calibrated_ranges(bus) -> None:
    assert bus._normalize({"joint1": 0, "gripper": 500}) == {
        "joint1": 0.0,
        "gripper": 50.0,
    }
    assert bus._unnormalize({"joint1": 150.0, "gripper": -10.0}) == {
        "joint1": 1000,
        "gripper": 0,
    }


def test_fixed_joint_ranges_match_piper_sdk_limits() -> None:
    tables = importlib.import_module("lerobot_real.devices.piper.tables")
    expected = {
        "joint1": (-150000, 150000),
        "joint2": (0, 180000),
        "joint3": (-170000, 0),
        "joint4": (-100000, 100000),
        "joint5": (-70000, 70000),
        "joint6": (-120000, 120000),
    }
    actual = {
        name: (tables.CALIBRATION[name].range_min, tables.CALIBRATION[name].range_max)
        for name in expected
    }
    assert actual == expected


def test_large_gripper_calibration_uses_configured_width() -> None:
    tables = importlib.import_module("lerobot_real.devices.piper.tables")

    calibration = tables.make_calibration(0.098)

    assert calibration["gripper"].range_min == 0
    assert calibration["gripper"].range_max == 98000
    assert tables.CALIBRATION["gripper"].range_max == 68000


def test_feedback_validation_rejects_missing_and_stale_frames(bus) -> None:
    with pytest.raises(RuntimeError, match="no valid joint state timestamp"):
        bus._validate_feedback(SimpleNamespace(Hz=0, time_stamp=0), "joint state")

    with pytest.raises(RuntimeError, match="stale"):
        bus._validate_feedback(
            SimpleNamespace(Hz=100, time_stamp=time.time() - 1.0),
            "joint state",
        )

    message = SimpleNamespace(Hz=100, time_stamp=time.time())
    assert bus._validate_feedback(message, "joint state") is message

    # The SDK can briefly report Hz=0 while retaining a fresh frame timestamp.
    fresh_zero_hz = SimpleNamespace(Hz=0, time_stamp=time.time())
    assert bus._validate_feedback(fresh_zero_hz, "joint state") is fresh_zero_hz


def test_normal_gripper_commands_do_not_clear_errors_every_frame(bus) -> None:
    bus.set_gripper_percent(50.0, effort=800)
    bus.clear_gripper()

    assert bus.piper.gripper_calls[0][1:] == (800, 0x01, 0)
    assert bus.piper.gripper_calls[1][2] == 0x03


def test_physical_joint_state_converts_radians_and_metres_to_sdk_units(bus) -> None:
    target = (0.01, -0.01, 0.005, -0.005, 0.0, 0.001, 0.0005)

    sent = bus.set_joint_state(target, speed_percent=40, gripper_effort=800)

    assert bus.piper.mode_calls == [(0x01, 0x01, 40, 0x00)]
    assert bus.piper.joint_calls == [(573, -573, 286, -286, 0, 57)]
    assert bus.piper.gripper_calls == [(500, 800, 0x01, 0)]
    assert sent == pytest.approx(
        (
            0.010000736,
            -0.010000736,
            0.004991642,
            -0.004991642,
            0.0,
            0.000994838,
            0.0005,
        ),
        abs=1e-9,
    )


def test_physical_joint_state_reads_radians_and_metres(bus) -> None:
    bus.piper.GetArmJointMsgs = lambda: SimpleNamespace(
        Hz=100,
        time_stamp=time.time(),
        joint_state=SimpleNamespace(
            joint_1=1000,
            joint_2=-2000,
            joint_3=3000,
            joint_4=-4000,
            joint_5=5000,
            joint_6=-6000,
        ),
    )
    bus.piper.GetArmGripperMsgs = lambda: SimpleNamespace(
        Hz=100,
        time_stamp=time.time(),
        gripper_state=SimpleNamespace(grippers_angle=500),
    )

    state = bus.get_joint_state()

    assert state[:6] == pytest.approx(
        tuple(math.radians(value) for value in (1, -2, 3, -4, 5, -6))
    )
    assert state[6] == pytest.approx(0.0005)


def test_physical_joint_state_rejects_joint_or_gripper_outside_calibration(bus) -> None:
    with pytest.raises(ValueError, match="joint1 command"):
        bus.set_joint_state((0.1, 0, 0, 0, 0, 0, 0.0005))

    with pytest.raises(ValueError, match="gripper command"):
        bus.set_joint_state((0, 0, 0, 0, 0, 0, 0.002))

    assert bus.piper.joint_calls == []
    assert bus.piper.gripper_calls == []


def test_normalized_joint_command_delegates_to_physical_joint_backend(bus) -> None:
    action = {f"joint{index}": 0.0 for index in range(1, 7)}
    action["gripper"] = 50.0

    sent = bus.set_joint_position(action, speed_percent=20)

    assert bus.piper.mode_calls == [(0x01, 0x01, 20, 0x00)]
    assert bus.piper.joint_calls == [(0, 0, 0, 0, 0, 0)]
    assert bus.piper.gripper_calls == [(500, 1000, 0x01, 0)]
    assert sent == pytest.approx(action)


def test_cartesian_stream_sends_motion_mode_once(bus) -> None:
    first_pose = (300.0, 0.0, 250.0, 180.0, 0.0, 0.0)
    second_pose = (301.0, 0.0, 250.0, 180.0, 0.0, 0.0)

    bus.set_end_pose(first_pose, move_mode="move_p", speed_percent=5)
    bus.set_end_pose(second_pose, move_mode="move_p", speed_percent=5)

    assert bus.piper.mode_calls == [(0x01, 0x00, 5, 0x00)]
    assert len(bus.piper.end_pose_calls) == 2


def test_arm_fault_blocks_follower_commands(bus) -> None:
    bus.piper.GetArmStatus = lambda: SimpleNamespace(
        Hz=100,
        time_stamp=time.time(),
        arm_status=SimpleNamespace(err_code=0x0002),
    )

    with pytest.raises(RuntimeError, match="arm fault 0x0002"):
        bus.assert_follower_ready()


def test_disabled_joint_blocks_follower_commands(bus) -> None:
    bus.piper.GetArmStatus = lambda: SimpleNamespace(
        Hz=100,
        time_stamp=time.time(),
        arm_status=SimpleNamespace(arm_status=0, err_code=0),
    )
    bus.piper.GetArmLowSpdInfoMsgs = lambda: make_motor_status_message(
        enabled=(True, True, True, False, True, True)
    )

    with pytest.raises(RuntimeError, match="not fully enabled"):
        bus.assert_follower_ready()


def test_motor_fault_blocks_follower_commands(bus) -> None:
    bus.piper.GetArmStatus = lambda: SimpleNamespace(
        Hz=100,
        time_stamp=time.time(),
        arm_status=SimpleNamespace(arm_status=0, err_code=0),
    )
    bus.piper.GetArmLowSpdInfoMsgs = lambda: make_motor_status_message(
        fault=(3, "collision_status")
    )

    with pytest.raises(RuntimeError, match="joint 3: collision protection"):
        bus.assert_follower_ready()


def test_leader_holds_last_valid_gripper_when_only_gripper_frame_is_stale(bus) -> None:
    joint_ctrl = SimpleNamespace(
        joint_1=1,
        joint_2=2,
        joint_3=3,
        joint_4=4,
        joint_5=5,
        joint_6=6,
    )
    bus.piper.GetArmJointCtrl = lambda: SimpleNamespace(
        Hz=100,
        time_stamp=time.time(),
        joint_ctrl=joint_ctrl,
    )
    gripper_messages = iter(
        [
            SimpleNamespace(
                Hz=100,
                time_stamp=time.time(),
                gripper_ctrl=SimpleNamespace(grippers_angle=42),
            ),
            SimpleNamespace(
                Hz=0,
                time_stamp=time.time() - 1.0,
                gripper_ctrl=SimpleNamespace(grippers_angle=0),
            ),
        ]
    )
    bus.piper.GetArmGripperCtrl = lambda: next(gripper_messages)
    bus._normalize = lambda values: values

    assert bus.get_leader_position()["gripper"] == 42
    assert bus.get_leader_position()["gripper"] == 42
