"""LeRobot motor-bus adapter for AgileX Piper arms.

Adapted from AgRoboticsResearch/lerobot_robot_piper and Kane1440/lerobot_piper2
under Apache-2.0.
"""

import logging
import math
import time
from collections.abc import Callable, Sequence
from typing import Any

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from piper_sdk import C_PiperInterface_V2

from lerobot_real.devices.piper.tables import PARKING_POSITION

logger = logging.getLogger(__name__)

MOTOR_FAULT_FIELDS = {
    "voltage_too_low": "undervoltage",
    "motor_overheating": "motor overtemperature",
    "driver_overcurrent": "driver overcurrent",
    "driver_overheating": "driver overtemperature",
    "collision_status": "collision protection",
    "driver_error_status": "driver error",
    "stall_status": "stall protection",
}


class PiperFeedbackError(RuntimeError):
    """Raised when Piper feedback is missing, stale, or reports a fault."""


class PiperFeedbackStaleError(PiperFeedbackError):
    """Raised when a valid Piper feedback frame has stopped updating."""


class PiperMotorsBus:
    """Adapt Piper CAN commands to the interface used by this plugin."""

    apply_drive_mode = False

    def __init__(
        self,
        id: str,
        port: str,
        motors: dict[str, Motor],
        calibration: dict[str, MotorCalibration],
        feedback_timeout_s: float = 0.5,
    ) -> None:
        self.port = port
        self.motors = motors
        self.calibration = calibration
        self.id = id
        self.feedback_timeout_s = feedback_timeout_s
        self.piper = C_PiperInterface_V2(port)
        self._is_connected = False
        self._requires_can_init = False
        self._last_leader_gripper: float | None = None
        self._last_motion_ctrl: tuple[int, int, int, int] | None = None

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, handshake: bool = True) -> None:
        if self._is_connected:
            return
        self.piper.ConnectPort(can_init=self._requires_can_init)
        get_connect_status = getattr(self.piper, "get_connect_status", None)
        if get_connect_status is not None and not get_connect_status():
            self.piper.DisconnectPort()
            self._requires_can_init = True
            raise ConnectionError(f"Failed to open CAN port {self.port!r} for {self.id}")
        self._requires_can_init = False
        self._last_leader_gripper = None
        self._last_motion_ctrl = None
        self._is_connected = True

    def disconnect(self, disable_torque: bool = True, park: bool = False) -> None:
        if not self._is_connected:
            return
        first_error: Exception | None = None
        try:
            if park:
                try:
                    self.parking()
                except Exception as exc:
                    first_error = first_error or exc
            if disable_torque:
                try:
                    self.disable_torque()
                except Exception as exc:
                    first_error = first_error or exc
        finally:
            try:
                self.piper.DisconnectPort()
            except Exception as exc:
                first_error = first_error or exc
            finally:
                # The SDK requires CAN reinitialization after DisconnectPort.
                self._requires_can_init = True
                self._last_motion_ctrl = None
                self._is_connected = False
        if first_error is not None:
            raise first_error

    def read(self, data_name: str, motor: str) -> int | float:
        return self.get_joint_position().get(motor, 0.0)

    def write(self, data_name: str, motor: str, value: int | float) -> None:
        current = self.get_joint_position()
        current[motor] = value
        self.set_joint_position(current)

    def sync_read(
        self, data_name: str, motors: str | list[str] | None = None
    ) -> dict[str, int | float]:
        position = self.get_joint_position()
        if motors is None:
            return position
        selected = [motors] if isinstance(motors, str) else motors
        return {motor: position[motor] for motor in selected if motor in position}

    def sync_write(self, data_name: str, values: dict[str, int | float]) -> None:
        self.set_joint_position(values)

    def enable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        retries = num_retry if num_retry > 0 else 50
        enable_piper = getattr(self.piper, "EnablePiper", None)
        if callable(enable_piper):
            while retries > 0:
                if enable_piper():
                    return
                retries -= 1
                time.sleep(0.1)
            raise TimeoutError(f"Timed out enabling Piper arm {self.id} on {self.port}")

        enable_arm = getattr(self.piper, "EnableArm", None)
        if not callable(enable_arm):
            raise AttributeError("Piper SDK exposes neither EnablePiper nor EnableArm")

        last_error: PiperFeedbackError | None = None
        while retries > 0:
            enable_arm()
            time.sleep(0.1)
            try:
                self.get_motor_status(require_enabled=True)
                return
            except PiperFeedbackError as exc:
                last_error = exc
            retries -= 1
        raise TimeoutError(f"Timed out enabling Piper arm {self.id} on {self.port}") from last_error

    def disable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        retries = num_retry if num_retry > 0 else 50
        disable_piper = getattr(self.piper, "DisablePiper", None)
        if callable(disable_piper):
            while retries > 0:
                if not disable_piper():
                    return
                retries -= 1
                time.sleep(0.1)
            raise TimeoutError(f"Timed out disabling Piper arm {self.id} on {self.port}")

        disable_arm = getattr(self.piper, "DisableArm", None)
        if not callable(disable_arm):
            raise AttributeError("Piper SDK exposes neither DisablePiper nor DisableArm")

        last_error: PiperFeedbackError | None = None
        while retries > 0:
            disable_arm()
            time.sleep(0.1)
            try:
                message = self.get_motor_status(require_enabled=False)
                enabled = [
                    bool(getattr(message, f"motor_{joint_index}").foc_status.driver_enable_status)
                    for joint_index in range(1, 7)
                ]
                if not any(enabled):
                    return
            except PiperFeedbackError as exc:
                last_error = exc
            retries -= 1
        raise TimeoutError(
            f"Timed out disabling Piper arm {self.id} on {self.port}"
        ) from last_error

    def read_calibration(self) -> dict[str, MotorCalibration]:
        return self.calibration

    def write_calibration(
        self, calibration_dict: dict[str, MotorCalibration], cache: bool = True
    ) -> None:
        self.calibration = calibration_dict

    def clear_gripper(self) -> None:
        self.piper.GripperCtrl(0, 1000, 0x03, 0)

    def set_follower(self) -> None:
        self.piper.MasterSlaveConfig(0xFC, 0, 0, 0)

    def set_leader(self) -> None:
        self.piper.MasterSlaveConfig(0xFA, 0, 0, 0)

    def _validate_feedback(self, message: Any, label: str) -> Any:
        try:
            hz = float(message.Hz)
            timestamp = float(message.time_stamp)
        except (AttributeError, TypeError, ValueError) as exc:
            raise PiperFeedbackError(
                f"Piper {self.id} returned malformed {label} feedback"
            ) from exc

        if not math.isfinite(hz) or hz < 0:
            raise PiperFeedbackError(f"Piper {self.id} returned malformed {label} frequency")
        if not math.isfinite(timestamp) or timestamp <= 0:
            raise PiperFeedbackError(f"Piper {self.id} has no valid {label} timestamp")

        age_s = time.time() - timestamp
        if age_s > self.feedback_timeout_s:
            raise PiperFeedbackStaleError(
                f"Piper {self.id} {label} feedback is stale ({age_s:.3f}s old)"
            )
        return message

    def _wait_for_feedback(
        self,
        reader: Callable[[], object | None],
        timeout_s: float,
        label: str,
    ) -> None:
        deadline = time.monotonic() + timeout_s
        last_error: PiperFeedbackError | None = None
        while time.monotonic() < deadline:
            try:
                reader()
                return
            except PiperFeedbackError as exc:
                last_error = exc
                time.sleep(0.05)
        raise TimeoutError(
            f"Timed out waiting {timeout_s:.1f}s for {self.id} {label} feedback"
        ) from last_error

    def wait_for_follower_feedback(self, control_space: str, timeout_s: float) -> None:
        def read_required_feedback() -> None:
            self.get_arm_status()
            self.get_motor_status(require_enabled=False)
            self.get_joint_position()
            if control_space == "cartesian":
                self.get_end_pose()

        self._wait_for_feedback(read_required_feedback, timeout_s, control_space)

    def wait_for_leader_feedback(self, timeout_s: float) -> None:
        self._wait_for_feedback(self.get_leader_position, timeout_s, "leader control")

    def get_arm_status(self) -> Any:
        message = self._validate_feedback(self.piper.GetArmStatus(), "arm status")
        status = getattr(message, "arm_status", None)
        if status is None:
            raise PiperFeedbackError(f"Piper {self.id} returned malformed arm status")
        arm_state = int(getattr(status, "arm_status", 0))
        if arm_state:
            raise PiperFeedbackError(
                f"Piper {self.id} reports non-normal arm status 0x{arm_state:02X}"
            )
        error_code = int(getattr(status, "err_code", 0))
        if error_code:
            raise PiperFeedbackError(f"Piper {self.id} reports arm fault 0x{error_code:04X}")
        return status

    def get_motor_status(self, *, require_enabled: bool) -> Any:
        message = self._validate_feedback(self.piper.GetArmLowSpdInfoMsgs(), "motor status")
        enabled: list[bool] = []
        faults: list[str] = []
        for joint_index in range(1, 7):
            motor = getattr(message, f"motor_{joint_index}", None)
            status = getattr(motor, "foc_status", None)
            required_fields = (*MOTOR_FAULT_FIELDS, "driver_enable_status")
            if status is None or any(not hasattr(status, field) for field in required_fields):
                raise PiperFeedbackError(
                    f"Piper {self.id} returned malformed motor {joint_index} status"
                )
            enabled.append(bool(status.driver_enable_status))
            faults.extend(
                f"joint {joint_index}: {label}"
                for field, label in MOTOR_FAULT_FIELDS.items()
                if bool(getattr(status, field))
            )

        if faults:
            raise PiperFeedbackError(f"Piper {self.id} reports motor faults: {', '.join(faults)}")
        if require_enabled and not all(enabled):
            raise PiperFeedbackError(
                f"Piper {self.id} is not fully enabled (joint status: {enabled})"
            )
        return message

    def assert_follower_ready(self) -> None:
        self.get_arm_status()
        self.get_motor_status(require_enabled=True)

    def parking(self, speed_percent: int = 5) -> None:
        self.assert_follower_ready()
        self.set_joint_position(PARKING_POSITION, speed_percent=speed_percent)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            status = self.get_arm_status()
            if not status.motion_status:
                return
            time.sleep(0.1)
        raise TimeoutError(f"Piper arm {self.id} did not finish parking in 10 seconds")

    def get_joint_position(self) -> dict[str, float]:
        joint_message = self._validate_feedback(self.piper.GetArmJointMsgs(), "joint state")
        gripper_message = self._validate_feedback(self.piper.GetArmGripperMsgs(), "gripper state")
        joint = joint_message.joint_state
        gripper = gripper_message.gripper_state
        raw = {
            "joint1": float(joint.joint_1),
            "joint2": float(joint.joint_2),
            "joint3": float(joint.joint_3),
            "joint4": float(joint.joint_4),
            "joint5": float(joint.joint_5),
            "joint6": float(joint.joint_6),
            "gripper": float(gripper.grippers_angle),
        }
        return self._normalize(raw)

    def get_joint_radians(self) -> tuple[float, ...]:
        """Read the six physical joint angles in radians for model-based IK."""
        joint_message = self._validate_feedback(self.piper.GetArmJointMsgs(), "joint state")
        joint = joint_message.joint_state
        return tuple(
            math.radians(float(getattr(joint, f"joint_{index}")) / 1000.0)
            for index in range(1, 7)
        )

    def get_joint_state(self) -> tuple[float, ...]:
        """Read six joint radians followed by gripper width in metres."""
        joints = self.get_joint_radians()
        gripper_message = self._validate_feedback(
            self.piper.GetArmGripperMsgs(), "gripper state"
        )
        gripper_width_m = float(gripper_message.gripper_state.grippers_angle) / 1_000_000.0
        return (*joints, gripper_width_m)

    def get_leader_position(self) -> dict[str, float]:
        joint_message = self._validate_feedback(
            self.piper.GetArmJointCtrl(), "leader joint control"
        )
        joint = joint_message.joint_ctrl
        gripper = None
        try:
            gripper_message = self._validate_feedback(
                self.piper.GetArmGripperCtrl(), "leader gripper control"
            )
            gripper = gripper_message.gripper_ctrl
        except PiperFeedbackError:
            if self._last_leader_gripper is None:
                raise
        raw = {
            "joint1": float(joint.joint_1),
            "joint2": float(joint.joint_2),
            "joint3": float(joint.joint_3),
            "joint4": float(joint.joint_4),
            "joint5": float(joint.joint_5),
            "joint6": float(joint.joint_6),
        }
        if gripper is not None:
            raw["gripper"] = float(gripper.grippers_angle)
            normalized = self._normalize(raw)
            self._last_leader_gripper = normalized["gripper"]
        else:
            raw["gripper"] = 0.0
            normalized = self._normalize(raw)
            normalized["gripper"] = self._last_leader_gripper
        return normalized

    def get_end_pose(self) -> tuple[float, float, float, float, float, float]:
        message = self._validate_feedback(self.piper.GetArmEndPoseMsgs(), "end pose")
        pose = message.end_pose
        # SDK units are 0.001 mm and 0.001 degree.
        return (
            float(pose.X_axis) / 1000.0,
            float(pose.Y_axis) / 1000.0,
            float(pose.Z_axis) / 1000.0,
            float(pose.RX_axis) / 1000.0,
            float(pose.RY_axis) / 1000.0,
            float(pose.RZ_axis) / 1000.0,
        )

    def set_joint_position(
        self, action: dict[str, float], speed_percent: int = 30
    ) -> dict[str, float]:
        missing = set(self.motors) - set(action)
        if missing:
            raise KeyError(f"Piper joint action is missing: {sorted(missing)}")
        raw = self._unnormalize(action)
        target = (
            *(math.radians(raw[f"joint{index}"] / 1000.0) for index in range(1, 7)),
            raw["gripper"] / 1_000_000.0,
        )
        sent = self.set_joint_state(target, speed_percent=speed_percent)
        sent_raw = {
            **{
                f"joint{index}": round(math.degrees(sent[index - 1]) * 1000.0)
                for index in range(1, 7)
            },
            "gripper": round(sent[6] * 1_000_000.0),
        }
        return self._normalize(sent_raw)

    def set_joint_state(
        self,
        target_joint: Sequence[float],
        *,
        speed_percent: int = 30,
        gripper_effort: int = 1000,
    ) -> tuple[float, ...]:
        """Send six joint radians and gripper width in metres through JointCtrl."""
        if len(target_joint) != 7:
            raise ValueError("Piper joint state must contain six radians and gripper metres")
        if not 1 <= speed_percent <= 100:
            raise ValueError("Piper joint command speed_percent must be in [1, 100]")
        if not 0 <= gripper_effort <= 5000:
            raise ValueError("Piper gripper effort must be in [0, 5000]")

        raw_joints: list[int] = []
        for index, value in enumerate(target_joint[:6], start=1):
            angle = float(value)
            if not math.isfinite(angle):
                raise ValueError("Piper joint command must contain only finite values")
            raw_value = int(round(math.degrees(angle) * 1000.0))
            calibration = self.calibration[f"joint{index}"]
            if not calibration.range_min <= raw_value <= calibration.range_max:
                raise ValueError(
                    f"Piper joint{index} command {math.degrees(angle):.3f} deg is outside "
                    f"[{calibration.range_min / 1000.0:.3f}, "
                    f"{calibration.range_max / 1000.0:.3f}]"
                )
            raw_joints.append(raw_value)

        gripper_width_m = float(target_joint[6])
        if not math.isfinite(gripper_width_m):
            raise ValueError("Piper joint command must contain only finite values")
        raw_gripper = int(round(gripper_width_m * 1_000_000.0))
        gripper_calibration = self.calibration["gripper"]
        if not gripper_calibration.range_min <= raw_gripper <= gripper_calibration.range_max:
            raise ValueError(
                f"Piper gripper command {gripper_width_m:.6f} m is outside "
                f"[{gripper_calibration.range_min / 1_000_000.0:.6f}, "
                f"{gripper_calibration.range_max / 1_000_000.0:.6f}]"
            )

        self._set_motion_ctrl(0x01, 0x01, speed_percent, 0x00)
        self.piper.JointCtrl(*raw_joints)
        self.piper.GripperCtrl(raw_gripper, gripper_effort, 0x01, 0)
        return (
            *(math.radians(value / 1000.0) for value in raw_joints),
            raw_gripper / 1_000_000.0,
        )

    def set_joint_radians(
        self,
        joints_rad: tuple[float, ...] | list[float],
        *,
        gripper_percent: float,
        speed_percent: int = 30,
    ) -> tuple[float, ...]:
        """Compatibility adapter for callers using gripper percent."""
        if len(joints_rad) != 6:
            raise ValueError("Piper joint command must contain six radians")
        gripper_raw = self._unnormalize({"gripper": gripper_percent})["gripper"]
        sent = self.set_joint_state(
            (*joints_rad, gripper_raw / 1_000_000.0),
            speed_percent=speed_percent,
        )
        return sent[:6]

    def set_end_pose(
        self,
        pose_mm_rpy_deg: tuple[float, float, float, float, float, float],
        *,
        move_mode: str,
        speed_percent: int,
    ) -> None:
        move_mode_code = 0x00 if move_mode == "move_p" else 0x02
        raw = [int(round(value * 1000.0)) for value in pose_mm_rpy_deg]
        self._set_motion_ctrl(0x01, move_mode_code, speed_percent, 0x00)
        self.piper.EndPoseCtrl(*raw)

    def set_gripper_percent(self, value: float, effort: int = 1000) -> None:
        raw = self._unnormalize({"gripper": value})["gripper"]
        self.piper.GripperCtrl(abs(int(raw)), effort, 0x01, 0)

    def _set_motion_ctrl(
        self,
        ctrl_mode: int,
        move_mode: int,
        speed_percent: int,
        mit_mode: int,
    ) -> None:
        command = (ctrl_mode, move_mode, speed_percent, mit_mode)
        if self._last_motion_ctrl == command:
            return
        mode_ctrl = getattr(self.piper, "ModeCtrl", None)
        if callable(mode_ctrl):
            mode_ctrl(*command)
        else:
            self.piper.MotionCtrl_2(*command)
        self._last_motion_ctrl = command
        # Piper needs one control cycle before accepting the first pose target.
        time.sleep(0.02)

    def _normalize(self, raw_values: dict[str, float]) -> dict[str, float]:
        result: dict[str, float] = {}
        for motor, value in raw_values.items():
            calibration = self.calibration[motor]
            minimum, maximum = calibration.range_min, calibration.range_max
            bounded = min(maximum, max(minimum, value))
            if self.motors[motor].norm_mode is MotorNormMode.RANGE_M100_100:
                result[motor] = ((bounded - minimum) / (maximum - minimum)) * 200.0 - 100.0
            elif self.motors[motor].norm_mode is MotorNormMode.RANGE_0_100:
                result[motor] = ((bounded - minimum) / (maximum - minimum)) * 100.0
            else:
                raise NotImplementedError(self.motors[motor].norm_mode)
        return result

    def _unnormalize(self, values: dict[str, float]) -> dict[str, int]:
        result: dict[str, int] = {}
        for motor, value in values.items():
            calibration = self.calibration[motor]
            minimum, maximum = calibration.range_min, calibration.range_max
            if self.motors[motor].norm_mode is MotorNormMode.RANGE_M100_100:
                bounded = min(100.0, max(-100.0, float(value)))
                raw = ((bounded + 100.0) / 200.0) * (maximum - minimum) + minimum
            elif self.motors[motor].norm_mode is MotorNormMode.RANGE_0_100:
                bounded = min(100.0, max(0.0, float(value)))
                raw = (bounded / 100.0) * (maximum - minimum) + minimum
            else:
                raise NotImplementedError(self.motors[motor].norm_mode)
            result[motor] = int(raw)
        return result
