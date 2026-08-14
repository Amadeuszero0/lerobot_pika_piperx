"""Single- and dual-arm AgileX Piper follower implementations.

Adapted from AgRoboticsResearch/lerobot_robot_piper under Apache-2.0.
"""

import logging
import math
import time
from collections.abc import Callable
from concurrent.futures import Future, ThreadPoolExecutor
from functools import cached_property
from typing import Any, TypeVar

from lerobot.cameras import Camera
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots import Robot
from lerobot.robots.utils import ensure_safe_goal_position
from lerobot.utils.errors import DeviceNotConnectedError

from lerobot_real.configs.piper import DualPiperFollowerConfig, PiperFollowerConfig
from lerobot_real.devices.piper.piper_motors_bus import (
    PiperFeedbackError,
    PiperFeedbackStaleError,
    PiperMotorsBus,
)
from lerobot_real.devices.piper.pose import (
    axis_angle_to_rpy_degrees,
    clamp,
    rotation_distance,
    rotation_step_towards,
    rpy_degrees_to_axis_angle,
    vector_step_towards,
)
from lerobot_real.devices.piper.tables import CALIBRATION, MOTORS

logger = logging.getLogger(__name__)
T = TypeVar("T")

JOINT_KEYS = tuple(f"joint{i}.pos" for i in range(1, 7)) + ("gripper.pos",)
JOINT_ANGLE_KEYS = tuple(f"joint{i}.angle_rad" for i in range(1, 7))
POSE_KEYS = ("pose.x", "pose.y", "pose.z", "pose.rx", "pose.ry", "pose.rz")


class PiperFollower(Robot):
    config_class = PiperFollowerConfig
    name = "piper_follower"

    def __init__(self, config: PiperFollowerConfig, prefix: str = "") -> None:
        super().__init__(config)
        self.config = config
        self.prefix = f"{prefix}." if prefix else ""
        self.bus = PiperMotorsBus(
            id=config.id or prefix or "piper",
            port=config.port,
            motors=MOTORS.copy(),
            calibration=CALIBRATION.copy(),
            feedback_timeout_s=config.feedback_timeout_s,
        )
        self.cameras: dict[str, Camera] = make_cameras_from_configs(config.cameras)
        self._force_step_cartesian = False
        self._official_ik = None
        self._last_ik_joint_command: tuple[float, ...] | None = None
        self._last_ik_action: dict[str, float] | None = None
        self._ik_over_limit = False
        self._feedback_stale_since: float | None = None
        self._last_observation: dict[str, Any] | None = None
        if config.cartesian_command_mode == "official_ik":
            self._official_ik = self._create_official_ik_worker()
        self._camera_executor: ThreadPoolExecutor | None = None
        if self.cameras:
            self._camera_executor = ThreadPoolExecutor(
                max_workers=len(self.cameras), thread_name_prefix=f"{prefix or 'piper'}-camera"
            )

    def _create_official_ik_worker(self) -> Any:
        from lerobot_real.devices.piper.official_kinematics import OfficialPiperIKWorker

        return OfficialPiperIKWorker(
            self.config.ik_urdf_path or "",
            self.config.ik_package_dir or "",
            name=self.id or "piper",
        )

    def _key(self, local_key: str) -> str:
        return f"{self.prefix}{local_key}"

    @cached_property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        state = {self._key(key): float for key in self._state_keys}
        if getattr(self.config, "record_joint_angles", False):
            state.update({self._key(key): float for key in JOINT_ANGLE_KEYS})
        images = {
            self._key(name): (camera.height, camera.width, 3)
            for name, camera in self.cameras.items()
        }
        return {**state, **images}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {self._key(key): float for key in self._state_keys}

    @property
    def _state_keys(self) -> tuple[str, ...]:
        if self.config.control_space == "joint":
            return JOINT_KEYS
        return POSE_KEYS + ("gripper.pos",)

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(
            camera.is_connected for camera in self.cameras.values()
        )

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def connect(self, calibrate: bool = True) -> None:
        if (
            getattr(self.config, "cartesian_command_mode", "step") == "official_ik"
            and getattr(self, "_official_ik", None) is None
        ):
            self._official_ik = self._create_official_ik_worker()
        self.bus.connect()
        try:
            if self.config.configure_role_on_connect:
                self.bus.set_follower()
                time.sleep(0.1)
            self.bus.wait_for_follower_feedback(
                self.config.control_space,
                self.config.feedback_startup_timeout_s,
            )
            self.bus.enable_torque()
            self.bus.assert_follower_ready()
            # This integration uses the Piper's fixed factory ranges, so it is
            # normally already calibrated. Avoid moving the gripper merely
            # because LeRobot calls connect(calibrate=True) by default.
            if calibrate and not self.is_calibrated:
                self.calibrate()
            if self.config.park_on_connect:
                self.bus.parking()
            for camera in self.cameras.values():
                camera.connect()
            if self.cameras and self._camera_executor is None:
                self._camera_executor = ThreadPoolExecutor(
                    max_workers=len(self.cameras),
                    thread_name_prefix=f"{self.id or 'piper'}-camera",
                )
        except BaseException:
            for camera in self.cameras.values():
                try:
                    if camera.is_connected:
                        camera.disconnect()
                except Exception:
                    logger.exception("Failed to close camera after Piper connect failure")
            try:
                self.bus.disconnect(
                    disable_torque=self.config.disable_torque_on_disconnect,
                    park=False,
                )
            except Exception:
                logger.exception("Failed to close Piper bus after connect failure")
            raise
        logger.info("Connected Piper follower %s on %s", self.id, self.config.port)

    def calibrate(self) -> None:
        self.bus.clear_gripper()

    def configure(self) -> None:
        # Role and motion mode are refreshed on connect/send_action respectively.
        return None

    def move_to_tcp_pose(
        self,
        pose_mm_rpy_deg: tuple[float, ...] | list[float],
        *,
        timeout_s: float = 30.0,
        translation_tolerance_mm: float = 2.0,
        rotation_tolerance_rad: float = 0.02,
    ) -> None:
        """Move gradually to a configured TCP pose before capturing a live teleop reference."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        if self.config.control_space != "cartesian":
            raise RuntimeError("Piper TCP pre-positioning requires cartesian control")
        if len(pose_mm_rpy_deg) != 6:
            raise ValueError("Piper TCP pose must contain six values")

        target = tuple(float(value) for value in pose_mm_rpy_deg)
        if not all(math.isfinite(value) for value in target):
            raise ValueError("Piper TCP pose must contain only finite values")
        for name, value in (
            ("timeout_s", timeout_s),
            ("translation_tolerance_mm", translation_tolerance_mm),
            ("rotation_tolerance_rad", rotation_tolerance_rad),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for axis, value, bounds in zip(
            "xyz",
            target[:3],
            (self.config.workspace_x, self.config.workspace_y, self.config.workspace_z),
            strict=True,
        ):
            if bounds is not None and not bounds[0] <= value <= bounds[1]:
                raise ValueError(
                    f"Piper {self.id} startup {axis}={value:.3f} is outside "
                    f"workspace [{bounds[0]}, {bounds[1]}]"
                )

        target_rotation = rpy_degrees_to_axis_angle(*target[3:6])
        gripper_unit = self.bus.get_joint_position()["gripper"] / 100.0
        local_action = dict(zip(POSE_KEYS, (*target[:3], *target_rotation), strict=True))
        local_action["gripper.pos"] = gripper_unit
        action = {self._key(key): value for key, value in local_action.items()}
        deadline = time.monotonic() + timeout_s

        logger.info("Moving Piper %s to startup TCP pose %s", self.id, target)
        while True:
            current = self.bus.get_end_pose()
            current_rotation = rpy_degrees_to_axis_angle(*current[3:6])
            translation_error = math.dist(current[:3], target[:3])
            angular_error = rotation_distance(current_rotation, target_rotation)
            if (
                translation_error <= translation_tolerance_mm
                and angular_error <= rotation_tolerance_rad
            ):
                logger.info(
                    "Piper %s reached startup TCP pose (translation error %.3f mm, "
                    "rotation error %.4f rad)",
                    self.id,
                    translation_error,
                    angular_error,
                )
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Piper {self.id} did not reach startup TCP pose within {timeout_s:.1f}s "
                    f"(translation error {translation_error:.3f} mm, "
                    f"rotation error {angular_error:.4f} rad)"
                )
            previous_force_step = getattr(self, "_force_step_cartesian", False)
            self._force_step_cartesian = True
            try:
                self.send_action(action)
            finally:
                self._force_step_cartesian = previous_force_step
            time.sleep(0.02)

    def setup_motors(self) -> None:
        self.bus.connect()
        self.bus.set_follower()

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        try:
            if self.config.control_space == "joint":
                local = {
                    f"{motor}.pos": value
                    for motor, value in self.bus.get_joint_position().items()
                }
            else:
                x, y, z, roll, pitch, yaw = self.bus.get_end_pose()
                rx, ry, rz = rpy_degrees_to_axis_angle(roll, pitch, yaw)
                gripper = self.bus.get_joint_position()["gripper"] / 100.0
                local = dict(zip(POSE_KEYS, (x, y, z, rx, ry, rz), strict=True))
                local["gripper.pos"] = gripper
            if getattr(self.config, "record_joint_angles", False):
                local.update(
                    dict(
                        zip(
                            JOINT_ANGLE_KEYS,
                            self.bus.get_joint_radians(),
                            strict=True,
                        )
                    )
                )
        except PiperFeedbackStaleError as exc:
            self._hold_for_stale_feedback(exc)
            if self._last_observation is None:
                raise
            return self._last_observation.copy()

        observation = {self._key(key): value for key, value in local.items()}
        if self._camera_executor is not None:
            futures = {
                name: self._camera_executor.submit(camera.async_read)
                for name, camera in self.cameras.items()
            }
            for name, future in futures.items():
                observation[self._key(name)] = future.result()
        self._mark_feedback_recovered()
        self._last_observation = observation.copy()
        return observation

    def _hold_for_stale_feedback(self, exc: PiperFeedbackStaleError) -> None:
        if self.config.cartesian_command_mode != "official_ik":
            raise exc
        now = time.monotonic()
        if self._feedback_stale_since is None:
            self._feedback_stale_since = now
            logger.warning(
                "Piper %s feedback is temporarily stale; holding the last valid "
                "joint target: %s",
                self.id,
                exc,
            )
        recovery_timeout_s = self.config.feedback_startup_timeout_s
        if now - self._feedback_stale_since >= recovery_timeout_s:
            raise PiperFeedbackStaleError(
                f"Piper {self.id} feedback remained stale for {recovery_timeout_s:.1f}s"
            ) from exc

    def _mark_feedback_recovered(self) -> None:
        if self._feedback_stale_since is not None:
            logger.info("Piper %s feedback recovered; joint streaming resumed", self.id)
            self._feedback_stale_since = None

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")

        local = self._strip_and_validate(action)
        try:
            self.bus.assert_follower_ready()
            if self.config.control_space == "joint":
                sent = self._send_joint_action(local)
            else:
                sent = self._send_cartesian_action(local)
        except PiperFeedbackStaleError as exc:
            self._hold_for_stale_feedback(exc)
            sent = self._last_ik_action.copy() if self._last_ik_action is not None else local
        else:
            self._mark_feedback_recovered()
        return {self._key(key): value for key, value in sent.items()}

    def _strip_and_validate(self, action: dict[str, Any]) -> dict[str, float]:
        local: dict[str, float] = {}
        for key, value in action.items():
            if self.prefix:
                if not key.startswith(self.prefix):
                    continue
                key = key[len(self.prefix) :]
            numeric_value = float(value)
            if not math.isfinite(numeric_value):
                raise ValueError(f"Action for {self.id} contains non-finite {key}={value!r}")
            local[key] = numeric_value
        missing = set(self._state_keys) - set(local)
        if missing:
            raise KeyError(f"Action for {self.id} is missing keys: {sorted(missing)}")
        return {key: local[key] for key in self._state_keys}

    def _send_joint_action(self, local: dict[str, float]) -> dict[str, float]:
        goal = {key.removesuffix(".pos"): value for key, value in local.items()}
        if self.config.max_relative_target is not None:
            present = self.bus.get_joint_position()
            paired = {key: (value, present[key]) for key, value in goal.items()}
            goal = ensure_safe_goal_position(paired, self.config.max_relative_target)
        sent = self.bus.set_joint_position(goal, speed_percent=self.config.move_speed_percent)
        return {f"{motor}.pos": value for motor, value in sent.items()}

    def _send_cartesian_action(self, local: dict[str, float]) -> dict[str, float]:
        x, y, z, roll, pitch, yaw = self.bus.get_end_pose()
        current_pose = (x, y, z, roll, pitch, yaw)
        if not all(math.isfinite(value) for value in current_pose):
            raise PiperFeedbackError(
                f"Piper {self.id} returned a non-finite Cartesian pose; refusing command"
            )
        current_rx, current_ry, current_rz = rpy_degrees_to_axis_angle(roll, pitch, yaw)
        current_xyz = (x, y, z)
        current_rotation = (current_rx, current_ry, current_rz)
        for axis, value, bounds in zip(
            "xyz",
            current_xyz,
            (self.config.workspace_x, self.config.workspace_y, self.config.workspace_z),
            strict=True,
        ):
            if bounds is not None and not bounds[0] <= value <= bounds[1]:
                raise PiperFeedbackError(
                    f"Piper {self.id} current {axis}={value:.3f} is outside "
                    f"workspace [{bounds[0]}, {bounds[1]}]; refusing Cartesian command"
                )
        target = tuple(local[key] for key in POSE_KEYS)
        # Clamp the target before limiting the step. Clamping the already-limited
        # command could cause a large jump when the current pose is outside bounds.
        bounded_xyz = (
            clamp(target[0], self.config.workspace_x),
            clamp(target[1], self.config.workspace_y),
            clamp(target[2], self.config.workspace_z),
        )
        official_ik_command = (
            getattr(self.config, "cartesian_command_mode", "step") == "official_ik"
            and not getattr(self, "_force_step_cartesian", False)
        )
        if official_ik_command:
            return self._send_official_ik_action(
                (*bounded_xyz, *target[3:6]),
                current_xyz,
                current_rotation,
                local["gripper.pos"],
            )
        direct_command = (
            getattr(self.config, "cartesian_command_mode", "step") == "direct"
            and not getattr(self, "_force_step_cartesian", False)
        )
        if direct_command:
            translation_error = math.dist(current_xyz, bounded_xyz)
            angular_error = rotation_distance(current_rotation, target[3:6])
            if translation_error > self.config.max_cartesian_following_error_mm:
                raise PiperFeedbackError(
                    f"Piper {self.id} direct target is {translation_error:.3f} mm away; "
                    f"limit is {self.config.max_cartesian_following_error_mm:.3f} mm"
                )
            if angular_error > self.config.max_rotation_following_error_rad:
                raise PiperFeedbackError(
                    f"Piper {self.id} direct target is {angular_error:.4f} rad away; "
                    f"limit is {self.config.max_rotation_following_error_rad:.4f} rad"
                )
            limited = [*bounded_xyz, *target[3:6]]
        else:
            limited_xyz = vector_step_towards(
                current_xyz, bounded_xyz, self.config.max_cartesian_step_mm
            )
            limited_rotation = rotation_step_towards(
                current_rotation, target[3:6], self.config.max_rotation_step_rad
            )
            limited = [*limited_xyz, *limited_rotation]
        if not all(math.isfinite(value) for value in limited):
            raise PiperFeedbackError(
                f"Piper {self.id} computed a non-finite Cartesian command; refusing to send"
            )

        roll_deg, pitch_deg, yaw_deg = axis_angle_to_rpy_degrees(*limited[3:6])
        command_pose = (*limited[:3], roll_deg, pitch_deg, yaw_deg)
        if not all(math.isfinite(value) for value in command_pose):
            raise PiperFeedbackError(
                f"Piper {self.id} computed a non-finite Cartesian command; refusing to send"
            )
        self.bus.set_end_pose(
            command_pose,
            move_mode=self.config.move_mode,
            speed_percent=self.config.move_speed_percent,
        )
        gripper_unit = min(1.0, max(0.0, local["gripper.pos"]))
        self.bus.set_gripper_percent(gripper_unit * 100.0, effort=self.config.gripper_effort)
        sent = dict(zip(POSE_KEYS, limited, strict=True))
        sent["gripper.pos"] = gripper_unit
        return sent

    def _send_official_ik_action(
        self,
        target_pose: tuple[float, ...],
        current_xyz: tuple[float, ...],
        current_rotation: tuple[float, ...],
        gripper_position: float,
    ) -> dict[str, float]:
        if self._official_ik is None:
            raise RuntimeError("Piper official IK was not initialized")

        current_joints = self.bus.get_joint_radians()
        gripper_unit = min(1.0, max(0.0, float(gripper_position)))
        solved_target = target_pose
        update_target = getattr(self._official_ik, "update_target", None)
        if update_target is not None:
            update = update_target(
                target_pose,
                current_joints,
                gripper_width_m=gripper_unit * 0.068,
            )
            if update is None:
                return self._stream_last_ik_command(
                    current_xyz,
                    current_rotation,
                    gripper_unit,
                )
            solved_target = update.target_pose
            result = update.result
        else:
            result = self._official_ik.solve_native_pose(
                target_pose,
                current_joints,
                gripper_width_m=gripper_unit * 0.068,
            )
        if result is None:
            if not self._ik_over_limit:
                logger.warning(
                    "Piper %s official IK over_limit=True; holding the last valid joint target",
                    self.id,
                )
            self._ik_over_limit = True
            return self._stream_last_ik_command(
                current_xyz,
                current_rotation,
                gripper_unit,
            )

        if self._ik_over_limit:
            logger.info("Piper %s official IK over_limit=False; joint streaming resumed", self.id)
        self._ik_over_limit = False
        reference = self._last_ik_joint_command or current_joints
        max_joint_change = max(
            abs(target - previous)
            for target, previous in zip(result.joints_rad, reference, strict=True)
        )
        if max_joint_change > math.radians(30.0):
            steps = max(1, int(max_joint_change / math.radians(1.0)))
            sent_joints = reference
            for step in range(1, steps + 1):
                ratio = step / steps
                interpolated = tuple(
                    previous + (target - previous) * ratio
                    for previous, target in zip(reference, result.joints_rad, strict=True)
                )
                sent_state = self.bus.set_joint_state(
                    (*interpolated, gripper_unit * 0.068),
                    speed_percent=self.config.move_speed_percent,
                    gripper_effort=self.config.gripper_effort,
                )
                sent_joints = sent_state[:6]
                if step < steps:
                    time.sleep(1.0 / 200.0)
        else:
            sent_state = self.bus.set_joint_state(
                (*result.joints_rad, gripper_unit * 0.068),
                speed_percent=self.config.move_speed_percent,
                gripper_effort=self.config.gripper_effort,
            )
            sent_joints = sent_state[:6]

        self._last_ik_joint_command = sent_joints
        sent = dict(zip(POSE_KEYS, solved_target, strict=True))
        sent["gripper.pos"] = gripper_unit
        self._last_ik_action = sent.copy()
        return sent

    def _stream_last_ik_command(
        self,
        current_xyz: tuple[float, ...],
        current_rotation: tuple[float, ...],
        gripper_unit: float,
    ) -> dict[str, float]:
        if self._last_ik_joint_command is not None:
            sent_state = self.bus.set_joint_state(
                (*self._last_ik_joint_command, gripper_unit * 0.068),
                speed_percent=self.config.move_speed_percent,
                gripper_effort=self.config.gripper_effort,
            )
            self._last_ik_joint_command = sent_state[:6]
        if self._last_ik_action is not None:
            held = self._last_ik_action.copy()
            held["gripper.pos"] = gripper_unit
            return held
        held = dict(zip(POSE_KEYS, (*current_xyz, *current_rotation), strict=True))
        held["gripper.pos"] = gripper_unit
        return held

    def parking(self) -> None:
        self.bus.parking()

    def _hold_position_before_disconnect(self) -> None:
        if getattr(self, "_official_ik", None) is not None:
            joint_state = self.bus.get_joint_state()
            self.bus.set_joint_state(
                joint_state,
                speed_percent=self.config.move_speed_percent,
                gripper_effort=self.config.gripper_effort,
            )
            time.sleep(0.05)
            logger.info("Piper %s commanded to hold its current joints", self.id)
            return
        pose = self.bus.get_end_pose()
        if not all(math.isfinite(value) for value in pose):
            raise PiperFeedbackError(
                f"Piper {self.id} returned a non-finite pose; refusing hold command"
            )
        self.bus.set_end_pose(
            pose,
            move_mode=self.config.move_mode,
            speed_percent=self.config.move_speed_percent,
        )
        time.sleep(0.05)
        logger.info("Piper %s commanded to hold its current pose", self.id)

    def disconnect(self) -> None:
        first_error: Exception | None = None
        if (
            self.bus.is_connected
            and self.config.control_space == "cartesian"
            and not self.config.disable_torque_on_disconnect
            and getattr(self.config, "hold_position_on_disconnect", False)
        ):
            try:
                self._hold_position_before_disconnect()
            except Exception as exc:
                logger.exception("Failed to hold Piper %s before disconnect", self.id)
                first_error = first_error or exc
        for camera in self.cameras.values():
            try:
                if camera.is_connected:
                    camera.disconnect()
            except Exception as exc:
                first_error = first_error or exc
        try:
            if self._camera_executor is not None:
                self._camera_executor.shutdown(wait=True)
                self._camera_executor = None
        except Exception as exc:
            first_error = first_error or exc
        try:
            self.bus.disconnect(
                disable_torque=self.config.disable_torque_on_disconnect,
                park=self.config.park_on_disconnect,
            )
            if not self.config.disable_torque_on_disconnect:
                logger.warning(
                    "Disconnected Piper follower %s without sending an automatic "
                    "motor-disable command",
                    self.id,
                )
        except Exception as exc:
            first_error = first_error or exc
        if getattr(self, "_official_ik", None) is not None:
            close_ik = getattr(self._official_ik, "close", None)
            if close_ik is not None:
                try:
                    close_ik()
                except Exception as exc:
                    first_error = first_error or exc
                self._official_ik = None
        if first_error is not None:
            raise first_error


class DualPiperFollower(Robot):
    config_class = DualPiperFollowerConfig
    name = "dual_piper_follower"

    def __init__(self, config: DualPiperFollowerConfig) -> None:
        super().__init__(config)
        self.config = config
        self.robots: dict[str, PiperFollower] = {}
        for side, robot_config in config.robots.items():
            if not isinstance(robot_config, PiperFollowerConfig):
                raise TypeError(
                    f"{side} must use type lerobot_real::piper, got {robot_config.type}"
                )
            self.robots[side] = PiperFollower(robot_config, prefix=side)
        self.cameras = {
            f"{side}.{name}": camera
            for side, robot in self.robots.items()
            for name, camera in robot.cameras.items()
        }
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="dual-piper")

    @property
    def observation_features(self) -> dict:
        return self._merge(lambda robot: robot.observation_features)

    @property
    def action_features(self) -> dict:
        return self._merge(lambda robot: robot.action_features)

    @property
    def is_connected(self) -> bool:
        return all(robot.is_connected for robot in self.robots.values())

    @property
    def is_calibrated(self) -> bool:
        return all(robot.is_calibrated for robot in self.robots.values())

    def _merge(self, getter: Callable[[PiperFollower], dict]) -> dict:
        merged: dict = {}
        for robot in self.robots.values():
            merged.update(getter(robot))
        return merged

    def _parallel(self, fn: Callable[[PiperFollower], T]) -> list[T]:
        futures: list[Future[T]] = [
            self._executor.submit(fn, robot) for robot in self.robots.values()
        ]
        return [future.result() for future in futures]

    def connect(self, calibrate: bool = True) -> None:
        try:
            if self.config.parallel_connect:
                self._parallel(lambda robot: robot.connect(calibrate=calibrate))
            else:
                for robot in self.robots.values():
                    robot.connect(calibrate=calibrate)
        except BaseException:
            for robot in self.robots.values():
                if robot.bus.is_connected:
                    try:
                        robot.disconnect()
                    except Exception:
                        logger.exception("Failed to clean up %s after a partial connect", robot.id)
            raise

    def calibrate(self) -> None:
        for robot in self.robots.values():
            robot.calibrate()

    def configure(self) -> None:
        for robot in self.robots.values():
            robot.configure()

    def get_observation(self) -> dict[str, Any]:
        observations = (
            self._parallel(lambda robot: robot.get_observation())
            if self.config.parallel_observation
            else [robot.get_observation() for robot in self.robots.values()]
        )
        merged: dict[str, Any] = {}
        for observation in observations:
            merged.update(observation)
        return merged

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        def send(robot: PiperFollower) -> dict[str, Any]:
            return robot.send_action(action)

        sent_actions = (
            self._parallel(send)
            if self.config.parallel_action
            else [send(robot) for robot in self.robots.values()]
        )
        merged: dict[str, Any] = {}
        for sent in sent_actions:
            merged.update(sent)
        return merged

    def disconnect(self) -> None:
        try:
            self._parallel(lambda robot: robot.disconnect())
        finally:
            self._executor.shutdown(wait=True)
