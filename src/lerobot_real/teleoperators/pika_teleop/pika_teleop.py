import logging
import math
from collections import deque
from threading import Event, Lock, Thread
from typing import Any

import numpy as np
from lerobot.utils.errors import DeviceNotConnectedError

from lerobot_real.devices.pika import PikaDevice
from lerobot_real.devices.umi.vive_tracker.transformations import Transformations

from ..base_teleop import BaseTeleop
from .pika_teleop_config import PikaTeleopConfig

logger = logging.getLogger(__name__)
POSE_KEYS = ("pose.x", "pose.y", "pose.z", "pose.rx", "pose.ry", "pose.rz")
TRACKER_FILTER_WINDOW = 3


class PikaTeleop(BaseTeleop, Thread):
    config_class = PikaTeleopConfig
    name = "Pika Teleop"

    def __init__(self, config: PikaTeleopConfig, prefix: str = "") -> None:
        super().__init__(config)
        Thread.__init__(self, name=f"{prefix or 'pika'}-command-state", daemon=True)
        self.stop_event = Event()
        self.config = config
        self._is_connected = False
        self._is_calibrated = True
        self._data_lock = Lock()
        self._teleop_enabled = False
        self._last_action: dict[str, float] | None = None
        self._session_generation = 0
        self._thread_error: Exception | None = None
        self._activation_ready = False
        self._pending_robot_sync = False
        self._gesture_closed = False
        self._gesture_opened = False
        self.prefix = "" if not prefix else f"{prefix}."

        tracker_to_robot_eef = list(self.config.tracker_to_robot_eef[:3]) + list(
            map(math.radians, self.config.tracker_to_robot_eef[3:6])
        )
        self.tracker_to_robot_matrix = Transformations.xyzrpy_to_rotation_matrix(
            *tracker_to_robot_eef
        )
        self.robot_eef_to_tracker_matrix = np.linalg.inv(self.tracker_to_robot_matrix)
        world_to_base_rpy = tuple(
            map(math.radians, self.config.tracker_world_to_robot_base_rpy)
        )
        self.tracker_world_to_robot_base_rotation = Transformations.rpy_to_rotation_matrix(
            *world_to_base_rpy
        )
        robot_base_pose = list(self.config.robot_base_pose[:3]) + list(
            map(math.radians, self.config.robot_base_pose[3:6])
        )
        self.robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*robot_base_pose)
        self.begin_tracker_matrix = None
        self.begin_tracker_robot_matrix = None
        self._tracker_matrix_window: deque[np.ndarray] = deque(
            maxlen=TRACKER_FILTER_WINDOW
        )
        self._last_robot_pose = Transformations.rotation_matrix_to_xyzrxryrz(self.robot_base_matrix)
        self._last_gripper_pos = 0.0

        self.pika_device = PikaDevice(
            1,
            pika_sense_port=self.config.port,
            pika_tracker_device=self.config.tracker_device_id,
        )
        self.pika_sense = self.pika_device.pika_sense

    @property
    def action_features(self) -> dict:
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (7,),
                "names": {
                    "pose.x": 0,
                    "pose.y": 1,
                    "pose.z": 2,
                    "pose.rx": 3,
                    "pose.ry": 4,
                    "pose.rz": 5,
                    "gripper.pos": 6,
                },
            }
        else:
            return {
                "dtype": "float32",
                "shape": (6,),
                "names": {
                    "pose.x": 0,
                    "pose.y": 1,
                    "pose.z": 2,
                    "pose.rx": 3,
                    "pose.ry": 4,
                    "pose.rz": 5,
                },
            }

    @property
    def feedback_features(self) -> dict:
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (7,),
                "names": {
                    "pose.x": 0,
                    "pose.y": 1,
                    "pose.z": 2,
                    "pose.rx": 3,
                    "pose.ry": 4,
                    "pose.rz": 5,
                    "gripper.pos": 6,
                },
            }
        else:
            return {
                "dtype": "float32",
                "shape": (6,),
                "names": {
                    "pose.x": 0,
                    "pose.y": 1,
                    "pose.z": 2,
                    "pose.rx": 3,
                    "pose.ry": 4,
                    "pose.rz": 5,
                },
            }

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    @property
    def uses_activation_gesture(self) -> bool:
        return self.config.activation_mode == "gripper_gesture"

    def calibrate(self) -> None:
        # CHECK!!
        pass

    def configure(self) -> None:
        pass

    def connect(self, calibrate: bool = False) -> None:
        if self._is_connected:
            return
        if self.ident is not None:
            raise RuntimeError(
                "A disconnected PikaTeleop cannot be reconnected; create a new instance"
            )
        super().connect(calibrate)
        self.stop_event.clear()
        self._thread_error = None
        self._is_connected = True
        try:
            self.start()
        except BaseException:
            self._is_connected = False
            super().disconnect()
            raise

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.is_alive():
            self.join(timeout=2.0)
        with self._data_lock:
            self._is_connected = False
            self._teleop_enabled = False
            self._activation_ready = False
            self._pending_robot_sync = False
            self._gesture_closed = False
            self._gesture_opened = False
            self._session_generation += 1
        try:
            self.pika_device.disconnect()
        finally:
            if self.is_alive():
                self.join(timeout=2.0)
            if self.is_alive():
                logger.warning("Pika command-state thread did not stop during disconnect")
            super().disconnect()

    def _hold_action_locked(self) -> dict[str, float]:
        action = {
            f"{self.prefix}{key}": float(value)
            for key, value in zip(POSE_KEYS, self._last_robot_pose, strict=True)
        }
        if self.config.use_gripper:
            action[f"{self.prefix}gripper.pos"] = float(self._last_gripper_pos)
        return action

    def _current_action_locked(self) -> dict[str, float]:
        if self._last_action is None:
            self._last_action = self._hold_action_locked()
        return self._last_action.copy()

    def _set_reference_locked(self, obs: dict) -> None:
        self._last_robot_pose = [float(obs[f"{self.prefix}{key}"]) for key in POSE_KEYS]
        if self.config.use_gripper:
            self._last_gripper_pos = float(obs[f"{self.prefix}gripper.pos"])
        self.robot_base_matrix = Transformations.xyzrxryrz_to_rotation_matrix(
            *self._last_robot_pose
        )

    @staticmethod
    def _pose_xyzrpy_degrees(matrix: np.ndarray) -> tuple[float, ...]:
        pose = Transformations.rotation_matrix_to_xyzrpy(matrix)
        return (
            *(float(value) for value in pose[:3]),
            *(math.degrees(float(value)) for value in pose[3:]),
        )

    def _activation_frame_mapping(
        self, tracker_matrix: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        gripper_base_matrix = (
            self.robot_base_matrix @ self.robot_eef_to_tracker_matrix
        )
        if self.config.control_frame == "robot_base":
            axis_mapping = self.tracker_world_to_robot_base_rotation
        else:
            axis_mapping = (
                gripper_base_matrix[:3, :3] @ tracker_matrix[:3, :3].T
            )
        return gripper_base_matrix, axis_mapping

    @staticmethod
    def _rotation_distance_radians(first: np.ndarray, second: np.ndarray) -> float:
        cosine = (np.trace(first.T @ second) - 1.0) / 2.0
        return math.acos(float(np.clip(cosine, -1.0, 1.0)))

    def _filter_tracker_matrix(self, tracker_matrix: np.ndarray) -> np.ndarray:
        if not self._tracker_matrix_window:
            for _ in range(TRACKER_FILTER_WINDOW):
                self._tracker_matrix_window.append(tracker_matrix.copy())
        else:
            self._tracker_matrix_window.append(tracker_matrix.copy())

        matrices = tuple(self._tracker_matrix_window)
        filtered = matrices[-1].copy()
        filtered[:3, 3] = np.median(
            np.stack([matrix[:3, 3] for matrix in matrices]), axis=0
        )

        # Select the measured rotation closest to the other samples, avoiding
        # quaternion averaging artifacts while rejecting a one-frame spike.
        rotations = tuple(matrix[:3, :3] for matrix in matrices)
        scores = [
            sum(
                self._rotation_distance_radians(candidate, other)
                for other in rotations
            )
            for candidate in rotations
        ]
        filtered[:3, :3] = rotations[min(range(len(scores)), key=scores.__getitem__)]
        return filtered

    def _official_control_target(self, tracker_robot_matrix: np.ndarray) -> np.ndarray:
        if self.begin_tracker_robot_matrix is None:
            raise RuntimeError("Pika tracker reference has not been captured")

        return (
            self.robot_base_matrix
            @ np.linalg.inv(self.begin_tracker_robot_matrix)
            @ tracker_robot_matrix
        )

    def _robot_base_control_target(self, tracker_matrix: np.ndarray) -> np.ndarray:
        if self.begin_tracker_matrix is None:
            raise RuntimeError("Pika tracker reference has not been captured")

        gripper_base_matrix = (
            self.robot_base_matrix @ self.robot_eef_to_tracker_matrix
        )
        mapping = self.tracker_world_to_robot_base_rotation
        delta_rotation = (
            tracker_matrix[:3, :3] @ self.begin_tracker_matrix[:3, :3].T
        )

        target_gripper_matrix = gripper_base_matrix.copy()
        target_gripper_matrix[:3, :3] = (
            mapping
            @ delta_rotation
            @ mapping.T
            @ gripper_base_matrix[:3, :3]
        )
        target_gripper_matrix[:3, 3] = (
            gripper_base_matrix[:3, 3]
            + mapping
            @ (tracker_matrix[:3, 3] - self.begin_tracker_matrix[:3, 3])
        )
        return target_gripper_matrix @ self.tracker_to_robot_matrix

    def _log_activation_frames(self, tracker_matrix: np.ndarray) -> None:
        gripper_base_matrix, axis_mapping = self._activation_frame_mapping(
            tracker_matrix
        )
        p0_pose = self._pose_xyzrpy_degrees(tracker_matrix)
        g0_pose = self._pose_xyzrpy_degrees(gripper_base_matrix)
        label = f"{self.prefix}PIKA"
        logger.info("[%s] control_frame = %s", label, self.config.control_frame)
        logger.info(
            "[%s] P0 [xyz mm, rpy deg] = %s",
            label,
            [round(value, 3) for value in p0_pose],
        )
        logger.info(
            "[%s] G0 [xyz mm, rpy deg] = %s",
            label,
            [round(value, 3) for value in g0_pose],
        )
        for axis, vector in zip("XYZ", axis_mapping.T, strict=True):
            logger.info(
                "[%s] Pika world +%s -> robot-base XYZ %s",
                label,
                axis,
                [round(float(value), 3) for value in vector],
            )

    def _activate_locked(self) -> None:
        self.begin_tracker_matrix = None
        self.begin_tracker_robot_matrix = None
        self._tracker_matrix_window.clear()
        self._pending_robot_sync = False
        self._teleop_enabled = True
        self._activation_ready = False
        self._gesture_closed = False
        self._gesture_opened = False
        self._last_action = self._hold_action_locked()
        self._session_generation += 1

    def _update_gripper_gesture_locked(self, distance_mm: float) -> bool:
        if self._teleop_enabled or not self._activation_ready:
            self._gesture_closed = False
            self._gesture_opened = False
            return False
        if not self._gesture_closed:
            if distance_mm <= self.config.activation_close_threshold_mm:
                self._gesture_closed = True
            return False
        if not self._gesture_opened:
            if distance_mm >= self.config.activation_open_threshold_mm:
                self._gesture_opened = True
            return False
        if distance_mm <= self.config.activation_close_threshold_mm:
            self._activation_ready = False
            self._pending_robot_sync = True
            self._gesture_closed = False
            self._gesture_opened = False
            return True
        return False

    @property
    def is_teleop_enabled(self) -> bool:
        with self._data_lock:
            return self._teleop_enabled

    def has_pending_robot_sync(self) -> bool:
        with self._data_lock:
            return self._pending_robot_sync

    def apply_pending_robot_sync(self, obs: dict) -> bool:
        with self._data_lock:
            if not self._pending_robot_sync:
                return False
            self._set_reference_locked(obs)
            self._activate_locked()
        print(f"[{self.prefix}PIKA] Teleoperation started")
        return True

    def set_teleop_enabled(self, enabled: bool, obs: dict | None = None) -> None:
        message: str
        with self._data_lock:
            if enabled:
                if self.uses_activation_gesture:
                    if obs is None:
                        raise ValueError(
                            "A robot observation is required to arm Pika gesture activation"
                        )
                    self._set_reference_locked(obs)
                    self._teleop_enabled = False
                    self._activation_ready = True
                    self._pending_robot_sync = False
                    self._gesture_closed = False
                    self._gesture_opened = False
                    self.begin_tracker_matrix = None
                    self.begin_tracker_robot_matrix = None
                    self._tracker_matrix_window.clear()
                    self._last_action = self._hold_action_locked()
                    self._session_generation += 1
                    message = (
                        f"[{self.prefix}PIKA] Waiting for a full close-open-close gesture "
                        "to start teleoperation"
                    )
                else:
                    if obs is not None:
                        self._set_reference_locked(obs)
                    self._activate_locked()
                    message = f"[{self.prefix}PIKA] Teleoperation started"
            else:
                if obs is None and self._last_action is not None:
                    self._last_robot_pose = [
                        self._last_action[f"{self.prefix}{key}"] for key in POSE_KEYS
                    ]
                    if self.config.use_gripper:
                        self._last_gripper_pos = self._last_action[f"{self.prefix}gripper.pos"]
                self._teleop_enabled = False
                self._activation_ready = False
                self._pending_robot_sync = False
                self._gesture_closed = False
                self._gesture_opened = False
                self.begin_tracker_matrix = None
                self.begin_tracker_robot_matrix = None
                self._tracker_matrix_window.clear()
                self._last_action = self._hold_action_locked()
                self._session_generation += 1
                message = f"[{self.prefix}PIKA] Teleoperation paused"
        print(message)

    def run(self) -> None:
        try:
            sleep_time = 1.0 / self.config.frequency

            if self.uses_activation_gesture:
                while not self.stop_event.wait(sleep_time):
                    raw_distance = self.pika_sense.get_gripper_distance()
                    if raw_distance is None:
                        continue
                    distance = float(raw_distance)
                    if not math.isfinite(distance):
                        continue
                    with self._data_lock:
                        triggered = self._update_gripper_gesture_locked(distance)
                    if triggered:
                        print(
                            f"[{self.prefix}PIKA] Gesture complete; synchronizing "
                            "the current robot pose"
                        )
                return

            initial_state = self.pika_sense.get_command_state()
            current_state = initial_state

            while not self.stop_event.wait(sleep_time):
                state = self.pika_sense.get_command_state()
                if state == current_state:
                    continue
                current_state = state
                with self._data_lock:
                    enabled = self._teleop_enabled
                if not enabled and current_state != initial_state:
                    self.set_teleop_enabled(True)
                elif enabled and current_state == initial_state:
                    self.set_teleop_enabled(False)
        except Exception as exc:
            logger.exception("Pika command-state thread stopped")
            with self._data_lock:
                self._thread_error = exc
                self._is_connected = False
            self.stop_event.set()

    def get_action(self) -> dict[str, Any] | None:
        if not self.is_connected:
            if self._thread_error is not None:
                raise RuntimeError("Pika command-state reader failed") from self._thread_error
            raise DeviceNotConnectedError(
                "PikaTeleop is not connected. You need to run `connect()` before `get_action()`."
            )

        with self._data_lock:
            if not self._teleop_enabled:
                return None
            generation = self._session_generation

        try:
            pose = self.pika_sense.get_pose(self.pika_device.pika_tracker_device)
        except Exception:
            logger.warning(
                "Failed to read Pika tracker pose; holding the previous action", exc_info=True
            )
            pose = None

        distance: float | None = None
        if self.config.use_gripper:
            try:
                raw_distance = self.pika_sense.get_gripper_distance()
                if raw_distance is not None:
                    candidate = float(raw_distance)
                    if math.isfinite(candidate):
                        distance = min(100.0, max(0.0, candidate))
            except Exception:
                logger.warning(
                    "Failed to read Pika gripper; holding the previous action", exc_info=True
                )

        with self._data_lock:
            if generation != self._session_generation or not self._teleop_enabled:
                return None
            self._current_action_locked()

            if pose is not None:
                try:
                    position = [float(value) for value in pose.position[:3]]
                    quaternion = [float(value) for value in pose.rotation]
                    if all(math.isfinite(value) for value in (*position, *quaternion)):
                        x, y, z = (value * 1000.0 * self.config.scale_xyz for value in position)
                        tracker_matrix = Transformations.xyzq_to_rotation_matrix(
                            x, y, z, quaternion
                        )
                        tracker_matrix = self._filter_tracker_matrix(tracker_matrix)
                        tracker_robot_matrix = tracker_matrix @ self.tracker_to_robot_matrix
                        if self.begin_tracker_robot_matrix is None:
                            self.begin_tracker_matrix = tracker_matrix.copy()
                            self.begin_tracker_robot_matrix = tracker_robot_matrix.copy()
                            self._log_activation_frames(tracker_matrix)
                        if self.config.control_frame == "robot_base":
                            robot_target_matrix = self._robot_base_control_target(tracker_matrix)
                            robot_target_pose = (
                                Transformations.rotation_matrix_to_xyzrxryrz(
                                    robot_target_matrix
                                )
                            )
                        else:
                            robot_target_matrix = self._official_control_target(
                                tracker_robot_matrix
                            )
                            robot_target_pose = (
                                Transformations.rotation_matrix_to_xyzrxryrz(
                                    robot_target_matrix
                                )
                            )
                        if all(math.isfinite(float(value)) for value in robot_target_pose):
                            for key, value in zip(POSE_KEYS, robot_target_pose, strict=True):
                                self._last_action[f"{self.prefix}{key}"] = float(value)
                except Exception:
                    logger.warning(
                        "Invalid Pika tracker sample; holding the previous action", exc_info=True
                    )

            if distance is not None:
                input_min = float(getattr(self.config, "gripper_input_min_mm", 0.0))
                input_max = float(getattr(self.config, "gripper_input_max_mm", 100.0))
                gripper_unit = (distance - input_min) / (input_max - input_min)
                self._last_action[f"{self.prefix}gripper.pos"] = min(
                    1.0, max(0.0, gripper_unit)
                )

            return self._current_action_locked()

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError
