#!/usr/bin/env python

# Copyright 2025 UFACTORY Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from dataclasses import dataclass

from lerobot.teleoperators import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("lerobot_real::pika_teleop")
@dataclass
class PikaTeleopConfig(TeleoperatorConfig):
    # Port to connect to the pika
    port: str = None
    # Persistent Vive hardware serial (LHR-*). Required for dual-Pika setups.
    tracker_device_id: str | None = None
    frequency: int = 100 # hz
    use_gripper: bool = True
    scale_xyz: float = 1.0
    activation_mode: str = "command"
    activation_close_threshold_mm: float = 15.0
    activation_open_threshold_mm: float = 70.0
    # Calibrate the physical Pika endpoints into the normalized [0, 1]
    # gripper action. Defaults preserve the original direct 0..100 mm mapping.
    gripper_input_min_mm: float = 0.0
    gripper_input_max_mm: float = 100.0
    # "official" applies the Pika delta in the local gripper frame. "robot_base"
    # maps the Pika tracking-world delta into the robot base frame.
    control_frame: str = "official"
    # Fixed tracking-world to robot-base rotation for robot_base control mode.
    # Official mode uses the unmodified G0 @ inverse(P0) @ P formula.
    tracker_world_to_robot_base_rpy: tuple[float, ...] = (0, 0, 0)
    # [x, y, z, roll(deg), pitch(deg), yaw(deg)]
    tracker_to_robot_eef: tuple[float, ...] = (0, 0, 0, 180, -90, 0)
    # When false, capture the current robot pose as the teleoperation reference
    # without commanding an automatic startup move.
    move_to_base_on_start: bool = True
    robot_base_pose: tuple[float, ...] = (400, 0, 400, 180, 0, 0)

    def __post_init__(self):
        self.id = 'pika_teleop' if self.id is None else self.id
        if self.tracker_device_id is not None and not self.tracker_device_id.strip():
            raise ValueError("Pika tracker_device_id must not be empty")
        if self.frequency <= 0:
            raise ValueError("Pika frequency must be positive")
        if not math.isfinite(self.scale_xyz) or self.scale_xyz <= 0:
            raise ValueError("Pika scale_xyz must be finite and positive")
        if self.activation_mode not in {"command", "gripper_gesture"}:
            raise ValueError("Pika activation_mode must be 'command' or 'gripper_gesture'")
        if self.control_frame not in {"official", "robot_base"}:
            raise ValueError("Pika control_frame must be 'official' or 'robot_base'")
        world_to_base_rpy = self.tracker_world_to_robot_base_rpy
        if len(world_to_base_rpy) != 3 or not all(
            math.isfinite(value) for value in world_to_base_rpy
        ):
            raise ValueError(
                "Pika tracker_world_to_robot_base_rpy must contain three finite values"
            )
        close_threshold = self.activation_close_threshold_mm
        open_threshold = self.activation_open_threshold_mm
        if (
            not math.isfinite(close_threshold)
            or not math.isfinite(open_threshold)
            or close_threshold < 0
            or open_threshold > 100
            or close_threshold >= open_threshold
        ):
            raise ValueError(
                "Pika activation thresholds must satisfy "
                "0 <= close_threshold_mm < open_threshold_mm <= 100"
            )
        if (
            not math.isfinite(self.gripper_input_min_mm)
            or not math.isfinite(self.gripper_input_max_mm)
            or self.gripper_input_min_mm < 0
            or self.gripper_input_max_mm > 100
            or self.gripper_input_min_mm >= self.gripper_input_max_mm
        ):
            raise ValueError(
                "Pika gripper input endpoints must satisfy "
                "0 <= gripper_input_min_mm < gripper_input_max_mm <= 100"
            )
        for name in ("tracker_to_robot_eef", "robot_base_pose"):
            values = getattr(self, name)
            if len(values) != 6 or not all(math.isfinite(value) for value in values):
                raise ValueError(f"Pika {name} must contain six finite values")
