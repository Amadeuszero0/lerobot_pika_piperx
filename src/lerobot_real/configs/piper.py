"""Configuration types for the integrated AgileX Piper support.

Adapted from AgRoboticsResearch/lerobot_robot_piper under Apache-2.0.
"""

import math
from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig
from lerobot.teleoperators import TeleoperatorConfig


@RobotConfig.register_subclass("lerobot_real::piper")
@dataclass(kw_only=True)
class PiperFollowerConfig(RobotConfig):
    """One Piper follower arm.

    ``cartesian`` actions use millimetres for xyz and radians as an axis-angle
    vector for rx/ry/rz, matching the Pika teleoperator.
    """

    port: str
    control_space: str = "joint"
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    # Add the six physical joint angles (radians) to observations without
    # changing the configured command space or action schema.
    record_joint_angles: bool = False

    configure_role_on_connect: bool = True
    park_on_connect: bool = False
    park_on_disconnect: bool = False
    disable_torque_on_disconnect: bool = True
    hold_position_on_disconnect: bool = False
    feedback_timeout_s: float = 0.5
    feedback_startup_timeout_s: float = 5.0

    # Joint-mode actions remain normalized for dataset compatibility. The
    # hardware backend converts them to radians/metres before calling JointCtrl.
    max_relative_target: float | dict[str, float] | None = 0.5

    # Cartesian/Pika safety limits. Translation is mm, rotation is rad.
    cartesian_command_mode: str = "step"
    ik_urdf_path: str | None = None
    ik_package_dir: str | None = None
    max_cartesian_step_mm: float = 1.0
    max_rotation_step_rad: float = 0.01
    max_cartesian_following_error_mm: float = 100.0
    max_rotation_following_error_rad: float = 0.5
    workspace_x: tuple[float, float] | None = None
    workspace_y: tuple[float, float] | None = None
    workspace_z: tuple[float, float] | None = None
    move_mode: str = "move_p"
    move_speed_percent: int = 5
    gripper_effort: int = 1000

    def __post_init__(self) -> None:
        super().__post_init__()
        self.id = "piper_follower" if self.id is None else self.id
        if self.control_space not in ("joint", "cartesian"):
            raise ValueError(f"Unsupported Piper control_space: {self.control_space}")
        if self.cartesian_command_mode not in ("step", "direct", "official_ik"):
            raise ValueError(
                "cartesian_command_mode must be 'step', 'direct', or 'official_ik'"
            )
        if self.cartesian_command_mode == "official_ik" and (
            not self.ik_urdf_path or not self.ik_package_dir
        ):
            raise ValueError(
                "official_ik requires both ik_urdf_path and ik_package_dir"
            )
        if not isinstance(self.hold_position_on_disconnect, bool):
            raise ValueError("hold_position_on_disconnect must be boolean")
        if not isinstance(self.record_joint_angles, bool):
            raise ValueError("record_joint_angles must be boolean")
        if self.move_mode not in ("move_p", "move_l"):
            raise ValueError(f"Unsupported Piper move_mode: {self.move_mode}")
        if not 1 <= self.move_speed_percent <= 100:
            raise ValueError("move_speed_percent must be in [1, 100]")
        if not math.isfinite(self.max_cartesian_step_mm) or self.max_cartesian_step_mm <= 0:
            raise ValueError("max_cartesian_step_mm must be finite and positive")
        if not math.isfinite(self.max_rotation_step_rad) or self.max_rotation_step_rad <= 0:
            raise ValueError("max_rotation_step_rad must be finite and positive")
        if (
            not math.isfinite(self.max_cartesian_following_error_mm)
            or self.max_cartesian_following_error_mm <= 0
        ):
            raise ValueError("max_cartesian_following_error_mm must be finite and positive")
        if (
            not math.isfinite(self.max_rotation_following_error_rad)
            or self.max_rotation_following_error_rad <= 0
        ):
            raise ValueError("max_rotation_following_error_rad must be finite and positive")

        for name in ("feedback_timeout_s", "feedback_startup_timeout_s"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.max_relative_target is not None:
            limits = (
                self.max_relative_target.values()
                if isinstance(self.max_relative_target, dict)
                else (self.max_relative_target,)
            )
            if any(not math.isfinite(value) or value <= 0 for value in limits):
                raise ValueError("max_relative_target values must be finite and positive")
        if not 0 <= self.gripper_effort <= 5000:
            raise ValueError("gripper_effort must be in [0, 5000]")
        if self.control_space == "cartesian" and any(
            bounds is None for bounds in (self.workspace_x, self.workspace_y, self.workspace_z)
        ):
            raise ValueError("Cartesian Piper control requires workspace_x/y/z")
        for name in ("workspace_x", "workspace_y", "workspace_z"):
            bounds = getattr(self, name)
            if bounds is not None:
                if len(bounds) != 2 or not all(math.isfinite(value) for value in bounds):
                    raise ValueError(f"{name} must contain two finite values")
                if bounds[0] >= bounds[1]:
                    raise ValueError(f"{name} must be ordered as (min, max)")


@RobotConfig.register_subclass("lerobot_real::dual_piper")
@dataclass(kw_only=True)
class DualPiperFollowerConfig(RobotConfig):
    robots: dict[str, RobotConfig] = field(default_factory=dict)
    parallel_connect: bool = True
    parallel_observation: bool = True
    parallel_action: bool = True

    def __post_init__(self) -> None:
        super().__post_init__()
        self.id = "dual_piper_follower" if self.id is None else self.id
        if len(self.robots) != 2:
            raise ValueError("lerobot_real::dual_piper requires exactly two follower arms")


@TeleoperatorConfig.register_subclass("lerobot_real::piper_leader")
@dataclass(kw_only=True)
class PiperLeaderConfig(TeleoperatorConfig):
    port: str
    configure_role_on_connect: bool = True
    disable_torque_on_disconnect: bool = True
    feedback_timeout_s: float = 0.5
    feedback_startup_timeout_s: float = 5.0

    def __post_init__(self) -> None:
        self.id = "piper_leader" if self.id is None else self.id
        for name in ("feedback_timeout_s", "feedback_startup_timeout_s"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")


@TeleoperatorConfig.register_subclass("lerobot_real::dual_piper_leader")
@dataclass(kw_only=True)
class DualPiperLeaderConfig(TeleoperatorConfig):
    teleops: dict[str, TeleoperatorConfig] = field(default_factory=dict)
    parallel_connect: bool = True
    parallel_read: bool = True

    def __post_init__(self) -> None:
        self.id = "dual_piper_leader" if self.id is None else self.id
        if len(self.teleops) != 2:
            raise ValueError("lerobot_real::dual_piper_leader requires exactly two leader arms")


@TeleoperatorConfig.register_subclass("lerobot_real::dual_pika_teleop")
@dataclass(kw_only=True)
class DualPikaTeleopConfig(TeleoperatorConfig):
    teleops: dict[str, TeleoperatorConfig] = field(default_factory=dict)
    parallel_read: bool = True

    def __post_init__(self) -> None:
        self.id = "dual_pika" if self.id is None else self.id
        if len(self.teleops) != 2:
            raise ValueError("lerobot_real::dual_pika_teleop requires exactly two Pika devices")
        ports = [getattr(teleop, "port", None) for teleop in self.teleops.values()]
        if any(port is None for port in ports):
            raise ValueError("each dual Pika teleoperator must define a serial port")
        if len(set(ports)) != len(ports):
            raise ValueError("dual Pika teleoperators must use distinct serial ports")
        tracker_ids = [
            getattr(teleop, "tracker_device_id", None) for teleop in self.teleops.values()
        ]
        if not all(tracker_ids):
            raise ValueError("each dual Pika teleoperator must define tracker_device_id")
        if len(set(tracker_ids)) != len(tracker_ids):
            raise ValueError("dual Pikas must use distinct tracker_device_id values")
