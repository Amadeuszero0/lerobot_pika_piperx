from dataclasses import dataclass, field
from lerobot.robots import RobotConfig
from .xarm_robot_config import XArmRobotConfig

@RobotConfig.register_subclass("lerobot_real::multiple_xarm")
@dataclass
class MultipleXArmRobotConfig(RobotConfig):
    robots: dict[str, RobotConfig] = field(
        default_factory=lambda: {}
    )
    async_connect: bool = True
    async_configure: bool = True
    async_action: bool = False
    cameras_args: dict = None

    def __post_init__(self):
        super().__post_init__()
        self.id = 'multiple_xarm_robot' if self.id is None else self.id
