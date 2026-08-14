from dataclasses import dataclass, field
from lerobot.robots import RobotConfig
from .mock_robot_config import MockRobotConfig

@RobotConfig.register_subclass("lerobot_real::multiple_mock_robot")
@dataclass
class MultipleMockRobotConfig(RobotConfig):
    robots: dict[str, RobotConfig] = field(
        default_factory=lambda: {}
    )

    def __post_init__(self):
        super().__post_init__()
        self.id = 'multiple_mock_robot' if self.id is None else self.id
