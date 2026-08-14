from lerobot.teleoperators import Teleoperator

from lerobot_real.context import register_teleop, unregister_teleop


class BaseTeleop(Teleoperator):
    config_class = None
    name = "Lerobot-Real Base Teleoperator"

    def __init__(self, config):
        super().__init__(config)
        self.config = config

    def connect(self, calibrate: bool = False) -> None:
        register_teleop(self)

    def disconnect(self):
        unregister_teleop(self)

    def set_teleop_enabled(self, enabled: bool, obs=None):
        """
        启用/停用遥操作
        当enabled为True且obs不为None时, 顺便设置机械臂初始位置映射
        """
        pass

    def has_pending_robot_sync(self) -> bool:
        """Return whether a device transition needs live robot feedback."""
        return False

    def apply_pending_robot_sync(self, obs) -> bool:
        """Apply pending device state; return whether it enabled teleoperation."""
        return False
