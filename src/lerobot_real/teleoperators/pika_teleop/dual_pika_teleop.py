"""Dual Pika teleoperator used by the Piper Cartesian-control workflow."""

from concurrent.futures import ThreadPoolExecutor
from typing import Any

from lerobot_real.configs.piper import DualPikaTeleopConfig
from lerobot_real.teleoperators.base_teleop import BaseTeleop
from lerobot_real.teleoperators.pika_teleop.pika_teleop import PikaTeleop
from lerobot_real.teleoperators.pika_teleop.pika_teleop_config import PikaTeleopConfig


class DualPikaTeleop(BaseTeleop):
    config_class = DualPikaTeleopConfig
    name = "dual_pika_teleop"

    def __init__(self, config: DualPikaTeleopConfig) -> None:
        super().__init__(config)
        self.config = config
        self.teleops: dict[str, PikaTeleop] = {}
        try:
            for side, teleop_config in config.teleops.items():
                if not isinstance(teleop_config, PikaTeleopConfig):
                    raise TypeError(
                        f"{side} must use type lerobot_real::pika_teleop, got {teleop_config.type}"
                    )
                self.teleops[side] = PikaTeleop(teleop_config, prefix=side)
        except BaseException:
            for teleop in self.teleops.values():
                try:
                    teleop.disconnect()
                except Exception:
                    pass
            raise
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="dual-pika")

    @property
    def action_features(self) -> dict[str, type]:
        result: dict[str, type] = {}
        for side, teleop in self.teleops.items():
            names = teleop.action_features["names"]
            result.update({f"{side}.{name}": float for name in names})
        return result

    @property
    def feedback_features(self) -> dict[str, type]:
        return self.action_features

    @property
    def is_connected(self) -> bool:
        return all(teleop.is_connected for teleop in self.teleops.values())

    @property
    def is_calibrated(self) -> bool:
        return all(teleop.is_calibrated for teleop in self.teleops.values())

    def connect(self, calibrate: bool = True) -> None:
        super().connect(calibrate)
        try:
            for teleop in self.teleops.values():
                teleop.connect(calibrate=calibrate)
        except BaseException:
            for teleop in self.teleops.values():
                try:
                    teleop.disconnect()
                except Exception:
                    pass
            super().disconnect()
            raise

    def calibrate(self) -> None:
        for teleop in self.teleops.values():
            teleop.calibrate()

    def configure(self) -> None:
        for teleop in self.teleops.values():
            teleop.configure()

    def set_teleop_enabled(self, enabled: bool, obs: dict | None = None) -> None:
        for teleop in self.teleops.values():
            teleop.set_teleop_enabled(enabled, obs)

    def has_pending_robot_sync(self) -> bool:
        states = [
            (teleop.has_pending_robot_sync(), teleop.is_teleop_enabled)
            for teleop in self.teleops.values()
        ]
        return any(pending for pending, _ in states) and all(
            pending or enabled for pending, enabled in states
        )

    def apply_pending_robot_sync(self, obs: dict) -> bool:
        if not self.has_pending_robot_sync():
            return False
        applied = [
            teleop.apply_pending_robot_sync(obs)
            for teleop in self.teleops.values()
            if teleop.has_pending_robot_sync()
        ]
        return any(applied) and all(
            teleop.is_teleop_enabled for teleop in self.teleops.values()
        )

    def get_action(self) -> dict[str, Any] | None:
        if self.config.parallel_read:
            futures = [self._executor.submit(teleop.get_action) for teleop in self.teleops.values()]
            actions = [future.result() for future in futures]
        else:
            actions = [teleop.get_action() for teleop in self.teleops.values()]
        merged: dict[str, Any] = {}
        for action in actions:
            if action is None:
                return None
            merged.update(action)
        return merged

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    def disconnect(self) -> None:
        first_error: Exception | None = None
        try:
            for teleop in self.teleops.values():
                try:
                    teleop.disconnect()
                except Exception as exc:
                    first_error = first_error or exc
        finally:
            try:
                self._executor.shutdown(wait=True)
            except Exception as exc:
                first_error = first_error or exc
            try:
                super().disconnect()
            except Exception as exc:
                first_error = first_error or exc
        if first_error is not None:
            raise first_error
