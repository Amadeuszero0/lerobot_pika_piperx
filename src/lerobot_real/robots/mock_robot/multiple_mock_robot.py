#!/usr/bin/env python

from lerobot.processor.core import RobotAction, RobotObservation
from lerobot.robots import Robot

from .mock_robot import MockRobot
from .multiple_mock_robot_config import MultipleMockRobotConfig


class MultipleMockRobot(Robot):
    config_class = MultipleMockRobotConfig
    name = "Lerobot-Real Multiple Mock Robots"

    def __init__(self, config: MultipleMockRobotConfig):
        super().__init__(config)
        self.config = config
        self.robots: dict[str, MockRobot] = {}
        for key, robot_config in self.config.robots.items():
            self.robots[key] = MockRobot(robot_config, prefix=key)

        self.cameras = {
            f"{side}.{name}": camera
            for side, robot in self.robots.items()
            for name, camera in robot.cameras.items()
        }

    @property
    def observation_features(self) -> dict:
        observation_features = {}
        for robot in self.robots.values():
            observation_features.update(robot.observation_features)
        return observation_features

    @property
    def action_features(self) -> dict:
        action_features = {}
        for robot in self.robots.values():
            action_features.update(robot.action_features)
        return action_features

    @property
    def is_connected(self) -> bool:
        return all(robot.is_connected for robot in self.robots.values())

    @property
    def is_calibrated(self) -> bool:
        return all(robot.is_calibrated for robot in self.robots.values())

    def connect(self, calibrate: bool = True) -> None:
        connected: list[MockRobot] = []
        try:
            for robot in self.robots.values():
                robot.connect(calibrate=calibrate)
                connected.append(robot)
        except BaseException:
            for robot in reversed(connected):
                try:
                    robot.disconnect()
                except Exception:
                    pass
            raise

    def calibrate(self) -> None:
        for robot in self.robots.values():
            robot.calibrate()

    def configure(self) -> None:
        for robot in self.robots.values():
            robot.configure()

    def disconnect(self) -> None:
        first_error: Exception | None = None
        for robot in self.robots.values():
            try:
                robot.disconnect()
            except Exception as exc:
                first_error = first_error or exc
        if first_error is not None:
            raise first_error

    def get_observation(self) -> RobotObservation:
        observations = [robot.get_observation() for robot in self.robots.values()]
        combined_observation = RobotObservation()
        for obs in observations:
            combined_observation.update(obs)
        return combined_observation

    def send_action(self, action: RobotAction) -> RobotAction:
        sent_action = RobotAction()
        for key, robot in self.robots.items():
            action_subset = {k: v for k, v in action.items() if k.startswith(f"{key}.")}
            sent_action.update(robot.send_action(action_subset))
        return sent_action
