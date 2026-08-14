import numpy as np
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots import Robot
from lerobot_real.context import get_active_teleop
from .mock_robot_config import MockRobotConfig


CARTESIAN_OBS_KEYS = [
    "pose.x", "pose.y", "pose.z", "pose.rx", "pose.ry", "pose.rz",
    # un-comment if you need more features below:
    # "velo.x", "velo.y", "velo.z", "velo.rx", "velo.ry", "velo.rz",
]

CARTESIAN_ACTION_KEYS = [
    "pose.x", "pose.y", "pose.z", "pose.rx", "pose.ry", "pose.rz",
]


class MockRobot(Robot):

    config_class = MockRobotConfig
    name = "Lerobot-Real Mock Robot"

    def __init__(self, config: MockRobotConfig, prefix=''):
        super().__init__(config)
        self.prefix = '' if not prefix else f"{prefix}."
        self.config = config
        self._dof = config.robot_dof
        self._control_space = self.config.control_space
        if self._control_space == "joint" and (self._dof == None or (not self._dof in (5, 6, 7))):
            raise ValueError(f"Please specify the correct DOF mock robot!, got {self._dof}")
        self._jnt_obs_has_vel = config.observe_joint_vel if self._control_space == "joint" else False
        self._is_connected = False
        self._is_calibrated =True
        self._cache_num = self.config.state_offset_action
        self._teleop_actions = []

        self.cameras = make_cameras_from_configs(config.cameras)

        self._gripper_type = self.config.gripper_type

    @property
    def _robot_state_features(self)-> dict:
        if self._control_space == "joint":
            state_features = {f"{self.prefix}J{motor}.pos": float for motor in range(1, self._dof+1)}
            if self._jnt_obs_has_vel:
                state_features.update({f"{self.prefix}J{motor}.vel": float for motor in range(1, self._dof+1)})
            if self._gripper_type > 0:
                state_features.update({f"{self.prefix}gripper.pos": float})
        elif self._control_space == "cartesian":
            state_features = {f"{self.prefix}{key}": float for key in CARTESIAN_OBS_KEYS}
            if self._gripper_type > 0:
                state_features.update({f"{self.prefix}gripper.pos": float})
        else:
            raise ValueError(f"Please check the given control space of mock robot! got {self._control_space}")
        return state_features
    
    @property
    # CHECK!! channel first or last?
    def _cam_features(self) -> dict:
        cam_ft = {}
        for cam_key, cam in self.cameras.items():
            cam_ft[f"{self.prefix}{cam_key}"] = (cam.height, cam.width, 3)
        return cam_ft

    @property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._robot_state_features, **self._cam_features}

    @property
    def action_features(self)-> dict:
        if self._control_space == "joint":
            action_ft = {f"{self.prefix}J{motor}.pos": float for motor in range(1, self._dof+1)}
        elif self._control_space == "cartesian":
            action_ft = {f"{self.prefix}{key}": float for key in CARTESIAN_ACTION_KEYS}
        else:
            raise ValueError(f"Please check the given control space of mock robot! got {self._control_space}")
        # Consider adding velocity configuration ??
        if self._gripper_type > 0:
            action_ft.update({f"{self.prefix}gripper.pos": float})
        return action_ft
    
    def connect(self, calibrate: bool = True) -> None:
        try:
            for cam in self.cameras.values():
                cam.connect()
            self.configure()
            if calibrate:
                self.calibrate()
        except BaseException:
            for cam in self.cameras.values():
                try:
                    if cam.is_connected:
                        cam.disconnect()
                except Exception:
                    pass
            self._is_connected = False
            raise
        self._is_connected = all(cam.is_connected for cam in self.cameras.values())
        if not self._is_connected:
            self.disconnect()
            raise ConnectionError(f"One or more cameras failed to connect for {self.config.id}")

    def configure(self) -> None:
        pass

    def calibrate(self) -> None:
        self._is_calibrated = True
        pass # CHECK! currently No-op

    def get_observation(self) -> dict[str, np.ndarray]:
        teleop = get_active_teleop(self.config.teleop_id)
        new_act = teleop.get_action() if teleop else {}
        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            new_act[f"{self.prefix}{cam_key}"] = cam.async_read()
        
        # if len(self._teleop_actions) >= self._cache_num:
        #     act = self._teleop_actions.pop(0)
        # else:
        #     act = new_act
        # self._teleop_actions.append(new_act)

        return new_act

    def send_action(self, action: dict) -> np.ndarray:
        return action

    def disconnect(self) -> None:
        first_error = None
        for cam in self.cameras.values():
            try:
                if cam.is_connected:
                    cam.disconnect()
            except Exception as exc:
                first_error = first_error or exc

        self._is_connected = False
        if first_error is not None:
            raise first_error
    
    @property
    def is_calibrated(self) -> bool:
        """Whether the robot is currently calibrated or not. Should be always `True` if not applicable"""
        return self._is_calibrated

    @property
    def is_connected(self) -> bool:
        """Whether the robot is currently calibrated or not. Should be always `True` if not applicable"""
        return self._is_connected

