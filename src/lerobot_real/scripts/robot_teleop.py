import sys
import argparse
import logging
import time
from pathlib import Path
from dataclasses import asdict, dataclass
from pprint import pformat
import lerobot_real  # register plugin configs
from lerobot.scripts.lerobot_record import register_third_party_plugins
from lerobot.processor import (
    make_default_processors,
)
from lerobot.robots import RobotConfig
from lerobot.teleoperators import TeleoperatorConfig
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import (
    init_logging,
)
from lerobot_real.configs import parser
from lerobot_real.robots.utils import make_robot_from_config
from lerobot_real.scripts._device_cleanup import disconnect_devices
from lerobot_real.scripts._teleop_setup import move_robot_to_teleop_base
from lerobot_real.teleoperators.utils import make_teleoperator_from_config
from lerobot_real.utils.utils import is_headless, init_keyboard_listener
from lerobot_real.teleoperators.base_teleop import BaseTeleop


@dataclass
class TeleopConfig:
    robot: RobotConfig
    teleop: TeleoperatorConfig
    fps: int = 30

    def __post_init__(self):
        if hasattr(self.robot, 'robots'):
            for _, robot in self.robot.robots.items():
                robot.cameras = {}
        else:
            self.robot.cameras = {}


def _activate_real_teleop(robot, teleop, *, reset_to_base: bool) -> None:
    if reset_to_base:
        move_robot_to_teleop_base(robot, teleop)

    obs = robot.get_observation()
    teleop.set_teleop_enabled(True, obs)


def _teleop_loop_impl(cfg: TeleopConfig, cleanup_devices: list):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    teleop = make_teleoperator_from_config(cfg.teleop)
    cleanup_devices.append(teleop)
    if hasattr(cfg.robot, "teleop"):
        cfg.robot.teleop = teleop
    robot = make_robot_from_config(cfg.robot)
    cleanup_devices.insert(0, robot)

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    robot.connect()
    teleop.connect()

    sleep_time_s = 1 / cfg.fps

    is_evt = not is_headless()
    is_real_teleop = isinstance(teleop, BaseTeleop)

    is_reset = False
    is_paused = True
    has_started = False
    events = {"exit": False}
    listener = None
    key_dict = {}

    if is_evt:
        from pynput import keyboard

        key_dict = {
            keyboard.Key.esc: 0,    # exit
            keyboard.Key.left: 0,   # reset and pause
            keyboard.Key.space: 0,  # start/pause
            keyboard.Key.enter: 0,  # help
        }

        def on_press(key):
            if key_dict.get(key, 1) == 0:
                try:
                    if key == keyboard.Key.esc:
                        events["exit"] = True
                        print("\nEscape key pressed. Stopping ...")
                except Exception as e:
                    print(f"Error handling key press: {e}")
            if key in key_dict:
                key_dict[key] = True

        def on_release(key):
            try:
                if key == keyboard.Key.enter:
                    if is_paused:
                        if is_reset:
                            print('⌨   [ESC] Exit  [Space] Reset / Start  [←] Reset')
                        else:
                            print('⌨   [ESC] Exit  [Space] Start  [←] Reset')
                    else:
                        print('⌨   [ESC] Exit  [Space] Pause  [←] Pause / Reset')
            except Exception as e:
                print(f"Error handling key release: {e}")
            if key in key_dict:
                key_dict[key] = False

        listener, events = init_keyboard_listener(events=events, on_press=on_press, on_release=on_release)
        print("\n********** Teleop Control Loop Start **********")
        print('⌨   [ESC] Exit  [Space] Start  [←] Reset')
    else:
        input('⌨   Press Enter to start teleop >>> ')
        if is_real_teleop:
            _activate_real_teleop(robot, teleop, reset_to_base=True)
            has_started = True
        is_paused = False
        is_reset = False
        print("\n********** Teleop Control Loop Start **********")

    key_space_pressed = False
    key_left_pressed = False

    while not events["exit"]:
        start_loop_t = time.perf_counter()

        if is_evt:
            if key_dict[keyboard.Key.left] and not key_left_pressed:
                key_left_pressed = True
                is_reset = True
                if not is_paused:
                    is_paused = True
                    if is_real_teleop:
                        teleop.set_teleop_enabled(False)
                print('⌨   [ESC] Exit  [Space] Reset / Start  [←] Reset')
            elif not key_dict[keyboard.Key.left] and key_left_pressed:
                key_left_pressed = False

            if key_dict[keyboard.Key.space] and not key_space_pressed:
                key_space_pressed = True
                is_paused = not is_paused
                if is_paused:
                    if is_real_teleop:
                        teleop.set_teleop_enabled(False)
                    # print('========== Teleop is paused ==========')
                    print('⌨   [ESC] Exit  [Space] Start  [←] Reset')
                else:
                    reset_to_base = is_reset or not has_started
                    if is_reset:
                        is_reset = False
                    if is_real_teleop:
                        _activate_real_teleop(
                            robot, teleop, reset_to_base=reset_to_base
                        )
                        has_started = True
                    elif reset_to_base:
                        robot.configure()
                    # print('========== Teleop is start ==========')
                    print('⌨   [ESC] Exit  [Space] Pause  [←] Reset')
                continue
            elif not key_dict[keyboard.Key.space] and key_space_pressed:
                key_space_pressed = False

        if not is_reset and is_real_teleop and teleop.has_pending_robot_sync():
            obs = robot.get_observation()
            if teleop.apply_pending_robot_sync(obs):
                is_paused = False
                if is_evt:
                    print('⌨   [ESC] Exit  [Space] Pause  [←] Reset')

        if is_evt and (is_reset or is_paused):
            continue

        # Get robot observation
        obs = robot.get_observation()

        act = teleop.get_action()
        if act is None:
            dt_s = time.perf_counter() - start_loop_t
            precise_sleep(max(sleep_time_s - dt_s, 0.0))
            continue

        act_processed_teleop = teleop_action_processor((act, obs))

        robot_action_to_send = robot_action_processor((act_processed_teleop, obs))
        robot.send_action(robot_action_to_send)

        dt_s = time.perf_counter() - start_loop_t
        precise_sleep(sleep_time_s - dt_s)
    
    print("\n********** Teleop Control Loop Exit **********")
    if is_evt and listener is not None:
        listener.stop()


def teleop_loop(cfg: TeleopConfig):
    cleanup_devices: list = []
    try:
        result = _teleop_loop_impl(cfg, cleanup_devices)
    except BaseException:
        disconnect_devices(cleanup_devices, suppress_errors=True)
        raise
    disconnect_devices(cleanup_devices, suppress_errors=False)
    return result

@parser.wrap()
def get_cfg(cfg: TeleopConfig) -> TeleopConfig:
    return cfg

def main():
    parser = argparse.ArgumentParser(description='configuration args')
    args, unknown = parser.parse_known_args()
    sys.argv = [sys.argv[0]] + unknown
    register_third_party_plugins()
    cfg = get_cfg()
    teleop_loop(cfg)


if __name__ == "__main__":
    main()
