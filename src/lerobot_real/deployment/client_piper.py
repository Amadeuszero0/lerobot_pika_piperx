#!/usr/bin/env python

"""Run a remote policy on the integrated AgileX Piper follower."""

from __future__ import annotations

import argparse
import logging
from typing import Any

from .camera_utils import add_realsense_arguments, make_realsense_configs
from .client_common import (
    RobotDeploymentSpec,
    add_common_rollout_arguments,
    policy_client_from_args,
    rollout_config_from_args,
    run_robot_client,
)

logger = logging.getLogger(__name__)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the provided remote-policy protocol on an AgileX Piper arm"
    )
    add_common_rollout_arguments(parser)
    add_realsense_arguments(parser)

    parser.add_argument("--can-interface", required=True, help="SocketCAN interface, e.g. can0")
    parser.add_argument("--robot-id", default="piper_follower")
    parser.add_argument(
        "--control-space",
        required=True,
        choices=("joint", "cartesian"),
        help="Must match the state/action representation used during training",
    )
    parser.add_argument(
        "--configure-role-on-connect",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--park-on-connect", action="store_true")
    parser.add_argument("--park-on-disconnect", action="store_true")
    parser.add_argument(
        "--disconnect-mode",
        choices=("disable", "hold", "keep-enabled"),
        default="disable",
        help=(
            "disable motor torque, command the current Cartesian pose before disconnecting, "
            "or disconnect without a final motor command"
        ),
    )
    parser.add_argument("--feedback-timeout-s", type=float, default=0.5)
    parser.add_argument("--feedback-startup-timeout-s", type=float, default=5.0)

    parser.add_argument(
        "--max-relative-target",
        type=float,
        default=0.5,
        help="Joint-mode maximum normalized target change per command",
    )
    parser.add_argument(
        "--no-joint-step-limit",
        action="store_true",
        help="Disable the joint-mode per-command target limit",
    )
    parser.add_argument(
        "--cartesian-command-mode",
        choices=("step", "direct"),
        default="step",
    )
    parser.add_argument("--max-cartesian-step-mm", type=float, default=1.0)
    parser.add_argument("--max-rotation-step-rad", type=float, default=0.01)
    parser.add_argument("--max-cartesian-following-error-mm", type=float, default=100.0)
    parser.add_argument("--max-rotation-following-error-rad", type=float, default=0.5)
    parser.add_argument("--workspace-x", type=float, nargs=2, metavar=("MIN", "MAX"))
    parser.add_argument("--workspace-y", type=float, nargs=2, metavar=("MIN", "MAX"))
    parser.add_argument("--workspace-z", type=float, nargs=2, metavar=("MIN", "MAX"))
    parser.add_argument("--move-mode", choices=("move_p", "move_l"), default="move_p")
    parser.add_argument("--move-speed-percent", type=int, default=5)
    parser.add_argument("--gripper-effort", type=int, default=1000)
    parser.add_argument(
        "--startup-tcp-pose",
        type=float,
        nargs=6,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Optional pre-rollout TCP pose in mm and RPY degrees",
    )
    parser.add_argument("--startup-timeout-s", type=float, default=30.0)
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.action_type.endswith("joint") and args.control_space != "joint":
        parser.error("a *_joint action type requires --control-space=joint")
    if args.action_type.endswith("endpose") and args.control_space != "cartesian":
        parser.error("a *_endpose action type requires --control-space=cartesian")
    if args.control_space == "cartesian" and any(
        bounds is None for bounds in (args.workspace_x, args.workspace_y, args.workspace_z)
    ):
        parser.error("Cartesian control requires --workspace-x, --workspace-y, and --workspace-z")
    if args.control_space != "cartesian" and args.startup_tcp_pose is not None:
        parser.error("--startup-tcp-pose requires --control-space=cartesian")
    if args.disconnect_mode == "hold" and args.control_space != "cartesian":
        parser.error("--disconnect-mode=hold is only available in Cartesian control")
    if args.startup_timeout_s <= 0:
        parser.error("--startup-timeout-s must be positive")


def _make_robot(args: argparse.Namespace) -> Any:
    from lerobot_real.configs.piper import PiperFollowerConfig
    from lerobot_real.robots.piper.piper_follower import PiperFollower

    disable_torque = args.disconnect_mode == "disable"
    hold_position = args.disconnect_mode == "hold"
    config = PiperFollowerConfig(
        id=args.robot_id,
        port=args.can_interface,
        control_space=args.control_space,
        cameras=make_realsense_configs(args),
        configure_role_on_connect=args.configure_role_on_connect,
        park_on_connect=args.park_on_connect,
        park_on_disconnect=args.park_on_disconnect,
        disable_torque_on_disconnect=disable_torque,
        hold_position_on_disconnect=hold_position,
        feedback_timeout_s=args.feedback_timeout_s,
        feedback_startup_timeout_s=args.feedback_startup_timeout_s,
        max_relative_target=None if args.no_joint_step_limit else args.max_relative_target,
        cartesian_command_mode=args.cartesian_command_mode,
        max_cartesian_step_mm=args.max_cartesian_step_mm,
        max_rotation_step_rad=args.max_rotation_step_rad,
        max_cartesian_following_error_mm=args.max_cartesian_following_error_mm,
        max_rotation_following_error_rad=args.max_rotation_following_error_rad,
        workspace_x=None if args.workspace_x is None else tuple(args.workspace_x),
        workspace_y=None if args.workspace_y is None else tuple(args.workspace_y),
        workspace_z=None if args.workspace_z is None else tuple(args.workspace_z),
        move_mode=args.move_mode,
        move_speed_percent=args.move_speed_percent,
        gripper_effort=args.gripper_effort,
    )
    return PiperFollower(config)


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    _validate_args(parser, args)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    robot = _make_robot(args)
    action_keys = tuple(robot.action_features)
    spec = RobotDeploymentSpec(
        action_space=args.control_space,
        state_keys=action_keys,
        action_keys=action_keys,
        camera_names=tuple(robot.cameras),
        gripper_bounds=(0.0, 100.0) if args.control_space == "joint" else (0.0, 1.0),
    )
    rollout_config = rollout_config_from_args(args)
    policy = policy_client_from_args(args)

    prepare_episode = None
    if args.startup_tcp_pose is not None:
        startup_pose = tuple(args.startup_tcp_pose)

        def prepare_episode(robot_instance, _episode_index: int) -> None:
            robot_instance.move_to_tcp_pose(startup_pose, timeout_s=args.startup_timeout_s)

    logger.info(
        "Configured Piper %s control with state/action keys %s",
        args.control_space,
        action_keys,
    )
    run_robot_client(
        robot,
        policy,
        spec,
        rollout_config,
        prepare_episode=prepare_episode,
    )


if __name__ == "__main__":
    main()
