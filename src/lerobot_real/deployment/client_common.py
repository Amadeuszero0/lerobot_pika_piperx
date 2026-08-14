"""Shared fail-closed rollout loop for remote real-robot clients."""

from __future__ import annotations

import argparse
import logging
import os
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

from .action_utils import (
    ActionSpace,
    DeltaReference,
    PolicyObservationBuilder,
    extract_action_chunk,
    extract_state,
    make_robot_action,
    normalize_action_type,
    process_action_chunk,
)
from .tools.websocket_policy_client import WebsocketClientPolicy

if TYPE_CHECKING:
    from lerobot.robots import Robot


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class RobotDeploymentSpec:
    action_space: ActionSpace
    state_keys: tuple[str, ...]
    action_keys: tuple[str, ...]
    camera_names: tuple[str, ...]
    gripper_bounds: tuple[float, float]

    def __post_init__(self) -> None:
        if len(self.state_keys) != 7 or len(self.action_keys) != 7:
            raise ValueError("Deployment requires six arm values plus one gripper value")
        if self.gripper_bounds[0] >= self.gripper_bounds[1]:
            raise ValueError("gripper_bounds must be ordered as (min, max)")


@dataclass(frozen=True)
class RolloutConfig:
    task: str
    control_fps: float = 10.0
    execution_horizon: int = 8
    action_type: str = "absolute"
    delta_reference: DeltaReference = "previous"
    gripper_is_delta: bool = False
    action_key: str | None = None
    episode_time_s: float = 60.0
    num_episodes: int = 1
    state_key: str = "state"
    task_key: str = "annotation.human.task_description"
    camera_prefix: str = "video"
    start_immediately: bool = False
    log_actions: bool = False

    def __post_init__(self) -> None:
        if not self.task:
            raise ValueError("task must not be empty")
        if self.control_fps <= 0:
            raise ValueError("control_fps must be positive")
        if self.execution_horizon <= 0:
            raise ValueError("execution_horizon must be positive")
        if self.episode_time_s <= 0 or self.num_episodes <= 0:
            raise ValueError("episode_time_s and num_episodes must be positive")
        normalize_action_type(self.action_type)


def add_common_rollout_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--host", default="127.0.0.1", help="Policy server host")
    parser.add_argument("--port", type=int, default=5555, help="Policy server port")
    parser.add_argument(
        "--api-key",
        default=os.environ.get("LEROBOT_REAL_POLICY_API_KEY"),
        help="Policy server API key; defaults to LEROBOT_REAL_POLICY_API_KEY",
    )
    parser.add_argument("--connect-timeout-s", type=float, default=300.0)
    parser.add_argument("--request-timeout-s", type=float, default=120.0)
    parser.add_argument("--task", required=True, help="Task text used during policy training")
    parser.add_argument("--control-fps", type=float, default=10.0)
    parser.add_argument(
        "--execution-horizon",
        type=int,
        default=8,
        help="Number of predicted steps sent before requesting a new chunk",
    )
    parser.add_argument(
        "--action-type",
        required=True,
        choices=(
            "absolute",
            "delta",
            "absolute_joint",
            "absolute_endpose",
            "delta_joint",
            "delta_endpose",
        ),
        help="Must match the action representation used during policy training",
    )
    parser.add_argument(
        "--delta-reference",
        choices=("previous", "observation"),
        default="previous",
        help="Accumulate each delta from the previous target or the latest observation",
    )
    parser.add_argument(
        "--gripper-is-delta",
        action="store_true",
        help="Treat the last predicted value as a gripper delta; default is absolute",
    )
    parser.add_argument(
        "--action-key",
        help="Optional response key containing the complete [T, 7] action chunk",
    )
    parser.add_argument("--state-key", default="state", help="Policy request state key")
    parser.add_argument(
        "--task-key",
        default="annotation.human.task_description",
        help="Policy request task key",
    )
    parser.add_argument(
        "--camera-prefix",
        default="video",
        help="Prefix used to form camera request keys, for example video.wrist",
    )
    parser.add_argument("--episode-time-s", type=float, default=60.0)
    parser.add_argument("--num-episodes", type=int, default=1)
    parser.add_argument(
        "--start-immediately",
        action="store_true",
        help="Skip the operator confirmation before connecting and moving the robot",
    )
    parser.add_argument("--log-actions", action="store_true")


def rollout_config_from_args(args: argparse.Namespace) -> RolloutConfig:
    return RolloutConfig(
        task=args.task,
        control_fps=args.control_fps,
        execution_horizon=args.execution_horizon,
        action_type=args.action_type,
        delta_reference=args.delta_reference,
        gripper_is_delta=args.gripper_is_delta,
        action_key=args.action_key,
        episode_time_s=args.episode_time_s,
        num_episodes=args.num_episodes,
        state_key=args.state_key,
        task_key=args.task_key,
        camera_prefix=args.camera_prefix,
        start_immediately=args.start_immediately,
        log_actions=args.log_actions,
    )


def policy_client_from_args(args: argparse.Namespace) -> WebsocketClientPolicy:
    return WebsocketClientPolicy(
        host=args.host,
        port=args.port,
        api_key=args.api_key,
        connect_timeout_s=args.connect_timeout_s,
        request_timeout_s=args.request_timeout_s,
    )


def run_episode(
    robot: Robot,
    policy: WebsocketClientPolicy,
    spec: RobotDeploymentSpec,
    config: RolloutConfig,
) -> int:
    observation_builder = PolicyObservationBuilder(
        camera_names=spec.camera_names,
        state_keys=spec.state_keys,
        state_key=config.state_key,
        task_key=config.task_key,
        camera_prefix=config.camera_prefix,
    )
    deadline = time.monotonic() + config.episode_time_s
    control_period_s = 1.0 / config.control_fps
    executed_steps = 0

    while time.monotonic() < deadline:
        observation = robot.get_observation()
        state = extract_state(observation, spec.state_keys)
        policy_observation = observation_builder.build(observation, config.task)

        inference_start = time.perf_counter()
        policy_action, _ = policy.get_action(policy_observation)
        inference_ms = (time.perf_counter() - inference_start) * 1000.0
        action_chunk = extract_action_chunk(
            policy_action,
            action_space=spec.action_space,
            action_keys=spec.action_keys,
            action_key=config.action_key,
        )
        target_chunk = process_action_chunk(
            state,
            action_chunk,
            action_type=config.action_type,
            delta_reference=config.delta_reference,
            gripper_is_delta=config.gripper_is_delta,
        )
        horizon = min(config.execution_horizon, target_chunk.shape[0])
        logger.info(
            "Policy inference %.1f ms; executing %d/%d predicted steps",
            inference_ms,
            horizon,
            target_chunk.shape[0],
        )

        for values in target_chunk[:horizon]:
            if time.monotonic() >= deadline:
                return executed_steps
            loop_start = time.perf_counter()
            action = make_robot_action(
                values,
                spec.action_keys,
                gripper_bounds=spec.gripper_bounds,
            )
            sent_action = robot.send_action(action)
            executed_steps += 1
            if config.log_actions:
                logger.info("Sent action: %s", sent_action)
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(control_period_s - elapsed, 0.0))

    return executed_steps


def run_robot_client(
    robot: Robot,
    policy: WebsocketClientPolicy,
    spec: RobotDeploymentSpec,
    config: RolloutConfig,
    *,
    prepare_episode: Callable[[Robot, int], None] | None = None,
) -> None:
    """Connect, execute episodes, and close the policy and hardware on every exit path."""
    primary_error: BaseException | None = None
    try:
        if not policy.ping():
            raise ConnectionError("Policy server ping failed")
        metadata = policy.get_server_metadata()
        action_dim = metadata.get("action_dim")
        if action_dim is not None and int(action_dim) != len(spec.action_keys):
            raise ValueError(
                f"Policy action dimension {action_dim} does not match robot dimension "
                f"{len(spec.action_keys)}"
            )
        logger.info("Policy server metadata: %s", metadata)

        if not config.start_immediately:
            input(
                "Check the workspace, load support, emergency stop, and policy action "
                "semantics; press Enter to connect the robot: "
            )
        robot.connect()

        for episode_index in range(config.num_episodes):
            if episode_index > 0 and not config.start_immediately:
                input(f"Prepare episode {episode_index}; press Enter to continue: ")
            if prepare_episode is not None:
                prepare_episode(robot, episode_index)
            policy.reset()
            logger.info("Starting rollout episode %d", episode_index)
            executed = run_episode(robot, policy, spec, config)
            logger.info("Finished episode %d after %d control steps", episode_index, executed)
    except BaseException as exc:
        primary_error = exc
        raise
    finally:
        try:
            policy.close()
        except Exception:
            logger.exception("Failed to close policy connection")
        try:
            if robot.is_connected:
                robot.disconnect()
        except Exception:
            if primary_error is None:
                raise
            logger.exception("Failed to disconnect robot while handling another error")
