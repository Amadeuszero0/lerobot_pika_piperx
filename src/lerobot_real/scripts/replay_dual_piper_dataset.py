"""Safely replay one dual-Piper episode from a LeRobot v3 dataset.

The replay source is the 14-D ``action`` feature produced by this project:
six joint targets in radians plus one normalized gripper target for each arm.
No camera, Pika, Vive calibration, or inverse kinematics is used during replay.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from lerobot_real.robots.piper.piper_follower import DUAL_JOINT_STATE_NAMES
from lerobot_real.utils.dataset_hardware import read_gripper_widths


SIDES = ("left", "right")
JOINT_LIMITS_DEG = (
    (-150.0, 150.0),
    (0.0, 180.0),
    (-170.0, 0.0),
    (-100.0, 100.0),
    (-70.0, 70.0),
    (-120.0, 120.0),
)


@dataclass(frozen=True)
class EpisodeReplay:
    root: Path
    episode_index: int
    fps: float
    frame_indices: np.ndarray
    timestamps_s: np.ndarray
    actions: np.ndarray
    gripper_max_width_m_by_side: dict[str, float]
    legacy_gripper_metadata: bool

    @property
    def duration_s(self) -> float:
        if len(self.timestamps_s) < 2:
            return 0.0
        return float(self.timestamps_s[-1] - self.timestamps_s[0])


@dataclass(frozen=True)
class ReplaySafetyReport:
    max_joint_step_rad: float
    max_joint_step_frame: int
    max_gripper_step: float
    max_gripper_step_frame: int


def _read_feature_names(info: dict[str, Any], feature_name: str) -> list[str]:
    try:
        feature = info["features"][feature_name]
        names = feature["names"]
        shape = tuple(feature["shape"])
    except (KeyError, TypeError) as exc:
        raise ValueError(
            f"meta/info.json does not describe required feature {feature_name!r}"
        ) from exc
    if shape != (14,) or not isinstance(names, list) or len(names) != 14:
        raise ValueError(
            f"{feature_name} must have shape [14] and 14 names, got shape={shape}, "
            f"names={names!r}"
        )
    return [str(name) for name in names]


def load_episode(dataset_root: Path, episode_index: int) -> EpisodeReplay:
    """Load and order one logical episode without decoding its videos."""
    if episode_index < 0:
        raise ValueError("episode index must be non-negative")
    root = dataset_root.expanduser().resolve()
    info_path = root / "meta" / "info.json"
    if not info_path.is_file():
        raise FileNotFoundError(f"dataset metadata not found: {info_path}")

    info = json.loads(info_path.read_text(encoding="utf-8"))
    gripper_widths, legacy_gripper_metadata = read_gripper_widths(root)
    recorded_names = _read_feature_names(info, "action")
    expected_names = list(DUAL_JOINT_STATE_NAMES)
    if set(recorded_names) != set(expected_names):
        missing = sorted(set(expected_names) - set(recorded_names))
        unexpected = sorted(set(recorded_names) - set(expected_names))
        raise ValueError(
            "dataset action is not the joint-action schema required for replay; "
            f"missing={missing}, unexpected={unexpected}"
        )
    reorder = [recorded_names.index(name) for name in expected_names]

    try:
        fps = float(info["fps"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("meta/info.json must contain a finite positive fps") from exc
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("meta/info.json must contain a finite positive fps")

    parquet_files = sorted((root / "data").glob("chunk-*/file-*.parquet"))
    if not parquet_files:
        raise FileNotFoundError(f"no data parquet files found below {root / 'data'}")

    try:
        import pyarrow.compute as pc
        import pyarrow.parquet as pq
    except ImportError as exc:
        raise RuntimeError("pyarrow is required to read a LeRobot dataset") from exc

    actions: list[list[float]] = []
    frame_indices: list[int] = []
    timestamps: list[float] = []
    required_columns = ["action", "episode_index", "frame_index", "timestamp"]
    for parquet_path in parquet_files:
        table = pq.read_table(parquet_path, columns=required_columns)
        selected = table.filter(pc.equal(table["episode_index"], episode_index))
        if selected.num_rows == 0:
            continue
        actions.extend(selected["action"].to_pylist())
        frame_indices.extend(int(value) for value in selected["frame_index"].to_pylist())
        timestamps.extend(float(value) for value in selected["timestamp"].to_pylist())

    if not actions:
        raise ValueError(f"episode {episode_index} was not found in {root}")

    action_array = np.asarray(actions, dtype=np.float64)
    frame_array = np.asarray(frame_indices, dtype=np.int64)
    timestamp_array = np.asarray(timestamps, dtype=np.float64)
    if action_array.ndim != 2 or action_array.shape[1] != 14:
        raise ValueError(f"episode action data has invalid shape {action_array.shape}")

    order = np.argsort(frame_array, kind="stable")
    action_array = action_array[order][:, reorder]
    frame_array = frame_array[order]
    timestamp_array = timestamp_array[order]
    expected_frames = np.arange(len(frame_array), dtype=np.int64)
    if not np.array_equal(frame_array, expected_frames):
        raise ValueError(
            "episode frame_index must be unique and contiguous from zero; "
            f"got first={frame_array[0]}, last={frame_array[-1]}, count={len(frame_array)}"
        )
    if not np.isfinite(action_array).all() or not np.isfinite(timestamp_array).all():
        raise ValueError("episode contains NaN or infinity")
    if len(timestamp_array) > 1 and np.any(np.diff(timestamp_array) <= 0):
        raise ValueError("episode timestamps must be strictly increasing")

    return EpisodeReplay(
        root=root,
        episode_index=episode_index,
        fps=fps,
        frame_indices=frame_array,
        timestamps_s=timestamp_array,
        actions=action_array,
        gripper_max_width_m_by_side=gripper_widths,
        legacy_gripper_metadata=legacy_gripper_metadata,
    )


def validate_episode(
    episode: EpisodeReplay,
    *,
    max_frame_joint_step_rad: float,
    max_frame_gripper_step: float,
) -> ReplaySafetyReport:
    """Reject malformed, out-of-range, or discontinuous commands before CAN access."""
    if max_frame_joint_step_rad <= 0 or not math.isfinite(max_frame_joint_step_rad):
        raise ValueError("max frame joint step must be finite and positive")
    if max_frame_gripper_step <= 0 or not math.isfinite(max_frame_gripper_step):
        raise ValueError("max frame gripper step must be finite and positive")

    for side_index, side in enumerate(SIDES):
        offset = side_index * 7
        for joint_index, (minimum_deg, maximum_deg) in enumerate(JOINT_LIMITS_DEG):
            values = episode.actions[:, offset + joint_index]
            minimum_rad = math.radians(minimum_deg)
            maximum_rad = math.radians(maximum_deg)
            if np.any(values < minimum_rad) or np.any(values > maximum_rad):
                bad = int(np.flatnonzero((values < minimum_rad) | (values > maximum_rad))[0])
                raise ValueError(
                    f"frame {bad} {side}.joint{joint_index + 1}="
                    f"{math.degrees(values[bad]):.3f} deg is outside "
                    f"[{minimum_deg:.3f}, {maximum_deg:.3f}]"
                )
        gripper = episode.actions[:, offset + 6]
        if np.any(gripper < 0.0) or np.any(gripper > 1.0):
            bad = int(np.flatnonzero((gripper < 0.0) | (gripper > 1.0))[0])
            raise ValueError(
                f"frame {bad} {side}.gripper.pos={gripper[bad]:.6f} is outside [0, 1]"
            )

    if len(episode.actions) < 2:
        return ReplaySafetyReport(0.0, 0, 0.0, 0)

    joint_columns = [*range(0, 6), *range(7, 13)]
    gripper_columns = [6, 13]
    joint_steps = np.abs(np.diff(episode.actions[:, joint_columns], axis=0))
    gripper_steps = np.abs(np.diff(episode.actions[:, gripper_columns], axis=0))
    joint_flat_index = int(np.argmax(joint_steps))
    gripper_flat_index = int(np.argmax(gripper_steps))
    joint_transition, _ = np.unravel_index(joint_flat_index, joint_steps.shape)
    gripper_transition, _ = np.unravel_index(gripper_flat_index, gripper_steps.shape)
    max_joint_step = float(joint_steps.flat[joint_flat_index])
    max_gripper_step = float(gripper_steps.flat[gripper_flat_index])
    if max_joint_step > max_frame_joint_step_rad:
        raise ValueError(
            f"joint command jumps {math.degrees(max_joint_step):.3f} deg between frames "
            f"{joint_transition} and {joint_transition + 1}; limit is "
            f"{math.degrees(max_frame_joint_step_rad):.3f} deg"
        )
    if max_gripper_step > max_frame_gripper_step:
        raise ValueError(
            f"gripper command jumps {max_gripper_step:.4f} between frames "
            f"{gripper_transition} and {gripper_transition + 1}; limit is "
            f"{max_frame_gripper_step:.4f}"
        )
    return ReplaySafetyReport(
        max_joint_step_rad=max_joint_step,
        max_joint_step_frame=int(joint_transition),
        max_gripper_step=max_gripper_step,
        max_gripper_step_frame=int(gripper_transition),
    )


def _split_action(
    action: np.ndarray, gripper_max_width_m_by_side: dict[str, float]
) -> dict[str, tuple[float, ...]]:
    result: dict[str, tuple[float, ...]] = {}
    for side_index, side in enumerate(SIDES):
        values = action[side_index * 7 : (side_index + 1) * 7]
        result[side] = (
            *[float(value) for value in values[:6]],
            float(values[6]) * gripper_max_width_m_by_side[side],
        )
    return result


def _print_summary(episode: EpisodeReplay, report: ReplaySafetyReport, rate: float) -> None:
    print(f"dataset: {episode.root}")
    print(f"episode: {episode.episode_index}")
    print(f"frames: {len(episode.actions)}")
    print(f"dataset_fps: {episode.fps:.3f}")
    print(f"recorded_duration_s: {episode.duration_s:.3f}")
    print(f"replay_rate: {rate:.3f}x")
    print(f"planned_duration_s: {episode.duration_s / rate:.3f}")
    print(
        "gripper_metadata: "
        + ("legacy fallback (68 mm)" if episode.legacy_gripper_metadata else "dataset")
    )
    for side in SIDES:
        print(
            f"{side}_gripper_max_width_m: "
            f"{episode.gripper_max_width_m_by_side[side]:.5f}"
        )
    print(
        "max_frame_joint_step_deg: "
        f"{math.degrees(report.max_joint_step_rad):.4f} "
        f"(transition {report.max_joint_step_frame}->{report.max_joint_step_frame + 1})"
    )
    print(
        "max_frame_gripper_step: "
        f"{report.max_gripper_step:.4f} "
        f"(transition {report.max_gripper_step_frame}->{report.max_gripper_step_frame + 1})"
    )
    first = _split_action(
        episode.actions[0], episode.gripper_max_width_m_by_side
    )
    for side in SIDES:
        joints_deg = [math.degrees(value) for value in first[side][:6]]
        print(
            f"{side}_first_joint_deg: "
            + " ".join(f"{value:.3f}" for value in joints_deg)
            + f" | gripper_m={first[side][6]:.5f}"
        )
    print("dataset_preflight=PASS")


def _make_bus(
    side: str,
    port: str,
    feedback_timeout_s: float,
    gripper_max_width_m: float,
) -> Any:
    # Delay hardware imports so the default dry-run works without opening CAN.
    from lerobot_real.devices.piper.piper_motors_bus import PiperMotorsBus
    from lerobot_real.devices.piper.tables import MOTORS, make_calibration

    return PiperMotorsBus(
        id=f"{side}_piper_replay",
        port=port,
        motors=MOTORS.copy(),
        calibration=make_calibration(gripper_max_width_m),
        feedback_timeout_s=feedback_timeout_s,
    )


def _check_start_pose(
    current: dict[str, tuple[float, ...]],
    first: dict[str, tuple[float, ...]],
    *,
    max_joint_error_rad: float,
    max_gripper_error_m: float,
) -> None:
    failures: list[str] = []
    for side in SIDES:
        joint_errors = [
            abs(a - b)
            for a, b in zip(current[side][:6], first[side][:6], strict=True)
        ]
        gripper_error = abs(current[side][6] - first[side][6])
        print(
            f"{side}_start_max_joint_error_deg="
            f"{math.degrees(max(joint_errors)):.3f} "
            f"gripper_error_m={gripper_error:.5f}"
        )
        if max(joint_errors) > max_joint_error_rad:
            failures.append(
                f"{side} maximum start joint error is "
                f"{math.degrees(max(joint_errors)):.3f} deg (limit "
                f"{math.degrees(max_joint_error_rad):.3f} deg)"
            )
        if gripper_error > max_gripper_error_m:
            failures.append(
                f"{side} start gripper error is {gripper_error:.5f} m "
                f"(limit {max_gripper_error_m:.5f} m)"
            )
    if failures:
        raise RuntimeError(
            "robot is not close enough to the episode's first action; no replay command "
            "was sent:\n  - " + "\n  - ".join(failures)
        )


def _start_stop_listener(stop_event: threading.Event) -> threading.Thread:
    def listen() -> None:
        while not stop_event.is_set():
            line = sys.stdin.readline()
            if not line:
                return
            if line.strip().lower() == "q":
                stop_event.set()
                return

    thread = threading.Thread(target=listen, name="replay-stop-listener", daemon=True)
    thread.start()
    return thread


def _hold_current(buses: dict[str, Any], executor: ThreadPoolExecutor, speed_percent: int) -> None:
    def hold(side: str) -> None:
        try:
            current = buses[side].get_joint_state()
            buses[side].set_joint_state(current, speed_percent=speed_percent)
        except Exception as exc:  # Best-effort emergency hold; preserve the original error.
            print(f"WARNING: failed to hold {side} Piper during replay cleanup: {exc}")

    list(executor.map(hold, SIDES))


def execute_replay(episode: EpisodeReplay, args: argparse.Namespace) -> None:
    ports = {"left": args.left_can, "right": args.right_can}
    if len(set(ports.values())) != 2:
        raise ValueError("left and right CAN interface names must be different")

    buses = {
        side: _make_bus(
            side,
            ports[side],
            args.feedback_timeout_s,
            episode.gripper_max_width_m_by_side[side],
        )
        for side in SIDES
    }
    connected: list[str] = []
    stop_event = threading.Event()
    commands_started = False
    with ThreadPoolExecutor(max_workers=2, thread_name_prefix="piper-replay") as executor:
        try:
            for side in SIDES:
                buses[side].connect()
                connected.append(side)
                if args.configure_role:
                    buses[side].set_follower()
                    time.sleep(0.1)
                buses[side].wait_for_follower_feedback("joint", args.startup_timeout_s)
                buses[side].enable_torque()
                buses[side].assert_follower_ready()
                print(f"{side}: connected and ready on {ports[side]}")

            current_values = list(executor.map(lambda side: buses[side].get_joint_state(), SIDES))
            current = dict(zip(SIDES, current_values, strict=True))
            first = _split_action(
                episode.actions[0], episode.gripper_max_width_m_by_side
            )
            _check_start_pose(
                current,
                first,
                max_joint_error_rad=math.radians(args.max_start_joint_error_deg),
                max_gripper_error_m=args.max_start_gripper_error_m,
            )

            phrase = f"REPLAY {episode.episode_index}"
            print("\nWARNING: the next step will physically move both Piper arms.")
            print("Keep the workspace clear and keep the emergency stop within reach.")
            if input(f"Type {phrase!r} to begin: ").strip() != phrase:
                print("Confirmation did not match; replay cancelled and no command was sent.")
                return

            print("Replay started. Type q then Enter, or press Ctrl+C, to stop and hold.")
            _start_stop_listener(stop_event)
            start_time = time.perf_counter()
            timestamp_zero = float(episode.timestamps_s[0])
            completed = 0

            for frame_index, (timestamp_s, action) in enumerate(
                zip(episode.timestamps_s, episode.actions, strict=True)
            ):
                deadline = start_time + (float(timestamp_s) - timestamp_zero) / args.rate
                while not stop_event.is_set():
                    remaining = deadline - time.perf_counter()
                    if remaining <= 0:
                        break
                    stop_event.wait(min(remaining, 0.02))
                if stop_event.is_set():
                    break
                lag_s = time.perf_counter() - deadline
                if lag_s > args.max_timing_lag_s:
                    raise RuntimeError(
                        f"replay fell {lag_s:.3f}s behind at frame {frame_index}; "
                        "aborting instead of sending catch-up commands"
                    )

                targets = _split_action(
                    action, episode.gripper_max_width_m_by_side
                )

                def send(side: str) -> None:
                    buses[side].assert_follower_ready()
                    buses[side].set_joint_state(
                        targets[side],
                        speed_percent=args.speed_percent,
                        gripper_effort=args.gripper_effort,
                    )

                commands_started = True
                list(executor.map(send, SIDES))
                completed = frame_index + 1
                if completed == 1 or completed % max(1, round(episode.fps)) == 0:
                    print(f"replayed_frames={completed}/{len(episode.actions)}")

            if stop_event.is_set():
                print(f"Replay stopped by operator after {completed} frames.")
            else:
                print(f"Replay completed: {completed} frames.")
            if commands_started:
                _hold_current(buses, executor, args.speed_percent)
        except KeyboardInterrupt:
            stop_event.set()
            print("\nCtrl+C received; replay is stopping.")
            if commands_started:
                print("Commanding both arms to hold current feedback position.")
                _hold_current(buses, executor, args.speed_percent)
        except BaseException:
            stop_event.set()
            if commands_started:
                print("Replay error; commanding both arms to hold current feedback position.")
                _hold_current(buses, executor, args.speed_percent)
            raise
        finally:
            for side in reversed(connected):
                try:
                    buses[side].disconnect(disable_torque=args.disable_on_exit, park=False)
                except Exception as exc:
                    print(f"WARNING: failed to disconnect {side} Piper cleanly: {exc}")
            if not args.disable_on_exit and connected:
                print("Piper CAN clients disconnected without disabling motor torque.")
            print(f"robot_commands_sent={str(commands_started).lower()}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Preflight or physically replay one dual-Piper LeRobot episode"
    )
    parser.add_argument("dataset_root", type=Path, help="LeRobot dataset root directory")
    parser.add_argument("--episode", type=int, required=True, help="logical episode index")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="open CAN and move the robots; without this flag only dataset preflight runs",
    )
    parser.add_argument("--left-can", default="can_left")
    parser.add_argument("--right-can", default="can_right")
    parser.add_argument(
        "--rate", type=float, default=0.5, help="replay time scale in (0, 1]; default 0.5x"
    )
    parser.add_argument("--speed-percent", type=int, default=10)
    parser.add_argument("--gripper-effort", type=int, default=1000)
    parser.add_argument("--max-frame-joint-step-deg", type=float, default=5.0)
    parser.add_argument("--max-frame-gripper-step", type=float, default=0.25)
    parser.add_argument("--max-start-joint-error-deg", type=float, default=3.0)
    parser.add_argument("--max-start-gripper-error-m", type=float, default=0.015)
    parser.add_argument("--max-timing-lag-s", type=float, default=0.2)
    parser.add_argument("--feedback-timeout-s", type=float, default=1.0)
    parser.add_argument("--startup-timeout-s", type=float, default=5.0)
    parser.add_argument(
        "--configure-role",
        action="store_true",
        help="explicitly send follower/slave mode before replay; off by default",
    )
    parser.add_argument(
        "--disable-on-exit",
        action="store_true",
        help="disable both arms on exit; off by default to avoid an unsupported arm dropping",
    )
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if not math.isfinite(args.rate) or not 0 < args.rate <= 1:
        raise ValueError("--rate must be in (0, 1]; replay faster than recorded is refused")
    if not 1 <= args.speed_percent <= 100:
        raise ValueError("--speed-percent must be in [1, 100]")
    if not 0 <= args.gripper_effort <= 5000:
        raise ValueError("--gripper-effort must be in [0, 5000]")
    for name in (
        "max_frame_joint_step_deg",
        "max_frame_gripper_step",
        "max_start_joint_error_deg",
        "max_start_gripper_error_m",
        "max_timing_lag_s",
        "feedback_timeout_s",
        "startup_timeout_s",
    ):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"--{name.replace('_', '-')} must be finite and positive")


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    try:
        _validate_args(args)
        episode = load_episode(args.dataset_root, args.episode)
        report = validate_episode(
            episode,
            max_frame_joint_step_rad=math.radians(args.max_frame_joint_step_deg),
            max_frame_gripper_step=args.max_frame_gripper_step,
        )
        _print_summary(episode, report, args.rate)
        if not args.execute:
            print("robot_commands_sent=false")
            print("Dry-run only. Add --execute after reviewing this report.")
            return
        execute_replay(episode, args)
    except Exception as exc:
        parser.exit(1, f"ERROR: {exc}\n")


if __name__ == "__main__":
    main()
