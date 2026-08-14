"""Observation and action helpers for remote real-robot rollout."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Literal

import numpy as np

from .tools.image_tools import convert_to_uint8

ActionSpace = Literal["joint", "cartesian"]
ActionType = Literal["absolute", "delta"]
DeltaReference = Literal["previous", "observation"]


def normalize_action_type(value: str) -> ActionType:
    aliases = {
        "absolute": "absolute",
        "absolute_joint": "absolute",
        "absolute_endpose": "absolute",
        "delta": "delta",
        "delta_joint": "delta",
        "delta_endpose": "delta",
    }
    try:
        return aliases[value]
    except KeyError as exc:
        raise ValueError(f"Unsupported action type: {value!r}") from exc


def extract_state(
    observation: Mapping[str, Any], state_keys: Sequence[str]
) -> np.ndarray:
    missing = [key for key in state_keys if key not in observation]
    if missing:
        raise KeyError(f"Robot observation is missing state keys: {missing}")
    state = np.asarray([observation[key] for key in state_keys], dtype=np.float32)
    if state.ndim != 1 or not np.all(np.isfinite(state)):
        raise ValueError("Robot state must be a finite one-dimensional vector")
    return state


@dataclass
class PolicyObservationBuilder:
    """Build the flat batched request used by the provided deployment client."""

    camera_names: tuple[str, ...]
    state_keys: tuple[str, ...]
    state_key: str = "state"
    task_key: str = "annotation.human.task_description"
    camera_prefix: str = "video"

    def __post_init__(self) -> None:
        if not self.state_keys:
            raise ValueError("At least one state key is required")
        if not self.state_key or not self.task_key:
            raise ValueError("Policy state and task keys must not be empty")

    def reset(self) -> None:
        return None

    def build(self, observation: Mapping[str, Any], task: str) -> dict[str, Any]:
        if not task:
            raise ValueError("Task description must not be empty")

        request: dict[str, Any] = {
            self.state_key: extract_state(observation, self.state_keys)[None, ...],
            self.task_key: [task],
        }
        for name in self.camera_names:
            if name not in observation:
                raise KeyError(f"Robot observation is missing camera {name!r}")
            image = convert_to_uint8(np.asarray(observation[name]))
            if image.ndim != 3 or image.shape[-1] != 3:
                raise ValueError(
                    f"Camera {name!r} must return HWC RGB data, got shape {image.shape}"
                )
            policy_key = f"{self.camera_prefix}.{name}" if self.camera_prefix else name
            request[policy_key] = np.ascontiguousarray(image)[None, ...]
        return request


def _direct_chunk(value: Any, *, label: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.ndim == 3:
        if array.shape[0] != 1:
            raise ValueError(f"{label} must have batch size 1, got shape {array.shape}")
        array = array[0]
    if array.ndim == 1:
        array = array[None, :]
    if array.ndim != 2 or array.shape[0] == 0:
        raise ValueError(
            f"{label} must have shape [T, D] or [1, T, D], got {array.shape}"
        )
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{label} contains NaN or infinity")
    return array


def _component_chunk(value: Any, *, width: int, label: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.ndim == 3:
        if array.shape[0] != 1:
            raise ValueError(f"{label} must have batch size 1, got shape {array.shape}")
        array = array[0]
    if width == 1:
        if array.ndim == 0:
            array = array.reshape(1, 1)
        elif array.ndim == 1:
            array = array[:, None]
        elif array.ndim == 2 and array.shape[0] == 1 and array.shape[1] != 1:
            array = array.reshape(-1, 1)
    elif array.ndim == 1 and array.shape[0] == width:
        array = array[None, :]
    if array.ndim != 2 or array.shape[1] != width or array.shape[0] == 0:
        raise ValueError(f"{label} must resolve to [T, {width}], got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{label} contains NaN or infinity")
    return array


def _find(mapping: Mapping[str, Any], names: Sequence[str]) -> tuple[str, Any] | None:
    for name in names:
        if name in mapping:
            return name, mapping[name]
    return None


def _combine_components(*components: np.ndarray) -> np.ndarray:
    horizons = {component.shape[0] for component in components}
    if len(horizons) != 1:
        raise ValueError(
            f"Policy action components have different horizons: {sorted(horizons)}"
        )
    return np.concatenate(components, axis=1)


def extract_action_chunk(
    action: Mapping[str, Any],
    *,
    action_space: ActionSpace,
    action_keys: Sequence[str] | None = None,
    action_key: str | None = None,
) -> np.ndarray:
    """Normalize common seven-value policy outputs to one finite ``[T, 7]`` chunk."""
    if not isinstance(action, Mapping) or not action:
        raise ValueError("Policy returned an empty or invalid action dictionary")

    if action_key is not None:
        if action_key not in action:
            raise KeyError(
                f"Policy action does not contain configured key {action_key!r}"
            )
        chunk = _direct_chunk(action[action_key], label=action_key)
    elif action_keys is not None and all(key in action for key in action_keys):
        chunk = _combine_components(
            *(_component_chunk(action[key], width=1, label=key) for key in action_keys)
        )
    elif action_space == "cartesian":
        direct = _find(action, ("actions", "action", "action.eef_pose", "eef_pose"))
        if direct is not None:
            chunk = _direct_chunk(direct[1], label=direct[0])
        else:
            position = _find(action, ("action.position", "position"))
            rotation = _find(action, ("action.rotation", "rotation"))
            gripper = _find(action, ("action.gripper", "gripper"))
            if position is None or rotation is None or gripper is None:
                if len(action) != 1:
                    raise KeyError(
                        "Cartesian output needs actions/eef_pose or "
                        "position/rotation/gripper components"
                    )
                key, value = next(iter(action.items()))
                chunk = _direct_chunk(value, label=key)
            else:
                chunk = _combine_components(
                    _component_chunk(position[1], width=3, label=position[0]),
                    _component_chunk(rotation[1], width=3, label=rotation[0]),
                    _component_chunk(gripper[1], width=1, label=gripper[0]),
                )
    else:
        direct = _find(action, ("actions", "action"))
        joints = _find(action, ("action.joint", "action.joints", "action.pos", "joint"))
        gripper = _find(action, ("action.gripper", "gripper"))
        if direct is not None:
            chunk = _direct_chunk(direct[1], label=direct[0])
        elif joints is None:
            if len(action) != 1:
                raise KeyError("Joint output needs actions or action.joint/action.pos")
            key, value = next(iter(action.items()))
            chunk = _direct_chunk(value, label=key)
        else:
            joint_chunk = _direct_chunk(joints[1], label=joints[0])
            if joint_chunk.shape[1] == 7:
                chunk = joint_chunk
            elif joint_chunk.shape[1] == 6 and gripper is not None:
                chunk = _combine_components(
                    joint_chunk,
                    _component_chunk(gripper[1], width=1, label=gripper[0]),
                )
            else:
                raise ValueError(
                    f"Joint action must contain six joints plus gripper, got {joint_chunk.shape}"
                )

    if chunk.shape[1] != 7:
        raise ValueError(
            f"Policy action must have seven values per step, got {chunk.shape}"
        )
    return chunk


def process_action_chunk(
    state: np.ndarray,
    action_chunk: np.ndarray,
    *,
    action_type: str,
    delta_reference: DeltaReference = "previous",
    gripper_is_delta: bool = False,
) -> np.ndarray:
    """Convert absolute or component-wise delta predictions to absolute targets."""
    normalized_type = normalize_action_type(action_type)
    state_array = np.asarray(state, dtype=np.float64)
    chunk = np.asarray(action_chunk, dtype=np.float64)
    if (
        state_array.ndim != 1
        or chunk.ndim != 2
        or chunk.shape[1] != state_array.shape[0]
    ):
        raise ValueError(
            f"State/action shape mismatch: state={state_array.shape}, action={chunk.shape}"
        )
    if not np.all(np.isfinite(state_array)) or not np.all(np.isfinite(chunk)):
        raise ValueError("State and action chunk must contain only finite values")
    if normalized_type == "absolute":
        return chunk.copy()
    if delta_reference not in ("previous", "observation"):
        raise ValueError(f"Unsupported delta reference: {delta_reference!r}")

    output = np.empty_like(chunk)
    if delta_reference == "observation":
        output[:] = chunk + state_array
        if not gripper_is_delta:
            output[:, -1] = chunk[:, -1]
        return output

    running = state_array.copy()
    for index, delta in enumerate(chunk):
        running[:-1] += delta[:-1]
        running[-1] = running[-1] + delta[-1] if gripper_is_delta else delta[-1]
        output[index] = running
    return output


def make_robot_action(
    values: np.ndarray,
    action_keys: Sequence[str],
    *,
    gripper_bounds: tuple[float, float] | None = None,
) -> dict[str, float]:
    vector = np.asarray(values, dtype=np.float64)
    expected_shape = (len(action_keys),)
    if vector.shape != expected_shape or not np.all(np.isfinite(vector)):
        raise ValueError(
            f"Robot action must be finite with shape {expected_shape}, got {vector.shape}"
        )
    vector = vector.copy()
    if gripper_bounds is not None:
        vector[-1] = np.clip(vector[-1], *gripper_bounds)
    return {key: float(value) for key, value in zip(action_keys, vector, strict=True)}
