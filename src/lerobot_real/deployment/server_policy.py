"""Serve LeRobot checkpoints or user-provided policies over WebSocket."""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import inspect
import ipaddress
import json
import logging
import math
import os
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np

from .tools.websocket_policy_server import WebsocketPolicyServer

logger = logging.getLogger(__name__)


def _is_loopback(host: str) -> bool:
    if host.lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(host.strip("[]")).is_loopback
    except ValueError:
        return False


def _split_symbol(reference: str) -> tuple[str, str]:
    module_reference, separator, attribute = reference.rpartition(":")
    if not separator or not module_reference or not attribute:
        raise ValueError(f"Expected MODULE:ATTRIBUTE or FILE.py:ATTRIBUTE, got {reference!r}")
    return module_reference, attribute


def _import_module_reference(reference: str) -> ModuleType:
    path = Path(reference).expanduser()
    if path.suffix == ".py" or path.is_file():
        if not path.is_file():
            raise FileNotFoundError(f"Python module file does not exist: {path}")
        module_name = f"lerobot_real_deployment_{path.stem}_{abs(hash(path.resolve()))}"
        spec = importlib.util.spec_from_file_location(module_name, path)
        if spec is None or spec.loader is None:
            raise ImportError(f"Cannot load Python module from {path}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        try:
            spec.loader.exec_module(module)
        except BaseException:
            sys.modules.pop(module_name, None)
            raise
        return module
    return importlib.import_module(reference)


def _load_symbol(reference: str) -> Any:
    module_reference, attribute = _split_symbol(reference)
    value: Any = _import_module_reference(module_reference)
    for part in attribute.split("."):
        value = getattr(value, part)
    return value


def _parse_assignments(values: Sequence[str], *, parse_json: bool) -> dict[str, Any]:
    parsed: dict[str, Any] = {}
    for value in values:
        key, separator, raw = value.partition("=")
        if not separator or not key:
            raise ValueError(f"Expected KEY=VALUE, got {value!r}")
        if key in parsed:
            raise ValueError(f"Duplicate key: {key!r}")
        if parse_json:
            try:
                parsed[key] = json.loads(raw)
            except json.JSONDecodeError:
                parsed[key] = raw
        else:
            parsed[key] = raw
    return parsed


def _accepts_parameter(method, name: str) -> bool:
    try:
        return name in inspect.signature(method).parameters
    except (TypeError, ValueError):
        return False


def _call_with_optional_options(method, observation: Mapping[str, Any], options) -> Any:
    if _accepts_parameter(method, "options"):
        return method(observation, options=options)
    return method(observation)


class ImportedPolicyAdapter:
    """Normalize a user policy exposing get_action, predict_action, or infer."""

    def __init__(
        self,
        policy: Any,
        *,
        method_name: str = "auto",
        action_dim: int | None = None,
    ) -> None:
        candidates = (
            (method_name,)
            if method_name != "auto"
            else ("get_action", "predict_action", "infer")
        )
        for candidate in candidates:
            method = getattr(policy, candidate, None)
            if callable(method):
                self._method = method
                self.method_name = candidate
                break
        else:
            raise TypeError(
                "Custom policy must expose get_action(), predict_action(), or infer()"
            )
        if action_dim is not None and action_dim <= 0:
            raise ValueError("action_dim must be positive")
        self.policy = policy
        self.action_dim = action_dim

    @staticmethod
    def _normalize_action(action: Any) -> dict[str, Any]:
        if isinstance(action, Mapping):
            return dict(action)
        array = np.asarray(action)
        if array.ndim == 0:
            raise TypeError("Custom policy returned a scalar action")
        return {"actions": array}

    def get_action(
        self,
        observation: Mapping[str, Any],
        options: Mapping[str, Any] | None = None,
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        result = _call_with_optional_options(self._method, observation, options)
        info: Mapping[str, Any] = {}
        if isinstance(result, tuple):
            if len(result) != 2:
                raise TypeError("Custom policy tuple must contain (action, info)")
            result, info = result
            if not isinstance(info, Mapping):
                raise TypeError("Custom policy info must be a dictionary")
        action = self._normalize_action(result)
        if self.action_dim is not None and "actions" in action:
            if np.asarray(action["actions"]).shape[-1] != self.action_dim:
                raise ValueError(
                    f"Custom policy action dimension does not match {self.action_dim}"
                )
        return action, dict(info)

    def reset(self, options: Mapping[str, Any] | None = None) -> Any:
        reset = getattr(self.policy, "reset", None)
        if not callable(reset):
            return {}
        if _accepts_parameter(reset, "options"):
            return reset(options=options)
        return reset()

    def get_modality_config(self) -> Any:
        getter = getattr(self.policy, "get_modality_config", None)
        if callable(getter):
            return getter()
        return {
            "adapter": "custom",
            "method": self.method_name,
            "action_dim": self.action_dim,
        }


class LeRobotPolicyAdapter:
    """Thin inference adapter for a saved LeRobot policy checkpoint."""

    def __init__(
        self,
        policy_path: str,
        *,
        device: str | None = None,
        strict: bool = False,
        input_map: Mapping[str, str] | None = None,
        task_key: str = "annotation.human.task_description",
        robot_type: str | None = None,
        register_modules: Sequence[str] = (),
    ) -> None:
        for module_reference in register_modules:
            _import_module_reference(module_reference)

        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.configs.types import FeatureType
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors
        from lerobot.utils.import_utils import register_third_party_plugins
        from lerobot.utils.utils import get_safe_torch_device

        register_third_party_plugins()
        config = PreTrainedConfig.from_pretrained(policy_path)
        if config.use_peft:
            raise ValueError(
                "The thin server cannot infer a PEFT base model automatically; load it through "
                "--custom-policy instead"
            )
        if device is not None:
            config.device = str(get_safe_torch_device(device, log=True))
            if not config.device.startswith("cuda"):
                config.use_amp = False

        policy_class = get_policy_class(config.type)
        policy = policy_class.from_pretrained(
            pretrained_name_or_path=policy_path,
            config=config,
            strict=strict,
        )
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=config,
            pretrained_path=policy_path,
            preprocessor_overrides={
                "device_processor": {"device": str(config.device)},
            },
        )

        self.policy_path = policy_path
        self.config = config
        self.policy = policy
        self.preprocessor = preprocessor
        self.postprocessor = postprocessor
        self.device = get_safe_torch_device(config.device, log=True)
        self.feature_type = FeatureType
        self.input_features = dict(config.input_features or {})
        self.output_features = dict(config.output_features or {})
        if not self.input_features:
            raise ValueError("LeRobot checkpoint does not define input_features")
        self.input_map = dict(input_map or {})
        unknown_targets = set(self.input_map.values()) - set(self.input_features)
        if unknown_targets:
            raise ValueError(
                f"Input map targets are not checkpoint features: {sorted(unknown_targets)}"
            )
        if len(set(self.input_map.values())) != len(self.input_map):
            raise ValueError("Each policy feature may only have one explicit request mapping")
        self.task_key = task_key
        self.robot_type = robot_type
        self.action_dim = self._compute_action_dim()

    def _compute_action_dim(self) -> int:
        dimensions = [
            math.prod(feature.shape)
            for feature in self.output_features.values()
            if feature.type is self.feature_type.ACTION
        ]
        if not dimensions:
            raise ValueError("LeRobot checkpoint does not define an action output feature")
        return sum(dimensions)

    def _source_for_feature(
        self,
        target: str,
        observation: Mapping[str, Any],
    ) -> str:
        explicit_sources = [
            source for source, mapped_target in self.input_map.items() if mapped_target == target
        ]
        if explicit_sources:
            source = explicit_sources[0]
            if source not in observation:
                raise KeyError(f"Policy request is missing explicitly mapped key {source!r}")
            return source
        if target in observation:
            return target

        feature = self.input_features[target]
        candidates: list[str] = []
        if feature.type is self.feature_type.VISUAL:
            suffix = target.removeprefix("observation.images.")
            candidates.extend((f"video.{suffix}", suffix))
        elif feature.type in (self.feature_type.STATE, self.feature_type.ENV):
            state_features = [
                item
                for item, value in self.input_features.items()
                if value.type in (self.feature_type.STATE, self.feature_type.ENV)
            ]
            if len(state_features) == 1:
                candidates.append("state")
            candidates.append(target.removeprefix("observation."))

        matches = [candidate for candidate in candidates if candidate in observation]
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            raise KeyError(
                f"Multiple request keys can map to {target!r}: {matches}; use --input-map"
            )
        raise KeyError(
            f"Policy request has no value for checkpoint feature {target!r}; use "
            "--input-map REQUEST_KEY=POLICY_FEATURE"
        )

    def _coerce_feature(self, source: str, target: str, value: Any) -> np.ndarray:
        array = np.asarray(value)
        expected = tuple(self.input_features[target].shape)
        if array.ndim == len(expected) + 1 and array.shape[0] == 1:
            array = array[0]
        feature = self.input_features[target]
        if feature.type is self.feature_type.VISUAL:
            if array.ndim != 3 or array.shape[-1] not in (1, 3, 4):
                raise ValueError(
                    f"Visual request {source!r} must be unbatched HWC after conversion, "
                    f"got {array.shape}"
                )
        elif tuple(array.shape) != expected:
            raise ValueError(
                f"Request {source!r} mapped to {target!r} has shape {array.shape}; "
                f"expected {expected}"
            )
        if array.dtype.kind in {"f", "i", "u"} and not np.all(np.isfinite(array)):
            raise ValueError(f"Request feature {source!r} contains NaN or infinity")
        return np.ascontiguousarray(array)

    def _task_from_request(
        self,
        observation: Mapping[str, Any],
        options: Mapping[str, Any] | None,
    ) -> str | None:
        if options is not None and isinstance(options.get("task"), str):
            return options["task"]
        value = observation.get(self.task_key)
        if isinstance(value, str):
            return value
        if isinstance(value, Sequence) and value and isinstance(value[0], str):
            return value[0]
        return None

    def get_action(
        self,
        observation: Mapping[str, Any],
        options: Mapping[str, Any] | None = None,
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        from lerobot.utils.control_utils import predict_action

        frame = {}
        for target in self.input_features:
            source = self._source_for_feature(target, observation)
            frame[target] = self._coerce_feature(source, target, observation[source])

        robot_type = self.robot_type
        if options is not None and isinstance(options.get("robot_type"), str):
            robot_type = options["robot_type"]
        action = predict_action(
            observation=frame,
            policy=self.policy,
            device=self.device,
            preprocessor=self.preprocessor,
            postprocessor=self.postprocessor,
            use_amp=self.config.use_amp,
            task=self._task_from_request(observation, options),
            robot_type=robot_type,
        )
        if hasattr(action, "detach"):
            action = action.detach().cpu().numpy()
        action_array = np.asarray(action)
        if action_array.ndim == 2 and action_array.shape[0] == 1:
            action_array = action_array[0]
        if action_array.ndim != 1 or action_array.shape[0] != self.action_dim:
            raise ValueError(
                f"LeRobot policy returned action shape {action_array.shape}; "
                f"expected ({self.action_dim},)"
            )
        if not np.all(np.isfinite(action_array)):
            raise ValueError("LeRobot policy returned NaN or infinity")
        return {"actions": action_array.astype(np.float32, copy=False)}, {}

    def reset(self, options: Mapping[str, Any] | None = None) -> dict[str, Any]:
        del options
        self.policy.reset()
        self.preprocessor.reset()
        self.postprocessor.reset()
        return {}

    def get_modality_config(self) -> dict[str, Any]:
        return {
            "adapter": "lerobot",
            "policy_type": self.config.type,
            "input_features": self.input_features,
            "output_features": self.output_features,
            "input_map": self.input_map,
            "task_key": self.task_key,
            "robot_type": self.robot_type,
            "action_dim": self.action_dim,
        }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Serve a LeRobot checkpoint or a user policy over WebSocket"
    )
    policy_group = parser.add_mutually_exclusive_group(required=True)
    policy_group.add_argument(
        "--policy-path",
        help="Local checkpoint directory or Hugging Face policy repository",
    )
    policy_group.add_argument(
        "--custom-policy",
        help="User factory as MODULE:ATTRIBUTE or FILE.py:ATTRIBUTE",
    )
    parser.add_argument(
        "--register-module",
        action="append",
        default=[],
        help="Import a module/file that registers a custom LeRobot policy; repeat as needed",
    )
    parser.add_argument(
        "--custom-policy-arg",
        action="append",
        default=[],
        metavar="KEY=JSON_VALUE",
        help="Keyword argument passed to the custom policy factory",
    )
    parser.add_argument(
        "--custom-policy-method",
        choices=("auto", "get_action", "predict_action", "infer"),
        default="auto",
    )
    parser.add_argument(
        "--input-map",
        action="append",
        default=[],
        metavar="REQUEST_KEY=POLICY_FEATURE",
        help="Map a client request key to a LeRobot checkpoint input feature",
    )
    parser.add_argument("--task-key", default="annotation.human.task_description")
    parser.add_argument("--robot-type")
    parser.add_argument("--action-dim", type=int, help="Expected custom-policy action width")
    parser.add_argument("--device", help="Override the device saved in a LeRobot checkpoint")
    parser.add_argument("--strict", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument(
        "--api-key",
        default=os.environ.get("LEROBOT_REAL_POLICY_API_KEY"),
        help="Defaults to LEROBOT_REAL_POLICY_API_KEY",
    )
    parser.add_argument(
        "--allow-unauthenticated",
        action="store_true",
        help="Allow a non-loopback plaintext server without an API key",
    )
    parser.add_argument("--max-message-mb", type=int, default=128)
    return parser


def _create_policy(args: argparse.Namespace) -> tuple[Any, dict[str, Any]]:
    if args.policy_path is not None:
        if args.custom_policy_arg:
            raise ValueError("--custom-policy-arg requires --custom-policy")
        adapter = LeRobotPolicyAdapter(
            args.policy_path,
            device=args.device,
            strict=args.strict,
            input_map=_parse_assignments(args.input_map, parse_json=False),
            task_key=args.task_key,
            robot_type=args.robot_type,
            register_modules=args.register_module,
        )
        return adapter, {
            "backend": "lerobot",
            "policy_path": args.policy_path,
            "policy_type": adapter.config.type,
            "action_dim": adapter.action_dim,
        }

    if args.input_map:
        raise ValueError("--input-map is only used with --policy-path")
    if args.register_module:
        raise ValueError("--register-module is only used with --policy-path")
    factory = _load_symbol(args.custom_policy)
    if not callable(factory):
        raise TypeError(f"Custom policy factory is not callable: {args.custom_policy}")
    policy = factory(**_parse_assignments(args.custom_policy_arg, parse_json=True))
    adapter = ImportedPolicyAdapter(
        policy,
        method_name=args.custom_policy_method,
        action_dim=args.action_dim,
    )
    return adapter, {
        "backend": "custom",
        "factory": args.custom_policy,
        "method": adapter.method_name,
        "action_dim": adapter.action_dim,
    }


def main() -> None:
    args = build_parser().parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    if args.max_message_mb <= 0:
        raise ValueError("--max-message-mb must be positive")
    if not _is_loopback(args.host) and args.api_key is None and not args.allow_unauthenticated:
        raise ValueError(
            "Refusing to expose an unauthenticated policy server. Set --api-key (or "
            "LEROBOT_REAL_POLICY_API_KEY), bind to localhost, or explicitly pass "
            "--allow-unauthenticated on a trusted isolated network."
        )

    policy, metadata = _create_policy(args)
    server = WebsocketPolicyServer(
        policy,
        host=args.host,
        port=args.port,
        api_key=args.api_key,
        metadata=metadata,
        max_message_bytes=args.max_message_mb * 1024 * 1024,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info("Policy server interrupted")
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
