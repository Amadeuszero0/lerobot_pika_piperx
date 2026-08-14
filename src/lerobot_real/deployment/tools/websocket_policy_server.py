"""Threaded WebSocket server for a policy exposing ``get_action``."""

from __future__ import annotations

import dataclasses
import enum
import hmac
import http
import inspect
import logging
import threading
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np
from websockets.sync.server import Server, ServerConnection, serve

from . import msgpack_numpy

logger = logging.getLogger(__name__)


def _to_wire(value: Any) -> Any:
    """Convert policy metadata and tensor-like values to MessagePack types."""
    if value is None or isinstance(value, (str, bytes, bool, int, float, np.ndarray, np.generic)):
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, enum.Enum):
        return _to_wire(value.value)
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return _to_wire(dataclasses.asdict(value))
    if isinstance(value, Mapping):
        return {str(key): _to_wire(item) for key, item in value.items()}
    if isinstance(value, Sequence):
        return [_to_wire(item) for item in value]
    if hasattr(value, "detach") and hasattr(value, "cpu") and hasattr(value, "numpy"):
        return value.detach().cpu().numpy()
    if hasattr(value, "__dict__"):
        return {key: _to_wire(item) for key, item in vars(value).items() if not key.startswith("_")}
    raise TypeError(f"Cannot serialize policy value of type {type(value).__name__}")


def _invoke_with_options(method, *args: Any, options: Mapping[str, Any] | None) -> Any:
    parameters = inspect.signature(method).parameters
    if "options" in parameters:
        return method(*args, options=options)
    if options:
        raise ValueError(f"{method.__qualname__} does not accept request options")
    return method(*args)


class WebsocketPolicyServer:
    """Expose a policy adapter over a small binary MessagePack protocol."""

    def __init__(
        self,
        policy: Any,
        *,
        host: str = "127.0.0.1",
        port: int = 5555,
        api_key: str | None = None,
        metadata: Mapping[str, Any] | None = None,
        max_message_bytes: int = 128 * 1024 * 1024,
    ) -> None:
        if not 1 <= port <= 65535:
            raise ValueError("port must be in [1, 65535]")
        if max_message_bytes <= 0:
            raise ValueError("max_message_bytes must be positive")
        self.policy = policy
        self.host = host
        self.port = port
        self.api_key = api_key
        self.max_message_bytes = max_message_bytes
        self.metadata = {
            "protocol_version": 1,
            "policy_type": type(policy).__name__,
            **({} if metadata is None else _to_wire(dict(metadata))),
        }
        self._policy_lock = threading.Lock()
        self._server: Server | None = None

    def _process_request(self, connection: ServerConnection, request):
        if self.api_key is None:
            return None
        expected = f"Api-Key {self.api_key}"
        received = request.headers.get("Authorization", "")
        if not hmac.compare_digest(received, expected):
            return connection.respond(http.HTTPStatus.UNAUTHORIZED, "Invalid API key\n")
        return None

    def _get_action(self, request: Mapping[str, Any]) -> dict[str, Any]:
        observation = request.get("observation")
        if not isinstance(observation, Mapping):
            raise ValueError("get_action requires an observation dictionary")
        options = request.get("options")
        if options is not None and not isinstance(options, Mapping):
            raise ValueError("get_action options must be a dictionary or null")

        result = _invoke_with_options(
            self.policy.get_action,
            dict(observation),
            options=options,
        )
        if isinstance(result, tuple):
            if len(result) != 2:
                raise TypeError("policy.get_action() tuple must contain (action, info)")
            action, info = result
        else:
            action, info = result, {}
        if not isinstance(action, Mapping):
            raise TypeError("policy.get_action() must return an action dictionary")
        if info is None:
            info = {}
        if not isinstance(info, Mapping):
            raise TypeError("policy get_action info must be a dictionary")
        return {"action": _to_wire(action), "info": _to_wire(info)}

    def _reset(self, request: Mapping[str, Any]) -> Any:
        reset = getattr(self.policy, "reset", None)
        if not callable(reset):
            return {}
        options = request.get("options")
        if options is not None and not isinstance(options, Mapping):
            raise ValueError("reset options must be a dictionary or null")
        result = _invoke_with_options(reset, options=options)
        return {} if result is None else _to_wire(result)

    def _get_modality_config(self) -> Any:
        getter = getattr(self.policy, "get_modality_config", None)
        if not callable(getter):
            raise NotImplementedError("Policy does not expose get_modality_config()")
        return _to_wire(getter())

    def _dispatch(self, request: Mapping[str, Any]) -> Any:
        endpoint = request.get("endpoint")
        if not isinstance(endpoint, str):
            raise ValueError("Policy request is missing a string endpoint")
        if endpoint == "ping":
            return {"status": "ok"}
        if endpoint == "metadata":
            return self.metadata
        with self._policy_lock:
            if endpoint == "get_action":
                return self._get_action(request)
            if endpoint == "reset":
                return self._reset(request)
            if endpoint == "get_modality_config":
                return self._get_modality_config()
        raise ValueError(f"Unknown policy endpoint: {endpoint}")

    def _handle_connection(self, websocket: ServerConnection) -> None:
        websocket.send(msgpack_numpy.packb(self.metadata))
        for message in websocket:
            endpoint = "unknown"
            try:
                if not isinstance(message, bytes):
                    raise TypeError("Policy requests must be binary MessagePack messages")
                request = msgpack_numpy.unpackb(message)
                if not isinstance(request, Mapping):
                    raise TypeError("Policy request must decode to a dictionary")
                endpoint = str(request.get("endpoint", "unknown"))
                result = self._dispatch(request)
                response = {"ok": True, "result": _to_wire(result)}
            except Exception as exc:
                logger.exception("Policy request %s failed", endpoint)
                response = {
                    "ok": False,
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                }
            websocket.send(msgpack_numpy.packb(response))

    def serve_forever(self) -> None:
        logger.info("Policy server listening on ws://%s:%d", self.host, self.port)
        with serve(
            self._handle_connection,
            self.host,
            self.port,
            process_request=self._process_request,
            compression=None,
            max_size=self.max_message_bytes,
            max_queue=4,
            ping_interval=20,
            ping_timeout=60,
        ) as server:
            self._server = server
            try:
                server.serve_forever()
            finally:
                self._server = None

    def shutdown(self) -> None:
        if self._server is not None:
            self._server.shutdown()
