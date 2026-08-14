"""Synchronous WebSocket client for remote policy inference."""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Mapping
from typing import Any

from websockets.exceptions import ConnectionClosed
from websockets.sync.client import ClientConnection, connect

from . import msgpack_numpy

logger = logging.getLogger(__name__)


class WebsocketClientPolicy:
    """Call a remote policy through the deployment WebSocket protocol."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int | None = 5555,
        api_key: str | None = None,
        *,
        connect_timeout_s: float = 300.0,
        request_timeout_s: float = 120.0,
        max_message_bytes: int = 128 * 1024 * 1024,
    ) -> None:
        if host in {"0.0.0.0", "::", "[::]"}:
            raise ValueError("A wildcard bind address cannot be used as a client host")
        if connect_timeout_s <= 0 or request_timeout_s <= 0:
            raise ValueError("WebSocket timeouts must be positive")
        if max_message_bytes <= 0:
            raise ValueError("max_message_bytes must be positive")

        if host.startswith(("ws://", "wss://")):
            self._uri = host.rstrip("/")
        else:
            self._uri = f"ws://{host}"
        if port is not None:
            self._uri += f":{port}"

        self._api_key = api_key
        self._connect_timeout_s = connect_timeout_s
        self._request_timeout_s = request_timeout_s
        self._max_message_bytes = max_message_bytes
        self._packer = msgpack_numpy.Packer()
        self._lock = threading.Lock()
        self._ws: ClientConnection | None = None
        self._server_metadata: dict[str, Any] = {}
        self._connect()

    def _open(self, open_timeout_s: float) -> tuple[ClientConnection, dict[str, Any]]:
        headers = {"Authorization": f"Api-Key {self._api_key}"} if self._api_key else None
        connection = connect(
            self._uri,
            compression=None,
            max_size=self._max_message_bytes,
            additional_headers=headers,
            open_timeout=open_timeout_s,
            ping_interval=20,
            ping_timeout=60,
            proxy=None,
        )
        try:
            message = connection.recv(timeout=self._request_timeout_s)
            if not isinstance(message, bytes):
                raise RuntimeError("Policy server metadata must be a binary MessagePack message")
            metadata = msgpack_numpy.unpackb(message)
            if not isinstance(metadata, dict):
                raise RuntimeError("Policy server returned invalid metadata")
            if metadata.get("protocol_version") != 1:
                raise RuntimeError(
                    f"Unsupported policy protocol version: {metadata.get('protocol_version')!r}"
                )
            return connection, metadata
        except BaseException:
            connection.close()
            raise

    def _connect(self) -> None:
        deadline = time.monotonic() + self._connect_timeout_s
        logger.info("Waiting for policy server at %s", self._uri)
        last_error: BaseException | None = None
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                self._ws, self._server_metadata = self._open(min(10.0, remaining))
                logger.info("Connected to policy server at %s", self._uri)
                return
            except (OSError, TimeoutError) as exc:
                last_error = exc
                time.sleep(min(2.0, max(remaining, 0.0)))
        raise TimeoutError(
            f"Failed to connect to policy server {self._uri} within {self._connect_timeout_s:.1f}s"
        ) from last_error

    def get_server_metadata(self) -> dict[str, Any]:
        return self._server_metadata.copy()

    def _request(self, endpoint: str, **payload: Any) -> Any:
        request = {"endpoint": endpoint, **payload}
        with self._lock:
            if self._ws is None:
                self._connect()
            assert self._ws is not None
            try:
                self._ws.send(self._packer.pack(request))
                message = self._ws.recv(timeout=self._request_timeout_s)
            except (ConnectionClosed, OSError, TimeoutError) as exc:
                self._close_unlocked()
                raise ConnectionError(
                    f"Policy request {endpoint!r} failed; robot rollout must stop"
                ) from exc

        if not isinstance(message, bytes):
            raise RuntimeError(f"Policy server returned a text response: {message}")
        response = msgpack_numpy.unpackb(message)
        if not isinstance(response, Mapping) or "ok" not in response:
            raise RuntimeError("Policy server returned an invalid response envelope")
        if not response["ok"]:
            error_type = response.get("error_type", "PolicyServerError")
            raise RuntimeError(f"{error_type}: {response.get('error', 'unknown server error')}")
        return response.get("result")

    def ping(self) -> bool:
        result = self._request("ping")
        return isinstance(result, Mapping) and result.get("status") == "ok"

    def get_action(
        self,
        observation: Mapping[str, Any],
        options: Mapping[str, Any] | None = None,
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        result = self._request(
            "get_action",
            observation=dict(observation),
            options=None if options is None else dict(options),
        )
        if not isinstance(result, Mapping) or not isinstance(result.get("action"), Mapping):
            raise RuntimeError("Policy server get_action result is missing an action dictionary")
        info = result.get("info", {})
        if not isinstance(info, Mapping):
            raise RuntimeError("Policy server get_action result contains invalid info")
        return dict(result["action"]), dict(info)

    def predict_action(self, query_info: Mapping[str, Any]) -> dict[str, Any]:
        """Backward-compatible alias returning only the action dictionary."""
        action, _ = self.get_action(query_info)
        return action

    def reset(self, options: Mapping[str, Any] | None = None) -> dict[str, Any]:
        result = self._request("reset", options=None if options is None else dict(options))
        return dict(result) if isinstance(result, Mapping) else {}

    def get_modality_config(self) -> Any:
        return self._request("get_modality_config")

    def _close_unlocked(self) -> None:
        if self._ws is not None:
            try:
                self._ws.close()
            finally:
                self._ws = None

    def close(self) -> None:
        with self._lock:
            self._close_unlocked()

    def __enter__(self) -> WebsocketClientPolicy:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
