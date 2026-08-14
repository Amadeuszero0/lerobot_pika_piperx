import socket
import threading

import numpy as np

from lerobot_real.deployment.server_policy import ImportedPolicyAdapter
from lerobot_real.deployment.tools.websocket_policy_client import WebsocketClientPolicy
from lerobot_real.deployment.tools.websocket_policy_server import WebsocketPolicyServer


class _FakeImportedPolicy:
    def __init__(self) -> None:
        self.reset_count = 0

    def infer(self, observation):
        assert observation["state"].shape == (1, 7)
        return np.arange(7, dtype=np.float32)[None, :]

    def reset(self) -> None:
        self.reset_count += 1


def _unused_local_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def test_imported_policy_adapter_accepts_infer_and_array_output() -> None:
    implementation = _FakeImportedPolicy()
    adapter = ImportedPolicyAdapter(implementation, action_dim=7)

    action, info = adapter.get_action({"state": np.zeros((1, 7), dtype=np.float32)})

    np.testing.assert_array_equal(action["actions"], np.arange(7)[None, :])
    assert info == {}
    assert adapter.method_name == "infer"


def test_websocket_policy_round_trip_with_custom_adapter() -> None:
    implementation = _FakeImportedPolicy()
    adapter = ImportedPolicyAdapter(implementation, action_dim=7)
    port = _unused_local_port()
    server = WebsocketPolicyServer(
        adapter,
        host="127.0.0.1",
        port=port,
        metadata={"backend": "custom", "action_dim": 7},
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    client = WebsocketClientPolicy(
        host="127.0.0.1",
        port=port,
        connect_timeout_s=5.0,
        request_timeout_s=5.0,
    )
    try:
        assert client.ping()
        action, info = client.get_action(
            {
                "state": np.zeros((1, 7), dtype=np.float32),
                "video.wrist": np.zeros((1, 2, 3, 3), dtype=np.uint8),
                "annotation.human.task_description": ["test task"],
            }
        )
        np.testing.assert_array_equal(action["actions"], np.arange(7)[None, :])
        assert info == {}
        client.reset()
        assert implementation.reset_count == 1
        assert client.get_server_metadata()["action_dim"] == 7
    finally:
        client.close()
        server.shutdown()
        thread.join(timeout=5.0)
    assert not thread.is_alive()
