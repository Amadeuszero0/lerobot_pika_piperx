from textwrap import dedent

import numpy as np

from lerobot_real.deployment.server_policy import ImportedPolicyAdapter, _load_symbol


def test_custom_factory_can_be_loaded_from_python_file(tmp_path) -> None:
    module_path = tmp_path / "my_policy.py"
    module_path.write_text(
        dedent(
            """
            from dataclasses import dataclass
            import numpy as np

            @dataclass
            class Policy:
                scale: float

                def predict_action(self, observation):
                    return np.ones((1, 7), dtype=np.float32) * self.scale

            def create_policy(scale=1.0):
                return Policy(float(scale))
            """
        ),
        encoding="utf-8",
    )

    factory = _load_symbol(f"{module_path}:create_policy")
    adapter = ImportedPolicyAdapter(factory(scale=2.0), action_dim=7)
    action, _ = adapter.get_action({"state": np.zeros((1, 7), dtype=np.float32)})

    np.testing.assert_array_equal(action["actions"], np.full((1, 7), 2.0))
