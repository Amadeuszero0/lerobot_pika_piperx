import numpy as np
import torch
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.act.processor_act import make_act_pre_post_processors

from lerobot_real.deployment.server_policy import LeRobotPolicyAdapter


def test_lerobot_adapter_loads_checkpoint_and_saved_processors(tmp_path) -> None:
    state_feature = "observation.state"
    environment_feature = "observation.environment_state"
    config = ACTConfig(
        input_features={
            state_feature: PolicyFeature(type=FeatureType.STATE, shape=(7,)),
            environment_feature: PolicyFeature(type=FeatureType.ENV, shape=(7,)),
        },
        output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(7,))},
        device="cpu",
        chunk_size=2,
        n_action_steps=2,
        dim_model=16,
        n_heads=4,
        dim_feedforward=32,
        n_encoder_layers=1,
        n_decoder_layers=1,
        use_vae=False,
        pretrained_backbone_weights=None,
    )
    stats = {
        state_feature: {"mean": torch.zeros(7), "std": torch.ones(7)},
        environment_feature: {"mean": torch.zeros(7), "std": torch.ones(7)},
        "action": {"mean": torch.zeros(7), "std": torch.ones(7)},
    }
    policy = ACTPolicy(config)
    preprocessor, postprocessor = make_act_pre_post_processors(config, dataset_stats=stats)
    policy.save_pretrained(tmp_path)
    preprocessor.save_pretrained(tmp_path)
    postprocessor.save_pretrained(tmp_path)

    adapter = LeRobotPolicyAdapter(
        str(tmp_path),
        device="cpu",
        input_map={
            "state": state_feature,
            "environment": environment_feature,
        },
    )
    action, info = adapter.get_action(
        {
            "state": np.zeros((1, 7), dtype=np.float32),
            "environment": np.zeros((1, 7), dtype=np.float32),
            "annotation.human.task_description": ["test task"],
        }
    )

    assert action["actions"].shape == (7,)
    assert np.all(np.isfinite(action["actions"]))
    assert info == {}
    assert adapter.action_dim == 7
