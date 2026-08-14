# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from lerobot.teleoperators.utils import make_teleoperator_from_config as lerobot_make_teleoperator_from_config
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator


def make_teleoperator_from_config(config: TeleoperatorConfig) -> Teleoperator:
    if config.type == "lerobot_real::gello_teleop":
        from .gello_teleop import GelloTeleop
        return GelloTeleop(config)
    elif config.type == "lerobot_real::pika_teleop":
        from .pika_teleop import PikaTeleop
        return PikaTeleop(config)
    elif config.type == "lerobot_real::dual_pika_teleop":
        from .pika_teleop.dual_pika_teleop import DualPikaTeleop
        return DualPikaTeleop(config)
    elif config.type == "lerobot_real::spacemouse_teleop":
        from .space_mouse import SpaceMouseTeleop
        return SpaceMouseTeleop(config)
    elif config.type == "lerobot_real::xarm_mock_teleop":
        from .xarm_mock_teleop import XArmMockTeleop
        return XArmMockTeleop(config)
    elif config.type == "lerobot_real::umi_teleop":
        from .umi_teleop import UmiTeleop
        return UmiTeleop(config)
    elif config.type == "lerobot_real::multiple_umi_teleop":
        from .umi_teleop import MultipleUmiTeleop
        return MultipleUmiTeleop(config)
    elif config.type == "lerobot_real::piper_leader":
        from .piper_leader.piper_leader import PiperLeader
        return PiperLeader(config)
    elif config.type == "lerobot_real::dual_piper_leader":
        from .piper_leader.piper_leader import DualPiperLeader
        return DualPiperLeader(config)
    else:
        return lerobot_make_teleoperator_from_config(config)
