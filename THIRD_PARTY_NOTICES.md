# Third-party notices

Lerobot-Real includes adapted code from the following Apache-2.0 projects:

- UFACTORY LeRobot:
  https://github.com/xArm-Developer/lerobot_robot_ufactory
- AgileX Piper LeRobot integration:
  https://github.com/AgRoboticsResearch/lerobot_robot_piper
- LeRobot:
  https://github.com/huggingface/lerobot

The Piper-derived portions include motor definitions and calibration ranges,
normalization, pose conversion, CAN follower/leader adapters, dual-arm
composition, configuration templates, and configuration validation. Those
portions were reorganized to use Lerobot-Real's package layout, plugin
factories, and shared command-line entry points.

The project optionally interoperates with the official AgileX Piper SDK:

- Piper SDK: https://github.com/agilexrobotics/piper_sdk (MIT License)

The Piper SDK is an external dependency and is not bundled in this repository.
Other hardware SDKs and Python dependencies remain subject to their respective
licenses.

No copyright or license ownership is transferred by these adaptations. See
`LICENSE` and the linked upstream repositories for the applicable terms.
