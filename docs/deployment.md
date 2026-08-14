# Piper Remote Policy Deployment

The deployment package uses a small MessagePack-over-WebSocket protocol. The Piper
client owns cameras, robot feedback, action conversion, and hardware safety limits.
The server owns policy loading and inference.

This path does not create or load a LeRobot dataset and does not reuse the evaluation
episode loop. A LeRobot checkpoint is loaded directly together with its saved policy
preprocessor and postprocessor.

## Install

Install the Piper and deployment extras in the robot environment:

```bash
python -m pip install -e ".[piper,deployment]"
```

## Start A LeRobot Policy Server

```bash
python -m lerobot_real.deployment.server_policy \
  --policy-path outputs/train/example/checkpoints/last/pretrained_model \
  --device cuda:0 \
  --robot-type piper_follower \
  --host 127.0.0.1 \
  --port 5555
```

The server automatically maps these common request names:

| Client request | LeRobot checkpoint feature |
| --- | --- |
| `state` | The only state/environment input feature |
| `video.wrist` | `observation.images.wrist` |
| `video.front` | `observation.images.front` |

Use an explicit mapping when a checkpoint has multiple state features or different
names:

```bash
python -m lerobot_real.deployment.server_policy \
  --policy-path /path/to/pretrained_model \
  --input-map state=observation.state.eef_pose \
  --input-map video.wrist=observation.images.hand \
  --device cuda:0
```

The output width is read from the checkpoint. The Piper client refuses to connect the
robot when it is not seven.

## Load A New LeRobot Policy Type

If a local module registers a custom `PreTrainedConfig`, policy, and processor factory,
import it before loading the checkpoint:

```bash
python -m lerobot_real.deployment.server_policy \
  --register-module /path/to/register_my_policy.py \
  --policy-path /path/to/pretrained_model \
  --device cuda:0
```

Installed third-party LeRobot entry-point plugins are also discovered automatically.

## Load An Independent Policy

An independent policy does not need to inherit a LeRobot class. Provide a factory that
returns an object with `get_action`, `predict_action`, or `infer`:

```python
class MyPolicy:
    def __init__(self, checkpoint: str):
        self.checkpoint = checkpoint

    def infer(self, observation):
        # Return a [T, 7], [1, T, 7], or named action dictionary.
        return {"actions": run_model(observation)}

    def reset(self):
        pass


def create_policy(checkpoint: str):
    return MyPolicy(checkpoint)
```

Start it with:

```bash
python -m lerobot_real.deployment.server_policy \
  --custom-policy /path/to/my_policy.py:create_policy \
  --custom-policy-arg checkpoint=/path/to/model \
  --action-dim 7
```

`--custom-policy-arg` values are parsed as JSON when possible. Repeat the option for
additional factory keyword arguments.

## Run Piper In Joint Space

Joint observations/actions use `joint1.pos` through `joint6.pos` in `[-100, 100]` and
`gripper.pos` in `[0, 100]`.

```bash
lerobot-real-deploy-piper \
  --host 127.0.0.1 \
  --port 5555 \
  --can-interface can0 \
  --control-space joint \
  --action-type absolute_joint \
  --task "pick and place the block" \
  --camera wrist=REALSENSE_SERIAL \
  --control-fps 10 \
  --execution-horizon 8 \
  --max-relative-target 0.5 \
  --move-speed-percent 5
```

## Run Piper In Cartesian Space

Cartesian observations/actions use `pose.x/y/z` in millimetres,
`pose.rx/ry/rz` as an axis-angle vector in radians, and `gripper.pos` in `[0, 1]`.
Workspace bounds are mandatory.

```bash
lerobot-real-deploy-piper \
  --host 127.0.0.1 \
  --port 5555 \
  --can-interface can0 \
  --control-space cartesian \
  --action-type delta_endpose \
  --task "pick and place the block" \
  --camera wrist=REALSENSE_SERIAL \
  --workspace-x 50 600 \
  --workspace-y -500 500 \
  --workspace-z 50 600 \
  --cartesian-command-mode step \
  --max-cartesian-step-mm 1 \
  --max-rotation-step-rad 0.01 \
  --move-mode move_p \
  --move-speed-percent 5 \
  --disconnect-mode hold
```

For `delta` actions, each arm component is accumulated from the previous target by
default. The last value is treated as an absolute gripper command unless
`--gripper-is-delta` is set. This must match the action representation used for
training. The deployment client does not reinterpret rotation deltas as SO(3)
composition.

`--cartesian-command-mode=step` applies Piper's per-command translation and rotation
limits. `direct` sends the complete target after checking workspace and following-error
bounds. Start with `step`, low speed, an unloaded arm, a reachable workspace, and an
accessible emergency stop.

## Network Exposure

The server defaults to loopback. For a non-loopback bind, set
`LEROBOT_REAL_POLICY_API_KEY` on both machines or pass `--api-key`. The protocol is
plaintext WebSocket; use it only on a trusted isolated network or place it behind a
TLS/VPN tunnel.
