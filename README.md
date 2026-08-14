# Lerobot-Real

> [中文版本](README_ZH.md)

Lerobot-Real is a lab-oriented LeRobot integration project for real-world robot learning across multiple robot-arm platforms. It currently supports UFACTORY xArm and AgileX Piper integrations through a shared plugin and configuration layer.

> [!NOTE]
> This repository is derived from [UFACTORY LeRobot](https://github.com/xArm-Developer/lerobot_robot_ufactory). The original project and bundled third-party components remain subject to their respective copyright notices and licenses; see [LICENSE](LICENSE).
> The Piper integration is adapted from [AgRoboticsResearch/lerobot_robot_piper](https://github.com/AgRoboticsResearch/lerobot_robot_piper). See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for source and license details.

## Training & Inference Results

[Test datasets](https://drive.google.com/drive/folders/1Ms25rd2YYGdh3tHPEsTTMU-m1fE7uNYY) used during development, **for reference only, do NOT reuse**. As the robot arm and camera positions during development differ from user setups.

<table>
<tr>
  <td width="50%">
    <a href="https://www.youtube.com/watch?v=wTiWLiHciT8" target="_blank">
      <img src="https://img.youtube.com/vi/wTiWLiHciT8/maxresdefault.jpg" width="100%">
    </a>
  </td>
  <td width="50%">
    <a href="https://www.youtube.com/watch?v=IiyvewZh5OY" target="_blank">
      <img src="https://img.youtube.com/vi/IiyvewZh5OY/maxresdefault.jpg" width="100%">
    </a>
  </td>
</tr>
<tr>
  <td width="50%">
    <a href="https://youtu.be/wBwZH6POk38" target="_blank">
      <img src="https://img.youtube.com/vi/wBwZH6POk38/maxresdefault.jpg" width="100%">
    </a>
  </td>
</tr>
</table>

## Features

- 🤖 Robot control for [UFACTORY xArm](https://www.ufactory.cc/) and AgileX Piper
- 🦾 Single/dual Piper follower, single/dual Pika-to-Piper, and Piper leader/follower workflows ([guide](docs/piper.md))
- 🎮 Multiple teleop modes: GELLO / [Pika](https://global.agilex.ai/products/pika) / [UMI](https://lumosumi.lumosbot.tech/pro/) / [SpaceMouse](https://3dconnexion.com/sg/product/spacemouse-wireless/)
- 📷 Multi-camera data collection ([RealSense](https://www.realsenseai.com/products/depth-camera-d435i/) / UMI camera)
- 📊 Dataset recording & management (LeRobot-compatible)
- 🧠 Imitation learning training (ACT / Diffusion Policy / etc.)
- 🚀 Policy evaluation & real-time inference
- 🌐 Remote Piper deployment with native LeRobot checkpoints or custom policies ([guide](docs/deployment.md))
- 🔧 Mock mode (teleop device only, no physical robot needed)

## Requirements

- Ubuntu 22.04 / 24.04
- Python >= 3.10
- CUDA >= 12.0 (recommended for GPU training)
- UFACTORY xArm (optional)
- AgileX Piper with CAN interface (optional)

## Installation

### Base Install

```bash
git clone https://github.com/jianliuuu/Lerobot-Real.git
cd Lerobot-Real

# Create conda environment
conda create -n lerobot_real python=3.10 -y
conda activate lerobot_real

# Install project
pip install -e .
```

Includes: `lerobot==0.4.3`, `xarm-python-sdk`, `numpy`, `pyyaml`. LeRobot already pulls in torch, opencv, wandb, etc.

### Peripheral Modules

Peripheral dependencies are available as optional extras via `[module]` install.

#### GELLO Teleop

Dynamixel-based leader arm, joint-space control.
* Once data collection starts, the **relative position** between the robot arm and camera (D435 / D435i) **must remain unchanged**.
* The camera position during inference must match the collection setup. If the robot arm or camera changes, previously collected data becomes invalid.

```bash
# 1. Install GELLO module
pip install -e ".[gello]"

# 2. Add serial port permissions (re-login required)
sudo usermod -aG dialout $USER
```

#### Pika Teleop

Pika Sense handheld + Vive Tracker, Cartesian-space control.
* No requirement for the relative position of the two base stations and the robot arm. Only need to ensure the Pika Sense is within base station range during collection, but **base stations must be recalibrated after moving**.
* Base station positions for collection and inference do not need to be the same.

```bash
# 1. Install peripheral deps (skip GUI/RealSense transitive deps)
python -m pip install agx-pypika --no-deps
python -m pip install pysurvive --no-deps
```

If `pysurvive` reports `No matching distribution found` (for example, when
PyPI has no Linux wheel for a newer Python version), build the official
libsurvive bindings from source. Do not use a shallow clone: `setup.py` uses
Git tags to determine its version.

```bash
# Optional when this machine needs an HTTP proxy
export HTTP_PROXY=http://PROXY_HOST:PROXY_PORT
export HTTPS_PROXY="$HTTP_PROXY"

sudo apt-get update
sudo apt-get install -y \
  build-essential git libusb-1.0-0-dev zlib1g-dev libx11-dev

python -m pip install "scikit-build>=0.13" "cmake>=3.18" ninja
git clone --recursive https://github.com/collabora/libsurvive.git /tmp/libsurvive-src
cd /tmp/libsurvive-src
git fetch --tags
python -m pip install --no-build-isolation --no-deps .

# Verify the Python binding before connecting Pika/Vive hardware
python -c "import pysurvive; print(pysurvive.__file__)"
```

For an existing shallow clone, run `git fetch --unshallow --tags` before the
build. This source-build path was verified with Python 3.12.13 and
`pysurvive 1.1.197`. Then return to the Lerobot-Real repository and install the
device rules:

```bash
# 2. Install udev rules (re-plug devices afterwards)
sudo cp rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> Calibrate Vive Tracker before first use: `lerobot-real-vive-calibrate`

#### AgileX Piper

Install the official Piper SDK integration as an optional extra:

```bash
pip install -e ".[piper]"
```

For remote policy inference and Piper deployment, install both optional extras:

```bash
pip install -e ".[piper,deployment]"
```

Pika-to-Piper workflows also require the Pika dependencies from the previous
section. Activate each CAN interface at 1 Mbps before connecting, then replace
all hardware placeholders in `config/piper/*.yaml`.

The Piper templates include AgileX's standard Pika TCP transform
`Ry(-90 deg) @ Tx(190 mm)`. It must be remeasured after changing the gripper or
tool geometry. On each teleop start, the configured base-pose fallback is
replaced with live Piper end-pose feedback; workspace bounds remain site-specific
safety templates. The host reads Pika's `Command` state and does not implement
the rapid-gripper-gesture timing itself.

> Setup, safety defaults, and single/dual-arm examples: [Piper integration guide](docs/piper.md)

#### UMI Teleop

Universal Manipulation Interface + Vive Tracker, supports dual-arm.

```bash
# 1. Install XVSDK (system-level, Ubuntu Focal only)
curl -sL https://raw.githubusercontent.com/xArm-Developer/ufactory_resources/main/fastumi/sdk/XVSDK_focal_amd64.deb -o /tmp/xvsdk.deb && sudo dpkg -i /tmp/xvsdk.deb
sudo apt install -y --fix-broken

# 2. Install peripheral deps
pip install pysurvive --no-deps

# 3. Install udev rules (re-plug devices afterwards)
sudo cp src/rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> Calibrate Vive Tracker before first use: `lerobot-real-vive-calibrate`

**Multi-UMI device configuration** (two or more devices):

```bash
# Increase USB buffer size
sudo sed -i '/GRUB_CMDLINE_LINUX_DEFAULT/s/quiet splash/quiet splash usbcore.usbfs_memory_mb=128/' /etc/default/grub
sync
sudo update-grub
sudo reboot
```

#### SpaceMouse Teleop

3Dconnexion SpaceMouse / SpaceNavigator.

```bash
# 1. Install SpaceMouse module
pip install -e ".[spacemouse]"

# 2. Install udev rules (re-plug device afterwards)
sudo cp src/rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```


## Usage

### 1. Teleop Testing

Test teleop-to-robot control loop without recording.

```bash
# Generic usage
lerobot-real-teleop --config_path path/to/config.yaml
lerobot-real-teleop --config_path path/to/config.yaml --fps 60  # specify frequency

# Example: xArm6 + UMI teleop
lerobot-real-teleop --config_path config/umi/xarm6_umi_record_config.yaml

# Example: single Pika -> single Piper
lerobot-real-teleop --config_path config/piper/pika_piper.yaml

# Example: dual Pika -> dual Piper
lerobot-real-teleop --config_path config/piper/dual_pika_piper.yaml
```

### 2. Data Collection

Record datasets via teleop.

```bash
# Generic usage
lerobot-real-record --config_path path/to/record_config.yaml
lerobot-real-record --resume --config_path path/to/config.yaml             # low-level resume

# Example: xArm6 + UMI data collection
lerobot-real-record --config_path config/umi/xarm6_umi_record_config.yaml

# Example: single Piper leader -> single Piper follower
lerobot-real-record --config_path config/piper/piper_leader_follower.yaml

# Example: dual Piper leader -> dual Piper follower
lerobot-real-record --config_path config/piper/dual_piper_leader_follower.yaml
```

If a formal dual-Pika/dual-Piper session stops because of an error, resume it
by explicitly naming the original dataset. In this launcher, `--episodes 50`
means the desired final total; already saved episodes are validated and
subtracted automatically. An interrupted, unsaved episode is recorded again.

```bash
bash scripts/collect_dual_pika_piper_dataset.sh \
  --resume /home/star/lerobot_data/dual_pika_piper_dataset_YYYYMMDD_HHMMSS \
  --episodes 50
```

### 3. Policy Training

Train imitation learning policies on collected data.

```bash
# Generic usage
lerobot-train --policy act --dataset your_dataset_name

# Example: train ACT on xArm6 UMI dataset
lerobot-train --policy act --dataset your_hf_username/xarm6_umi_datas
```

Important parameters:

```bash
# Note: repo_id is the same as in the record config
# Policy type: ACT, training steps: 800k
# Checkpoints saved every 20k steps, output to lerobot_datas/train (sibling of lerobot directory)
lerobot-train \
  --dataset.root=../../../../lerobot_datas/record/xarm6_umi_datas \
  --dataset.repo_id=your_hf_username/xarm6_umi_datas \
  --policy.type=act \
  --policy.device=cuda \
  --policy.repo_id=your_hf_username/xarm6_umi_datas \
  --output_dir=../../../../lerobot_datas/train/xarm6_umi_datas \
  --job_name=xarm6_umi_datas \
  --steps=800000 \
  --batch_size=8 \
  --save_freq=20000
```

### 4. Inference & Evaluation

Run inference with a trained policy.

```bash
# Generic usage
lerobot-real-eval --config_path path/to/config.yaml --policy.path your_train_path

# Example: run inference with trained ACT policy
lerobot-real-eval --config_path config/umi/xarm6_umi_record_config.yaml --policy.path ../../../../lerobot_datas/train/xarm6_umi_datas/checkpoints/last/pretrained_model/
```

### 5. Remote Piper Policy Deployment

Run policy inference on a server and execute the returned action chunks on a Piper
robot client. Native LeRobot checkpoints are loaded directly with their saved
preprocessor and postprocessor; independently implemented policies can also be loaded
through a custom factory.

```bash
# Policy server
lerobot-real-policy-server \
  --policy-path /path/to/pretrained_model \
  --device cuda:0 \
  --host 127.0.0.1 \
  --port 5555

# Piper client (run in the robot environment)
lerobot-real-deploy-piper \
  --host 127.0.0.1 \
  --port 5555 \
  --can-interface can0 \
  --control-space joint \
  --action-type absolute_joint \
  --task "pick and place the block" \
  --camera wrist=REALSENSE_SERIAL
```

Cartesian control requires explicit workspace bounds. Start physical tests at low
speed with an accessible emergency stop. See the [Piper remote deployment guide](docs/deployment.md)
for Cartesian examples, request/action formats, custom policy loading, and network
security settings.

## Tools

### 1. Camera Viewer

View and stitch multiple camera feeds.

```bash
lerobot-real-camera-view -l                           # list all cameras
lerobot-real-camera-view -l -T xvisio                 # list XVisio cameras only
lerobot-real-camera-view -T xvisio                    # view XVisio cameras (default 1280x1280 YU12)
lerobot-real-camera-view -T xvisio -W 640 -H 1920 -F NV12  # specify format
lerobot-real-camera-view -T other                     # view other camera types
```

### 2. LeRobot Dataset Tools

LeRobot provides dataset utilities for inspecting, editing and managing collected datasets.

#### View an episode:
e.g. view episode index 17:
```bash
lerobot-dataset-viz \
  --root=../../../../lerobot_datas/record/xarm7_record_datas \
  --repo-id your_hf_username/xarm7_record_datas \
  --display-compressed-images true \
  --episode-index 17
```

#### Delete specific episodes:
e.g. delete episodes 18 and 19:
```bash
lerobot-edit-dataset \
  --root=../../../../lerobot_datas/record/xarm7_record_datas \
  --repo_id your_hf_username/xarm7_record_datas \
  --new_repo_id ../xarm7_record_datas_new \
  --operation.type delete_episodes \
  --operation.episode_indices "[18, 19]"
```

#### Merge datasets:
```bash
lerobot-edit-dataset \
  --root=../../../../lerobot_datas/record \
  --repo_id your_hf_username/xarm7_record_datas_merge_1_2 \
  --operation.type merge \
  --operation.repo_ids "['your_hf_username/xarm7_record_datas_1', 'your_hf_username/xarm7_record_datas_2']"
```

## Teleop Comparison

| Feature | GELLO | Pika | UMI | SpaceMouse |
|---------|-------|------|-----|------------|
| Control space | Joint space | Cartesian space | Cartesian space | Cartesian space |
| Tracking | Dynamixel servos | Vive Tracker | UMI SLAM / Vive | 3D mouse |
| Dual-arm | ❌ | ✅ | ✅ | ❌ |
| System dep | dialout group | — | XVSDK deb | — |

## Project Structure

```
Lerobot-Real/
├── src/
│   ├── lerobot_real/      # LeRobot plugin package
│   │   ├── robots/                 # Robot control
│   │   │   ├── xarm/           #   xArm physical robot
│   │   │   ├── mock_robot/      #   Mock robot simulator
│   │   │   └── piper/              #   Single/dual Piper follower
│   │   ├── teleoperators/          # Teleop drivers
│   │   │   ├── base_teleop/        #   Shared base class
│   │   │   ├── gello_teleop/       #   GELLO (Dynamixel leader)
│   │   │   ├── pika_teleop/        #   Single/dual Pika Sense
│   │   │   ├── piper_leader/       #   Single/dual Piper leader
│   │   │   ├── xarm_mock_teleop/    #   xArm automated mock teleoperator
│   │   │   ├── umi_teleop/         #   UMI (dual-arm support)
│   │   │   └── space_mouse/        #   SpaceMouse (3D mouse)
│   │   ├── cameras/                # Camera modules
│   │   │   └── umi_camera/         #   UMI camera
│   │   ├── deployment/             # Remote policy server and Piper rollout client
│   │   ├── devices/                # External device drivers
│   │   │   ├── pika/               #   Pika serial driver
│   │   │   ├── piper/              #   Piper CAN adapter and pose helpers
│   │   │   └── umi/                #   XVLib / Vive Tracker
│   │   ├── scripts/                # Entry-point scripts
│   │   │   ├── robot_teleop.py     # Teleop testing
│   │   │   ├── lerobot_record.py   # Data recording
│   │   │   ├── lerobot_eval.py     # Policy evaluation
│   │   │   ├── camera_view.py      # Camera viewer tool
│   │   │   ├── piper_check_config.py  # Piper config validation
│   │   │   └── vive_calibrate.py      # Vive Tracker calibration
│   │   ├── context.py              # Teleop context registry
│   │   └── utils/                  # Utilities
├── config/                         # YAML config files
│   ├── gello/
│   ├── pika/
│   ├── piper/
│   ├── umi/
│   └── spacemouse/
├── docs/
│   ├── deployment.md               # Piper remote policy deployment guide
│   └── piper.md                    # Piper setup and safety guide
├── rules/                          # udev device rules
├── pyproject.toml
└── README.md
```

## Important Notes

Users are expected to thoroughly study the codebase and configuration parameters.  
The provided configurations are **not guaranteed to work for all scenarios** and must be adjusted based on actual hardware setups and task requirements.

In particular, for **diffusion policies**, the default parameters in LeRobot are primarily designed for simulation and **are not optimized for real-world robots**.

## License

This project is released under the Apache License 2.0. See [LICENSE](LICENSE).
