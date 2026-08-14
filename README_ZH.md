# Lerobot-Real

> [English Version](README.md)

Lerobot-Real 是面向实验室真实机械臂的 LeRobot 集成项目，通过统一插件和配置层支持 UFACTORY xArm 与 AgileX Piper，并可继续扩展实验室的其他机械臂。

> [!NOTE]
> 本仓库参考并基于原项目 [UFACTORY LeRobot](https://github.com/xArm-Developer/lerobot_robot_ufactory) 修改。原项目及仓库内第三方组件的版权与许可证声明继续有效，详见 [LICENSE](LICENSE)。
> Piper 集成参考 [AgRoboticsResearch/lerobot_robot_piper](https://github.com/AgRoboticsResearch/lerobot_robot_piper) 并适配到本仓库结构；来源与许可证详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 训练推理效果

点击下载开发时[采集的数据集](https://drive.google.com/drive/folders/1Ms25rd2YYGdh3tHPEsTTMU-m1fE7uNYY)，**仅供参考，不可复用**。因为用户机械臂和摄像头位置和开发测试时不一致。

<table>
<tr>
  <td width="50%">
    <a href="https://www.bilibili.com/video/BV12xFjzzEaX" target="_blank">
      <img src="https://i2.hdslb.com/bfs/archive/7b325df5fb4c16e922b66d27b56d8fb6534f8b46.jpg" width="100%">
    </a>
  </td>
  <td width="50%">
    <a href="https://www.bilibili.com/video/BV16ccizHE2P" target="_blank">
      <img src="https://i2.hdslb.com/bfs/archive/8c5d5e9370577fa89d06a175aceea282d9b2eb9a.jpg" width="100%">
    </a>
  </td>
</tr>
<tr>
  <td width="50%">
    <a href="https://www.bilibili.com/video/BV1xGEy6mE3i" target="_blank">
      <img src="https://i2.hdslb.com/bfs/archive/c8bbac04a736b7043a20b753e11afea7627bdae2.jpg" width="100%">
    </a>
  </td>
</tr>
</table>

## 功能特性

- 🤖 支持 [UFACTORY xArm](https://www.ufactory.cc/) 与 AgileX Piper
- 🦾 支持单/双 Piper follower、单/双 Pika 遥操 Piper 和 Piper 主从模式（[使用指南](docs/piper.md)）
- 🎮 多种遥操作方式：GELLO / [Pika](https://global.agilex.ai/products/pika) / [UMI](https://lumosumi.lumosbot.tech/pro/) / [SpaceMouse](https://3dconnexion.com/sg/product/spacemouse-wireless/)
- 📷 多摄像头数据采集（[RealSense](https://www.realsenseai.com/products/depth-camera-d435i/) / UMI 相机）
- 📊 数据集录制与管理（兼容 LeRobot 格式）
- 🧠 模仿学习训练（ACT / Diffusion Policy 等）
- 🚀 策略评估与实时推理
- 🔧 Mock 机器人模拟（只用遥操作设备采集数据）

## 环境要求

- Ubuntu 22.04 / 24.04
- Python >= 3.10
- CUDA >= 12.0（GPU 训练推荐）
- UFACTORY 机械臂（xArm 系列，可选）
- AgileX Piper 与 CAN 接口（可选）

## 安装

### 基础项目安装

```bash
git clone https://github.com/jianliuuu/Lerobot-Real.git
cd Lerobot-Real

# 创建 conda 环境
conda create -n lerobot_real python=3.10 -y
conda activate lerobot_real

# 安装项目
pip install -e .
```

包含：`lerobot==0.4.3`、`xarm-python-sdk`、`numpy`、`pyyaml`（lerobot 已自动携带 torch、opencv、wandb 等训练相关依赖）。

### 外设模块安装

外设依赖以可选模块形式提供，通过 `[模块名]` 安装。

#### GELLO 遥操作

适用于 GELLO 示教臂（Dynamixel 舵机方案），控制空间为关节空间。
* 一旦开始数据采集，机械臂与摄像头（D435 / D435i）的**相对位置必须保持不变**。
* 推理时的摄像头位置必须与采集时相同。若机械臂或摄像头发生变化，此前采集的数据将无效。

```bash
# 1. 安装 GELLO 模块
pip install -e ".[gello]"

# 2. 添加串口权限（重新登录后生效）
sudo usermod -aG dialout $USER
```

#### Pika 遥操作

适用于 Pika Sense 手持示教器 + Vive Tracker，控制空间为笛卡尔空间。
* 两个基站和机械臂相对位置没有要求，只需要保证采集时pika sense在基站范围内，但**基站移动后需要重新校准**。
* 采集和推理时基站位置可不相同。

```bash
# 1. 安装外设依赖（跳过 GUI/RealSense 等传递依赖）
python -m pip install agx-pypika --no-deps
python -m pip install pysurvive --no-deps
```

如果安装 `pysurvive` 时提示 `No matching distribution found`（例如 PyPI
尚未提供较高 Python 版本对应的 Linux wheel），需要从官方 libsurvive
源码编译 Python 绑定。不要只做浅克隆，因为 `setup.py` 会读取 Git 标签
生成版本号。

```bash
# 当前机器需要 HTTP 代理时可选设置
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

# 连接 Pika/Vive 硬件前先验证 Python 绑定
python -c "import pysurvive; print(pysurvive.__file__)"
```

如果已有的是浅克隆，请在编译前执行 `git fetch --unshallow --tags`。
该源码编译流程已在 Python 3.12.13、`pysurvive 1.1.197` 上验证通过。
随后返回 Lerobot-Real 仓库并安装设备规则：

```bash
# 2. 安装 udev 规则（重新插拔设备后生效）
sudo cp rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> Vive Tracker 首次使用前需校准：`lerobot-real-vive-calibrate`

#### AgileX Piper

通过可选依赖安装 Piper SDK：

```bash
pip install -e ".[piper]"
```

Pika 遥操 Piper 还需要上一节的 Pika 依赖。连接前按 Piper SDK 官方
说明以 1 Mbps 激活每个 CAN 接口，并替换 `config/piper/*.yaml` 中的
所有硬件占位符。

Piper 模板已按 AgileX 官方参考设置标准 Pika TCP 变换
`Ry(-90°) @ Tx(190 mm)`；更换夹爪或工具几何后必须重新测量。每次开启遥操时，
程序会用 Piper 实时末端反馈替换配置中的基座位姿后备值；工作空间仍是必须按现场
修改的安全模板。主机只读取 Pika 的 `Command` 状态，不负责识别快速开合手势的
时间窗口。

> 单/双臂模式的配置、安全默认值和运行方法见 [Piper 集成指南](docs/piper.md)。

#### UMI 遥操作

适用于 UMI（Universal Manipulation Interface）方案，含 Vive Tracker 追踪，支持双机械臂。

```bash
# 1. 安装 XVSDK（系统级依赖，仅支持 Ubuntu Focal）
curl -sL https://raw.githubusercontent.com/xArm-Developer/ufactory_resources/main/fastumi/sdk/XVSDK_focal_amd64.deb -o /tmp/xvsdk.deb && sudo dpkg -i /tmp/xvsdk.deb
sudo apt install -y --fix-broken

# 2. 安装外设依赖
pip install pysurvive --no-deps

# 3. 安装 udev 规则（重新插拔设备后生效）
sudo cp rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> Vive Tracker 首次使用前需校准：`lerobot-real-vive-calibrate`

**多 UMI 设备配置**（使用两台及以上时）：

```bash
# 增加 USB 缓冲区大小
sudo sed -i '/GRUB_CMDLINE_LINUX_DEFAULT/s/quiet splash/quiet splash usbcore.usbfs_memory_mb=128/' /etc/default/grub
sync
sudo update-grub
sudo reboot
```

#### SpaceMouse 遥操作

适用于 3Dconnexion SpaceMouse / SpaceNavigator。

```bash
# 1. 安装 SpaceMouse 模块
pip install -e ".[spacemouse]"

# 2. 安装 udev 规则（重新插拔设备后生效）
sudo cp rules/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 使用

### 1. 遥操作测试

测试遥操作设备与机械臂的联动，不录制数据。

```bash
# 通用格式
lerobot-real-teleop --config_path path/to/config.yaml
lerobot-real-teleop --config_path path/to/config.yaml --fps 60  # 指定频率

# 示例: xArm6 + UMI 遥操作
lerobot-real-teleop --config_path config/umi/xarm6_umi_record_config.yaml

# 示例：单 Pika -> 单 Piper
lerobot-real-teleop --config_path config/piper/pika_piper.yaml

# 示例：双 Pika -> 双 Piper
lerobot-real-teleop --config_path config/piper/dual_pika_piper.yaml
```

### 2. 数据采集

通过遥操作录制数据集。

```bash
# 通用格式
lerobot-real-record --config_path path/to/record_config.yaml
lerobot-real-record --resume --config_path path/to/config.yaml          # 底层续录

# 示例: xArm6 + UMI 数据采集
lerobot-real-record --config_path config/umi/xarm6_umi_record_config.yaml

# 示例：单 Piper leader -> 单 Piper follower
lerobot-real-record --config_path config/piper/piper_leader_follower.yaml

# 示例：双 Piper leader -> 双 Piper follower
lerobot-real-record --config_path config/piper/dual_piper_leader_follower.yaml
```

双 Pika + 双 Piper 正式采集如果因异常中止，可显式指定原数据集目录续采。这里的
`--episodes 50` 表示续采完成后的目标总轮数；脚本会先验证旧数据集，并自动扣除已经
完整保存的轮数。异常发生时尚未保存完成的当前轮不会计入，下次会重新采这一轮。

```bash
bash scripts/collect_dual_pika_piper_dataset.sh \
  --resume /home/star/lerobot_data/dual_pika_piper_dataset_YYYYMMDD_HHMMSS \
  --episodes 50
```

### 3. Lerobot训练

采集数据后，使用 LeRobot 训练管道进行模仿学习训练。

```bash
# 通用格式
lerobot-train --policy act --dataset your_dataset_name
```

参数示例：

```bash
# 注意: repo_id就是采集时配置文件里面的repo_id
# 这里训练策略policy.type选用act，训练steps为80w次
# 训练过程每2w次保存一次结果，结果输出到和lerobot同级目录下的lerobot_datas/train里面
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

### 4. 推理

指定模型进行推理

```bash
# 通用格式
lerobot-real-eval --config_path path/to/config.yaml --policy.path your_train_path

# 示例：使用训练好的 ACT 策略进行推理
lerobot-real-eval --config_path config/umi/xarm6_umi_record_config.yaml --policy.path ../../../../lerobot_datas/train/xarm6_umi_datas/checkpoints/last/pretrained_model/
```

## 工具集

### 1. 摄像头查看器

查看和拼接多路摄像头画面。

```bash
lerobot-real-camera-view -l                           # 列出所有摄像头
lerobot-real-camera-view -l -T xvisio                 # 仅列出 XVisio 摄像头
lerobot-real-camera-view -T xvisio                    # 查看 XVisio 摄像头（默认 1280x1280 YU12）
lerobot-real-camera-view -T xvisio -W 640 -H 1920 -F NV12  # 指定格式
lerobot-real-camera-view -T other                     # 查看其他类型摄像头
```

### 2. Lerobot数据集工具
Lerobot提供一些数据集工具，方便对采集的数据集进行增删查操作。

### 查看某个索引的episode:
例如查看索引号为17的episode:
```bash
lerobot-dataset-viz \
  --root=../../../../lerobot_datas/record/xarm7_record_datas \
  --repo-id your_hf_username/xarm7_record_datas \
  --display-compressed-images true \
  --episode-index 17
```

### 删除某些索引的episodes:
例如删除索引号为18和19的episode:
```bash
lerobot-edit-dataset \
  --root=../../../../lerobot_datas/record/xarm7_record_datas \
  --repo_id your_hf_username/xarm7_record_datas \
  --new_repo_id ../xarm7_record_datas_new \
  --operation.type delete_episodes \
  --operation.episode_indices "[18, 19]"
```

### 合并数据集
```bash
lerobot-edit-dataset \
  --root=../../../../lerobot_datas/record \
  --repo_id your_hf_username/xarm7_record_datas_merge_1_2 \
  --operation.type merge \
  --operation.repo_ids "['your_hf_username/xarm7_record_datas_1', 'your_hf_username/xarm7_record_datas_2']"
```


## 遥操作方式对比

| 特性 | GELLO | Pika | UMI | SpaceMouse |
|------|-------|------|-----|------------|
| 控制空间 | 关节空间 | 笛卡尔空间 | 笛卡尔空间 | 笛卡尔空间 |
| 跟踪方式 | Dynamixel 舵机 | Vive Tracker | UMI SLAM / Vive | 3D 鼠标 |
| 双臂支持 | ❌ | ✅ | ✅ | ❌ |
| 系统依赖 | dialout 组 | — | XVSDK deb | — |

## 项目结构

```
Lerobot-Real/
├── src/
│   ├── lerobot_real/      # LeRobot 插件包
│   │   ├── robots/                 # 机器人控制
│   │   │   ├── xarm/           #   xArm 实体机器人
│   │   │   ├── mock_robot/      #   仿真 Mock 机器人
│   │   │   └── piper/              #   单/双 Piper follower
│   │   ├── teleoperators/          # 遥操作器
│   │   │   ├── base_teleop/        #   共享基类
│   │   │   ├── gello_teleop/       #   GELLO (Dynamixel 示教臂)
│   │   │   ├── pika_teleop/        #   单/双 Pika Sense
│   │   │   ├── piper_leader/       #   单/双 Piper leader
│   │   │   ├── xarm_mock_teleop/    #   xArm 自动化 Mock 遥操作器
│   │   │   ├── umi_teleop/         #   UMI (含双机械臂)
│   │   │   └── space_mouse/        #   SpaceMouse (3D 鼠标)
│   │   ├── cameras/                # 摄像头模块
│   │   │   └── umi_camera/         #   UMI 相机
│   │   ├── devices/                # 外部设备驱动
│   │   │   ├── pika/               #   Pika 串口驱动
│   │   │   ├── piper/              #   Piper CAN 适配与位姿工具
│   │   │   └── umi/                #   XVLib / Vive Tracker
│   │   ├── scripts/                # 执行脚本
│   │   │   ├── robot_teleop.py     # 遥操作测试
│   │   │   ├── lerobot_record.py   # 数据采集
│   │   │   ├── lerobot_eval.py     # 策略评估
│   │   │   ├── camera_view.py      # 摄像头查看工具
│   │   │   ├── piper_check_config.py  # Piper 配置检查
│   │   │   └── vive_calibrate.py      # Vive Tracker 校准
│   │   ├── context.py              # Teleop 上下文注册
│   │   └── utils/                  # 工具函数
├── config/                         # YAML 配置文件
│   ├── gello/
│   ├── pika/
│   ├── piper/
│   ├── umi/
│   └── spacemouse/
├── docs/
│   └── piper.md                    # Piper 安装与安全指南
├── rules/                         # udev 设备规则
├── pyproject.toml
└── README.md
```

## 重要提示
用户需要全面研究整个代码库，并了解相关的配置参数，因为代码中所写的配置并非适用于所有使用场景和设置，所以用户需要研究代码或相关理论，以获取相关知识，并自行进行修改和调整。特别是对于扩散策略(diffusion policy)，LeRobot 中的默认参数可能仅用于模拟，并未针对实际机器人场景进行优化。


## 许可证

本项目基于 Apache License 2.0 发布，详见 [LICENSE](LICENSE) 文件。
