# 双臂 Pika–Piper 数据集说明

本数据集使用双 Pika 遥操作双 PiperX 机械臂采集，采用 LeRobot Dataset v3 格式，共包含 **50 个 episode**。

## 数据集概况

- 机器人：双 PiperX
- 遥操作设备：双 Pika Sense + Vive Tracker
- 相机：3 台 Intel RealSense D435i
- 图像类型：RGB
- 图像分辨率：640 × 480
- 图像帧率：30 FPS
- 数据格式：LeRobot Dataset v3
- Episode 数量：50

每个 episode 包含：

- 左、右 PiperX 的控制动作
- 左、右 PiperX 的机械臂状态
- 左、右 PiperX 的 6 个关节角度
- 第三人称视角 RGB 视频
- 左机械臂腕部 RGB 视频
- 右机械臂腕部 RGB 视频
- 任务、时间戳、episode 编号和帧编号

## 目录结构

```text
dataset_root/
├── data/
│   └── chunk-000/
│       ├── file-000.parquet
│       └── file-001.parquet
│
├── meta/
│   ├── episodes/
│   │   └── chunk-000/
│   │       ├── file-000.parquet
│   │       └── ...
│   ├── info.json
│   ├── stats.json
│   └── tasks.parquet
│
├── videos/
│   ├── observation.images.left.third_view/
│   │   └── chunk-000/
│   │       ├── file-000.mp4
│   │       └── file-001.mp4
│   ├── observation.images.left.wrist/
│   │   └── chunk-000/
│   │       ├── file-000.mp4
│   │       └── file-001.mp4
│   └── observation.images.right.wrist/
│       └── chunk-000/
│           ├── file-000.mp4
│           └── file-001.mp4
│
└── images/                         # 录制或编码过程中的临时图像
```

## 目录及文件说明

### `data/`

保存逐帧的结构化数据，文件格式为 Parquet。

主要字段包括：

- `observation.state`：机械臂观测状态和关节角度
- `action`：发送给机械臂的控制动作
- `episode_index`：当前帧所属的 episode
- `frame_index`：当前帧在 episode 内的编号
- `timestamp`：当前帧的时间戳
- `index`：当前帧在整个数据集中的全局编号
- `task_index`：当前帧对应的任务编号

左、右机械臂的关节角度以弧度保存于 `observation.state`，字段名称包括：

```text
left.joint1.angle_rad
left.joint2.angle_rad
left.joint3.angle_rad
left.joint4.angle_rad
left.joint5.angle_rad
left.joint6.angle_rad

right.joint1.angle_rad
right.joint2.angle_rad
right.joint3.angle_rad
right.joint4.angle_rad
right.joint5.angle_rad
right.joint6.angle_rad
```

`file-000.parquet`、`file-001.parquet` 等文件属于同一个逻辑数据集。出现多个文件通常是由续采或 LeRobot 自动分块造成的，不需要手动合并。

### `meta/`

保存数据集的索引、特征定义和统计信息。

#### `meta/info.json`

记录以下信息：

- LeRobot 数据集版本
- 机器人类型
- 数据集总 episode 数量
- 数据集总帧数
- 采集帧率
- 特征名称、形状和数据类型
- 数据及视频文件的路径模板

#### `meta/episodes/`

保存每个 episode 的元数据，包括：

- Episode 编号和帧数
- Episode 对应的数据文件
- Episode 对应的视频文件
- Episode 在视频文件中的起止时间
- Episode 的统计信息

LeRobot 根据这些元数据，将多个 Parquet 和 MP4 物理文件还原为连续的 50 个逻辑 episode。

#### `meta/stats.json`

保存状态、动作和图像等特征的统计值，例如：

- 均值
- 标准差
- 最小值
- 最大值

这些统计信息可供训练时进行数据归一化。

#### `meta/tasks.parquet`

保存任务文本及对应的 `task_index`。

### `videos/`

保存三路 D435i RGB 视频：

| LeRobot 字段 | 相机位置 |
|---|---|
| `observation.images.left.third_view` | 第三人称视角 |
| `observation.images.left.wrist` | 左机械臂腕部 |
| `observation.images.right.wrist` | 右机械臂腕部 |

同一相机目录中的多个 MP4 文件仍属于同一个逻辑视频数据流。每个 episode 对应的视频文件和时间段由 `meta/episodes/` 记录，训练时由 LeRobot 自动读取，不需要手动拼接 MP4。

### `images/`

该目录用于存放视频编码前的临时图像。

正常完成视频编码后，临时图像通常会被自动清理。如果录制过程曾异常终止，该目录中可能留有未完成 episode 的残余图片。只要数据集能够被 LeRobot 正常加载，这些残余文件不会影响已经完整保存的 episode。

## 数据集加载

```python
from pathlib import Path

from lerobot.datasets.lerobot_dataset import LeRobotDataset


root = Path("/home/star/lerobot_data/final_data")

dataset = LeRobotDataset(
    repo_id="local/final_data",
    root=root,
)

print("episodes:", dataset.num_episodes)
print("frames:", dataset.num_frames)
print(dataset)
```

正常情况下，输出的 episode 数量应为：

```text
episodes: 50
```

## 使用注意事项

- 不要手动拼接或修改 Parquet 和 MP4 文件。
- 不要单独重命名 `data/`、`videos/` 或 `meta/episodes/` 中的文件。
- 移动或备份数据集时，应整体移动或复制数据集根目录。
- 训练时将数据集根目录指向本目录即可。
- `file-000`、`file-001` 等是物理分块，逻辑上仍是一个完整数据集。
- 如果需要继续采集，应使用项目提供的 resume 功能，不要直接覆盖已有文件。

## 续采示例

```bash
cd ~/Lerobot-Real-Cam

bash scripts/collect_dual_pika_piper_dataset.sh \
  --resume /home/star/lerobot_data/final_data \
  --episodes 50
```

续采时，`--episodes` 表示完成后的目标 episode 总数。脚本会读取已经完整保存的 episode 数量，并只采集缺少的部分。
