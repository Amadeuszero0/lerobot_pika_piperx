# 双 Pika–双 PiperX 数据集格式说明（训练人员版）

本文档说明本项目采集的双臂遥操作数据如何存储、每个特征代表什么，以及训练代码应如何正确读取和对齐数据。

适用采集系统：

- 两台 Pika Sense + Vive Tracker：人工遥操作输入
- 两台 PiperX：左右从臂
- 三台 Intel RealSense D435i：第三视角、左腕、右腕 RGB
- LeRobot `0.4.3`
- LeRobot Dataset v3 目录格式

> 数据集本身的权威描述始终是该数据集目录中的 `meta/info.json`。本文描述的是当前项目生成的默认 schema；如果以后修改采集配置，应以实际 `info.json` 为准。

## 1. 一页速览

| 项目 | 当前格式 |
|---|---|
| 逻辑样本单位 | 一帧控制周期 |
| 数据集采样率 | 30 Hz |
| Episode | 一次经人工确认并保存的 B 动作轨迹 |
| 图像 | 3 路 RGB，640×480，30 FPS |
| 深度/IMU | 当前不保存 |
| `observation.state` | 14 维 `float32`，关节反馈 + 夹爪反馈 |
| `observation.state.endpose` | 12 维 `float32`，实际 EEF 位姿 |
| `action` | 14 维 `float32`，实际发送的关节目标 + 夹爪目标 |
| `action.endpose` | 12 维 `float32`，实际使用的 EEF 目标 |
| 关节角单位 | rad（弧度） |
| 末端位置单位 | mm（毫米） |
| 末端姿态 | 轴角向量，rad；不是 RPY 欧拉角 |
| 夹爪 | 归一化开合量，约 `0.0～1.0` |
| Action 类型 | 主 `action` 为双臂绝对关节目标 + 夹爪目标 |
| Action 是否为差值 | 否 |
| Action/State 是否人工错开一帧 | 否 |

训练时最重要的语义是：

```text
observation.state[t]         = 本周期开始时读取的关节与夹爪实际反馈
observation.state.endpose[t] = 本周期开始时读取的实际 EEF 位姿
action[t]                    = official IK 在本周期实际发送的关节目标和夹爪目标
action.endpose[t]            = 本周期经过限制与 IK 接受后的有效 EEF 目标
```

四者存放在同一行。动作的物理效果通常体现在后续帧中，但原始数据没有人为移动 action，也没有保存成差值。

## 2. 数据集目录结构

一个完整数据集目录类似：

```text
dataset_root/
├── data/
│   └── chunk-000/
│       ├── file-000.parquet
│       ├── file-001.parquet
│       └── ...
├── meta/
│   ├── episodes/
│   │   └── chunk-000/
│   │       ├── file-000.parquet
│   │       └── ...
│   ├── info.json
│   ├── robot_hardware.json
│   ├── stats.json
│   └── tasks.parquet
├── videos/
│   ├── observation.images.left.third_view/
│   │   └── chunk-000/
│   │       ├── file-000.mp4
│   │       ├── file-001.mp4
│   │       └── ...
│   ├── observation.images.left.wrist/
│   │   └── chunk-000/
│   │       └── ...
│   └── observation.images.right.wrist/
│       └── chunk-000/
│           └── ...
└── images/                         # 可能存在的编码临时目录
```

### 2.1 逻辑数据集与物理分块

`file-000.parquet`、`file-001.parquet` 或多个 MP4 不代表多个独立数据集，也不保证一个文件只对应一个 episode。它们只是 LeRobot 的物理分块。

出现多个 `file-xxx` 的常见原因包括：

- 使用 resume 向已有数据集续采；
- LeRobot 达到分块大小后自动创建新文件；
- 视频编码产生新的物理分块。

训练时应把整个 `dataset_root` 作为一个 LeRobot 数据集加载，由 `meta/` 恢复逻辑 episode。不要手工拼接 Parquet 或 MP4。

## 3. `data/`：逐帧结构化数据

`data/chunk-*/file-*.parquet` 保存每帧的状态、动作和索引。主要列为：

| Parquet 列 | 含义 |
|---|---|
| `observation.state` | 当前关节角和夹爪实际反馈，14 维 |
| `observation.state.endpose` | 当前双臂实际 EEF 位姿，12 维 |
| `action` | 本周期实际发送的关节和夹爪目标，14 维 |
| `action.endpose` | 本周期实际使用的双臂 EEF 目标，12 维 |
| `episode_index` | 当前帧所属的逻辑 episode |
| `frame_index` | 当前帧在本 episode 内的序号，从 0 开始 |
| `timestamp` | episode 内相对时间，单位为秒；不是 Unix 时间 |
| `index` | 帧在整个数据集中的全局索引 |
| `task_index` | 对应 `meta/tasks.parquet` 中的任务编号 |

每个 episode 的长度可以不同。采集者可以在达到最长时间前提前结束，因此训练代码不能假设每个 episode 都正好是 `episode_time_s × 30` 帧。

## 4. 两个 Observation feature

### 4.1 `observation.state`：14 维实际关节状态

当前正式录制配置开启 `record_joint_angles: true`。该 feature 的名称和顺序为：

```text
0   left.joint1.angle_rad
1   left.joint2.angle_rad
2   left.joint3.angle_rad
3   left.joint4.angle_rad
4   left.joint5.angle_rad
5   left.joint6.angle_rad
6   left.gripper.pos
7   right.joint1.angle_rad
8   right.joint2.angle_rad
9   right.joint3.angle_rad
10  right.joint4.angle_rad
11  right.joint5.angle_rad
12  right.joint6.angle_rad
13  right.gripper.pos
```

关节角单位为 rad，来源是 PiperX 反馈；夹爪是归一化反馈，约 `0～1`。现场新数据
使用98 mm有效行程，物理换算必须读取 `meta/robot_hardware.json`，不要把没有该文件的
旧版68 mm数据当作98 mm。不要只依赖数字下标，应读取：

```python
info["features"]["observation.state"]["names"]
```

### 4.2 `observation.state.endpose`：12 维实际 EEF 状态

顺序是左臂 `pose.x/y/z/rx/ry/rz`，随后是右臂同样六项。XYZ 单位为 mm，分别位于每台 PiperX 的基座坐标系；姿态为轴角向量（rotation vector），单位 rad，不是独立 RPY 欧拉角。

对于向量 `r = [rx, ry, rz]`：

```text
旋转角度 = ||r||
旋转轴   = r / ||r||       （当 ||r|| > 0）
```

如果训练需要相对旋转，不应简单相减，应先转换为旋转矩阵或四元数，再计算：

```text
R_relative = R_state^T · R_action
```

## 5. 两个 Action feature

### 5.1 `action`：14 维真实发送关节目标

名称和顺序与 `observation.state` 相同：左 6 关节 + 左夹爪 + 右 6 关节 + 右夹爪。

```text
0..5   left.joint1..6.angle_rad
6   left.gripper.pos
7..12  right.joint1..6.angle_rad
13  right.gripper.pos
```

关节项来自本周期 official IK 路径调用 `set_joint_state()` 后返回的最终 `sent_joints`，不是 `state[t+1]`，也不是重新离线求 IK 得到的值。夹爪项是本周期实际目标。全部为绝对目标，不是 delta。

### 5.2 `action.endpose`：12 维有效 EEF 目标

顺序与 `observation.state.endpose` 相同，不重复夹爪。它保存经过坐标转换、工作空间限制以及 IK 控制路径接受后的 EEF 目标。若 IK 无解并保持上一条有效命令，则保存相应的保持目标，而不是不可执行的原始 Pika 输入。

## 6. State 与 Action 的时间对齐

每个控制周期的顺序为：

```text
1. 读取机器人反馈，得到 observation.state[t] 和 observation.state.endpose[t]
2. 读取 Pika，生成本周期 EEF 请求
3. 经安全限制和 official IK 后发送给双 PiperX
4. 记录真实 sent joint target 到 action[t]
5. 记录有效 EEF target 到 action.endpose[t]
6. 四个低维 feature 与三路 RGB 写入同一帧
```

所以原始数据的行级对应关系是：

```text
frame[t] = {
    observation.state[t],
    observation.state.endpose[t],
    action[t],
    action.endpose[t],
    RGB[t],
}
```

没有执行以下处理：

```text
frame[t] = {observation[t], action[t+1]}       # 没有人工错位
action[t] = joint_state[t+1]                   # 新采数据没有用下一状态代理 action
action[t] = target[t] - state[t]               # 没有转成差值
```

从控制因果关系看：

```text
observation[t] → action[t] → observation[t+1 ...]
```

机械臂、CAN、IK 和控制器存在动态延迟，所以 action 的实际效果可能在后续一帧或多帧中逐步出现，并不保证严格延迟一帧。这是正常物理现象。

对于标准行为克隆，通常直接训练：

```text
policy(observation[t]) → action[t]
```

本批数据默认使用三路 RGB + `observation.state` 预测主 `action`。两个 `*.endpose` feature 作为 EEF 辅助监督、运动学一致性和质量检查数据保留。

不要为了“补偿延迟”在不了解模型假设的情况下先把整列 action 固定移动一帧。若需要系统辨识或精确延迟补偿，应先通过时间序列相关性单独估计每个数据集的延迟。

## 7. 三路 RGB 图像

当前保存三个视频特征：

| LeRobot 特征名 | 物理视角 | D435i 序列号 |
|---|---|---|
| `observation.images.left.third_view` | 第三人称视角 | `346122070530` |
| `observation.images.left.wrist` | 左腕视角 | `233522077815` |
| `observation.images.right.wrist` | 右腕视角 | `250122077305` |

默认参数：

```text
RGB: 640 × 480
FPS: 30
编码: MP4（由 LeRobot/FFmpeg 生成）
```

虽然设备是 D435i，但当前数据集只保存 RGB，不保存 depth、点云或 IMU。

`left.third_view` 中的 `left` 只是因为第三视角相机在软件配置中挂载到左侧机器人对象，用于组合统一 schema；它仍然是第三人称相机，不是左腕相机。

### 7.1 图像时间同步说明

三路相机和双臂状态在同一个 observation 周期内并行读取，因此是软件层面的近同步。当前没有配置 D435i 硬件触发同步，也没有声明三台相机曝光时刻完全相同。

训练通常可以按 LeRobot 同一 frame 使用三路图像。若研究要求亚毫秒级多相机同步，需要单独增加硬件同步和设备时间戳记录，不能仅依赖当前数据格式。

### 7.2 不要按文件名手动对齐视频

不要假设 `videos/.../file-001.mp4` 与 `data/.../file-001.parquet` 逐文件一一对应。应通过 LeRobot loader 和 `meta/episodes/` 的索引读取指定 frame 的图像。

## 8. Episode 的含义

当前任务描述中，总任务由 A+B+C 组成，本数据集记录的是 B。

每轮流程包含：

1. PREP：人工摇操恢复到 A 的结束状态；不记录。
2. RECORD B：正式记录 B；支持达到最长时长自动结束或提前结束。
3. REVIEW：人工决定保存、丢弃重做或退出；不记录。

因此，一个已保存 episode 只包含正式 B 轨迹，不包括：

- 回到 A 结束状态的准备运动；
- 轮次之间的等待；
- 审核阶段；
- 被 `r` 丢弃的失败轨迹；
- 使用 `q` 退出时尚未确认的当前轨迹。

### 8.1 变长 Episode

因为允许提前结束，每个 episode 的帧数可以不同。训练时必须：

- 根据 `episode_index` 或 episode 元数据划分序列；
- 在 episode 边界处终止 action chunk；
- 对 batch 中的变长序列进行 padding/mask，或按固定窗口采样；
- 不要让上下文窗口跨越两个 episode。

### 8.2 续采（resume）

resume 会向同一个逻辑数据集追加完整 episode，可能产生新的 Parquet 和 MP4 文件。采集完成后不需要再写脚本合并。

训练前只需要确认：

- `meta/info.json` 中总 episode 和总帧数正确；
- LeRobot 能从根目录加载全部 episode；
- 未完成 episode 的临时图片没有被误当作正式数据。

## 9. `meta/` 元数据

### 9.1 `meta/info.json`

这是训练代码理解 schema 的首要文件，通常包括：

- `codebase_version`
- `robot_type`
- `fps`
- `total_episodes`
- `total_frames`
- `total_tasks`
- `features`
- 数据和视频路径模板

训练前应至少校验：

```python
info["fps"]
info["total_episodes"]
info["total_frames"]
info["features"]["observation.state"]
info["features"]["action"]
```

### 9.2 `meta/episodes/`

保存 episode 级索引和统计信息，用于把物理分块恢复成逻辑 episode。不要单独删除或重命名。

### 9.3 `meta/stats.json`

保存状态、动作、图像等特征的统计量，例如 mean、std、min、max，可用于归一化和快速质量检查。

如果需要严格的 train/validation/test 划分，建议按 episode 划分后，只用训练集 episode 重新计算模型使用的归一化统计量，避免验证集信息泄漏。原始 `stats.json` 可以保留作为完整数据集统计参考。

### 9.4 `meta/tasks.parquet`

保存任务文本及其 `task_index`。当前通常是单任务数据集，但训练代码仍应通过 `task_index` 读取，不要假定它永远为 0。

### 9.5 `meta/robot_hardware.json`

这是本项目额外保存的物理控制语义，记录左右夹爪的 `max_width_m_by_side` 以及
`gripper.pos` 的归一化范围。现场100 mm大夹爪保留2 mm安全余量，因此新数据通常为：

```json
{
  "schema_version": 1,
  "gripper": {
    "normalized_range": [0.0, 1.0],
    "max_width_m_by_side": {"left": 0.098, "right": 0.098}
  }
}
```

没有该文件的数据由工具按旧版68 mm语义解释。恢复采集时，如果现有数据没有该文件而
当前配置为98 mm，程序会拒绝续采，以免同一数据集中出现两种不同的夹爪物理含义。

## 10. 推荐：使用 LeRobotDataset 加载

应优先使用项目环境中的 LeRobot 版本加载，而不是自行解析视频索引：

```python
from pathlib import Path

from lerobot.datasets.lerobot_dataset import LeRobotDataset


root = Path("/home/star/lerobot_data/final_data")

dataset = LeRobotDataset(
    repo_id=f"local/{root.name}",
    root=root,
)

print("episodes:", dataset.num_episodes)
print("frames:", dataset.num_frames)

sample = dataset[0]
for key, value in sample.items():
    shape = getattr(value, "shape", None)
    print(key, shape if shape is not None else type(value))
```

典型训练样本包含：

```text
observation.state
observation.state.endpose
observation.images.left.third_view
observation.images.left.wrist
observation.images.right.wrist
action
action.endpose
episode_index
frame_index
timestamp
task_index
```

具体键名以实际 loader 输出和 `meta/info.json` 为准。

## 11. 按名称解析 State 和 Action

下面的代码避免硬编码向量下标：

```python
from pathlib import Path
import json

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset


root = Path("/home/star/lerobot_data/final_data")
info = json.loads((root / "meta/info.json").read_text())

state_names = info["features"]["observation.state"]["names"]
action_names = info["features"]["action"]["names"]
endpose_names = info["features"]["action.endpose"]["names"]

dataset = LeRobotDataset(repo_id=f"local/{root.name}", root=root)
sample = dataset[0]

state = np.asarray(sample["observation.state"], dtype=np.float32)
action = np.asarray(sample["action"], dtype=np.float32)
action_endpose = np.asarray(sample["action.endpose"], dtype=np.float32)

state_by_name = dict(zip(state_names, state, strict=True))
action_by_name = dict(zip(action_names, action, strict=True))
endpose_by_name = dict(zip(endpose_names, action_endpose, strict=True))

left_joint_rad = np.array(
    [state_by_name[f"left.joint{i}.angle_rad"] for i in range(1, 7)],
    dtype=np.float32,
)

left_sent_joint_rad = np.array(
    [action_by_name[f"left.joint{i}.angle_rad"] for i in range(1, 7)],
    dtype=np.float32,
)
left_action_xyz_mm = np.array(
    [endpose_by_name[f"left.pose.{axis}"] for axis in "xyz"],
    dtype=np.float32,
)

print("left joint rad:", left_joint_rad)
print("left sent joint target rad:", left_sent_joint_rad)
print("left action xyz mm:", left_action_xyz_mm)
```

## 12. 直接读取 Parquet（仅用于分析/校验）

模型训练仍推荐使用 LeRobot loader。若只检查数值，可以直接读取所有物理分块：

```python
from pathlib import Path
import json

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq


root = Path("/home/star/lerobot_data/final_data")
info = json.loads((root / "meta/info.json").read_text())

files = sorted(root.glob("data/chunk-*/*.parquet"))
if not files:
    raise RuntimeError("No data parquet files found")

table = pa.concat_tables([pq.read_table(path) for path in files])
state = np.asarray(table["observation.state"].to_pylist(), dtype=np.float32)
state_endpose = np.asarray(
    table["observation.state.endpose"].to_pylist(), dtype=np.float32
)
action = np.asarray(table["action"].to_pylist(), dtype=np.float32)
action_endpose = np.asarray(table["action.endpose"].to_pylist(), dtype=np.float32)

state_names = info["features"]["observation.state"]["names"]
action_names = info["features"]["action"]["names"]

print("frames:", len(table))
print("state:", state.shape, state.dtype)
print("state endpose:", state_endpose.shape, state_endpose.dtype)
print("action:", action.shape, action.dtype)
print("action endpose:", action_endpose.shape, action_endpose.dtype)
print("state finite:", np.isfinite(state).all())
print("action finite:", np.isfinite(action).all())

assert state.shape[1] == len(state_names)
assert action.shape[1] == len(action_names)
assert state_endpose.shape[1] == 12
assert action_endpose.shape[1] == 12
assert np.isfinite(state).all()
assert np.isfinite(state_endpose).all()
assert np.isfinite(action).all()
assert np.isfinite(action_endpose).all()
```

直接读取 Parquet 不会自动解码 MP4，也不会自动处理训练用时间窗口，因此不能完全替代 LeRobot loader。

## 13. 训练数据组织建议

### 13.1 推荐输入

视觉策略通常可以使用：

```text
三路 RGB
+ 双臂关节角（12 维）
+ 双臂夹爪反馈（2 维）
```

也可以加入双臂实际末端位姿。关节角与末端位姿存在运动学相关性，是否同时输入应由模型设计和消融实验决定。

### 13.2 推荐监督目标

当前原生监督目标为：

```text
双臂绝对 joint action + gripper action，14 维
```

该 action 是采集时实际发送的目标。`action.endpose` 是同一周期有效 EEF 目标，可作为辅助监督。如果模型要求 delta action，可以在训练预处理时派生：

- 关节：`target_joint - current_joint`
- 平移：`target_xyz - current_xyz`（使用两个 endpose feature）
- 旋转：使用相对旋转 `R_state^T R_action`
- 夹爪：按模型定义使用绝对开合量或差值

不要修改原始数据集；应在训练代码中可复现地转换，并保存转换配置。

### 13.3 数据划分

必须按 episode 划分训练集和验证集，不要随机按帧划分。按帧随机划分会让同一条轨迹的相邻帧同时进入训练和验证，造成严重时间泄漏。

### 13.4 Action Chunk

如果模型一次预测未来 `H` 步动作：

- chunk 必须限制在当前 episode 内；
- episode 尾部应截断或 padding 并生成 mask；
- 不能把下一个 episode 的动作接到当前 episode 尾部；
- 图像、state 和 action 应依据相同 frame 索引采样。

### 13.5 归一化

建议分别处理不同物理量：

- XYZ：先统一 mm 或 m，再归一化；
- 轴角：保留 rad，或明确转换成 quaternion/rotation-6D；
- 关节角：rad；
- 夹爪：通常保留 `[0, 1]` 后再按模型需要归一化；
- 图像：按模型预训练规范归一化。

不要对位置、角度和夹爪使用同一个未经区分的物理缩放规则。

## 14. 训练前完整性检查清单

交给训练人员前建议逐项确认：

- [ ] `meta/info.json` 能正常解析。
- [ ] `total_episodes` 与预期保存轮数一致。
- [ ] `total_frames` 大于 0。
- [ ] `observation.state` 为 14 维，且只含双臂关节反馈和夹爪反馈。
- [ ] `observation.state.endpose` 为 12 维。
- [ ] `action` 为 14 维，且是实际发送的关节和夹爪目标。
- [ ] `action.endpose` 为 12 维。
- [ ] 四个低维 feature 均不包含 NaN 或 Inf。
- [ ] 12 个 `.angle_rad` 字段均存在且随动作合理变化。
- [ ] 三路视频特征均存在。
- [ ] 随机抽查每个视角，视频无黑屏、冻结、明显错位或损坏。
- [ ] 每个 episode 的 `frame_index` 从 0 开始并递增。
- [ ] 每个 episode 的 timestamp 单调递增。
- [ ] Episode 长度合理，过短轨迹已按项目标准处理。
- [ ] 使用 LeRobotDataset 能遍历首帧、随机帧和末帧。
- [ ] 训练/验证按 episode 划分。
- [ ] action chunk 不跨 episode。
- [ ] 训练配置记录了 XYZ 单位和旋转表示。

## 15. 已知边界与不能从本数据直接得到的信息

当前数据可直接提供：

- 三路 RGB；
- 双臂实际末端位姿；
- 双臂实际关节角；
- 双臂夹爪反馈；
- 双臂实际发送的关节和夹爪 action；
- 双臂控制路径实际采用的 EEF action；
- episode、帧、相对时间和任务索引。

当前数据不直接提供：

- 深度图或点云；
- 相机硬件同步时间戳；
- 关节速度、关节力矩或末端六维力；
- 原始 Vive/Pika 世界坐标轨迹；
- A 和 C 阶段轨迹；
- 左右机器人基座之间的统一世界外参。

如果训练模型依赖上述信息，不能从字段名推断或伪造，应重新采集或增加明确的离线派生步骤。

## 16. 数据集复制、备份和续采

- 移动或备份时复制整个 `dataset_root`，不要只复制 `data/` 或 `videos/`。
- 不要单独重命名物理分块。
- 不要手工合并 MP4 或 Parquet。
- 续采使用项目的 `--resume`，不要覆盖旧文件。
- resume 只能追加到相同四-feature schema 的数据集；旧 26/14 schema 会被兼容性检查拒绝。
- 续采后再次用 LeRobot loader 校验总 episode、总帧数和随机视频帧。

续采示例：

```bash
cd ~/Lerobot-Real-Cam
conda activate lerobot_real

bash scripts/collect_dual_pika_piper_dataset.sh \
  --resume /home/star/lerobot_data/final_data \
  --episodes 50
```

其中 `--episodes 50` 表示续采完成后的目标总 episode 数，不是额外再采 50 轮。

## 17. 交付训练人员时建议附带的信息

除完整数据集目录外，建议同时提供：

1. 本文档 `DATASET_README.md`；
2. 数据集根目录的准确名称和校验和；
3. 实际 episode 数量和总帧数；
4. 任务 B 的文字定义、成功标准和失败标准；
5. 相机安装位置示意图；
6. 训练/验证 episode 划分清单；
7. 所有训练预处理配置，包括单位转换、图像裁剪和 action 表示转换。

这样训练团队可以明确区分“原始采集事实”和“训练阶段派生表示”，避免因单位、旋转表示、时间对齐或 episode 边界理解错误而得到无效模型。

## 18. 将旧数据转换为 Joint/EEF 分离格式

旧数据没有保存 official IK 当时实际下发的 `sent_joints`，但已经保存了实际关节反馈、实际 EEF、EEF action、夹爪 action 和三路视频。因此可以按照训练方允许的后备规则，用 `state[t+1]` 构造代理关节 action：

```text
新 observation.state         = 旧数据的双臂关节反馈 + 夹爪反馈
新 observation.state.endpose = 旧数据的双臂实际 EEF
新 action                    = 下一帧双臂关节反馈 + 当前帧真实夹爪 action
新 action.endpose            = 旧数据的双臂 EEF action
```

转换脚本：

```text
scripts/convert_dual_pika_piper_dataset_schema.py
```

先只检查，不写任何文件：

```bash
cd ~/Lerobot-Real-Cam
conda activate lerobot_real

python scripts/convert_dual_pika_piper_dataset_schema.py \
  --source /home/star/lerobot_data/final_data \
  --output /home/star/lerobot_data/final_data_joint_eef \
  --dry-run
```

确认检查通过、磁盘空间充足后执行正式转换：

```bash
python scripts/convert_dual_pika_piper_dataset_schema.py \
  --source /home/star/lerobot_data/final_data \
  --output /home/star/lerobot_data/final_data_joint_eef
```

转换行为：

- 绝不修改源数据集；
- 输出路径必须不存在，防止覆盖已有数据；
- 通过 LeRobot loader 逐 episode 读取源数据；
- 三路 RGB 会解码并重新编码到新数据集；
- 每个 episode 的最后一帧没有 `state[t+1]`，因此不会写入转换数据集；
- 输出总帧数应为 `源总帧数 - episode 数量`；
- episode 数量、任务文本和 30 Hz 采样率保持不变；
- 自动重新生成 Parquet、MP4、`info.json`、episode metadata 和统计值；
- 输出根目录会增加 `CONVERSION_INFO.json`，明确关节 action 是下一状态代理值，不是真实历史关节命令；
- 转换完成后脚本会重新通过 LeRobot 加载输出并检查 episode 数、帧数和四个低维 feature 的 shape。

默认视频编码为 H.264。可以显式设置：

```bash
python scripts/convert_dual_pika_piper_dataset_schema.py \
  --source /home/star/lerobot_data/final_data \
  --output /home/star/lerobot_data/final_data_joint_eef \
  --vcodec h264 \
  --image-writer-threads 8
```

转换需要完整解码并重新编码三路视频，因此耗时较长，并且转换期间需要足够空间同时保存源数据和新数据。不要在正式采集期间运行转换，以免争用 CPU、磁盘和视频编码资源。
