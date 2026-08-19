# 双 Pika–双 PiperX 新数据集 Schema 确认稿

> 状态：训练组已确认，采集程序已按本格式实现。正式采集前仍需完成本文末尾的冒烟验证。

本文记录训练组已经确认的下一批双臂遥操作数据字段、单位、动作语义与时间对齐方式。采集端严格按照本文写入新数据；旧数据不纳入这批正式训练数据。

## 1. 采集系统

| 项目 | 配置 |
|---|---|
| 机器人 | 左、右两台 PiperX |
| 遥操作设备 | 左、右两套 Pika Sense + Vive Tracker |
| 相机 | 三台 Intel RealSense D435i |
| 数据格式 | LeRobot Dataset v3 |
| 数据采样率 | 30 Hz |
| 图像 | RGB，640×480，30 FPS |
| 深度/点云/IMU | 本批数据不保存 |
| Episode 内容 | 总任务 A+B+C 中的 B 阶段 |

相机角色保持如下：

| Feature | 物理视角 | D435i 序列号 |
|---|---|---|
| `observation.images.left.third_view` | 第三人称视角 | `346122070530` |
| `observation.images.left.wrist` | 左腕视角 | `233522077815` |
| `observation.images.right.wrist` | 右腕视角 | `250122077305` |

`left.third_view` 中的 `left` 只是当前软件中相机所属机器人对象的前缀；该相机的物理含义仍然是第三人称视角。

## 2. 拟采用的 Feature Schema

每帧包含四个低维机器人 feature 和三路 RGB：

| Feature | Dtype | Shape | 内容 |
|---|---|---:|---|
| `observation.state` | `float32` | `[14]` | 双臂实际关节角 + 实际夹爪反馈 |
| `observation.state.endpose` | `float32` | `[12]` | 双臂实际 EEF 位姿 |
| `action` | `float32` | `[14]` | 双臂实际发送的关节目标 + 夹爪目标 |
| `action.endpose` | `float32` | `[12]` | 双臂实际使用的 EEF 目标 |
| `observation.images.left.third_view` | `video` | `[480,640,3]` | 第三视角 RGB |
| `observation.images.left.wrist` | `video` | `[480,640,3]` | 左腕 RGB |
| `observation.images.right.wrist` | `video` | `[480,640,3]` | 右腕 RGB |

另外保留 LeRobot 自动生成的：

```text
episode_index
frame_index
timestamp
index
task_index
```

## 3. `observation.state`：实际关节状态

Shape：`[14]`

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

字段语义：

- `joint*.angle_rad`：PiperX 在本控制周期开始时反馈的实际关节角，单位 rad。
- `gripper.pos`：本控制周期开始时反馈的实际夹爪开合量，归一化约为 `[0,1]`。
- 约 `0` 表示闭合，约 `1` 表示最大张开。现场两台 Piper 使用 100 mm
  大夹爪，正式配置保留 2 mm 端点余量，因此新数据对应约 98 mm 有效行程。
- 新数据在 `meta/robot_hardware.json` 中记录左右夹爪的实际归一化范围。
  没有该文件的历史数据按旧版 68 mm语义解释，不能直接和98 mm新数据混用。

该 feature 不再混入 EEF 位姿。

## 4. `observation.state.endpose`：实际 EEF 状态

Shape：`[12]`

```text
0   left.pose.x
1   left.pose.y
2   left.pose.z
3   left.pose.rx
4   left.pose.ry
5   left.pose.rz

6   right.pose.x
7   right.pose.y
8   right.pose.z
9   right.pose.rx
10  right.pose.ry
11  right.pose.rz
```

字段语义：

- `pose.x/y/z`：PiperX 反馈的实际末端位置，单位 mm。
- 左右臂分别使用各自的机器人基座坐标系。
- `pose.rx/ry/rz`：轴角向量（rotation vector），单位 rad，不是三个独立的 RPY 欧拉角。
- 夹爪不在该 feature 中重复；实际夹爪反馈只保存在 `observation.state`。

如果训练端需要统一为米，应在预处理时同时将 state 和 action 的 XYZ 除以 1000，原始数据仍保留 mm。

## 5. `action`：实际发送的关节目标

Shape：`[14]`

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

关节 action 的来源不是 `state[t+1]`，而是当前控制周期 official IK 求解后实际发送给 PiperX 的关节目标：

```text
Pika EEF 目标
    ↓
坐标转换和工作空间限制
    ↓
official IK
    ↓
本周期最终 sent_joints
    ↓
PiperX JointCtrl
```

保存语义：

- 六个关节值：本周期最终实际发送的 `sent_joints`，单位 rad。
- 夹爪值：本周期实际发送的夹爪目标，归一化约为 `[0,1]`。
- 如果一个控制周期内部发生安全插值，保存该周期最终发送的关节目标。
- 如果 IK 无解或触发保持，保存本周期真正重新发送/保持的上一有效关节目标。

本系统的软件 IK 路径可以直接获得实际发送的关节目标，因此新数据不需要用下一帧反馈 `state[t+1]` 作为代理 action。

## 6. `action.endpose`：实际使用的 EEF 目标

Shape：`[12]`

```text
0   left.pose.x
1   left.pose.y
2   left.pose.z
3   left.pose.rx
4   left.pose.ry
5   left.pose.rz

6   right.pose.x
7   right.pose.y
8   right.pose.z
9   right.pose.rx
10  right.pose.ry
11  right.pose.rz
```

字段语义：

- `pose.x/y/z`：本周期用于控制的 EEF 目标，单位 mm。
- `pose.rx/ry/rz`：本周期用于控制的目标轴角向量，单位 rad。
- 保存经过坐标转换、工作空间限制以及控制路径接受后的有效目标。
- 如果 IK 无解并保持上一有效命令，保存相应的保持 EEF 目标，而不是不可执行的原始 Pika 输入。
- 夹爪目标只保存在主 `action` 中，不在 `action.endpose` 中重复。

## 7. State 与 Action 的时间关系

每个控制周期拟按以下顺序执行：

```text
1. 读取 Piper 实际反馈
   ├── observation.state[t]
   └── observation.state.endpose[t]

2. 读取 Pika 并生成本周期 EEF 目标

3. official IK 求解并发送
   ├── action[t]
   └── action.endpose[t]

4. 三路 RGB 与以上低维数据写入同一 frame[t]
```

因此同一行表示：

```text
frame[t] = {
    当前实际关节状态,
    当前实际 EEF 状态,
    本周期发送的关节目标,
    本周期使用的 EEF 目标,
    三路 RGB
}
```

原始数据不会执行以下处理：

```text
不把 action 人工移动一帧
不把 action 保存为 action - state
不使用 state[t+1] 代替新采数据的真实 sent_joints
```

物理因果关系仍然是：

```text
observation[t] → action[t] → observation[t+1 ...]
```

由于机械臂动力学和控制延迟，`action[t]` 的效果可能在后续一帧或多帧逐步出现在 observation 中。

## 8. Episode 定义

当前总任务由 A+B+C 组成，本批数据只正式记录 B。

每轮流程保持：

```text
PREP：人为恢复到 A 的结束状态，不记录
RECORD B：正式记录 B
REVIEW：决定保存、丢弃重做或退出，不记录
```

已保存 episode：

- 只包含 RECORD B；
- 可在最长时长前提前结束，因此 episode 允许变长；
- 不包含 PREP、等待和 REVIEW；
- `r` 丢弃的轨迹不保存；
- `q` 退出时未确认的当前轨迹不保存；
- resume 只向同一 schema 的数据集追加完整 episode。

训练切分必须按 episode 进行，action chunk 不得跨 episode。

## 9. 建议训练使用方式

主行为克隆路径：

```text
输入：三路 RGB + observation.state
输出：action
```

也就是：

```text
images[t] + joint_state[t]
    ↓
policy
    ↓
joint_target[t] + gripper_target[t]
```

额外 EEF feature：

```text
observation.state.endpose
action.endpose
```

可用于：

- EEF 辅助监督；
- 运动学一致性损失；
- 3D 空间建模；
- EEF 可视化和数据质量检查。

LeRobot 只负责保存这些 feature；训练代码需要显式配置是否读取 `.endpose`，不会仅凭字段存在就自动用于模型。

## 10. 部署语义

如果模型使用主输出：

```text
action = joint targets + gripper targets
```

部署端应按关节控制路径发送，不再对这些关节角执行 IK。

如果模型输出：

```text
action.endpose = EEF target
```

部署端应继续使用：

```text
EEF target → official IK → joint target → PiperX
```

两种 action 不能发送到错误的控制接口。

## 11. 保持不变的部分

本次 schema 修改不改变：

- Pika、Tracker、Piper 和相机左右绑定；
- Vive 标定方式；
- Pika 激活手势；
- 现有双臂摇操映射；
- official IK；
- 工作空间和关节安全限制；
- Piper 速度、控制频率和急停要求；
- 三路 RGB 分辨率与帧率；
- PREP/RECORD/REVIEW 操作流程；
- 提前结束、`r` 重采和 `q` 退出；
- 变长 episode；
- resume；
- LeRobot Parquet、MP4 和 metadata 分块方式。

本次只修改低维 feature 的组织方式，并记录此前没有写入数据集的真实 joint action。

## 12. 拟生成的 `meta/info.json` 核心结构

以下仅展示需要训练组确认的低维部分：

```json
{
  "features": {
    "observation.state": {
      "dtype": "float32",
      "shape": [14],
      "names": [
        "left.joint1.angle_rad",
        "left.joint2.angle_rad",
        "left.joint3.angle_rad",
        "left.joint4.angle_rad",
        "left.joint5.angle_rad",
        "left.joint6.angle_rad",
        "left.gripper.pos",
        "right.joint1.angle_rad",
        "right.joint2.angle_rad",
        "right.joint3.angle_rad",
        "right.joint4.angle_rad",
        "right.joint5.angle_rad",
        "right.joint6.angle_rad",
        "right.gripper.pos"
      ]
    },
    "observation.state.endpose": {
      "dtype": "float32",
      "shape": [12],
      "names": [
        "left.pose.x",
        "left.pose.y",
        "left.pose.z",
        "left.pose.rx",
        "left.pose.ry",
        "left.pose.rz",
        "right.pose.x",
        "right.pose.y",
        "right.pose.z",
        "right.pose.rx",
        "right.pose.ry",
        "right.pose.rz"
      ]
    },
    "action": {
      "dtype": "float32",
      "shape": [14],
      "names": [
        "left.joint1.angle_rad",
        "left.joint2.angle_rad",
        "left.joint3.angle_rad",
        "left.joint4.angle_rad",
        "left.joint5.angle_rad",
        "left.joint6.angle_rad",
        "left.gripper.pos",
        "right.joint1.angle_rad",
        "right.joint2.angle_rad",
        "right.joint3.angle_rad",
        "right.joint4.angle_rad",
        "right.joint5.angle_rad",
        "right.joint6.angle_rad",
        "right.gripper.pos"
      ]
    },
    "action.endpose": {
      "dtype": "float32",
      "shape": [12],
      "names": [
        "left.pose.x",
        "left.pose.y",
        "left.pose.z",
        "left.pose.rx",
        "left.pose.ry",
        "left.pose.rz",
        "right.pose.x",
        "right.pose.y",
        "right.pose.z",
        "right.pose.rx",
        "right.pose.ry",
        "right.pose.rz"
      ]
    }
  }
}
```

## 13. 请训练组确认

请对以下项目逐项回复“OK”或给出修改值：

| # | 待确认项 | 当前提案 |
|---:|---|---|
| 1 | 关节反馈 feature 名 | `observation.state` |
| 2 | 实际 EEF feature 名 | `observation.state.endpose` |
| 3 | 关节动作 feature 名 | `action` |
| 4 | EEF 动作 feature 名 | `action.endpose` |
| 5 | `observation.state` 内容 | 12 个关节角 + 2 个夹爪反馈，共 14 维 |
| 6 | `observation.state.endpose` 内容 | 双臂 12 维 EEF，不重复夹爪 |
| 7 | `action` 内容 | 12 个真实 sent joint targets + 2 个夹爪目标，共 14 维 |
| 8 | `action.endpose` 内容 | 双臂 12 维有效 EEF 目标，不重复夹爪 |
| 9 | 关节 action 来源 | 本周期 official IK 实际发送的 `sent_joints`，不用 `state[t+1]` |
| 10 | 时间对齐 | `observation[t]` 与本周期 `action[t]` 同一行，不人工错位 |
| 11 | XYZ 单位 | 原始保存 mm |
| 12 | 姿态格式 | 轴角向量，rad |
| 13 | 关节角单位 | rad |
| 14 | 图像 | 三路 RGB，640×480@30 FPS，不保存深度 |
| 15 | Episode | 只记录 B，允许变长，PREP/REVIEW 不记录 |
| 16 | 主训练目标 | 默认 `observation.state + RGB → action` |
| 17 | `.endpose` 用途 | 由训练代码显式选择作为输入、辅助监督或分析字段 |

只有以上 schema 得到确认后，才修改采集程序并进行新的正式采集。
