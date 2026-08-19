# 双 PiperX 数据集回放说明

本项目可以把新格式数据集中的某一个 episode 回放到左右 PiperX。回放读取：

```text
action = 左 6 关节目标(rad) + 左夹爪 + 右 6 关节目标(rad) + 右夹爪
```

它不会读取视频，不使用 Pika/Vive，不依赖基站标定，也不会重新进行 IK。`action.endpose`
只用于训练和检查，不作为默认回放命令。

新数据集会在 `meta/robot_hardware.json` 中记录夹爪物理行程。现场100 mm大夹爪使用
98 mm安全范围；没有该元数据文件的旧数据自动按历史68 mm范围回放，防止旧动作被错误
放大到98 mm。

> 真机回放有碰撞风险。第一次测试应清空工作区、准备急停，并使用 `0.25x` 或 `0.5x`。

## 1. 先做离线预检

预检不会连接 CAN，也不会发送机械臂命令：

```bash
cd ~/Lerobot-Real-Cam

python scripts/replay_dual_piper_dataset.py \
  /home/star/lerobot_data/你的数据集目录 \
  --episode 0
```

预检会检查：

- 数据集确实使用当前 14 维关节 `action` schema；
- episode 存在且帧号连续；
- 所有数值有限；
- 六个关节没有超过 PiperX 机械范围；
- 夹爪目标位于 `[0,1]`；
- 相邻帧没有异常关节或夹爪跳变。

最后必须出现：

```text
dataset_preflight=PASS
robot_commands_sent=false
```

## 2. 把机械臂恢复到该 episode 的起点

回放程序不会自动从任意姿态拉到第一帧，因为未知环境中的关节插值可能碰撞。应先用原有
A-end-state preparation 流程，把双臂恢复到该 episode 开始时的 A 结束状态。

默认要求每个关节距离第一帧不超过 `3°`，夹爪宽度误差不超过 `0.015 m`。超出时程序会在
发送第一条回放命令之前退出。

## 3. 第一次真机回放

```bash
python scripts/replay_dual_piper_dataset.py \
  /home/star/lerobot_data/你的数据集目录 \
  --episode 0 \
  --rate 0.25 \
  --speed-percent 10 \
  --execute
```

程序连接 `can_left` 和 `can_right`、检查反馈及起始姿态，然后要求输入：

```text
REPLAY 0
```

只有完全匹配才会开始运动。回放中输入 `q` 后按 Enter，或按 `Ctrl+C`，程序会停止继续读取
数据，并尝试让双臂保持在当前反馈位置。

验证轨迹和环境没有问题后，可改为原速：

```bash
python scripts/replay_dual_piper_dataset.py \
  /home/star/lerobot_data/你的数据集目录 \
  --episode 0 \
  --rate 1.0 \
  --speed-percent 10 \
  --execute
```

## 4. 常用参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `--episode` | 必填 | 逻辑 episode 编号，从 0 开始 |
| `--execute` | 关闭 | 加上后才会连接 CAN 并运动 |
| `--left-can` | `can_left` | 左 PiperX CAN 接口 |
| `--right-can` | `can_right` | 右 PiperX CAN 接口 |
| `--rate` | `0.5` | 时间倍速，只允许 `(0,1]`，不允许超速回放 |
| `--speed-percent` | `10` | Piper JointCtrl 速度百分比 |
| `--max-start-joint-error-deg` | `3.0` | 当前姿态与第一帧的最大允许单关节误差 |
| `--max-frame-joint-step-deg` | `5.0` | 数据中相邻帧的最大允许关节跳变量 |
| `--configure-role` | 关闭 | 显式发送 Piper follower/slave 模式；正常情况下不需要 |
| `--disable-on-exit` | 关闭 | 退出时失能双臂；机械臂无支撑时不要随意打开 |

如果当前主机的 CAN 命名变化，例如右臂临时为 `can0`：

```bash
python scripts/replay_dual_piper_dataset.py DATASET_ROOT \
  --episode 0 \
  --left-can can_left \
  --right-can can0
```

先不要通过增大安全阈值来绕过报错。应先确认数据、左右臂绑定和当前起始姿态是否正确。
