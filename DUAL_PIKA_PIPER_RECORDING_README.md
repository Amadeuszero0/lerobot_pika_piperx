# 双 Pika–双 PiperX 摇操数据采集使用手册

本文档介绍本项目中双 Pika Sense 遥操作双 PiperX，并通过三台 Intel RealSense D435i 采集 LeRobot 格式数据的完整使用方法。

Vive Tracker 和 Lighthouse 的重新标定流程参见：

```text
VIVE_CALIBRATION_README.md
```

## 1. 当前硬件绑定

本机配置使用固定设备名和硬件序列号，不依赖可能在重启后变化的 `/dev/videoN`。

| 角色 | 设备 |
|---|---|
| 左 Pika | `/dev/pika_left`，Tracker `LHR-818D4A5D` |
| 右 Pika | `/dev/pika_right`，Tracker `LHR-52C31F65` |
| 左 PiperX | `can_left` |
| 右 PiperX | `can_right` |
| 第三视角 D435i | `346122070530` |
| 左腕 D435i | `233522077815` |
| 右腕 D435i | `250122077305` |

相机绑定文件：

```text
config/piper/d435i_roles_local.yaml
```

双臂摇操基础配置：

```text
config/piper/dual_pika_piper_local.yaml
```

## 2. 采集内容

每个 episode 保存：

- 左、右 PiperX 的实际关节角度
- 左、右 PiperX 的机械臂观测状态
- 左、右 Pika 生成并发送给机械臂的动作
- 第三视角 D435i RGB 视频
- 左腕 D435i RGB 视频
- 右腕 D435i RGB 视频
- Episode、frame、timestamp 和 task 等 LeRobot 元数据

默认采集参数：

- 50 个 episode
- 每个 B episode 默认最长 30 秒，可提前结束并在审核界面决定是否保存
- RGB 分辨率 640 × 480
- RGB 帧率 30 FPS
- Piper 速度 40%
- 数据目录 `/home/star/lerobot_data`

## 3. 采集前准备

进入项目和 Conda 环境：

```bash
cd ~/Lerobot-Real-Cam
conda activate lerobot_real
```

确认：

1. 两台 PiperX 工作空间内没有人员和障碍物，急停可随时操作。
2. 两个 CAN 接口名称分别为 `can_left`、`can_right`，且均为 `UP`。
3. 两个 Pika 设备路径分别为 `/dev/pika_left`、`/dev/pika_right`。
4. 三台 D435i 均已连接并协商到 USB 3.x。
5. 两个 Vive 基站位置固定，两个 Tracker 在工作区域内具有稳定视野。
6. 没有其他程序占用相机、Pika 或 Piper。

快速查看 CAN：

```bash
ip -brief link show type can
```

快速查看 D435i：

```bash
python - <<'PY'
import pyrealsense2 as rs

for device in rs.context().query_devices():
    name = device.get_info(rs.camera_info.name)
    if "D435I" not in name.upper():
        continue
    print(
        device.get_info(rs.camera_info.serial_number),
        "USB",
        device.get_info(rs.camera_info.usb_type_descriptor),
        device.get_info(rs.camera_info.physical_port),
    )
PY
```

正常情况下应看到三个已绑定序列号，并且全部显示 `USB 3.2`。

## 4. 常用启动方式

### 4.1 正式采集：50 轮，每轮最长 30 秒

```bash
cd ~/Lerobot-Real-Cam
conda activate lerobot_real

bash scripts/collect_dual_pika_piper_dataset.sh
```

该命令会创建带时间戳的新数据集，例如：

```text
/home/star/lerobot_data/dual_pika_piper_dataset_20260815_103000
```

所有检查通过后，终端要求确认时输入：

```text
RECORD
```

### 4.2 练手：5 轮，每轮最长 30 秒

```bash
bash scripts/practice_dual_pika_piper_recording.sh
```

### 4.3 单轮低速冒烟测试

该模式采集 1 轮、10 秒、速度 10%，适合正式采集前验证完整流程：

```bash
bash scripts/run_dual_pika_piper_record.sh --smoke
```

### 4.4 只检查设备，不发送机械臂动作

```bash
bash scripts/run_dual_pika_piper_record.sh --check
```

该检查会验证相机、Pika、Piper 和运行配置，不会发送机械臂运动命令。

### 4.5 只测试双臂摇操，不保存数据

先做只读检查：

```bash
bash scripts/run_dual_pika_piper.sh --check
```

检查通过后启动纯摇操：

```bash
bash scripts/run_dual_pika_piper.sh --run
```

## 5. 自定义采集参数

正式包装脚本的完整参数会继续传递给底层启动脚本，因此可以直接在命令末尾添加参数。

查看帮助：

```bash
bash scripts/start_dual_pika_piper_recording.sh --help
```

### 参数说明

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `--session NAME` | `dual_pika_piper` | 新数据集目录的名称前缀。正式包装脚本默认使用 `dual_pika_piper_dataset`。只允许字母、数字、点、下划线和连字符。|
| `--task TEXT` | `Bimanual Pika to Piper teleoperation` | 写入 LeRobot 数据集的任务文本。相同训练任务建议始终使用相同描述。|
| `--episodes N` | `50` | 新建数据时表示本次采集轮数；续采时表示完成后的目标总轮数。|
| `--episode-seconds N` | `30` | B 正式记录的最长时间，支持正整数或小数；PREP 恢复阶段不计入。|
| `--timed-episodes` | 默认启用 | 使用最长时长：可提前结束，到达 `--episode-seconds` 后自动进入审核。|
| `--manual-episodes` | 关闭 | 取消 B 正式记录的最长时长，必须输入 `Enter`/`s`、`r` 或 `q` 才能结束当前 episode。|
| `--interactive-reset` | 默认启用 | 每轮 B 正式记录前，先开启不写数据的摇操阶段，用于人工恢复到 A 的结束状态。|
| `--no-interactive-reset` | 关闭 | 跳过 A 结束状态恢复阶段，直接进入 B 正式记录。|
| `--reset-seconds N` | `20` | 写入 LeRobot 录制配置的重置时间。当前终端流程主要通过人工确认、双臂回初始位姿和 Pika 手势控制换轮，并不是简单地自动等待该时长。|
| `--speed N` | `40` | Piper 运动速度百分比，允许 `1–100`。提高速度会增加运动风险，不建议未经低速测试直接提高。|
| `--dataset-base PATH` | `/home/star/lerobot_data` | 新数据集的父目录。续采时由 `--resume` 指定的旧数据集目录决定。|
| `--preflight-seconds N` | `10` | 正式开始前，同时打开三路 RGB 流进行稳定性测试的时长。|
| `--resume PATH` | 无 | 续采指定的现有 LeRobot 数据集，而不是创建新目录。|
| `-h`、`--help` | — | 显示帮助。|

### 常用示例

采集 50 轮，每轮最长 20 秒：

```bash
bash scripts/collect_dual_pika_piper_dataset.sh \
  --episodes 50 \
  --episode-seconds 20
```

采集 10 轮，每轮最长 15 秒，速度 20%：

```bash
bash scripts/collect_dual_pika_piper_dataset.sh \
  --episodes 10 \
  --episode-seconds 15 \
  --speed 20
```

自定义数据集名称和任务：

```bash
bash scripts/collect_dual_pika_piper_dataset.sh \
  --session bimanual_pick_cube \
  --task "Pick up and place the cube with both arms" \
  --episodes 50 \
  --episode-seconds 30
```

自定义数据保存位置：

```bash
bash scripts/collect_dual_pika_piper_dataset.sh \
  --dataset-base /mnt/robot_data \
  --episodes 50
```

包装脚本先提供默认值，再附加用户输入的参数，因此命令行中最后提供的 `--episodes`、`--episode-seconds` 等参数会覆盖默认值。

## 6. 续采已有数据集

如果采集因为程序异常、设备断开或人工退出而中止，可以继续向原数据集追加 episode：

```bash
bash scripts/collect_dual_pika_piper_dataset.sh \
  --resume /home/star/lerobot_data/原数据集目录 \
  --episodes 50 \
  --episode-seconds 30
```

续采确认时输入：

```text
RESUME
```

续采模式下：

- `--episodes 50` 表示数据集完成后总共应有 50 个 episode。
- 如果旧数据集已有 18 个完整 episode，本次只会追加 32 个。
- 已完整保存的 Parquet 和 MP4 不会被覆盖。
- 异常中止时尚未完整保存的当前 episode 不计入总数，下次重新采这一轮。
- 脚本会在机械臂动作前检查旧数据集能否被 LeRobot 正常加载。
- 如果旧数据集已经达到或超过目标轮数，脚本会拒绝继续追加。

续采时建议保持原数据集的 `--task`、`--episode-seconds` 和采集硬件配置一致。

## 7. 每轮采集流程

当前任务采集的是组合任务 A→B→C 中的 B。每个 episode 的正常流程如下：

1. 左 PiperX 自动回到配置中的左臂初始 TCP 位姿。
2. 右 PiperX 自动回到配置中的右臂初始 TCP 位姿。
3. 两个 Pika 分别提示等待完整的“夹紧—张开—夹紧”手势。
4. 两侧手势均完成后进入 PREP 恢复阶段：摇操有效，但不写入任何 episode 数据。
5. 人工把双臂、物体和场景恢复到 A 的结束动作和位姿。
6. 确认状态正确后按 `Enter`，从这一刻开始正式记录 B。
7. 输入 `Enter` 或 `s` + `Enter` 提前结束，或达到 `--episode-seconds` 的最长时间，本轮摇操暂停并进入审核。
8. 输入 `s` 保存；输入 `r` 丢弃并重新进入 PREP；输入 `q` 丢弃并退出。
9. 保存时程序同步写入三路 MP4、Parquet 和 episode 元数据，保存期间不会开始下一轮摇操。
10. 保存完成后按 `Enter`，进入下一轮的初始位姿、Pika 手势和 PREP 恢复阶段。

PREP 阶段没有时间限制，也不会进入训练数据。B 的正式记录时长可以不同，但默认不会超过 `--episode-seconds`。需要取消正式记录时限时添加 `--manual-episodes`。

Pika 激活手势必须完整完成，不要在机械臂回初始位置时提前移动或夹动 Pika。

## 8. 终端快捷操作

### PREP：恢复 A 的结束状态（不记录）

| 输入 | 作用 |
|---|---|
| `Enter` | 确认 A 的结束状态已经恢复，开始正式记录 B。|
| `q` + `Enter` | 不生成当前 episode，保留以前的数据并退出。|

只有两侧 Pika 激活手势都完成后，程序才接受开始记录的 `Enter`。如果过早按下，会提示先完成手势和恢复。

### B 正在正式录制时

| 输入 | 作用 |
|---|---|
| `Enter` 或 `s` + `Enter` | 提前结束当前 episode，进入审核。|
| `r` + `Enter` | 立即丢弃当前 episode，并回到 PREP 重新恢复 A 的结束状态。|
| `q` + `Enter` | 丢弃当前 episode，保留以前已保存的数据并安全退出。|

定时模式下如果没有输入命令，到达最长时长后也会进入审核，不会自动保存。

### Episode 审核

```text
Episode finished: [s] save  [r] discard and restore A-end state again  [q] discard and quit >>>
```

| 输入 | 作用 |
|---|---|
| `s` | 同步保存当前 episode。|
| `r` | 丢弃当前 episode，并回到 PREP 重新恢复 A 的结束状态。|
| `q` | 丢弃当前 episode，保留以前的数据并退出。|

保存采用与 LeRobot 官方相同的同步顺序。出现 MP4 编码输出时应等待保存完成，不要在此时操作下一轮。

### 两轮之间

终端显示：

```text
Press Enter to prepare the next episode, or q to stop >>>
```

| 输入 | 作用 |
|---|---|
| `Enter` | 双臂回初始位姿、等待 Pika 手势，然后开始下一轮。|
| `q` | 停止本次采集。|

遇到动作失误时应使用 `r` 丢弃重录，不要保存明显错误的 demonstration。发生危险时优先使用机械急停，不要只依赖终端输入。

## 9. 启动脚本自动进行的检查

正式启动脚本依次执行：

1. 检查 Conda 环境中的 Python 和项目命令是否存在。
2. 检查 `/dev/pika_left`、`/dev/pika_right`。
3. 检查 `can_left`、`can_right` 是否存在且为 `UP`。
4. 按序列号检查三台绑定的 D435i。
5. 要求三台 D435i 均协商到 USB 3.x。
6. 同时打开三路 640 × 480、30 FPS RGB 流进行预检。
7. 基于基础配置生成本次专用录制配置。
8. 验证 Piper 和 LeRobot 配置。
9. 对 Pika 和 Piper 执行只读硬件检查。
10. 显示最终数据目录、轮数、时长和速度，等待 `RECORD` 或 `RESUME` 确认。

预检失败时不会启动正式机械臂运动。

## 10. 数据保存位置和文件分块

新建数据集默认保存到：

```text
/home/star/lerobot_data/<session>_YYYYMMDD_HHMMSS
```

数据集包含：

```text
data/       # 状态、动作和逐帧索引 Parquet
meta/       # info、stats、tasks、episode 元数据和 robot_hardware.json
videos/     # 三路相机 MP4
images/     # 录制/编码过程中的临时图像
```

现场左右 Piper 均为100 mm大夹爪，正式配置使用98 mm安全行程。Pika 的实测0～98 mm
输入会归一化为 `gripper.pos=0～1`。每个新数据集的
`meta/robot_hardware.json` 会记录左右夹爪的实际物理行程，供训练、恢复采集和回放校验。
没有该文件的历史数据按旧版68 mm范围解释，不能直接和98 mm新数据混合。

续采后可能出现：

```text
file-000.parquet
file-001.parquet

file-000.mp4
file-001.mp4
```

这是正常的 LeRobot 物理分块。它们通过 `meta/episodes/` 组成一个逻辑数据集，不需要手动合并。

更详细的数据集目录说明参见项目根目录：

```text
DATASET_README.md
```

## 11. 数据集完成后验证

将路径替换为实际数据集目录：

```bash
python - <<'PY'
from pathlib import Path

from lerobot.datasets.lerobot_dataset import LeRobotDataset


root = Path("/home/star/lerobot_data/your_dataset")
dataset = LeRobotDataset(
    repo_id=f"local/{root.name}",
    root=root,
)

print("episodes:", dataset.num_episodes)
print("frames:", dataset.num_frames)
print("episode metadata rows:", len(dataset.meta.episodes))
print(dataset)
PY
```

对于完整的 50 轮数据集，应满足：

```text
episodes: 50
episode metadata rows: 50
```

## 12. 常见问题

### 提示某个 D435i 未连接

例如：

```text
FAIL: bound D435i cameras are not connected: 346122070530
```

表示序列号为 `346122070530` 的第三视角相机没有被 librealsense 枚举。检查相机、USB 线和对应 USB 3 端口，不要随意修改绑定序列号。

### 相机显示 USB 2.1

当前配置设置了 `require_usb3: true`，因此预检会拒绝 USB 2.x 相机。检查 USB 3 线材、接头和扩展卡端口，目标是三台 D435i 均显示 `USB 3.2`。

### CAN 接口存在但没有数据

`UP` 只表示 Linux 网络接口已启用，不代表 Piper 一定在发送反馈。需要结合 `candump`、Piper 主从/从臂模式、电源和 CAN 链路继续检查。

### 保存时出现大量 SVT-AV1 输出

三路 RGB 正在编码为 MP4，属于正常现象。等待终端出现：

```text
[Finish] Save episode N
```

再准备下一轮，不要在编码过程中强制关闭进程。

### Resume 提示缺少 `meta/episodes`

这通常表示上次进程在 LeRobot v3 将缓存的 episode 元数据和 Parquet footer
写回磁盘之前被终止。先运行只读检查：

```bash
python scripts/repair_lerobot_dataset.py \
    /home/star/lerobot_data/数据集目录
```

脚本会分别核对数据 Parquet 和每路 MP4，只保留从 episode 0 开始、数据和所有
相机视频均完整的连续前缀。确认计划后再执行：

```bash
python scripts/repair_lerobot_dataset.py \
    /home/star/lerobot_data/数据集目录 \
    --apply
```

修改前的元数据及被裁剪的尾部文件会备份到数据集内的
`.repair_backup_时间戳/`。修复成功后再用正式命令的 `--resume` 继续采集。
不要手动创建一个空的 `meta/episodes` 目录；空目录无法恢复 episode 索引、视频
时间范围和统计字段。

### 摇操突然持续抖动

先丢弃当前 episode，并用纯摇操模式复现：

```bash
bash scripts/run_dual_pika_piper.sh --run
```

如果纯摇操仍抖，检查 Tracker 遮挡、基站是否移动，以及终端是否持续出现 `MPFIT`、`Global solve` 或参考 Lighthouse 切换。不要将明显抖动的数据保存为训练 demonstration。

## 13. 高级环境变量

通常不需要设置。需要临时切换配置时可以使用：

| 环境变量 | 作用 |
|---|---|
| `PIPER_CONFIG_PATH` | 替换默认双臂基础配置文件。|
| `D435I_BINDINGS_PATH` | 替换默认 D435i 角色绑定文件。|
| `PIPER_RECORD_CONFIG_PATH` | 替换自动生成的本次录制配置输出路径。|

示例：

```bash
PIPER_CONFIG_PATH=/path/to/custom_dual_piper.yaml \
D435I_BINDINGS_PATH=/path/to/custom_camera_roles.yaml \
bash scripts/collect_dual_pika_piper_dataset.sh --episodes 10
```

基础配置和绑定配置涉及机械臂方向、工作空间、初始位姿、Tracker 身份及相机角色。修改后必须先运行只读检查和低速冒烟测试。
