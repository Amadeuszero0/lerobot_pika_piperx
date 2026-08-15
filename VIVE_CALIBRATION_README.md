# Pika / Vive Tracker 基站标定说明

本文档介绍双 Pika Sense 摇操系统所使用的 Vive Tracker 和 Lighthouse 基站标定流程。

## 1. 标定的作用

Vive 标定主要用于确定两台 Lighthouse 基站之间的相对位置和姿态，使 libsurvive 能够在统一追踪坐标系中稳定输出两个 Tracker 的位姿。

本标定不会修改：

- D435i 相机序列号和角色绑定
- PiperX 的 CAN 接口绑定
- PiperX 的关节零位
- Pika Tracker 到机械臂末端的固定安装变换
- PiperX 的初始 TCP 位姿

这些参数分别保存在项目的相机绑定和双臂基础配置中。

## 2. 什么时候需要重新标定

以下情况需要重新标定：

- 任意一台 Lighthouse 基站发生移动、转动或重新安装。
- 基站支架松动，无法确认位置是否保持不变。
- 更换了基站或改变了 Lighthouse 通道。
- Tracker 位姿持续漂移、跳变，且已经排除遮挡、反光和 USB 问题。
- 摇操过程中持续出现新的 `MPFIT`、`Global solve`，并伴随机械臂明显抖动。
- 当前 libsurvive 配置文件缺失、损坏或不能被正常加载。

以下情况通常不需要重新标定：

- D435i 相机更换 USB 端口。
- Piper CAN 口重命名或重新激活。
- 仅仅重启电脑、Pika 或 Piper，且基站没有移动。
- 单次短暂 Tracker 遮挡，恢复视野后位姿保持稳定。

重新标定不能修复长期遮挡、镜面反射、Tracker 供电异常或基站视野不足。标定前必须先改善这些问题。

## 3. 标定配置文件

libsurvive 配置文件默认位于：

```text
~/.config/libsurvive/config.json
```

虽然扩展名为 `.json`，当前 libsurvive 生成的内容可能采用它自己的逐项配置格式，并不一定是严格 JSON。因此不要使用 `python -m json.tool` 是否成功作为标定有效性的判断标准。

项目命令：

```text
lerobot-real-vive-calibrate
```

启动时会自动：

1. 加入 `--force-calibrate`。
2. 使用详细日志级别 `--v 100`。
3. 删除现有的 `~/.config/libsurvive/config.json`。
4. 创建新的 libsurvive 上下文并重新求解基站位置。

因此每次标定前必须先备份旧配置。

## 4. 标定前准备

### 4.1 固定基站

- 两台基站必须安装牢固，标定过程中不能移动或振动。
- 两台基站应覆盖双臂摇操的主要工作空间。
- 尽量让两个 Tracker 在大部分工作区内都能看到两台基站。
- 避免基站正对玻璃、镜子、金属亮面和强反光物体。
- 不要让相机、机械臂、人体长期遮挡 Tracker 与基站之间的视线。

### 4.2 停止占用 Vive/Pika 的程序

退出所有摇操、采集和标定进程：

```bash
pgrep -af 'lerobot-real-record|lerobot-real-teleop|lerobot-real-vive-calibrate'
```

如果没有输出，表示没有发现这些进程。不要在正式采集程序仍运行时启动标定。

### 4.3 确认环境

```bash
cd ~/Lerobot-Real-Cam
conda activate lerobot_real

which python
which lerobot-real-vive-calibrate
```

预期命令来自：

```text
/home/star/miniconda3/envs/lerobot_real/
```

### 4.4 备份旧标定

```bash
CALIB_CFG="$HOME/.config/libsurvive/config.json"
BACKUP="$HOME/.config/libsurvive/config.before_calibration_$(date +%Y%m%d_%H%M%S)"

if [[ -f "$CALIB_CFG" ]]; then
    cp -a "$CALIB_CFG" "$BACKUP"
    echo "Calibration backup: $BACKUP"
else
    echo "No existing calibration file; starting from scratch."
fi
```

保留终端打印的备份路径，以便标定失败时恢复。

## 5. 正式标定流程

### 5.1 启动标定

```bash
lerobot-real-vive-calibrate
```

程序会打印被发现的 Lighthouse 和 Tracker，以及持续更新的 Tracker 位姿。

### 5.2 采集标定场景

建议用时约 1–3 分钟，动作要缓慢、连续，并覆盖实际摇操空间。

推荐步骤：

1. 将两只 Pika 放在双臂工作区中央，确保两个 Tracker 都能稳定看到基站。
2. 保持其中一只 Pika 静止，缓慢移动另一只 Pika。
3. 覆盖左、右、前、后、上、下等不同位置。
4. 在不同位置缓慢改变 Tracker 朝向，但不要快速甩动。
5. 每到一个代表性位置，静止约 1–2 秒，让系统获得稳定观测。
6. 换另一只 Pika，重复相同步骤。
7. 最后让两只 Pika 都在实际摇操区域内缓慢移动一遍。

注意：

- 不要用身体挡住 Tracker。
- 不要把 Tracker 紧贴基站或移出基站视野。
- 不要快速摆动或制造剧烈运动模糊。
- 如果某个姿态会导致 Tracker 丢失，应调整基站覆盖，而不是反复在该盲区采样。

### 5.3 观察求解日志

正常标定过程中通常会看到：

```text
Got OOTX packet ...
MPFIT success ...
Global solve with ... scenes ...
Using LH ... as reference lighthouse
```

这些信息表示：

- 已读取基站 OOTX 信息。
- MPFIT 优化成功完成。
- 系统使用累计场景计算了基站全局关系。
- 选择了一台 Lighthouse 作为参考基站。

单独看到一次 `Global solve` 或参考基站选择不是错误。需要关注的是求解后 Tracker 位姿是否稳定，以及正常摇操时是否仍反复求解并导致坐标跳变。

### 5.4 结束标定

当满足以下条件后，可以结束：

- 两台 Lighthouse 均已被识别。
- 两个 Tracker 均持续输出位姿。
- 已出现成功的 `MPFIT` 和 `Global solve`。
- 两只 Pika 静止时，终端中的位置和姿态没有明显跳变。
- 在实际工作区移动时，没有频繁丢失 Tracker。

让两只 Pika 静止观察约 20–30 秒，然后按：

```text
Ctrl+C
```

## 6. 标定文件检查

检查文件是否重新生成：

```bash
CALIB_CFG="$HOME/.config/libsurvive/config.json"

ls -lh "$CALIB_CFG"
stat "$CALIB_CFG"
```

查看关键字段：

```bash
grep -nE \
'^"lighthouse[01]"|^"id"|^"mode"|^"pose"|^"variance"|^"OOTXSet"|^"PositionSet"' \
"$CALIB_CFG"
```

正常结果应包含两组 Lighthouse 配置，并且两组都具有：

```text
"OOTXSet":"1"
"PositionSet":"1"
```

不同 libsurvive 版本的具体排版可能略有差异。不要手工在该文件中增加大括号、逗号或修改 Lighthouse pose。

## 7. 标定后验证

不要标定完成后直接开始 50 轮正式采集。先进行以下测试。

### 7.1 只读硬件检查

```bash
cd ~/Lerobot-Real-Cam
conda activate lerobot_real

bash scripts/run_dual_pika_piper.sh --check
```

### 7.2 纯摇操测试

```bash
bash scripts/run_dual_pika_piper.sh --run
```

测试顺序：

1. 双臂回初始位姿。
2. 完成两个 Pika 的“夹紧—张开—夹紧”手势。
3. 两只 Pika 静止不动 10 秒，确认机械臂不会自行抖动。
4. 只移动左 Pika，右 Pika 保持静止。
5. 只移动右 Pika，左 Pika 保持静止。
6. 双侧同时进行平移和旋转。
7. 连续测试至少 1–2 分钟。

同时观察终端是否在摇操期间频繁出现：

```text
MPFIT
Global solve
Using LH
```

如果机械臂稳定，再进行低速冒烟采集：

```bash
bash scripts/run_dual_pika_piper_record.sh --smoke
```

冒烟测试正常后，再启动正式采集。

## 8. 标定失败后的恢复

如果新标定明显比旧标定差，先停止所有 Pika、摇操和采集程序，然后恢复备份：

```bash
CALIB_CFG="$HOME/.config/libsurvive/config.json"
BACKUP="/home/star/.config/libsurvive/config.before_calibration_YYYYMMDD_HHMMSS"

cp -a "$BACKUP" "$CALIB_CFG"
ls -lh "$CALIB_CFG"
```

将 `BACKUP` 替换为标定前实际打印的备份路径。恢复后重新启动纯摇操程序验证。

不要在 libsurvive 或 Pika 程序仍运行时替换配置文件，因为已经启动的进程通常不会自动重新加载配置。

## 9. 常见问题

### 9.1 配置文件不是合法 JSON

如果运行：

```bash
python -m json.tool "$HOME/.config/libsurvive/config.json"
```

出现 `Extra data`，不一定说明文件损坏。当前 libsurvive 可能生成逐项配置格式，应以它能否被 libsurvive 加载、两组 Lighthouse 是否具有 `OOTXSet` 和 `PositionSet`、Tracker 是否稳定为判断依据。

### 9.2 标定时一直没有 `Global solve`

依次检查：

- 两台基站是否都已通电。
- Lighthouse 通道是否冲突。
- Tracker 是否能看到两台基站。
- 是否存在明显遮挡或镜面反射。
- Pika/Tracker USB 是否被其他程序占用。
- 是否只在一个很小区域内移动，导致场景覆盖不足。

### 9.3 标定后仍然抖动

重新标定后仍抖，不应连续反复覆盖配置。先判断：

- 两只机械臂是否同时抖动。
- 只有某一个 Tracker 的更新率是否明显偏低。
- Pika 静止时 Tracker 位姿是否仍变化。
- 是否只有某些机械臂姿态附近出现 IK 振荡。
- 是否只在打开三台相机和录制时发生。

双臂同时抖动更偏向共享的 Vive/libsurvive 坐标解；单臂抖动更偏向对应 Tracker 遮挡、该侧 IK 或机械臂链路。

### 9.4 运行过程中偶尔出现 `Global solve`

一次全局求解不等于故障。若它发生在某个 episode 开始前，新的 Pika 起始参考 `P0` 通常可以吸收固定的坐标系变化。

如果求解发生在 episode 进行过程中，且 Tracker 坐标随后跳动，则可能直接造成机械臂目标跳变。此时应丢弃当前 episode，停止正式采集并检查基站覆盖和标定质量。

### 9.5 基站移动后旧数据还能不能训练

基站移动不会修改已经保存的数据，因此旧数据仍可用于训练。重新标定只影响之后的摇操追踪。

但为了保持新数据的示教质量，应在基站移动后重新标定并完成纯摇操和冒烟测试，再继续采集。

## 10. 推荐操作顺序

```text
固定两台基站
  ↓
排除遮挡和反光
  ↓
停止所有 Pika/摇操/采集程序
  ↓
备份旧 libsurvive 配置
  ↓
运行 lerobot-real-vive-calibrate
  ↓
缓慢覆盖实际工作空间和不同朝向
  ↓
确认 MPFIT / Global solve 成功
  ↓
静止观察 20–30 秒
  ↓
检查 OOTXSet=1、PositionSet=1
  ↓
纯摇操测试 1–2 分钟
  ↓
低速单轮冒烟测试
  ↓
正式采集
```

采集命令、续采、参数和快捷键说明参见：

```text
DUAL_PIKA_PIPER_RECORDING_README.md
```
