# AgileX Piper 集成

Piper 已直接集成到 Lerobot-Real 主包中，不再作为带有独立
`pyproject.toml` 和包装脚本的子项目维护。实现参考
[AgRoboticsResearch/lerobot_robot_piper](https://github.com/AgRoboticsResearch/lerobot_robot_piper)，
并按本仓库的插件工厂、配置解析和命令入口进行了适配。真机安全语义以
[AgileX 官方 Piper SDK](https://github.com/agilexrobotics/piper_sdk) 为准；同时对照了
[AgileX 官方 Piper-X URDF](https://github.com/agilexrobotics/agx_arm_urdf) 和
[PikaAnyArm 官方 Piper IK](https://github.com/agilexrobotics/PikaAnyArm)；还参考了
[Kane1440/lerobot_piper2](https://github.com/Kane1440/lerobot_piper2) 的 follower、
配置和物理关节命令接口，及
[piper_pika_lerobot](https://github.com/MataoDuan/piper_pika_lerobot)、
[lerobot-for-pika](https://github.com/Koorye/lerobot-for-pika) 和
[lerobot_robot_piper](https://github.com/WeGo-Robotics/lerobot_robot_piper)。参考实现中的
反馈频率检查、无效 Pika 数据保持和实际发送动作记录被选择性采用；强制使能主臂、
固定初始移动和高速度启动逻辑没有采用。

## 支持模式

| 模式 | 配置文件 | 动作空间 | 硬件 |
|---|---|---|---|
| 单 Pika 遥操单 Piper | `config/piper/pika_piper.yaml` | 末端位姿 + 夹爪 | 1×Pika + 1×Piper follower |
| 单 Piper 主从 | `config/piper/piper_leader_follower.yaml` | 关节位置 + 夹爪 | 1×Piper leader + 1×Piper follower |
| 双 Pika 直接采集 | `config/piper/dual_pika_direct.yaml` | 双侧末端位姿 + 夹爪 | 2×Pika + 相机，不驱动机械臂 |
| 双 Pika 遥操双 Piper | `config/piper/dual_pika_piper.yaml` | 双侧末端位姿 + 夹爪 | 2×Pika + 2×Piper follower |
| 双 Piper 主从 | `config/piper/dual_piper_leader_follower.yaml` | 双侧关节位置 + 夹爪 | 2×Piper leader + 2×Piper follower |

双臂模式的数据键使用 `left.` / `right.` 前缀。例如，笛卡尔模式包含
`left.pose.x`、`right.pose.rz` 和 `left.gripper.pos`；关节模式包含
`left.joint1.pos` 到 `right.joint6.pos`。

## 安装

在仓库根目录安装 Piper 可选依赖：

```bash
pip install -e ".[piper]"
```

使用 Pika 的模式还需要安装现有 Pika 依赖：

```bash
pip install pysurvive agx-pypika --no-deps
```

较高 Python 版本没有 `pysurvive` wheel 时，按主 README 的 Pika 安装章节从官方
libsurvive 源码编译。

Piper 支持基于 `piper_sdk>=0.3.0`，不需要安装 `wego_piper`。普通
xArm 安装仍可使用 `pip install -e .`，不会因导入主包而强制加载
`piper_sdk`。

`cartesian_command_mode: official_ik` 还需要带 CasADi 绑定的 Pinocchio。
PyPI 的 `pin` 包不提供项目所需的 `pinocchio.casadi`，请安装 conda-forge 构建：

```bash
conda install -n lerobot_real -c conda-forge pinocchio=4.1.0 casadi=3.7.2
```

Piper-X 必须使用对应型号的官方 URDF，不能使用普通 Piper 模型。示例现场配置
指向 `agx_arm_urdf/piper_x/urdf/piper_x_description.urdf`，并将包含
`agx_arm_description` 目录的父目录配置为 `ik_package_dir`，以便解析 mesh。

## 运行前配置

1. 按 Piper SDK 官方说明激活 CAN 接口，波特率为 1 Mbps。单 Pika
   遥操需要一路 follower 接口；单 Piper 主从需要 follower 和 leader
   各一路；双 follower 需要两路；双 Piper 主从需要四路。已经按现场规则
   命名接口后，可按需手动运行 `sudo ./scripts/setup_piper_can.sh` 配置
   `can_left` 和 `can_right`；该脚本不会注册或启用开机服务。
2. 修改所需 YAML，替换全部 `REPLACE_*` 占位符，包括 Pika 串口、
   相机稳定路径和任务描述。
3. 将示例中的 `can_follower`、`can_leader`、`can_follower1`、
   `can_follower2`、`can_leader1` 和 `can_leader2` 改为现场接口名。
4. 双 Pika 配置必须为左右侧填写不同的 `tracker_device_id`，并使用 Tracker
   的持久 `LHR-*` 硬件序列号。不要使用 libsurvive 启动时临时分配的
   `T20` / `T21` 名称，它们在重启后可能交换。双侧在进程内共享同一个
   libsurvive 上下文，避免第二个上下文因 USB 设备占用而启动失败。
5. 官方 PikaAnyArm 对普通 Piper 定义的夹爪中心工具帧为
   `Ry(-90 deg) @ Tx(190 mm)`。Piper-X 官方 URDF 的夹爪安装帧相对 J6
   又绕 Z 轴旋转 `+90 deg`，因此 Piper-X 使用
   `E_x = Rz(90 deg) @ Ry(-90 deg) @ Tx(190 mm)`，其 xyzrpy 表示为
   `[0, 0, 190, 0, -90, 90]`。Piper SDK 的末端反馈和命令使用原生 J6 帧，
   因此本项目配置其逆变换
   `tracker_to_robot_eef: [-190, 0, 0, -90, 0, -90]`。更换末端工具后需要重新
   标定该参数；每台机械臂还必须在本体坐标系下设置 `workspace_x/y/z`。
   `tracker_to_robot_eef` 只描述工具帧，不再用于修正 Pika 的世界坐标轴。
   `control_frame: robot_base` 使用
   `tracker_world_to_robot_base_rpy` 单独标定跟踪世界到 Piper 基座的固定旋转。
6. 检查配置：

```bash
lerobot-real-piper-check-config config/piper/pika_piper.yaml
lerobot-real-piper-check-config config/piper/piper_leader_follower.yaml
lerobot-real-piper-check-config config/piper/dual_pika_direct.yaml
lerobot-real-piper-check-config config/piper/dual_pika_piper.yaml
lerobot-real-piper-check-config config/piper/dual_piper_leader_follower.yaml
```

现场双 Pika/Piper 配置可先运行只读预检；需要连接双臂时必须显式使用
`--run`，并在脚本提示后输入确认词：

```bash
./scripts/run_dual_pika_piper.sh --check
./scripts/run_dual_pika_piper.sh --run
```

检查器会在占位符未替换、单臂端口缺失、双臂左右侧缺失或同一配置重复
使用端口时返回失败。

## 遥操作与录制

先在急停可达、机械臂周围净空的条件下低速验证遥操作：

```bash
lerobot-real-teleop --config_path config/piper/pika_piper.yaml
lerobot-real-teleop --config_path config/piper/piper_leader_follower.yaml
lerobot-real-teleop --config_path config/piper/dual_pika_piper.yaml
lerobot-real-teleop --config_path config/piper/dual_piper_leader_follower.yaml
```

验证动作方向、左右映射、工作空间和夹爪方向后再录制：

```bash
# 单 Pika -> 单 Piper
lerobot-real-record --config_path config/piper/pika_piper.yaml

# 单 Piper leader -> 单 Piper follower
lerobot-real-record --config_path config/piper/piper_leader_follower.yaml

# 双 Pika 直接采集
lerobot-real-record --config_path config/piper/dual_pika_direct.yaml

# 双 Pika -> 双 Piper
lerobot-real-record --config_path config/piper/dual_pika_piper.yaml

# 双 Piper leader -> 双 Piper follower
lerobot-real-record --config_path config/piper/dual_piper_leader_follower.yaml
```

续录等参数继续使用主仓库 `lerobot-real-record` 的现有选项。

## 控制约定

### Pika 到 Piper

Pika 输出 `xyz(mm) + 轴角(rad)`，目标始终先裁剪到配置工作空间。
`cartesian_command_mode: direct` 每个控制周期通过 `EndPoseCtrl` 发送完整目标；
`step` 模式使用 `max_cartesian_step_mm` 和 `max_rotation_step_rad` 逐步追赶。
`official_ik` 使用 Pinocchio/CasADi 求六轴关节解，并通过 `JointCtrl` 发送物理关节角。
无论运行期使用哪种模式，启动移动到 `robot_base_pose` 时都强制使用固件笛卡尔
限步路径。Pika 的 `gripper.pos` 为 0–1，发送给 Piper 时转换为 0–100。

设 Pika 启动和当前位姿为 `P0`、`Pt`，Piper SDK 启动末端为 `S0`，上述
工具变换为 `E`。官方先计算夹爪中心目标：
`G_t = (S0 E) P0^-1 Pt`。Piper SDK 使用原生 J6 帧，因此实际命令为：
`S_t = G_t E^-1 = S0 E P0^-1 Pt E^-1`。
`control_frame: official` 使用相对位姿公式
`S_t = S0 C^-1 P0^-1 Pt C`，两者相等要求 `C = E^-1`。
`tracker_to_robot_eef` 只保存 `C`，即 Piper J6 与官方夹爪中心之间的工具变换。
`official_ik` 将 `S_t E` 作为求解目标，以当前反馈关节角或上一帧有效解作为初值，
并使用 URDF 关节限位和官方碰撞对。相邻目标超过 30 度时按官方逻辑以约 1 度、
200 Hz 插值。IK 无解、超限或碰撞时不发布新的关节/夹爪目标，只上报
`over_limit=True` 并保持上一条有效指令；后续重新求得有效解会自动恢复。

底层关节接口统一使用七个物理量：`[q1..q6(rad), gripper(m)]`。它集中完成
弧度到 SDK `0.001 deg`、米到微米的换算以及关节/夹爪范围校验。现有主从关节动作
仍保留 `[-100, 100]`（夹爪 `[0, 100]`）的数据集表示，但发送前只做一次适配并调用
同一物理关节接口；TCP/IK 不经过归一化，直接复用该接口。

`control_frame: robot_base` 将跟踪世界的增量映射到 Piper 基座。设
`Q` 为 `tracker_world_to_robot_base_rpy` 对应的旋转，则先计算
`p_Gt = p_G0 + Q(p_Pt - p_P0)` 和
`R_Gt = Q(R_Pt R_P0^-1)Q^-1 R_G0`，最后仍以
`S_t = G_t E^-1` 转回 J6 命令。这样工具中心不变，平移方向也不再依赖
Pika 和机械臂激活时的初始朝向。

双侧完成闭合-打开-闭合手势后，主控制循环先用同一帧左右 Piper 反馈更新
`G0`，再分别以两只 Pika 的首个有效位姿作为 `P0`。程序会在发送首条动作前
记录 `P0`、映射后的夹爪中心 `G0`，以及 Pika 世界坐标 +X/+Y/+Z 到机器人基座
XYZ 的方向向量。
`official` 模式记录启动姿态形成的局部轴映射；`robot_base` 模式记录固定的
`Q`，便于现场核对坐标标定。

`lerobot-real-teleop` 的机器人命令循环由 YAML 顶层 `fps` 控制，Piper 示例
设为 50 Hz，与官方发布频率一致。`teleop.frequency` 只控制 Pika 命令或启动
手势轮询；`dataset.fps` 由录制命令使用，不改变该遥操作命令的控制频率。

连接阶段会等待机械臂状态、关节、夹爪以及末端位姿的有效反馈。运行中如果反馈
`Hz <= 0`、时间戳超过 `feedback_timeout_s`，或机械臂报告故障码、欠压、过温、
过流、碰撞、驱动器错误、堵转等状态，将拒绝发送动作。
如果机械臂当前位置已经处于配置工作空间之外，也会拒绝笛卡尔动作，而不是从错误
位置继续移动。直发模式还会比较目标与机械臂反馈：
`max_cartesian_following_error_mm` 或 `max_rotation_following_error_rad`
任一超限便停止发送。Tracker 或夹爪偶发返回 `None/NaN` 时保持上一条有效动作。

### Piper 主从

leader 与 follower 使用相同的归一化关节表示：六个关节为 -100–100，
夹爪为 0–100。`max_relative_target` 限制单周期关节变化，降低主从臂
初始姿态不一致造成大幅跳变的风险。leader 配置为 `0xFA` 示教输入臂后保持
可回拖状态，不调用 follower 使用的使能流程；只有收到有效的关节控制帧后才
向上层提供动作。启动时至少需要一帧有效夹爪控制数据，之后夹爪控制帧短暂掉帧
会保持上一有效夹爪值，关节控制帧掉帧则立即停止发送。

## 安全默认值

- `park_on_connect: false`：连接时不自动执行停车动作。
- `park_on_disconnect: false`：退出时不自动规划停车轨迹。
- `disable_torque_on_disconnect` 默认为 `true`；当前单臂 Pika 配置显式设为
  `false`，避免未支撑机械臂退出时突然失能跌落。
- `hold_position_on_disconnect: true`：在保留使能的退出路径上，先发送一次当前
  反馈位姿，避免机械臂继续追赶最后一个遥操作目标。
- `feedback_timeout_s: 0.5`：反馈超过 0.5 秒没有更新即停止发送。
- `feedback_startup_timeout_s: 5.0`：启动后最多等待 5 秒有效反馈，否则连接失败。
- 当前单臂 Pika 配置运行期使用完整目标直发、30% 速度和
  600 mm / 3.2 rad 紧急跟随误差保护；启动预定位仍使用 3 mm / 0.02 rad 限步。
- Piper 主从示例使用 5% 速度和 0.5 的归一化关节单周期限幅。
- 笛卡尔模式必须显式配置 `workspace_x/y/z`，不提供无限制默认工作空间。

关闭扭矩前必须评估负载和坠臂风险。示例工作空间、速度和相机路径只是
模板，不能替代现场安全标定。

## 代码位置

- `src/lerobot_real/devices/piper/`：CAN 适配、固定标定表和位姿转换。
- `src/lerobot_real/robots/piper/`：单/双 Piper follower。
- `src/lerobot_real/teleoperators/piper_leader/`：单/双 Piper leader。
- `src/lerobot_real/teleoperators/pika_teleop/`：单/双 Pika 遥操作。
- `src/lerobot_real/configs/piper.py`：Piper/Pika 配置类型注册。

## 已知边界

- `official_ik` 的运动学结果取决于 URDF 型号。Piper-X 必须使用
  `piper_x_description.urdf`；普通 Piper URDF 会产生错误的正逆解。
- Piper-X 基础 URDF 不含夹爪碰撞几何，因此当前碰撞检测覆盖机械臂本体，
  不覆盖夹爪与外部环境。
- 关节范围按 `piper_sdk>=0.3.0` 的固定限制设置；现场固件、末端工具、
  安装方向和实际安全空间仍需逐项核对。
