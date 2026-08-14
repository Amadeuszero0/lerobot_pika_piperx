# 双 Pika / Piper 三相机录制流程

这套录制路径与现有遥操配置隔离：

- 普通遥操继续使用 `config/piper/dual_pika_piper_local.yaml`；
- 相机角色保存在 `config/piper/d435i_roles_local.yaml`；
- 录制配置由工具生成到 `config/piper/dual_pika_piper_record_local.yaml`；
- 生成器不会覆盖普通遥操配置。

相机角色绑定到 RealSense 硬件序列号，不能绑定 `/dev/videoN` 或 USB 端口号。

## 当前角色

```yaml
third_view: "346122070530"
left_wrist: "233522077815"
right_wrist: null
```

## 新线到货后的绑定

三台相机全部接好并等待十秒，然后查看枚举和 USB 速率：

```bash
python scripts/prepare_dual_pika_piper_recording.py discover
```

三台都必须显示 USB 3.x。保存带序列号的画面用于最终核对位置：

```bash
python scripts/prepare_dual_pika_piper_recording.py discover \
    --snapshots /tmp/d435i_role_snapshots
```

确认第三台画面来自右臂后绑定：

```bash
python scripts/prepare_dual_pika_piper_recording.py bind \
    right_wrist RIGHT_CAMERA_SERIAL
```

绑定工具默认拒绝 USB 2.x 相机、未连接序列号和重复序列号。

## 只读门禁测试

```bash
bash scripts/run_dual_pika_piper_record.sh --check
```

这个命令依次完成：

1. 检查三台绑定相机全部在线且均为 USB 3.x；
2. 同时打开三路 `640x480@30` RGB 流十秒并检查帧率；
3. 生成独立的录制配置；
4. 校验 Piper 配置；
5. 只读检查左右 Piper 反馈和两套 Pika；
6. 不使能机械臂，不发送运动命令。

任一设备失败都会阻止进入录制阶段。

## 正式录制

支撑好机械臂、清空左右工作空间并准备好急停，然后执行：

```bash
bash scripts/run_dual_pika_piper_record.sh --run
```

只有手动输入 `RECORD` 后，脚本才会调用 `lerobot-real-record`。生成的录制配置会在连接时把两台 Piper 设置为 follower 模式，然后按原双臂遥操的动作、安全限幅、工作空间和 Pika 映射运行。

录制图像键为：

```text
left.third_view
left.wrist
right.wrist
```

默认以 LeRobot 视频数据集格式写入本地配置指定的 `dataset.root`，相机分辨率和数据集帧率均为 30 FPS。
