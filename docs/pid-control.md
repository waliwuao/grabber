# X位置外环PID速度控制

该节点读取 `/spear_recognition/result` 中指定目标的X坐标，以：

```text
error_x = desired_x - current_x
speed = Kp*error_x + Ki*integral(error_x) + Kd*d(error_x)/dt
```

输出速度。它是“位置到速度”的外环，不替代电机驱动器自身的速度/电流内环。

## 安全设计

- 默认 `enabled=false`，只输出零速度；
- 目标丢失、识别失败或超过4.5秒无新结果时，速度自动归零；
- 最大速度默认限制为 `0.01 m/s`；
- 每次收到新位置后只保持速度0.5秒，单次最多移动约5 mm；
- X误差进入 `±5 mm` 死区后速度归零并清除积分；
- 同时发布 `/cmd_vel` 和 `/spear_pid/speed_mps`。

首次连接实际运动机构时，应架空或使用急停，并先确认速度正方向。
下游电机控制器也必须配置命令超时看门狗，不能只依赖ROS节点退出时发送零速度。

## 启动

先启动雷达和识别节点，例如当前串口为 `/dev/ttyUSB1`、5个目标：

```bash
ros2 launch ldlidar_stl_ros2 viewer_stl27l.launch.py \
  port_name:=/dev/ttyUSB1

ros2 launch spear_locator recognition.launch.py expected_count:=5
```

先以禁用状态观察PID：

```bash
ros2 launch spear_locator position_pid.launch.py \
  target_id:=2 desired_x_m:=0.0 enabled:=false
```

查看状态：

```bash
ros2 topic echo /spear_pid/status
ros2 topic echo /spear_pid/speed_mps
```

确认 `error_x_m` 正负和机械运动方向后启用：

```bash
ros2 launch spear_locator position_pid.launch.py \
  target_id:=2 desired_x_m:=0.0 enabled:=true
```

如果正速度会使目标X误差增大，把 `position_pid.yaml` 中的
`direction_sign` 改为 `-1.0`。

## 初始调参顺序

1. 保持 `Ki=0`、`Kd=0`，从较小 `Kp` 开始。
2. 缓慢增加 `Kp`，直到响应足够快但尚未持续振荡。
3. 增加少量 `Kd` 抑制过冲。
4. 只有存在稳定静差时才增加很小的 `Ki`。

当前识别约每30帧更新一次，约3秒才产生一个新位置，因此默认采用短速度脉冲，而不是
在两次测量间持续盲走。实际连续运动若需要更平滑，应把识别窗口缩短或增加跨帧目标
跟踪；不要依靠提高PID增益或延长脉冲来弥补测量延迟。
