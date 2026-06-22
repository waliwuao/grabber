# 矛头相对位置处理节点

## 是否需要预设读取范围

需要，但应使用可配置 ROI，不应把范围永久写死在算法中。

预设范围有三个作用：

1. 排除墙面、机器人和人员等无关点。
2. 避免把整圈雷达中的所有小点簇误判为矛头。
3. 降低计算量，使分簇阈值更容易调试。

当前第一版参数位于：

```text
src/spear_locator/config/spear_locator.yaml
```

默认值：

```text
距离：0.08～0.43 m（目标真值要求不超过 0.40 m）
角度：绿色负轴周围 -135°～-45°
横向 X：-0.43～+0.43 m
测量侧 Y：-0.43～-0.08 m
```

ROS 激光坐标约定及当前布置为：

```text
红色 X：两个矛头的排列方向
绿色 -Y：雷达到矛头的测量方向
```

其中 0.43 m 包含 3 cm 测量余量，防止位于 0.40 m 的目标因为正测距误差被过滤；它不表示工作目标可以超过 0.40 m。接入真实雷达后，应先在 RViz2 中观察矛头所在区域，再把横向 ROI 缩小到目标运动范围外加少量余量。

建议采用两层范围：

- 驱动层保留完整 `/scan`，方便录包和重新调参。
- 处理层通过 YAML 截取 ROI，不修改原始数据。

## 当前已经实现

- 订阅 `sensor_msgs/msg/LaserScan`
- 距离、角度和二维矩形 ROI
- NaN、Inf 和越界点过滤
- 随距离和角分辨率变化的自适应相邻点分簇
- 点簇点数和宽度过滤
- 使用点簇坐标中位数估计目标中心
- 按雷达坐标 `Y` 从小到大编号
- 输出相对雷达坐标
- 输出相对 `id=0` 的二维位置
- 计算平均相邻间距
- 发布过滤点云、PoseArray、MarkerArray 和 JSON
- 合成等间距矛头数据，可在没有雷达转接板时测试

尚未实现：

- 基于已知标准间距的槽位拟合与误检剔除
- 矛尖两侧边缘拟合
- 跨帧 ID 跟踪
- 遮挡目标位置预测
- 雷达到工位/机器人坐标系的标定

## 编译

```bash
cd /home/charlie/2d雷达
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 无硬件测试

启动模拟雷达、处理节点和 RViz2：

```bash
cd /home/charlie/2d雷达
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch spear_locator synthetic_demo.launch.py
```

模拟场景包含 5 个目标：

```text
前向距离约 1.2 m
相邻间距约 0.2 m
```

不打开 RViz2，仅测试数据流：

```bash
ros2 launch spear_locator synthetic_pipeline.launch.py
```

另一个终端查看输出：

```bash
source /opt/ros/jazzy/setup.bash
source /home/charlie/2d雷达/install/setup.bash
ros2 topic echo /spear_locator/detections_json
```

## 接入真实 STL-27L

连接雷达并确认串口后：

```bash
cd /home/charlie/2d雷达
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch spear_locator stl27l_pipeline.launch.py
```

该 launch 会启动：

- STL-27L 驱动
- 矛头处理节点
- RViz2

## 输出话题

| 话题 | 类型 | 用途 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 原始雷达扫描 |
| `/spear_locator/filtered_points` | `sensor_msgs/PointCloud2` | ROI 中保留的点 |
| `/spear_locator/poses` | `geometry_msgs/PoseArray` | 各目标中心位置 |
| `/spear_locator/markers` | `visualization_msgs/MarkerArray` | RViz 编号和坐标 |
| `/spear_locator/detections_json` | `std_msgs/String` | 上位机易读取的 JSON |

JSON 中每个目标包含：

```text
id
x_m, y_m
relative_to_id0_x_m, relative_to_id0_y_m
range_m
bearing_deg
point_count
width_m
observed
```

## 真实数据调参顺序

1. 先只调整 `range_*`、`angle_*`、`x_*`、`y_*`，确保 ROI 只包围矛头区域。
2. 观察每个矛头在 `/spear_locator/filtered_points` 中有多少点。
3. 调整 `neighbor_base_gap_m` 和 `neighbor_gap_scale`，避免一个矛头被拆开或相邻矛头被合并。
4. 调整 `min_cluster_points`、`max_cluster_points` 和宽度范围。
5. 记录 rosbag，在同一份数据上重复调参。
6. 已加入6目标近似等间距约束；获得标准间距和完整架真实数据后，再收紧当前35%的相邻间距偏差门限。
