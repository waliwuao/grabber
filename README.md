# 2D LiDAR Target Recognition and X-Position Control

基于 STL-27L 与 ROS 2 Jazzy 的二维目标识别、X坐标补偿和位置外环PID项目。

## 仓库内容

- `src/spear_locator/`：目标识别、标定补偿、离线分析及PID节点
- `src/ldlidar_stl_ros2/`：包含本项目兼容修补的STL-27L驱动源码
- `datasets/`：本项目录制的MCAP rosbag、场景说明、识别结果和真值表
- `scripts/`：录制、处理、回放、渲染和表格工具
- `PROJECT_MEMORY.md`：完整项目进度与实验记录
- `RECOGNITION_RESULTS.md`：识别和精度结果
- `CALIBRATION_RESULTS.md`：标定方法与参数
- `PID_CONTROL.md`：X位置PID速度控制说明

## 克隆后的数据路径

本机开发环境中的 `data` 是指向外部大容量磁盘的软链接，因此没有直接提交该链接。
克隆仓库后可将仓库数据映射为既有脚本所使用的路径：

```bash
ln -s datasets data
```

## 构建

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动雷达

串口号可能随USB接口变化，请先检查：

```bash
ls /dev/ttyUSB*
```

例如：

```bash
ros2 launch ldlidar_stl_ros2 viewer_stl27l.launch.py \
  port_name:=/dev/ttyUSB1
```

## 处理已有数据

```bash
ros2 run spear_locator analyze_bag \
  datasets/bags/five_targets_0mm_r8_20260620_162610/bag \
  --expected-count 5
```

## 测试

```bash
pytest -q src/spear_locator/test
```

当前基线为22项测试通过。数据、参数适用于本项目的安装结构与目标定义，迁移到其他
雷达、目标或机械坐标系时应重新进行独立标定和验证。
