# STL-27L ROS 2 Jazzy 驱动

## 当前状态

- 官方源码：`src/ldlidar_stl_ros2`
- 官方提交：`bf668a89baf722a787dadc442860dcbf33a82f5a`
- 环境：Ubuntu 24.04 x86_64、ROS 2 Jazzy
- 编译状态：成功
- 雷达实机测试：尚未完成

厂商源码在 GCC 13 下缺少 `pthread.h`，本地已做最小兼容修复；同时将互斥锁析构时的错误 `pthread_mutex_unlock` 改为 `pthread_mutex_destroy`。

## 重新编译

```bash
cd /home/charlie/2d雷达
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 连接雷达后检查串口

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

默认启动文件使用 `/dev/ttyUSB0` 和 `921600` baud。USB 重新插拔后设备可能变成 `/dev/ttyUSB1`，此时不需要修改源码，启动时传入：

```bash
ros2 launch ldlidar_stl_ros2 viewer_stl27l.launch.py \
  port_name:=/dev/ttyUSB1
```

## 串口权限

当前用户 `charlie` 尚不在 `dialout` 组。推荐执行一次：

```bash
sudo usermod -aG dialout charlie
```

然后注销并重新登录。不要长期使用厂商 README 中的 `chmod 777 /dev/ttyUSB0`。

## 启动

仅启动雷达节点：

```bash
cd /home/charlie/2d雷达
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ldlidar_stl_ros2 stl27l.launch.py
```

如果实际串口为 `/dev/ttyUSB1`：

```bash
ros2 launch ldlidar_stl_ros2 stl27l.launch.py \
  port_name:=/dev/ttyUSB1
```

启动雷达并打开 RViz2：

```bash
cd /home/charlie/2d雷达
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ldlidar_stl_ros2 viewer_stl27l.launch.py
```

如果实际串口为 `/dev/ttyUSB1`：

```bash
ros2 launch ldlidar_stl_ros2 viewer_stl27l.launch.py \
  port_name:=/dev/ttyUSB1
```

检查数据：

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /scan --once
```

## 已验证内容

- `colcon build --symlink-install` 成功
- ROS 包和节点可执行文件可被发现
- STL-27L launch 文件可被 Jazzy 正常解析
- 节点可加载 STL-27L、921600 baud、`/scan` 等参数

用户已在主机上识别到 CP210x，设备编号曾从 `/dev/ttyUSB0` 变为 `/dev/ttyUSB1`。真实 `/scan` 的稳定性和录包仍待完成。
