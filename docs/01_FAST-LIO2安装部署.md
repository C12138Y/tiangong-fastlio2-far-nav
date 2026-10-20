# FAST-LIO2 安装与部署

## 1. 前置条件

实机环境：

- Ubuntu 24.04 ARM64
- ROS 2 Jazzy
- PCL 1.14、Eigen 3.4、yaml-cpp 0.8
- 厂家 Livox 驱动已运行
- `/livox/lidar`：`sensor_msgs/msg/PointCloud2`，约 10 Hz
- `/livox/imu`：`sensor_msgs/msg/Imu`，约 200 Hz
- 两路消息的 `frame_id` 均为 `livox_frame`

每个终端先执行：

```bash
source /home/nvidia/hric_tools/hric_env.sh
unset ROS_DOMAIN_ID
```

`ROS_DOMAIN_ID` 是进程启动时确定的。运行中修改环境变量不会把已有进程迁移到新 domain。实机实时链路使用未设置状态；离线回放若使用 `ROS_DOMAIN_ID=77`，所有相关进程必须在同一个 77 环境中重新启动。

## 2. 获取源码

普通浅克隆曾出现 `curl 28`、`GnuTLS recv error (-110)`、`expected flush after ref listing`。已验证的方式是强制 HTTP/1.1，并克隆到新目录：

```bash
mkdir -p ~/fastlio2_ws/src
cd ~/fastlio2_ws/src

git -c http.version=HTTP/1.1 clone \
  --depth 1 --single-branch --branch main \
  https://github.com/liangheming/FASTLIO2_ROS2.git \
  FASTLIO2_ROS2_http11
```

不要在失败后留下的半成品目录中继续编译。先检查：

```bash
git -C ~/fastlio2_ws/src/FASTLIO2_ROS2_http11 status
git -C ~/fastlio2_ws/src/FASTLIO2_ROS2_http11 rev-parse HEAD
```

## 3. 本地安装 Sophus

系统中缺少可直接使用的 Sophus，因此安装到工作区私有前缀，不污染系统：

```bash
mkdir -p ~/fastlio2_ws/third_party
cd ~/fastlio2_ws/third_party

git -c http.version=HTTP/1.1 clone \
  --depth 1 --branch 1.22.10 \
  https://github.com/strasdat/Sophus.git sophus-1.22.10

cmake -S sophus-1.22.10 -B sophus-1.22.10/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/fastlio2_ws/local/sophus" \
  -DSOPHUS_USE_BASIC_LOGGING=ON \
  -DBUILD_SOPHUS_TESTS=OFF \
  -DBUILD_SOPHUS_EXAMPLES=OFF

cmake --build sophus-1.22.10/build --parallel 2
cmake --install sophus-1.22.10/build
```

### CMake 必要修正

原工程虽然 `find_package(Sophus)`，但 `lio_node` 没有链接导出的目标，导致编译器找不到 `sophus/so3.hpp`。在 FAST-LIO2 的 `CMakeLists.txt` 中确保：

```cmake
target_link_libraries(lio_node
  yaml-cpp
  Sophus::Sophus
  ${PCL_LIBRARIES}
)
```

首次配置/清理后构建：

```bash
source /home/nvidia/hric_tools/hric_env.sh
cd ~/fastlio2_ws

colcon build --packages-select fastlio2 \
  --cmake-args \
    -DSophus_DIR="$HOME/fastlio2_ws/local/sophus/share/sophus/cmake" \
    -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct+
```

日常修改后可用：

```bash
cd ~/fastlio2_ws
colcon build --packages-select fastlio2 --symlink-install
source ~/fastlio2_ws/install/setup.bash
```

## 4. 解决 PointCloud2 与 CustomMsg 不匹配

选用的 FAST-LIO2 ROS 2 端口订阅 `livox_ros_driver2/msg/CustomMsg`，而天工现有厂家驱动发布 `sensor_msgs/msg/PointCloud2`。为避免改动厂家驱动，新增独立节点：

```text
/livox/lidar (PointCloud2)
      -> pc2_to_custom
      -> /fastlio2/livox_custom (livox_ros_driver2/CustomMsg)
```

机器人端源文件应归档：

```text
~/fastlio2_ws/src/FASTLIO2_ROS2_http11/fastlio2/src/pc2_to_custom.cpp
```

转换要点：

- 读取 `x/y/z/intensity/tag/line/timestamp`。
- Livox `timestamp` 字段是每个点的绝对纳秒时间。
- `CustomPoint.offset_time` 使用相对帧起点的纳秒偏移。
- 实测单帧点偏移约 `0～99.83 ms`，符合约 10 Hz 扫描周期。
- 不应把所有点的 `offset_time` 写成 0，否则会破坏去畸变。

`CMakeLists.txt` 和 `package.xml` 需要声明 `sensor_msgs`、`livox_ros_driver2` 等依赖，创建并安装 `pc2_to_custom` 可执行文件。当前完整源文件仍在机器人端，使用本归档中的 `collect_robot_sources.sh` 收集，禁止凭记忆重写覆盖。

启动转换器：

```bash
source /home/nvidia/hric_tools/hric_env.sh
source ~/fastlio2_ws/install/setup.bash
unset ROS_DOMAIN_ID
ros2 run fastlio2 pc2_to_custom
```

验证：

```bash
ros2 topic info /fastlio2/livox_custom --verbose
timeout 8 ros2 topic hz /fastlio2/livox_custom
```

## 5. FAST-LIO2 配置

源配置：

```text
~/fastlio2_ws/src/FASTLIO2_ROS2_http11/fastlio2/config/lio.yaml
```

本次最终使用的核心配置：

```yaml
imu_topic: /livox/imu
lidar_topic: /fastlio2/livox_custom
body_frame: fastlio_body
world_frame: fastlio_odom
r_il: [1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0]
t_il: [-0.011, -0.02329, 0.04412]
```

说明：

- 将默认 `lidar/body` 改成 `fastlio_odom/fastlio_body`，避免与 HRIC 原生 TF 冲突。
- 源码使用形式为 `p_imu = r_il * p_lidar + t_il`。
- 当前平移根据 MID360S 手册中 LiDAR/IMU 几何关系和上述变换约定换算；物理安装朝向和机器人底盘外参仍需现场标定，不能把手册值当成最终底盘外参。
- 修改源目录配置后必须重新构建并重新 `source install/setup.bash`。launch 实际读取安装空间中的文件。

## 6. 启动 FAST-LIO2

另开终端：

```bash
source /home/nvidia/hric_tools/hric_env.sh
source ~/fastlio2_ws/install/setup.bash
unset ROS_DOMAIN_ID
ros2 launch fastlio2 lio_launch.py
```

验证：

```bash
ros2 topic info /fastlio2/lio_odom
ros2 topic info /fastlio2/body_cloud
ros2 topic info /fastlio2/world_cloud

timeout 8 ros2 topic echo /fastlio2/lio_odom --once --field header
timeout 8 ros2 topic echo /fastlio2/world_cloud --once --field header

ros2 run tf2_ros tf2_echo fastlio_odom fastlio_body
```

正确结果应包含：

- `/fastlio2/lio_odom` 持续发布，frame 为 `fastlio_odom`。
- TF 只有本链路私有的 `fastlio_odom -> fastlio_body`。
- `/fastlio2/body_cloud` 为当前帧、机体系点云。
- `/fastlio2/world_cloud` 为当前帧变换到 `fastlio_odom` 后的点云。

特别注意：`/fastlio2/world_cloud` 不是累计全局地图。源码中它是当前处理帧的 world-frame 点云，不要根据名字把它误当成已经拼接完成的 PCD 地图。

## 7. 里程计适配和局部代价地图

独立适配包位于机器人：

```text
~/tiangong_nav_ws/src/fastlio_nav_adapter
```

它输出：

- `/my_nav/odom`
- 私有 TF `fastlio_odom -> my_nav_base_link`

启动：

```bash
source /home/nvidia/hric_tools/hric_env.sh
source ~/fastlio2_ws/install/setup.bash
source ~/tiangong_nav_ws/install/setup.bash
unset ROS_DOMAIN_ID
ros2 launch fastlio_nav_adapter adapter.launch.py
```

局部 costmap：

```bash
ros2 launch fastlio_nav_adapter local_costmap.launch.py
```

若 launch 没自动激活生命周期：

```bash
ros2 lifecycle set /my_nav/local_costmap configure
ros2 lifecycle set /my_nav/local_costmap activate
ros2 lifecycle get /my_nav/local_costmap
```

Jazzy 参数类型严格，`height`、`width` 这类声明为整数的参数必须写 `12`，不要写 `12.0`。

## 8. FAST-LIO2 独立验收

```bash
source /home/nvidia/hric_tools/hric_env.sh
source ~/fastlio2_ws/install/setup.bash
unset ROS_DOMAIN_ID

ros2 topic info /livox/lidar --verbose
ros2 topic info /livox/imu --verbose
ros2 topic info /fastlio2/livox_custom --verbose
ros2 topic info /fastlio2/lio_odom --verbose

timeout 8 ros2 topic hz /fastlio2/livox_custom
timeout 8 ros2 topic hz /fastlio2/lio_odom
timeout 8 ros2 topic echo /fastlio2/world_cloud --once --field width
ros2 run tf2_ros tf2_echo fastlio_odom fastlio_body
```

只有这些检查通过后，才应继续启动 terrain analysis 和 FAR。

