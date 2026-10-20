# 话题、TF、RViz 与参数字典

## 1. 原始传感器

| 名称 | 类型 | 发布者/来源 | 作用 |
|---|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 厂家 Livox 驱动 | MID360 原始点云，约 10 Hz |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 厂家 Livox 驱动 | IMU，约 200 Hz |
| `livox_frame` | frame | 厂家驱动 | 两路原始消息的 frame |

厂家配置实机路径曾确认是：

```text
/home/nvidia/xos/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json
```

## 2. FAST-LIO2

| 名称 | 类型 | 作用 | 备注 |
|---|---|---|---|
| `/fastlio2/livox_custom` | `livox_ros_driver2/msg/CustomMsg` | 转换后的雷达输入 | `pc2_to_custom` 发布 |
| `/fastlio2/lio_odom` | `nav_msgs/msg/Odometry` | FAST-LIO2 原始里程计 | world=`fastlio_odom` |
| `/fastlio2/lio_path` | `nav_msgs/msg/Path` | LIO 轨迹显示 | 非规划路径 |
| `/fastlio2/body_cloud` | `sensor_msgs/msg/PointCloud2` | 当前帧机体系点云 | frame=`fastlio_body`，给局部 costmap |
| `/fastlio2/world_cloud` | `sensor_msgs/msg/PointCloud2` | 当前帧世界系点云 | frame=`fastlio_odom`，不是累计地图 |

TF：

```text
fastlio_odom -> fastlio_body
```

## 3. 适配器与局部 costmap

| 名称 | 类型 | 作用 |
|---|---|---|
| `/my_nav/odom` | `nav_msgs/msg/Odometry` | 统一给 FAR、局部规划和跟踪器的里程计 |
| `/my_nav/costmap` | `nav_msgs/msg/OccupancyGrid` | 2D 局部代价地图 |
| `/my_nav/voxel_grid` | `nav2_msgs/msg/VoxelGrid` | 3D voxel 调试输出 |
| `/my_nav/local_avoid_path` | `nav_msgs/msg/Path` | 跟踪脚本发布的局部 A* 路径 |

TF：

```text
fastlio_odom -> my_nav_base_link
```

当前 costmap 实测/配置特征：

- rolling window：约 12 m × 12 m
- resolution：0.1 m
- robot radius：0.35 m
- inflation radius：0.55 m
- 点云源：`/fastlio2/body_cloud`
- 点云订阅 QoS：BEST_EFFORT
- 输出 costmap QoS：TRANSIENT_LOCAL

需重点复核的 voxel 参数：

```text
origin_z: 0.0
z_resolution: 0.125
z_voxels: 16
unknown_threshold: 15
mark_threshold: 0
min_obstacle_height: 0.08
max_obstacle_height: 1.8
obstacle_min_range: 0.3
obstacle_max_range: 8.0
raytrace_min_range: 0.3
raytrace_max_range: 10.0
```

## 4. Terrain 与 FAR

| 名称 | 类型 | 作用 |
|---|---|---|
| `/far_eval/terrain_map` | `sensor_msgs/msg/PointCloud2` | terrain analysis 输出 |
| `/far_eval/terrain_map_ext` | `sensor_msgs/msg/PointCloud2` | 扩展地形点云 |
| `/FAR_obs_debug` | `sensor_msgs/msg/PointCloud2` | FAR 障碍/观测调试点云 |
| `/far_eval/goal_point` | `geometry_msgs/msg/PointStamped` | FAR 最终目标输入 |
| `/far_eval/way_point` | `geometry_msgs/msg/PointStamped` | FAR 当前航点输出 |
| `/far_reach_goal_status` | `std_msgs/msg/Bool` | FAR 到达目标状态 |
| `/viz_path_topic` | `visualization_msgs/msg/Marker` | FAR 全局路径显示 |
| `/viz_graph_topic` | `visualization_msgs/msg/MarkerArray` | V-Graph 顶点和边显示 |

`/viz_path_topic` 的常见 Marker：

- `ns: global_path`
- `type: 4`，即 `LINE_STRIP`
- 蓝色粗线
- `points` 才是实际显示的路径顶点

## 5. RViz 目标接口

| RViz 工具 | 默认/当前 topic | 类型 | 是否被 FAR 原生订阅 |
|---|---|---|---|
| Publish Point | `/far_eval/goal_point` | `PointStamped` | 是 |
| 2D Goal Pose | `/goal_pose` | `PoseStamped` | 否，需转换脚本 |
| 2D Pose Estimate | 常为 `/initialpose` | `PoseWithCovarianceStamped` | 本无图链路不用 |

当前跟踪器将 `/goal_pose` 的 position 转为 `/far_eval/goal_point`，frame 必须能按 `fastlio_odom` 正确解释。

## 6. 运控接口

| 名称 | 类型 | 作用 |
|---|---|---|
| `/hric/robot/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 自定义跟踪器/遥控/原生 tracker 的速度入口 |
| `/hric/robot/cmd_vel_status` | `geometry_msgs/msg/TwistStamped` | 控制侧接收状态回传 |
| `/hric/robot/walk_speed` | `geometry_msgs/msg/TwistStamped` | 实际/报告步行速度 |
| `/hric/robot/rl_state` | `diagnostic_msgs/msg/DiagnosticStatus` | FSM 状态，实验要求 SWR |
| `/sbus_data` | `sensor_msgs/msg/Joy` | 遥控器输入 |

主要节点：

- `joystick_bridge_node`：遥控器到 `/hric/robot/cmd_vel`。
- `hric_topic_bridge`：订阅速度和 FSM 命令，连接运控。
- `hric_path_tracker`：HRIC 原生导航跟踪器；做自定义无图试验时不应同时发布速度。
- `xmigcs_control_node`：发布 `cmd_vel_status` 和 `walk_speed`。

## 7. HRIC 原生地图导航接口（本无图链路不依赖）

| 名称 | 类型 | 作用 |
|---|---|---|
| `/hric/loc/pose` | `nav_msgs/msg/Odometry` | HRIC 地图定位位姿 |
| `/hric/loc_laser/load_map` | `hric_msgs/srv/LoadMap` | 定位地图加载 |
| `/hric/loc/relocate` | `hric_msgs/srv/Relocate` | 地图重定位 |
| `/hric/nav/start_nav` | `hric_msgs/srv/StartNav` | 原生导航启动 |
| `/hric/nav/stop_nav` | `hric_msgs/srv/StopNav` | 原生导航停止 |
| `/hric/nav/status` | `hric_msgs/msg/NavStatus` | 导航状态 |
| `/hric/nav/task_status` | `hric_msgs/msg/TaskStatus` | 任务进度 |

只有加载地图、重定位成功并且原生 nav/tracker 链路完整时，以上接口才构成地图导航。它们不能替代本项目的 FAST-LIO 无图坐标链路。

## 8. RViz 推荐显示项

| Display 名称建议 | Display 类型 | Topic / Reference Frame | 作用 |
|---|---|---|---|
| FAST World Frame | Axes | `fastlio_odom` | 世界系原点 |
| FAST Body | Axes | `fastlio_body` | 雷达/IMU 位姿 |
| Nav Base | Axes | `my_nav_base_link` | 跟踪器使用的本体坐标 |
| Current Body Cloud | PointCloud2 | `/fastlio2/body_cloud` | 当前机体系点云 |
| Current World Cloud | PointCloud2 | `/fastlio2/world_cloud` | 当前世界系点云 |
| Terrain | PointCloud2 | `/far_eval/terrain_map` | 地形分析结果 |
| Terrain Ext | PointCloud2 | `/far_eval/terrain_map_ext` | 扩展地形 |
| FAR Obstacles | PointCloud2 | `/FAR_obs_debug` | FAR 障碍调试 |
| FAR Graph | MarkerArray | `/viz_graph_topic` | 可见性图 |
| FAR Path | Marker | `/viz_path_topic` | FAR 输出路径 |
| Local Costmap | Map | `/my_nav/costmap` | 近场障碍与膨胀 |
| Local Avoid Path | Path | `/my_nav/local_avoid_path` | 独立 A* 结果 |

点云颜色通常按 Intensity/AxisColor 显示，不同颜色不自动代表“可通行/障碍”。判断必须基于具体字段、分类话题或 cost 值。

## 9. 快速状态命令

```bash
ros2 topic info /fastlio2/world_cloud --verbose
ros2 topic info /my_nav/odom --verbose
ros2 topic info /far_eval/way_point --verbose
ros2 topic info /my_nav/costmap --verbose
ros2 topic info /hric/robot/cmd_vel --verbose

timeout 8 ros2 topic echo /my_nav/odom --once --field header
timeout 8 ros2 topic echo /far_eval/way_point --once
timeout 8 ros2 topic echo /viz_path_topic --once --field points
timeout 8 ros2 topic echo /hric/robot/rl_state --once
```

