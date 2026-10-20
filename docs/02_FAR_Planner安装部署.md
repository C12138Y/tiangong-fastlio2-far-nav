# FAR Planner 与 Terrain Analysis 安装部署

## 1. 获取 FAR Planner

```bash
source /home/nvidia/hric_tools/hric_env.sh
mkdir -p ~/far_planner_eval_ws
cd ~/far_planner_eval_ws

git -c http.version=HTTP/1.1 clone \
  --depth 1 --branch humble-jazzy \
  https://github.com/MichaelFYang/far_planner.git far_planner

cd far_planner
colcon list --base-paths src
```

应看到的主要包：

- `far_planner`
- `graph_decoder`
- `visibility_graph_msg`
- `boundary_handler`
- `goalpoint_rviz_plugin`
- `teleop_rviz_plugin`

## 2. 处理 `srv_msgs` 依赖问题

部署时发现 `package.xml` 中存在 `srv_msgs`，但实际源代码和 CMake 未使用它；这属于残留依赖。先检查再处理：

```bash
grep -Rns 'srv_msgs' ~/far_planner_eval_ws/far_planner/src
```

如果仍然只有清单声明、没有源码或 CMake 引用，可移除该无效依赖或在当前固定源码状态下按已验证包集合构建。不要在未知的新版本上直接照搬此结论。

```bash
cd ~/far_planner_eval_ws/far_planner
source /home/nvidia/hric_tools/hric_env.sh

colcon build --symlink-install \
  --packages-up-to far_planner graph_decoder \
  --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 3. 获取并构建 terrain analysis

```bash
cd ~/far_planner_eval_ws

git -c http.version=HTTP/1.1 clone \
  --depth 1 --single-branch --branch jazzy \
  https://github.com/HongbiaoZ/autonomous_exploration_development_environment.git \
  cmu_dev_jazzy

cd cmu_dev_jazzy
colcon list --base-paths src | grep -E '^terrain_analysis(_ext)?[[:space:]]'

source /home/nvidia/hric_tools/hric_env.sh
colcon build --symlink-install \
  --packages-select terrain_analysis terrain_analysis_ext \
  --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 硬编码 frame 修正

上游代码曾把输出 frame 写死为 `map`，与本链路的 `fastlio_odom` 不一致。已验证需要检查并修改：

```text
~/far_planner_eval_ws/cmu_dev_jazzy/src/terrain_analysis/src/terrainAnalysis.cpp
~/far_planner_eval_ws/cmu_dev_jazzy/src/terrain_analysis_ext/src/terrainAnalysisExt.cpp
```

将输出点云 header 的硬编码 `map` 改成 `fastlio_odom`，重新构建：

```bash
cd ~/far_planner_eval_ws/cmu_dev_jazzy
colcon build --symlink-install \
  --packages-select terrain_analysis terrain_analysis_ext
```

编译中 `terrainAnalysis.cpp` 某处关于 `&&` 与 `||` 优先级的提示是 warning，不是本次构建失败原因；但它是应当后续人工审查的逻辑风险，不能长期视为无害。

机器人端这两个仓库可能还存在其他本地修改。应使用：

```bash
git -C ~/far_planner_eval_ws/cmu_dev_jazzy status --short
git -C ~/far_planner_eval_ws/cmu_dev_jazzy --no-pager diff
git -C ~/far_planner_eval_ws/far_planner status --short
git -C ~/far_planner_eval_ws/far_planner --no-pager diff
```

若终端停在 `:`，这是分页器，不是卡死，按 `q` 退出；或者始终使用 `--no-pager`。

## 4. 启动 terrain analysis

终端 D：

```bash
source /home/nvidia/hric_tools/hric_env.sh
source ~/fastlio2_ws/install/setup.bash
source ~/far_planner_eval_ws/cmu_dev_jazzy/install/setup.bash
unset ROS_DOMAIN_ID

ros2 run terrain_analysis terrainAnalysis \
  --ros-args \
  -r /state_estimation:=/my_nav/odom \
  -r /registered_scan:=/fastlio2/world_cloud \
  -r /terrain_map:=/far_eval/terrain_map
```

终端 E：

```bash
source /home/nvidia/hric_tools/hric_env.sh
source ~/fastlio2_ws/install/setup.bash
source ~/far_planner_eval_ws/cmu_dev_jazzy/install/setup.bash
unset ROS_DOMAIN_ID

ros2 run terrain_analysis_ext terrainAnalysisExt \
  --ros-args \
  -r /state_estimation:=/my_nav/odom \
  -r /registered_scan:=/fastlio2/world_cloud \
  -r /terrain_map:=/far_eval/terrain_map \
  -r /terrain_map_ext:=/far_eval/terrain_map_ext
```

验证：

```bash
timeout 10 ros2 topic echo /far_eval/terrain_map --once --field width
timeout 10 ros2 topic echo /far_eval/terrain_map_ext --once --field width
timeout 10 ros2 topic echo /far_eval/terrain_map_ext --once --field header
```

frame 应为 `fastlio_odom`。

## 5. 启动 FAR Planner

上游默认话题名与本项目不同，必须显式重映射：

```bash
source /home/nvidia/hric_tools/hric_env.sh
source ~/fastlio2_ws/install/setup.bash
source ~/tiangong_nav_ws/install/setup.bash
source ~/far_planner_eval_ws/cmu_dev_jazzy/install/setup.bash
source ~/far_planner_eval_ws/far_planner/install/setup.bash
unset ROS_DOMAIN_ID

ros2 run far_planner far_planner \
  --ros-args \
  -p world_frame:=fastlio_odom \
  -r /odom_world:=/my_nav/odom \
  -r /terrain_cloud:=/far_eval/terrain_map_ext \
  -r /scan_cloud:=/far_eval/terrain_map \
  -r /terrain_local_cloud:=/fastlio2/world_cloud \
  -r /goal_point:=/far_eval/goal_point \
  -r /way_point:=/far_eval/way_point
```

关键映射：

| FAR 默认接口 | 本项目接口 | 作用 |
|---|---|---|
| `/odom_world` | `/my_nav/odom` | 机器人位姿 |
| `/terrain_cloud` | `/far_eval/terrain_map_ext` | 扩展地形点云 |
| `/scan_cloud` | `/far_eval/terrain_map` | 地形点云 |
| `/terrain_local_cloud` | `/fastlio2/world_cloud` | 当前帧 world 点云 |
| `/goal_point` | `/far_eval/goal_point` | 全局目标 |
| `/way_point` | `/far_eval/way_point` | 当前导航航点 |

## 6. FAR 参数和当前实测值

```bash
ros2 param get /far_planner world_frame
ros2 param get /far_planner local_planner_range
ros2 param get /far_planner terrain_range
ros2 param get /far_planner is_attempt_autoswitch
```

本轮观测值：

- `world_frame = fastlio_odom`
- `local_planner_range = 5.0`
- `terrain_range = 15.0`
- `is_attempt_autoswitch = true`

terrain 节点观测值：

| 节点 | 参数 | 观测值 |
|---|---|---:|
| `/terrainAnalysis` | `minRelZ` | -1.5 |
| `/terrainAnalysis` | `maxRelZ` | 0.2 |
| `/terrainAnalysis` | `disRatioZ` | 0.2 |
| `/terrainAnalysis` | `vehicleHeight` | 1.5 |
| `/terrainAnalysisExt` | `lowerBoundZ` | -1.5 |
| `/terrainAnalysisExt` | `upperBoundZ` | 1.0 |
| `/terrainAnalysisExt` | `disRatioZ` | 0.1 |
| `/terrainAnalysisExt` | `vehicleHeight` | 1.5 |

这些是当时运行值，不是已经标定完成的推荐值。尤其 `vehicleHeight`、`robot_dim`、高度阈值必须按天工本体和雷达安装实测。

## 7. FAR 独立验收

```bash
ros2 node info /far_planner
ros2 param get /far_planner world_frame
ros2 topic info /far_eval/way_point --verbose
ros2 topic info /far_reach_goal_status --verbose

timeout 10 ros2 topic echo /FAR_obs_debug --once --field width
timeout 20 ros2 topic echo /far_eval/way_point --once
timeout 20 ros2 topic echo /viz_path_topic --once --field points
```

正确理解：

- `Publisher count: 1` 只说明端点存在，不说明刚刚产生了新路径。
- 必须同时观察目标消息、航点时间戳、FAR 终端日志和 `/viz_path_topic`。
- `/viz_path_topic` 是 `visualization_msgs/msg/Marker`，不是 `MarkerArray`。
- `/viz_graph_topic` 才用于显示 V-Graph，通常是 `MarkerArray`。
- 只有两个点的蓝色粗线往往只是起点到目标的直连，不足以证明避障成功。

## 8. 目标发布方式

FAR 原生接收 `PointStamped`：

```bash
ros2 topic pub --once \
  /far_eval/goal_point \
  geometry_msgs/msg/PointStamped \
  "{header: {frame_id: fastlio_odom}, point: {x: 1.745, y: 7.972, z: -0.640}}"
```

RViz 的两个工具不同：

- `Publish Point` 发布 `PointStamped`，可以直接设为 `/far_eval/goal_point`，但只能点击当前视图中可投射的表面。
- `2D Goal Pose` 发布 `PoseStamped` 到 `/goal_pose`，FAR 不直接订阅。当前 `far_waypoint_follower_trial.py` 会把它转换为 `/far_eval/goal_point`。

因此，脚本运行时可用 `2D Goal Pose`；脚本未运行时，直接点击 `/goal_pose` 不会触发 FAR。

