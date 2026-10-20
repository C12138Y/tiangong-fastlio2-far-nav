# 配置说明

## FAST-LIO2

机器人源配置位置：

```text
~/fastlio2_ws/src/FASTLIO2_ROS2_http11/fastlio2/config/lio.yaml
```

需要合并的核心值见 `fastlio2_lio.required.yaml`。该文件是配置片段，不保证可以直接替换上游完整 `lio.yaml`。

此外，FAST-LIO2 的 `lio_node` 必须链接：

```cmake
Sophus::Sophus
```

并构建独立 `pc2_to_custom` 可执行文件。最终机器人端源码应通过 `scripts/collect_robot_sources.sh` 归档。

## FAST-LIO adapter

机器人源码：

```text
~/tiangong_nav_ws/src/fastlio_nav_adapter
```

需要提供：

- `/my_nav/odom`
- TF `fastlio_odom -> my_nav_base_link`
- `local_costmap.launch.py`
- voxel costmap 的 YAML

当前 costmap 已知参数和低矮障碍问题见 `docs/04_问题诊断与解决清单.md`。

## Terrain analysis

检查以下文件中是否仍硬编码输出 frame 为 `map`：

```text
~/far_planner_eval_ws/cmu_dev_jazzy/src/terrain_analysis/src/terrainAnalysis.cpp
~/far_planner_eval_ws/cmu_dev_jazzy/src/terrain_analysis_ext/src/terrainAnalysisExt.cpp
```

本项目要求输出 frame 为 `fastlio_odom`。

## FAR Planner

当前不是修改 FAR 默认配置文件，而是在启动时显式传递：

```text
world_frame:=fastlio_odom
/odom_world:=/my_nav/odom
/terrain_cloud:=/far_eval/terrain_map_ext
/scan_cloud:=/far_eval/terrain_map
/terrain_local_cloud:=/fastlio2/world_cloud
/goal_point:=/far_eval/goal_point
/way_point:=/far_eval/way_point
```

这些参数已固化在 `scripts/stack/far_stack_start.sh`。

## 环境文件

`runtime.env.example` 只记录工作区路径，供阅读和 CI 检查使用。ROS overlay 必须按顺序 `source`，不能简单把 setup 文件当普通变量执行。

