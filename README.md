# 天工 3.0 FAST-LIO2 + FAR Planner 无图导航集成

这是天工 3.0 机器人上的 FAST-LIO2、FAR Planner、局部代价地图和 HRIC 运控实验集成仓库。当前仓库整理的是已经实现并经过阶段性测试的**集成层代码**：启动脚本、目标/航点处理、无图速度跟踪、局部碰撞检查、局部 A*、测试和部署文档。

> 当前状态：研究与现场测试版本，不是生产级自主导航系统。低矮障碍识别和局部 A* 实机连续绕障仍未完成最终验收。

## 已实现功能

- 将 Livox `PointCloud2` 转换为 FAST-LIO2 使用的 `livox_ros_driver2/CustomMsg`。
- FAST-LIO2 实时里程计与当前帧点云。
- 私有坐标链 `fastlio_odom -> fastlio_body/my_nav_base_link`，避免与 HRIC 原生 TF 冲突。
- terrain analysis、FAR V-Graph、全局目标、航点和路径输出。
- RViz `2D Goal Pose` 到 FAR `PointStamped` 目标的转换。
- FAR 航点流到 `/hric/robot/cmd_vel` 的无图跟踪。
- 平台速度死区适配、弧线转向和转弯半径控制。
- GO 确认期间航点缓存和安全反馈刷新。
- FAR 提前到达信号与里程计距离的联合判定。
- Nav2 局部 costmap 碰撞扫掠、未知区域门控和 8 邻域 A*。
- `/my_nav/local_avoid_path` 可视化。
- A～F 感知/规划链路一键启动、状态检查和停止。
- 53 个纯 Python 单元测试。

## 技术链路

```text
/livox/lidar + /livox/imu
          │
          ├─ pc2_to_custom
          v
       FAST-LIO2 ──> /fastlio2/lio_odom
          │                    │
          │                    v
          │          fastlio_nav_adapter ──> /my_nav/odom
          │                                      │
          v                                      v
 /fastlio2/world_cloud                 terrain analysis
          │                                      │
          └──────────────────> FAR Planner <─────┘
                                      │
                           /far_eval/way_point
                                      │
       /my_nav/costmap ──> waypoint follower + local A*
                                      │
                           /hric/robot/cmd_vel
                                      │
                              HRIC motion bridge
```

## 仓库结构

```text
.
├── README.md
├── DEPENDENCIES.md
├── SHA256SUMS
├── UPSTREAM_VERSIONS.md
├── config/
│   ├── README.md
│   ├── fastlio2_lio.required.yaml
│   └── runtime.env.example
├── docs/
│   ├── 00_总览与阅读顺序.md
│   ├── 01_FAST-LIO2安装部署.md
│   ├── 02_FAR_Planner安装部署.md
│   ├── 03_完整链路重启与验收.md
│   ├── 04_问题诊断与解决清单.md
│   ├── 05_话题_TF_RViz_参数字典.md
│   ├── 06_代码资产清单与规范.md
│   ├── 07_当前结论与后续改进.md
│   ├── 08_历史资料与三段对话索引.md
│   ├── 09_机器人源码快照与工作区说明.md
│   └── implementation_notes/
├── legacy/
│   └── far_waypoint_to_hric_goal.py
├── extensions/
│   └── fastlio2/pc2_to_custom.cpp
├── patches/
│   ├── fastlio2.patch
│   └── terrain_analysis.patch
├── ros2_packages/
│   └── fastlio_nav_adapter/
├── scripts/
│   ├── collect_robot_sources.sh
│   ├── bootstrap_ros_sources.sh
│   ├── install_integration_tools.sh
│   ├── far_waypoint_shadow.py
│   ├── far_single_waypoint_trial.py
│   ├── far_waypoint_follower_trial.py
│   └── stack/
│       ├── far_stack_start.sh
│       ├── far_stack_status.sh
│       ├── far_stack_stop.sh
│       └── far_stack_rviz.sh
└── tests/
    ├── test_far_single_waypoint_trial.py
    └── test_far_waypoint_follower_trial.py
```

## 外部工程

本仓库没有重复提交以下完整上游工程：

- [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2)
- [FAR Planner](https://github.com/MichaelFYang/far_planner)
- [Autonomous Exploration Development Environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment)
- ROS 2、Nav2、Livox 驱动和 HRIC 厂家工作区

已从机器人快照提取并纳入仓库的项目代码包括：

- `ros2_packages/fastlio_nav_adapter`：完整自研 ROS 2 包和 costmap 配置。
- `extensions/fastlio2/pc2_to_custom.cpp`：Livox PointCloud2 到 CustomMsg 转换器。
- `patches/fastlio2.patch`：FAST-LIO2 构建、话题、TF 和 RViz 修改。
- `patches/terrain_analysis.patch`：terrain analysis 实机修改。

FAR Planner 工作区没有本地修改，因此只锁定上游 commit，不重复提交完整源码。版本和修改点见：

- [FAST-LIO2 安装部署](docs/01_FAST-LIO2安装部署.md)
- [FAR Planner 安装部署](docs/02_FAR_Planner安装部署.md)
- [配置说明](config/README.md)
- [上游版本锁定](UPSTREAM_VERSIONS.md)
- [补丁说明](patches/README.md)
- [机器人源码快照与工作区说明](docs/09_机器人源码快照与工作区说明.md)

要重新创建三个源码工作区，可在一台没有同名目录的新环境运行：

```bash
scripts/bootstrap_ros_sources.sh
```

要另外保存当前机器人端现场源码状态，可运行：

```bash
bash scripts/collect_robot_sources.sh
```

生成的压缩包用于私有备份，已被 `.gitignore` 排除。不要把厂家私有代码或大体积 bag 直接提交到公共仓库。

## 环境

实机已验证环境：

- Ubuntu 24.04 ARM64
- ROS 2 Jazzy
- 天工 3.0
- Livox MID360
- Python 3 / `rclpy`
- Nav2 costmap

完整依赖、专有组件和复现等级见 [DEPENDENCIES.md](DEPENDENCIES.md)。没有 HRIC 厂家运行环境时，可以复现感知、规划和离线控制逻辑，但不能直接驱动天工机器人。

每个机器人终端首先执行：

```bash
source /home/nvidia/hric_tools/hric_env.sh
unset ROS_DOMAIN_ID
```

## 快速安装集成脚本

在机器人上的仓库根目录执行：

```bash
chmod +x scripts/install_integration_tools.sh
scripts/install_integration_tools.sh /home/nvidia/hric_tools
```

安装脚本只复制本仓库的 Python/运维脚本；若目标文件已存在，会先保存带时间戳的备份。它不会安装或改写 FAST-LIO2、FAR、terrain、HRIC 厂家工程。

## 启动感知与规划栈

```bash
/home/nvidia/hric_tools/far_stack_start.sh
/home/nvidia/hric_tools/far_stack_status.sh
/home/nvidia/hric_tools/far_stack_rviz.sh
```

也可以按照 [完整链路重启与验收](docs/03_完整链路重启与验收.md) 分终端启动 A～G。

## 只读验证

```bash
python3 /home/nvidia/hric_tools/far_waypoint_shadow.py --max-distance 1.0
```

在 RViz 点击近距离空旷目标。该脚本不会创建速度发布者。

## 主跟踪器预检

```bash
python3 /home/nvidia/hric_tools/far_waypoint_follower_trial.py \
  --max-goal-distance 20.0 \
  --max-route-distance 20.0 \
  --max-travel 2.0 \
  --max-linear-speed 0.4 \
  --turn-linear-speed 0.4 \
  --arc-linear-speed 0.4 \
  --max-angular-speed 0.6 \
  --turn-radius 0.8 \
  --planner-timeout 30 \
  --timeout 60
```

不加 `--execute` 时不会创建速度发布者。

## 运动试验

运动前至少确认：

```bash
timeout 6 ros2 topic echo /hric/robot/rl_state --once
ros2 topic info /hric/robot/cmd_vel --verbose
ros2 lifecycle get /my_nav/local_costmap
```

`/hric/robot/cmd_vel` 不能同时存在 `hric_path_tracker` 和本脚本两个自主控制器。现场条件满足后，在上述主跟踪器命令中增加：

```text
--execute
```

脚本仍会要求人工输入大写 `GO`。

## 测试

这些测试只覆盖与 ROS 运行时解耦的控制和规划函数：

```bash
python3 -m py_compile scripts/*.py legacy/*.py
python3 -m unittest discover -s tests -p 'test_far_*.py' -v
bash -n scripts/*.sh scripts/stack/*.sh
```

当前结果：53 个 Python 测试通过。

代码和配置文件哈希记录在 `SHA256SUMS`。部署后可用 `sha256sum -c SHA256SUMS` 核对；目标机器上只复制部分文件时，也可以逐项核对对应行。

## 当前限制

- 低矮柜体可能被 terrain 或 voxel costmap 的高度窗口过滤。
- FAR 可能输出起点到终点的直连路径，不能只凭蓝色路径线判断可通行。
- FAR 不订阅 `/my_nav/costmap`；局部避障由跟踪器独立完成。
- 局部 A* 已实现并通过函数测试，但真实障碍下的连续绕行仍需继续验收。
- `0.4 m/s`、`0.6 rad/s` 是当前平台试验参数，不是通用安全参数。
- 最大目标范围 20 m 只是输入限制；实际运动仍应用 `--max-travel` 分段限制。
- 本项目当前不使用 HRIC 静态地图定位和原生 `StartNav`。

详细故障处理见 [问题诊断与解决清单](docs/04_问题诊断与解决清单.md)。

## 复现验收标准

建议按以下顺序验收，不要直接从编译跳到实机运动：

1. `bootstrap_ros_sources.sh` 能拉取锁定 commit 并应用两个补丁。
2. FAST-LIO2、terrain analysis、FAR Planner 和 `fastlio_nav_adapter` 均可编译。
3. 53 个 Python 单元测试全部通过。
4. 只读模式能收到 RViz 目标、FAST-LIO2 里程计和 FAR 航点。
5. RViz 中 TF、点云、terrain、V-Graph、路径和 costmap 坐标一致。
6. 实机端确认唯一自主速度发布者、SWR 状态和急停链路后，才进行分段运动测试。

## 上传 GitHub 前

1. 运行全部测试和 `bash -n`。
2. 确认没有机器人 IP、账号口令、地图、bag、厂家私有源码或日志。
3. 决定仓库可见性，并根据实际权利情况添加许可证；本整理未擅自添加许可证。
4. 记录机器人端上游仓库 commit 和本地 patch。
5. 若公开发布，审查 `legacy/` 是否仍有保留必要。

