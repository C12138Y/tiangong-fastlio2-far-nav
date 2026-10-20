# 依赖与复现边界

本文档说明哪些内容能够仅依赖本仓库复现，哪些内容仍依赖天工机器人和厂家运行环境。

## 已验证平台

| 项目 | 已验证配置 |
|---|---|
| 操作系统 | Ubuntu 24.04，ARM64 |
| ROS | ROS 2 Jazzy |
| 机器人 | 天工 3.0 |
| 激光雷达 | Livox MID360 |
| 定位 | FAST-LIO2 |
| 全局规划 | FAR Planner / V-Graph |
| 局部环境 | Nav2 rolling voxel costmap |
| 控制接口 | `geometry_msgs/msg/TwistStamped`，话题 `/hric/robot/cmd_vel` |

上游仓库和精确 commit 记录在 [UPSTREAM_VERSIONS.md](UPSTREAM_VERSIONS.md)。

## 开源侧依赖

- ROS 2 Jazzy 基础开发环境与 `colcon`。
- FAST-LIO2 及其依赖（Eigen、PCL、livox_ros_driver2 等）。
- FAR Planner 和 CMU terrain analysis 的依赖。
- Nav2 costmap 相关包。
- Python 3、`rclpy`、ROS 2 标准消息包。

推荐先运行 `scripts/bootstrap_ros_sources.sh` 拉取锁定版本并应用补丁，再按 `docs/01_FAST-LIO2安装部署.md` 和 `docs/02_FAR_Planner安装部署.md` 编译。

## 天工实机专有依赖

以下内容不随本仓库发布，实机运动时必须由目标机器人已有环境提供：

- HRIC 运动控制程序和硬件通信桥。
- `hric_msgs`、`bodyctrl_msgs` 等厂家消息/服务包。
- `/hric/robot/cmd_vel` 的实际执行端。
- 遥控器、SWR 状态机和实体急停链路。
- 雷达与机器人本体外参、设备标定和固件配置。

没有这些依赖时，仍可编译和验证开源算法链路，但不能据此直接驱动天工机器人。

## 复现等级

| 目标 | 当前支持情况 |
|---|---|
| 阅读技术路线与接口定义 | 支持 |
| 拉取同版本上游源码并应用补丁 | 支持 |
| 编译 FAST-LIO2、terrain、FAR 和适配包 | 支持，但需安装系统依赖 |
| 运行 Python 单元测试 | 支持，不要求连接机器人 |
| 用自备 rosbag 做离线回放 | 支持，仓库暂未附带公开样例 bag |
| 在相同天工 3.0 环境部署 | 支持，需厂家工作区和实机标定 |
| 在普通电脑上直接控制机器人 | 不支持 |
| 生产级无人值守避障 | 不支持，仍处于现场试验阶段 |

## 未纳入 Git 的内容

- `build/`、`install/`、`log/` 等编译产物。
- 地图、rosbag、诊断数据和现场日志。
- 机器人完整源码快照与厂家私有代码。
- 账号口令、私钥和现场网络信息。

源码快照仅用于私有备份；仓库通过锁定上游 commit、补丁、自研 ROS 2 包和脚本重建必要修改。
