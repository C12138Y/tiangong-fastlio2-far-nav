# 上游版本锁定

以下 commit 来自 2026-10-20 机器人源码快照。

| 工程 | 上游仓库 | 分支 | 实机 commit | 本项目处理方式 |
|---|---|---|---|---|
| FAST-LIO2 ROS 2 | `https://github.com/liangheming/FASTLIO2_ROS2.git` | `main` | `7baa59f7339ba88ce3b646fe35b23f61031c5d3f` | 克隆后应用 `patches/fastlio2.patch`，再复制 `extensions/fastlio2/pc2_to_custom.cpp` |
| terrain analysis | `https://github.com/HongbiaoZ/autonomous_exploration_development_environment.git` | `jazzy` | `8313dfed10533787582be8a6044483fe3299622f` | 克隆后应用 `patches/terrain_analysis.patch` |
| FAR Planner | `https://github.com/MichaelFYang/far_planner.git` | `humble-jazzy` | `29fb215886df4b0c53c27f487c51041f808c5eba` | 无工作区修改，直接检出固定 commit |
| fastlio_nav_adapter | 本项目代码 | — | 非独立 Git 仓库 | 完整包保存在 `ros2_packages/fastlio_nav_adapter` |

不要只依赖分支名复现。上游分支可能继续变化，应检出表中的 commit。

快照还确认：

- FAST-LIO2 有 3 个已修改文件和 1 个新增转换器。
- terrain analysis 有 2 个已修改源文件。
- FAR Planner 工作区干净。
- `/home/nvidia/tiangong_nav_ws` 不是 Git 仓库，因此 adapter 以完整 ROS 包归档。

