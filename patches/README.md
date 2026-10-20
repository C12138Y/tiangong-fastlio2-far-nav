# 上游补丁说明

## `fastlio2.patch`

基于 FAST-LIO2 commit：

```text
7baa59f7339ba88ce3b646fe35b23f61031c5d3f
```

内容：

- 创建并安装 `pc2_to_custom` 可执行文件。
- 给 converter 添加 ROS 依赖。
- `lio_node` 链接 `Sophus::Sophus`。
- 输入改为 `/fastlio2/livox_custom`。
- frame 改为 `fastlio_body/fastlio_odom`。
- RViz 添加 costmap、FAR graph/path、terrain、障碍点云和局部 A* 路径显示。

注意：Git patch 不包含当时尚未被 Git 跟踪的 `pc2_to_custom.cpp`，所以该文件单独保存在：

```text
extensions/fastlio2/pc2_to_custom.cpp
```

## `terrain_analysis.patch`

基于 autonomous exploration environment commit：

```text
8313dfed10533787582be8a6044483fe3299622f
```

内容不仅包括输出 frame 从 `map` 改为 `fastlio_odom`，还包含机器人姿态处理方面的实机修改。应用前应阅读完整 diff，不要把它当成适用于所有机器人的通用补丁。

## 应用

可以使用：

```bash
git apply --check /path/to/fastlio2.patch
git apply /path/to/fastlio2.patch
```

或运行仓库中的：

```bash
scripts/bootstrap_ros_sources.sh
```

