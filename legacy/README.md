# Legacy：HRIC 地图坐标候选转换

`far_waypoint_to_hric_goal.py` 是早期实验工具：读取 FAR 航点、FAST-LIO 里程计和 HRIC 地图定位，推算一个 HRIC `map` 坐标候选，并打印旧 `hric_goto_xy.py` 命令。

它不会主动触发机器人运动，但该方法依赖两套定位之间的平移、朝向和外参一致性；这些条件尚未被严格标定。因此它不属于当前无图导航主线，也不应直接用于生产导航。

当前主线使用：

```text
scripts/far_waypoint_follower_trial.py
```

