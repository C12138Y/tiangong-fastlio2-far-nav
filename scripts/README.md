# 脚本说明

| 脚本 | 是否发速度 | 用途 |
|---|---|---|
| `far_waypoint_shadow.py` | 否 | 只读核对 FAR 航点相对机器人前/左方向 |
| `far_single_waypoint_trial.py` | 仅 `--execute` 后 | 单航点短距离试验 |
| `far_waypoint_follower_trial.py` | 仅 `--execute` 且输入 `GO` 后 | FAR 航点流、costmap、局部 A* 和 HRIC 运控主程序 |
| `stack/far_stack_start.sh` | 否 | 启动 converter、FAST-LIO2、adapter、terrain 和 FAR |
| `stack/far_stack_status.sh` | 否 | 检查 A～F 进程和关键话题 |
| `stack/far_stack_stop.sh` | 否 | 逆序停止本脚本管理的 A～F 进程 |
| `stack/far_stack_rviz.sh` | 否 | 打开 FAST-LIO2/FAR RViz 配置 |
| `install_integration_tools.sh` | 否 | 备份并安装本仓库脚本到机器人 |
| `collect_robot_sources.sh` | 否 | 收集机器人端源码、commit、diff 和哈希 |

运动前必须查看：

```bash
ros2 topic info /hric/robot/cmd_vel --verbose
timeout 6 ros2 topic echo /hric/robot/rl_state --once
```

不要让 `hric_path_tracker` 与本脚本同时成为自主速度发布者。

