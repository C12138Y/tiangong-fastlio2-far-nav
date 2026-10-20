# FAR 航点原地转向试验（2026-10-19）

这是受限的运控接口试验，不是完整导航。遥控 bag 显示遥控原地转向使用 `v=0`、`angular.z` 常见约 0.3–0.6 rad/s，指令间隔约 23 ms；原 FAR 跟随脚本的弧线转向仅约 0.1 rad/s、约 100 ms 间隔，未观察到有效航向变化。一次脚本 `0.30 rad/s` 试验在约 4° 后停滞，而遥控 bag 中约 `-0.27 rad/s` 也曾持续转动，因此不能断定速度是唯一原因。新版默认试验角速度为 `0.50 rad/s`、可配置范围为 `0.20–0.60 rad/s`，每秒打印 `cmd_vel_status` 角速度以区分命令回读与实际航向。`/hric/robot/walk_speed` 在遥控真实转向期间也一直为零，因此以 FAST-LIO 里程计航向作为试验判据。

新版 `far_waypoint_follower_trial.py` 增加 `--turn-test-only`，须显式指定才启用。该模式仅原地转向，不跟踪前进；目标应在空旷区域、距机器人 0.2–1.0 m（上限由 `--max-route-distance` 指定）、绝对偏角 10–150°。预检会打印实测距离和偏角及具体拒绝原因。转向方向只取本次 RViz 目标与 FAST-LIO 里程计，不等待 FAR 航点；因此它只验证运控原地转向，不能证明 FAR 路线可行。普通 FAR 航点跟随模式的航点检查未放宽。单次默认最多转 10°；平移达到 0.05 m、航向反向超过 2°、5 秒未有效转向、反馈/定位过期、速度话题端点变化或超时均停止，并连续发布零速约 2 秒。目标发令频率约 40 Hz，实际频率在日志末尾显示。

先将新版作为 `/home/nvidia/hric_tools/far_waypoint_follower_turntest.py` 上传到机器人，保留原有脚本，并用 `python3 /home/nvidia/hric_tools/far_waypoint_follower_turntest.py --help` 确认有 `--turn-test-only`。不要同时运行新旧两个跟随脚本。所有终端使用同一 ROS 环境和 `ROS_DOMAIN_ID`。现场人员需核对环境空旷、E 上拨、机器人静止且手持实体急停。

先预检（不会发非零速度），启动后在 RViz 点一个新的目标：

```bash
python3 /home/nvidia/hric_tools/far_waypoint_follower_turntest.py \
  --turn-test-only --max-route-distance 1.0 --max-travel 0.2 \
  --turn-angular-speed 0.50 --max-turn-degrees 10 --timeout 12
```

仅在现场再次确认方向及安全后，添加 `--execute` 运行新的一次试验，并按提示输入大写 `GO`。在结果证明转向可靠前，不要把该模式扩展成连续 FAR 自动导航，不要直接使用遥控器峰值角速度。

本地只验证了 Python 逻辑和参数解析；未在机器人上验证角速度执行、实际发令频率或停机距离。
