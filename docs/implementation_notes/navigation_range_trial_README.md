# FAR 最大目标范围的短程试验

`--max-route-distance 5.0` 是 FAR 已确认的 `local_planner_range`，表示本次允许选择距机器人最多 5 m 的目标，不代表允许机器人走 5 m。新版脚本未指定 `--max-travel` 时默认只允许累计行走 0.2 m；显式指定更长距离仍须现场人员另行判断。普通跟随模式的转向上限仍为 `--max-angular-speed 0.30 rad/s`，不同于原地转向试验的 `--turn-angular-speed`。先在空旷区域进行预检，观察 FAR 航点是否落在范围内，不应把视觉上晃动或 FAST-LIO 的瞬时航向变化当作已成功转身。

已有机器人脚本可先不上传新版，用命令行显式设置同样的目标与行走上限：

```bash
python3 /home/nvidia/hric_tools/far_waypoint_follower_turntest.py \
  --max-route-distance 5.0 --max-travel 0.2 \
  --max-angular-speed 0.30 --timeout 20
```

该命令无 `--execute`，只预检。现场确认目标及第一航点在已扫描的空旷区域后，才能对新目标考虑添加 `--execute`；仍保留 GO、速度端点检查、定位/反馈时效和零速停机。尚未验证该机器人能按 FAR 航点完成转弯，因此不应试验 5 m 实际行走。
