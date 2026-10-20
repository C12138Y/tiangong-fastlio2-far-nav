#!/usr/bin/env python3
"""Read-only FAR waypoint check in the FAST-LIO robot frame.

This program creates subscriptions only. It never publishes a velocity, calls a
motion service, or arms a controller. Start it, then click a *new* RViz goal.
"""

import argparse
import math
import statistics
import time


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def waypoint_in_body(waypoint, position, yaw):
    dx = waypoint.x - position.x
    dy = waypoint.y - position.y
    return (
        math.cos(yaw) * dx + math.sin(yaw) * dy,
        -math.sin(yaw) * dx + math.cos(yaw) * dy,
    )


def timeout_diagnosis(received, counts, goal_time, now, sample_count):
    def age(key):
        return "never" if counts[key] == 0 else f"{now - received[key]:.1f}s ago"

    details = (
        f"goal={counts['goal']} ({age('goal')}), "
        f"waypoint={counts['waypoint']} ({age('waypoint')}), "
        f"odom={counts['odom']} ({age('odom')}), samples={sample_count}"
    )
    if goal_time is None:
        reason = "没有收到本次 RViz 新目标"
    elif counts["waypoint"] == 0 or received["waypoint"] < goal_time:
        reason = "目标已收到，但 FAR 未在目标之后发布航点"
    elif counts["odom"] == 0:
        reason = "收到航点，但 /my_nav/odom 没有消息"
    elif now - received["odom"] > 0.8:
        reason = "里程计曾到达，但已超过 0.8 秒新鲜度限制"
    else:
        reason = "数据偶尔同时新鲜，但未积累足够的两秒稳定样本"
    return f"{reason}；接收统计：{details}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=40.0)
    parser.add_argument("--min-distance", type=float, default=0.3)
    parser.add_argument("--max-distance", type=float, default=1.0)
    args = parser.parse_args()
    if args.timeout <= 3.0 or not 0.0 < args.min_distance < args.max_distance:
        parser.error("timeout must exceed 3 seconds; require 0 < min-distance < max-distance")

    import rclpy
    from geometry_msgs.msg import PointStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("far_waypoint_shadow_check")
    latest = {"goal": None, "waypoint": None, "odom": None}
    received = {key: 0.0 for key in latest}
    counts = {key: 0 for key in latest}

    def update(key):
        def callback(msg):
            latest[key] = msg
            received[key] = time.monotonic()
            counts[key] += 1
        return callback

    odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = [
        node.create_subscription(PointStamped, "/far_eval/goal_point", update("goal"), 10),
        node.create_subscription(PointStamped, "/far_eval/way_point", update("waypoint"), 10),
        node.create_subscription(Odometry, "/my_nav/odom", update("odom"), odom_qos),
    ]
    print("只读模式：请在 RViz 点击一个新的 FAR 目标；不会让机器人运动。", flush=True)
    goal_time = None
    samples = []
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if latest["goal"] is None:
                continue
            if goal_time is None:
                goal_time = received["goal"]
                print("收到新目标，等待 FAR 的后续航点和 FAST-LIO 里程计...", flush=True)
            if latest["waypoint"] is None or received["waypoint"] < goal_time:
                continue
            if latest["odom"] is None:
                continue
            # FAR may publish a single waypoint for a goal. Keep that post-goal
            # waypoint while collecting fresh odometry for the stability check.
            if now - received["odom"] > 0.8:
                continue

            goal = latest["goal"]
            waypoint = latest["waypoint"]
            odom = latest["odom"]
            if (goal.header.frame_id != "fastlio_odom" or
                    waypoint.header.frame_id != "fastlio_odom" or
                    odom.header.frame_id != "fastlio_odom" or
                    odom.child_frame_id != "my_nav_base_link"):
                raise RuntimeError("frame_id 不符；应为 fastlio_odom / my_nav_base_link")
            position = odom.pose.pose.position
            yaw = yaw_from_quaternion(odom.pose.pose.orientation)
            forward, left = waypoint_in_body(waypoint.point, position, yaw)
            values = (forward, left, position.x, position.y, yaw)
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError("位置或朝向不是有限数值")
            samples.append((now, *values))
            if len(samples) >= 10 and now - samples[0][0] >= 2.0:
                break
        else:
            raise RuntimeError(timeout_diagnosis(
                received, counts, goal_time, time.monotonic(), len(samples)
            ))

        recent = samples[-30:]
        forward = statistics.median(s[1] for s in recent)
        left = statistics.median(s[2] for s in recent)
        distance = math.hypot(forward, left)
        angle = math.degrees(math.atan2(left, forward))
        robot_x = statistics.median(s[3] for s in recent)
        robot_y = statistics.median(s[4] for s in recent)
        goal_distance = math.hypot(goal.point.x - robot_x, goal.point.y - robot_y)
        spread = max(math.hypot(s[1] - forward, s[2] - left) for s in recent)
        if spread > 0.08:
            raise RuntimeError(f"两秒内相对航点波动 {spread:.3f} m；先保持机器人静止")

        print(f"机器人 fastlio_odom: ({robot_x:.3f}, {robot_y:.3f})")
        print(f"RViz 目标 fastlio_odom: ({goal.point.x:.3f}, {goal.point.y:.3f})；距机器人 {goal_distance:.3f} m")
        print(f"FAR 航点 fastlio_odom: ({waypoint.point.x:.3f}, {waypoint.point.y:.3f})")
        print(f"相对 my_nav_base_link: 前方 {forward:+.3f} m，左方 {left:+.3f} m")
        print(f"平面距离 {distance:.3f} m，平面方向 {angle:+.1f}°，观测波动 {spread:.3f} m")
        if distance < args.min_distance:
            print(f"拒绝作为行走航点：距机器人不足 {args.min_distance:.2f} m；检查 FAR 规划结果。")
            return 2
        if distance > args.max_distance:
            print(f"超出 {args.max_distance:.2f} m 的初测范围；请选更近的空旷目标。")
        print("仅供坐标轴核对：未验证机器人前/左方向、外参、避障或停车机制；未发送运动命令。")
        return 0
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(f"未完成只读核对：{exc}")
        return 1
    finally:
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
