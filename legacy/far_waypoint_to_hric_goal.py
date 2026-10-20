#!/usr/bin/env python3
"""Capture one FAR waypoint and print a guarded HRIC map-frame goal.

This tool NEVER calls StartNav, StopNav, or publishes velocity commands.
It uses simultaneous robot poses only as a provisional 2-D alignment; the
operator must verify the candidate in the HRIC map before running it.
"""

import argparse
import math
import shlex
import statistics
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_of(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def convert_waypoint(waypoint, fast_pose, hric_pose):
    """Map-relative robot displacement, assuming aligned robot body axes."""
    fast_yaw = yaw_of(fast_pose.orientation)
    hric_yaw = yaw_of(hric_pose.orientation)
    rotation = wrap(hric_yaw - fast_yaw)
    dx = waypoint.x - fast_pose.position.x
    dy = waypoint.y - fast_pose.position.y
    x = hric_pose.position.x + math.cos(rotation) * dx - math.sin(rotation) * dy
    y = hric_pose.position.y + math.sin(rotation) * dx + math.cos(rotation) * dy
    return x, y, math.hypot(dx, dy), rotation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-distance", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=40.0)
    parser.add_argument(
        "--goto-script",
        default="/home/nvidia/hric_tools/hric_goto_xy.py",
        help="Existing, previously tested HRIC one-goal script; only printed, never run",
    )
    args = parser.parse_args()
    if not 0.3 <= args.max_distance <= 2.0 or args.timeout <= 3.0:
        parser.error("max-distance must be 0.3..2.0 m; timeout must exceed 3 s")

    rclpy.init()
    node = rclpy.create_node("far_hric_one_goal_preview")
    latest = {"goal": None, "waypoint": None, "fast": None, "hric": None}
    received = {key: 0.0 for key in latest}
    samples = []

    def update(key):
        def callback(message):
            latest[key] = message
            received[key] = time.monotonic()
        return callback

    cloud_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = [
        node.create_subscription(PointStamped, "/far_eval/goal_point", update("goal"), 10),
        node.create_subscription(PointStamped, "/far_eval/way_point", update("waypoint"), 10),
        node.create_subscription(Odometry, "/my_nav/odom", update("fast"), cloud_qos),
        node.create_subscription(Odometry, "/hric/loc/pose", update("hric"), cloud_qos),
    ]
    print("等待 RViz 在 /far_eval/goal_point 发布一个新的目标点；不会触发机器人运动。", flush=True)
    start = time.monotonic()
    goal_received_at = None
    try:
        while time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if latest["goal"] is None:
                continue
            if goal_received_at is None:
                goal_received_at = received["goal"]
                print("收到新目标，等待 FAR 输出其后续航点及两路定位...", flush=True)
            if latest["waypoint"] is None or received["waypoint"] < goal_received_at:
                continue
            if latest["fast"] is None or latest["hric"] is None:
                continue
            if any(now - received[key] > 0.6 for key in ("waypoint", "fast", "hric")):
                continue
            goal = latest["goal"]
            waypoint = latest["waypoint"]
            fast = latest["fast"]
            hric = latest["hric"]
            if (goal.header.frame_id != "fastlio_odom" or
                    waypoint.header.frame_id != "fastlio_odom" or
                    fast.header.frame_id != "fastlio_odom" or
                    hric.header.frame_id != "map"):
                raise RuntimeError("frame_id 不符合预期；需要 FAR/FAST=fastlio_odom、HRIC=map")
            if fast.child_frame_id != "my_nav_base_link" or hric.child_frame_id != "base_link":
                raise RuntimeError("机器人 body frame 不符合预期；先核对两种 base 外参")
            candidate = convert_waypoint(waypoint.point, fast.pose.pose, hric.pose.pose)
            if not all(math.isfinite(v) for v in candidate):
                raise RuntimeError("收到非有限坐标")
            samples.append((now, *candidate, fast.pose.pose.position.x,
                            fast.pose.pose.position.y, hric.pose.pose.position.x,
                            hric.pose.pose.position.y))
            if now - samples[0][0] < 2.0 or len(samples) < 10:
                continue
            break
        else:
            raise RuntimeError("超时：需要重新点一个目标并检查 FAR 航点及两路定位发布者")

        recent = samples[-30:]
        map_x = statistics.median(s[1] for s in recent)
        map_y = statistics.median(s[2] for s in recent)
        distance = statistics.median(s[3] for s in recent)
        spread = max(math.hypot(s[1] - map_x, s[2] - map_y) for s in recent)
        fast_motion = math.hypot(recent[-1][5] - recent[0][5], recent[-1][6] - recent[0][6])
        hric_motion = math.hypot(recent[-1][7] - recent[0][7], recent[-1][8] - recent[0][8])
        if distance < 0.3 or distance > args.max_distance:
            raise RuntimeError(f"候选航点距离 {distance:.2f}m，不在 0.3..{args.max_distance:.2f}m 内")
        if spread > 0.08 or fast_motion > 0.05 or hric_motion > 0.05:
            raise RuntimeError("两秒观测不稳定或机器人在移动；不要发送导航目标")

        print(f"FAR 航点 fastlio_odom: ({waypoint.point.x:.3f}, {waypoint.point.y:.3f})")
        print(f"HRIC 候选目标 map:    ({map_x:.3f}, {map_y:.3f})")
        print(f"相对距离: {distance:.3f} m；观测波动: {spread:.3f} m")
        print("注意：这是双定位位姿推算的候选值，尚未证明两种 base 朝向/外参一致。")
        print("必须在 HRIC 地图上核对位置、障碍、机器人足迹和实际通道，再由现场人员决定是否执行。")
        print("仅供核对的旧脚本命令（本工具不会运行）：")
        print(f"python3 {shlex.quote(args.goto_script)} --x {map_x:.3f} --y {map_y:.3f} --timeout 30")
        return 0
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(f"未生成可执行目标：{exc}")
        return 1
    finally:
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
