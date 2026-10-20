#!/usr/bin/env python3
"""Guarded, short FAR waypoint motion trial; NOT a navigation controller.

Requires a fresh RViz goal, a subsequent FAR waypoint, FAST-LIO odometry,
velocity feedback, and a human typing GO. Commands only forward velocity and
yaw rate. The translation limit is configurable up to 1.0 m. Keep a human at
the physical emergency stop throughout this trial.
"""

import argparse
import math
import signal
import time


class MotionAccumulator:
    """Track net displacement separately from odometry path-length jitter."""

    def __init__(self, x, y):
        self.start_x = x
        self.start_y = y
        self.last_x = x
        self.last_y = y
        self.path_length = 0.0
        self.net_displacement = 0.0

    def update(self, x, y):
        step = math.hypot(x - self.last_x, y - self.last_y)
        self.path_length += step
        self.net_displacement = math.hypot(x - self.start_x, y - self.start_y)
        self.last_x = x
        self.last_y = y
        return step


def clamp(value, lo, hi):
    return max(lo, min(value, hi))


def yaw_of(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def body_error(point, pose):
    yaw = yaw_of(pose.orientation)
    dx = point.x - pose.position.x
    dy = point.y - pose.position.y
    forward = math.cos(yaw) * dx + math.sin(yaw) * dy
    left = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return forward, left, math.hypot(dx, dy), math.atan2(left, forward)


def all_fresh_after(received, keys, cutoff):
    return all(received[key] > cutoff for key in keys)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="allow motion after typing GO")
    parser.add_argument(
        "--max-travel", type=float, default=1.0,
        help="maximum net translation in metres (0.05..1.0; default: 1.0)",
    )
    parser.add_argument(
        "--max-waypoint-distance", type=float, default=1.5,
        help="largest accepted FAR waypoint distance (0.3..1.5; default: 1.5)",
    )
    parser.add_argument(
        "--max-linear-speed", type=float, default=0.05,
        help="maximum forward speed in m/s (0.02..0.4; default: 0.05)",
    )
    args = parser.parse_args()
    if not 0.05 <= args.max_travel <= 1.0:
        parser.error("--max-travel 必须在 0.05..1.0 m 内")
    if not 0.3 <= args.max_waypoint_distance <= 1.5:
        parser.error("--max-waypoint-distance 必须在 0.3..1.5 m 内")
    if not 0.02 <= args.max_linear_speed <= 0.4:
        parser.error("--max-linear-speed 必须在 0.02..0.4 m/s 内")

    import rclpy
    from diagnostic_msgs.msg import DiagnosticStatus
    from geometry_msgs.msg import PointStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions

    # Keep the ROS context alive on Ctrl+C/SIGTERM so finally can send zeros.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def on_terminate(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_terminate)
    signal.signal(signal.SIGTERM, on_terminate)
    node = rclpy.create_node("far_single_waypoint_trial")
    latest = {"goal": None, "waypoint": None, "odom": None, "feedback": None,
              "walk_speed": None, "rl_state": None}
    received = {key: 0.0 for key in latest}

    def callback_for(key):
        def callback(msg):
            latest[key] = msg
            received[key] = time.monotonic()
        return callback

    best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = [
        node.create_subscription(PointStamped, "/far_eval/goal_point", callback_for("goal"), 10),
        node.create_subscription(PointStamped, "/far_eval/way_point", callback_for("waypoint"), 10),
        node.create_subscription(Odometry, "/my_nav/odom", callback_for("odom"), best_effort),
        node.create_subscription(TwistStamped, "/hric/robot/cmd_vel_status", callback_for("feedback"), best_effort),
        node.create_subscription(TwistStamped, "/hric/robot/walk_speed", callback_for("walk_speed"), best_effort),
        node.create_subscription(DiagnosticStatus, "/hric/robot/rl_state", callback_for("rl_state"), best_effort),
    ]
    publisher = None
    moving = False

    def send_velocity(forward=0.0, yaw_rate=0.0):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.twist.linear.x = forward
        msg.twist.angular.z = yaw_rate
        publisher.publish(msg)

    def stop_and_drain():
        if publisher is None:
            return
        for _ in range(20):
            send_velocity()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.1)

    try:
        print("等待 RViz 点击一个新的空旷目标；执行前还需人工确认 GO。", flush=True)
        started = time.monotonic()
        goal_time = None
        while time.monotonic() - started < 40.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if latest["goal"] is None:
                continue
            if goal_time is None:
                goal_time = received["goal"]
                print("收到新目标，等待对应 FAR 航点...", flush=True)
            if (latest["waypoint"] is None or received["waypoint"] < goal_time or
                    now - goal_time < 1.0):
                continue
            if (latest["odom"] is None or latest["feedback"] is None or
                    latest["walk_speed"] is None or latest["rl_state"] is None):
                continue
            if (now - received["odom"] > 0.5 or now - received["feedback"] > 1.0 or
                    now - received["walk_speed"] > 1.0):
                continue
            break
        else:
            raise RuntimeError("超时：缺新目标后的航点、里程计、速度反馈或 FSM 状态")

        goal = latest["goal"]
        waypoint = latest["waypoint"]
        odom = latest["odom"]
        if (goal.header.frame_id != "fastlio_odom" or
                waypoint.header.frame_id != "fastlio_odom" or
                odom.header.frame_id != "fastlio_odom" or
                odom.child_frame_id != "my_nav_base_link"):
            raise RuntimeError("坐标系不符，拒绝运动")
        forward, left, distance, heading = body_error(waypoint.point, odom.pose.pose)
        if not all(math.isfinite(v) for v in (forward, left, distance, heading)):
            raise RuntimeError("位姿无效，拒绝运动")
        if not 0.3 <= distance <= args.max_waypoint_distance:
            raise RuntimeError(
                f"航点距离 {distance:.3f} m，不在 0.3..{args.max_waypoint_distance:.2f} m 内"
            )
        state = {item.key: item.value for item in latest["rl_state"].values}
        if state.get("current_state") != "SWR" or state.get("status") != "running":
            raise RuntimeError(f"机器人初始 FSM 状态不符：{state}")
        if (abs(latest["feedback"].twist.linear.x) > 0.02 or
                abs(latest["feedback"].twist.angular.z) > 0.02 or
                abs(latest["walk_speed"].twist.linear.x) > 0.02 or
                abs(latest["walk_speed"].twist.angular.z) > 0.02):
            raise RuntimeError("初始速度反馈不是零；机器人可能正在运动")

        existing = {ep.node_name for ep in node.get_publishers_info_by_topic("/hric/robot/cmd_vel")}
        consumers = {ep.node_name for ep in node.get_subscriptions_info_by_topic("/hric/robot/cmd_vel")}
        if existing != {"joystick_bridge_node"} or "hric_topic_bridge" not in consumers:
            raise RuntimeError(f"速度话题端点与预期不符：发布者={existing} 订阅者={consumers}")

        print(f"目标 ({goal.point.x:.3f}, {goal.point.y:.3f})；FAR 航点 ({waypoint.point.x:.3f}, {waypoint.point.y:.3f})")
        print(f"航点在机器人前方 {forward:+.3f} m、左方 {left:+.3f} m；距离 {distance:.3f} m、转角 {math.degrees(heading):+.1f}°")
        print(
            f"本次最多净平移 {args.max_travel:.2f} m，"
            f"线速度 ≤{args.max_linear_speed:.2f} m/s、"
            "角速度 ≤0.10 rad/s、最长 25 s。"
        )
        if not args.execute:
            print("仅预检，未创建速度发布者。需要运动时加 --execute 并由现场人员输入 GO。")
            return 0
        print("确认目标方向与现实一致、通道空旷、E 上拨且专人手持实体急停。", flush=True)
        if input("确认后输入大写 GO；其他输入取消：").strip() != "GO":
            print("已取消，未创建速度发布者。")
            return 1

        # input() blocks ROS callbacks. After confirmation, deliberately collect
        # a fresh message from every safety-critical stream before publishing.
        confirmation_time = time.monotonic()
        refresh_deadline = confirmation_time + 3.0
        required = ("odom", "feedback", "walk_speed", "rl_state")
        while time.monotonic() < refresh_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if all_fresh_after(received, required, confirmation_time):
                break
        else:
            missing = [key for key in required if received[key] <= confirmation_time]
            raise RuntimeError(f"GO 后 3 秒仍缺少新反馈：{missing}，未启动")

        now = time.monotonic()
        if (now - received["odom"] > 0.5 or now - received["feedback"] > 1.0 or
                now - received["walk_speed"] > 1.0):
            raise RuntimeError("GO 后刷新到的数据已过期，未启动")
        if received["goal"] > goal_time:
            raise RuntimeError("确认期间收到另一个 RViz 目标，请重新运行")
        if received["waypoint"] < goal_time:
            raise RuntimeError("GO 后没有当前目标对应的 FAR 航点")
        waypoint = latest["waypoint"]
        current_error = body_error(waypoint.point, latest["odom"].pose.pose)
        if abs(current_error[2] - distance) > 0.05:
            raise RuntimeError("确认期间机器人位置或 FAR 航点变化超过 0.05 m，未启动")
        if not 0.3 <= current_error[2] <= args.max_waypoint_distance:
            raise RuntimeError(
                f"GO 后航点距离 {current_error[2]:.3f} m "
                f"超出 0.3..{args.max_waypoint_distance:.2f} m"
            )
        state = {item.key: item.value for item in latest["rl_state"].values}
        if state.get("current_state") != "SWR" or state.get("status") != "running":
            raise RuntimeError(f"GO 后 FSM 状态不符：{state}")
        if (abs(latest["feedback"].twist.linear.x) > 0.02 or
                abs(latest["feedback"].twist.angular.z) > 0.02 or
                abs(latest["walk_speed"].twist.linear.x) > 0.02 or
                abs(latest["walk_speed"].twist.angular.z) > 0.02):
            raise RuntimeError("GO 后速度反馈不是零，未启动")
        existing = {ep.node_name for ep in node.get_publishers_info_by_topic("/hric/robot/cmd_vel")}
        consumers = {ep.node_name for ep in node.get_subscriptions_info_by_topic("/hric/robot/cmd_vel")}
        if existing != {"joystick_bridge_node"} or "hric_topic_bridge" not in consumers:
            raise RuntimeError(f"GO 后速度话题端点变化：发布者={existing} 订阅者={consumers}")

        publisher = node.create_publisher(TwistStamped, "/hric/robot/cmd_vel", 10)
        moving = True
        discovery_deadline = time.monotonic() + 2.0
        while time.monotonic() < discovery_deadline and publisher.get_subscription_count() == 0:
            send_velocity()
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError("速度发布者未与 hric_topic_bridge 建立连接")
        # Establish the command stream with zeros before the first non-zero sample.
        for _ in range(5):
            send_velocity()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.1)

        motion_start = time.monotonic()
        initial_pose = latest["odom"].pose.pose
        motion = MotionAccumulator(initial_pose.position.x, initial_pose.position.y)
        initial_yaw = yaw_of(initial_pose.orientation)
        initial_distance = current_error[2]
        yaw_change = 0.0
        distance = initial_distance
        last_mode = None
        first_nonzero_command_time = None
        command_feedback_seen = False
        outcome = "时间上限"
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            if now - motion_start >= 25.0:
                break
            if (now - received["odom"] > 0.5 or now - received["feedback"] > 1.0 or
                    now - received["walk_speed"] > 1.0):
                outcome = "里程计或速度反馈中断"
                break
            odom = latest["odom"]
            if odom.header.frame_id != "fastlio_odom" or odom.child_frame_id != "my_nav_base_link":
                outcome = "坐标系变化"
                break
            position = odom.pose.pose.position
            step = motion.update(position.x, position.y)
            if step > 0.12:
                outcome = "里程计突然跳变"
                break
            yaw_change = math.atan2(
                math.sin(yaw_of(odom.pose.pose.orientation) - initial_yaw),
                math.cos(yaw_of(odom.pose.pose.orientation) - initial_yaw),
            )
            if abs(yaw_change) > 1.2:
                outcome = "达到 1.2 rad 旋转上限"
                break
            if (abs(latest["walk_speed"].twist.linear.x) > args.max_linear_speed + 0.05 or
                    abs(latest["walk_speed"].twist.angular.z) > 0.35):
                outcome = "实际速度反馈超限"
                break
            if (abs(latest["feedback"].twist.linear.x) > 0.01 or
                    abs(latest["feedback"].twist.angular.z) > 0.03):
                command_feedback_seen = True
            if (first_nonzero_command_time is not None and
                    now - first_nonzero_command_time > 2.0 and
                    not command_feedback_seen):
                outcome = "2 秒内 cmd_vel_status 未确认非零命令"
                break
            forward, left, distance, heading = body_error(waypoint.point, odom.pose.pose)
            if motion.net_displacement > 0.05 and distance > initial_distance + 0.05:
                outcome = "运动后距航点反而增大，方向可能不符"
                break
            if motion.net_displacement >= args.max_travel:
                outcome = f"达到 {args.max_travel:.2f} m 平移上限"
                break
            if distance <= 0.12:
                outcome = "接近航点"
                break
            if abs(heading) > 0.25:
                mode = "原地转向"
                linear = 0.0
                angular = clamp(0.5 * heading, -0.10, 0.10)
            else:
                mode = "低速前进"
                linear = clamp(0.4 * distance, 0.02, args.max_linear_speed)
                angular = clamp(0.4 * heading, -0.06, 0.06)
            if mode != last_mode:
                print(
                    f"运动阶段：{mode}；当前航向误差 {math.degrees(heading):+.1f}°；"
                    f"净平移 {motion.net_displacement:.3f} m",
                    flush=True,
                )
                last_mode = mode
            send_velocity(linear, angular)
            if first_nonzero_command_time is None and (linear != 0.0 or angular != 0.0):
                first_nonzero_command_time = now
            if now - motion_start > 5.0:
                if motion.net_displacement < 0.01 and abs(yaw_change) < 0.02:
                    outcome = "5 秒内未检测到运动，停止发送"
                    break
            time.sleep(0.1)
        print(
            f"停止原因：{outcome}；净平移 {motion.net_displacement:.3f} m；"
            f"里程计逐帧路径和 {motion.path_length:.3f} m；"
            f"航向变化 {math.degrees(yaw_change):+.1f}°；"
            f"距航点 {initial_distance:.3f} -> {distance:.3f} m；"
            f"cmd_vel_status 非零确认={'是' if command_feedback_seen else '否'}"
        )
        return 0
    except (RuntimeError, KeyboardInterrupt, EOFError) as exc:
        print(f"终止测试：{exc}")
        return 1
    finally:
        if moving:
            print("发送零速 2 秒；现场人员仍需确认机器人确实停稳。", flush=True)
        stop_and_drain()
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
