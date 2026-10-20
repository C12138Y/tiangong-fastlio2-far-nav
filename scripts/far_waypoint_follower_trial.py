#!/usr/bin/env python3
"""Guarded FAR waypoint-stream follower for an open-area supervised trial.

This is an experimental bridge, not a production navigation controller.  It
follows fresh /far_eval/way_point messages for at most the configured FAR local
planning range, watches /far_reach_goal_status, and always sends zero commands
on exit.  The isolated --turn-test-only mode uses the RViz goal direction and
FAST-LIO odometry, without requiring a FAR waypoint.  A human must keep the
physical emergency stop in hand.
"""

import argparse
import heapq
import math
import signal
import time


def clamp(value, lo, hi):
    return max(lo, min(value, hi))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


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


def accept_goal_reached(far_reached, goal_distance, tolerance):
    return bool(far_reached) and goal_distance <= tolerance


def waypoint_problem(goal_distance, waypoint_distance, max_range, tolerance):
    """Fail closed on an out-of-range or robot-self waypoint."""
    if waypoint_distance > max_range:
        return (f"FAR 航点距机器人 {waypoint_distance:.3f} m，"
                f"超过本次 {max_range:.2f} m 范围")
    if goal_distance > tolerance and waypoint_distance <= tolerance:
        return (f"FAR 航点仅距机器人 {waypoint_distance:.3f} m，"
                f"但全局目标仍有 {goal_distance:.3f} m")
    return None


def goal_range_problem(goal_distance, max_range):
    """Reject an out-of-scope goal before waiting for FAR's first waypoint."""
    if goal_distance > max_range:
        return (
            f"RViz 目标距机器人 {goal_distance:.3f} m，超过本次 "
            f"{max_range:.2f} m 范围；不会继续等待 FAR 航点"
        )
    return None


def planning_problem(
    goal_distance,
    waypoint_distance,
    max_goal_distance,
    max_waypoint_distance,
    tolerance,
):
    """Validate the distant final goal separately from the local waypoint."""
    return (
        goal_range_problem(goal_distance, max_goal_distance)
        or waypoint_problem(
            goal_distance, waypoint_distance, max_waypoint_distance, tolerance
        )
    )


def planner_timeout_message(goal_distance, wait_seconds):
    return (
        f"FAR 在 {wait_seconds:.0f} 秒内未生成首个航点；目标距机器人 "
        f"{goal_distance:.3f} m。目标可能不在当前连通 V-Graph、"
        "已扫描可通行区域或可达区域内"
    )


def cached_waypoint_problem(goal_time, waypoint_time, now, max_age=None):
    """A pre-GO waypoint is usable briefly if it belongs to this goal."""
    if waypoint_time < goal_time:
        return "航点早于当前目标，不能沿用"
    age = now - waypoint_time
    if max_age is not None and age > max_age:
        return f"FAR 航点已 {age:.1f} 秒未更新；请重新取点"
    return None


def pose_changed_problem(before, after):
    translation = math.hypot(
        after.position.x - before.position.x,
        after.position.y - before.position.y,
    )
    rotation = abs(wrap_angle(yaw_of(after.orientation) - yaw_of(before.orientation)))
    if translation > 0.05 or rotation > math.radians(5.0):
        return "GO 确认期间机器人位姿变化，不能沿用预检航点"
    return None


def motion_stalled(elapsed, translation, signed_yaw_progress, turning):
    """Require observable yaw for turns, or observable translation for travel."""
    if elapsed <= 5.0:
        return False
    if turning:
        return signed_yaw_progress < math.radians(2.0)
    return translation < 0.03


def turn_trial_command(heading, angular_speed):
    """One supervised pure-yaw probe; never mix forward speed into this mode."""
    return "原地转向试验", 0.0, math.copysign(angular_speed, heading)


def turn_trial_speed_problem(angular_speed):
    if not 0.20 <= angular_speed <= 0.60:
        return "--turn-angular-speed 必须在 0.20..0.60 rad/s 内"
    return None


def default_max_travel(max_route_distance):
    """A longer selectable FAR goal must not silently authorize longer motion."""
    return min(0.2, max_route_distance)


def turn_trial_input_problem(goal_error, max_distance):
    goal_distance, goal_heading = goal_error[2], goal_error[3]
    heading_degrees = math.degrees(goal_heading)
    if not 0.2 <= goal_distance <= max_distance:
        return (f"原地转向试验目标距离 {goal_distance:.3f} m，"
                f"要求 0.20..{max_distance:.2f} m")
    if not 10.0 <= abs(heading_degrees) <= 150.0:
        return (f"原地转向试验目标偏角 {heading_degrees:+.1f}°，"
                "要求绝对偏角 10..150°")
    return None


def turn_trial_guard(start_pose, current_pose, direction, max_degrees, elapsed):
    """Use FAST-LIO pose, not walk_speed, to bound a turn probe."""
    displacement = math.hypot(
        current_pose.position.x - start_pose.position.x,
        current_pose.position.y - start_pose.position.y,
    )
    progress = direction * wrap_angle(
        yaw_of(current_pose.orientation) - yaw_of(start_pose.orientation)
    )
    if displacement >= 0.05:
        return f"原地转向时平移 {displacement:.3f} m，超过 0.05 m 上限"
    if progress <= -math.radians(2.0):
        return f"航向反向变化 {math.degrees(progress):+.1f}°"
    if progress >= math.radians(max_degrees):
        return f"达到 {max_degrees:.1f}° 单次转角上限"
    if elapsed > 5.0 and progress < math.radians(2.0):
        return f"5 秒内未检测到有效转向；航向变化 {math.degrees(progress):+.1f}°"
    return None


def speed_publisher_names(node):
    return sorted(
        ep.node_name for ep in node.get_publishers_info_by_topic("/hric/robot/cmd_vel")
    )


class FilteredTravelAccumulator:
    """Accumulate motion in chunks so millimetre odometry jitter is ignored."""

    def __init__(self, x, y, chunk=0.02):
        self.start_x = x
        self.start_y = y
        self.anchor_x = x
        self.anchor_y = y
        self.chunk = chunk
        self.path_length = 0.0
        self.net_displacement = 0.0

    def update(self, x, y):
        self.net_displacement = math.hypot(x - self.start_x, y - self.start_y)
        delta = math.hypot(x - self.anchor_x, y - self.anchor_y)
        if delta >= self.chunk:
            self.path_length += delta
            self.anchor_x = x
            self.anchor_y = y
        return delta


def costmap_blockage(
    grid,
    pose,
    linear,
    angular,
    lookahead=1.2,
    footprint_radius=0.45,
    lethal_threshold=65,
    unknown_is_obstacle=True,
):
    """Return the first (x, y, cost) in the commanded swept footprint."""
    info = grid.info
    resolution = float(info.resolution)
    width = int(info.width)
    height = int(info.height)
    if resolution <= 0.0 or width <= 0 or height <= 0:
        return (pose.position.x, pose.position.y, -1)

    origin_yaw = yaw_of(info.origin.orientation)
    origin_cos = math.cos(origin_yaw)
    origin_sin = math.sin(origin_yaw)
    robot_yaw = yaw_of(pose.orientation)
    robot_cos = math.cos(robot_yaw)
    robot_sin = math.sin(robot_yaw)
    radius_cells = int(math.ceil(footprint_radius / resolution))
    sample_step = max(0.025, resolution * 0.5)
    distance = max(0.0, lookahead) if linear > 0.001 else 0.0
    sample_count = max(1, int(math.ceil(distance / sample_step)))
    checked = set()

    for index in range(sample_count + 1):
        travelled = distance * index / sample_count
        if abs(angular) < 1e-5 or abs(linear) < 1e-5:
            body_x, body_y = travelled, 0.0
        else:
            curvature = angular / linear
            turn = curvature * travelled
            body_x = math.sin(turn) / curvature
            body_y = (1.0 - math.cos(turn)) / curvature
        world_x = pose.position.x + robot_cos * body_x - robot_sin * body_y
        world_y = pose.position.y + robot_sin * body_x + robot_cos * body_y
        dx = world_x - info.origin.position.x
        dy = world_y - info.origin.position.y
        map_x = (origin_cos * dx + origin_sin * dy) / resolution
        map_y = (-origin_sin * dx + origin_cos * dy) / resolution
        centre_x = int(math.floor(map_x))
        centre_y = int(math.floor(map_y))

        for cell_y in range(centre_y - radius_cells, centre_y + radius_cells + 1):
            for cell_x in range(centre_x - radius_cells, centre_x + radius_cells + 1):
                key = (cell_x, cell_y)
                if key in checked:
                    continue
                local_x = (cell_x + 0.5 - map_x) * resolution
                local_y = (cell_y + 0.5 - map_y) * resolution
                if math.hypot(local_x, local_y) > footprint_radius:
                    continue
                checked.add(key)
                if not (0 <= cell_x < width and 0 <= cell_y < height):
                    return (world_x, world_y, -1)
                cost = int(grid.data[cell_y * width + cell_x])
                if cost >= lethal_threshold or (cost < 0 and unknown_is_obstacle):
                    cell_map_x = (cell_x + 0.5) * resolution
                    cell_map_y = (cell_y + 0.5) * resolution
                    cell_world_x = (
                        info.origin.position.x
                        + origin_cos * cell_map_x - origin_sin * cell_map_y
                    )
                    cell_world_y = (
                        info.origin.position.y
                        + origin_sin * cell_map_x + origin_cos * cell_map_y
                    )
                    return (cell_world_x, cell_world_y, cost)
    return None


def _grid_geometry(grid):
    info = grid.info
    yaw = yaw_of(info.origin.orientation)
    return float(info.resolution), math.cos(yaw), math.sin(yaw)


def _world_to_cell(grid, world_x, world_y):
    resolution, cosine, sine = _grid_geometry(grid)
    dx = world_x - grid.info.origin.position.x
    dy = world_y - grid.info.origin.position.y
    return (
        int(math.floor((cosine * dx + sine * dy) / resolution)),
        int(math.floor((-sine * dx + cosine * dy) / resolution)),
    )


def _cell_to_world(grid, cell):
    resolution, cosine, sine = _grid_geometry(grid)
    local_x = (cell[0] + 0.5) * resolution
    local_y = (cell[1] + 0.5) * resolution
    return (
        grid.info.origin.position.x + cosine * local_x - sine * local_y,
        grid.info.origin.position.y + sine * local_x + cosine * local_y,
    )


def _cell_cost(grid, cell):
    x, y = cell
    if not (0 <= x < int(grid.info.width) and 0 <= y < int(grid.info.height)):
        return None
    return int(grid.data[y * int(grid.info.width) + x])


def _cell_traversable(grid, cell, lethal_threshold, unknown_is_obstacle):
    cost = _cell_cost(grid, cell)
    if cost is None or cost >= lethal_threshold:
        return False
    return cost >= 0 or not unknown_is_obstacle


def _nearest_traversable(grid, desired, lethal_threshold, unknown_is_obstacle):
    width, height = int(grid.info.width), int(grid.info.height)
    desired = (
        min(max(desired[0], 0), width - 1),
        min(max(desired[1], 0), height - 1),
    )
    if _cell_traversable(grid, desired, lethal_threshold, unknown_is_obstacle):
        return desired
    max_radius = max(width, height)
    for radius in range(1, max_radius + 1):
        candidates = []
        for x in range(desired[0] - radius, desired[0] + radius + 1):
            candidates.append((x, desired[1] - radius))
            candidates.append((x, desired[1] + radius))
        for y in range(desired[1] - radius + 1, desired[1] + radius):
            candidates.append((desired[0] - radius, y))
            candidates.append((desired[0] + radius, y))
        for candidate in candidates:
            if _cell_traversable(
                grid, candidate, lethal_threshold, unknown_is_obstacle
            ):
                return candidate
    return None


def costmap_path(
    grid,
    start_world,
    goal_world,
    lethal_threshold=65,
    unknown_is_obstacle=True,
):
    """Plan an 8-connected local A* path over a Nav2 OccupancyGrid."""
    if (float(grid.info.resolution) <= 0.0 or int(grid.info.width) <= 0 or
            int(grid.info.height) <= 0):
        return None
    start = _nearest_traversable(
        grid, _world_to_cell(grid, *start_world), lethal_threshold,
        unknown_is_obstacle,
    )
    goal = _nearest_traversable(
        grid, _world_to_cell(grid, *goal_world), lethal_threshold,
        unknown_is_obstacle,
    )
    if start is None or goal is None:
        return None

    def heuristic(cell):
        return math.hypot(goal[0] - cell[0], goal[1] - cell[1])

    frontier = [(heuristic(start), 0.0, start)]
    came_from = {start: None}
    best_cost = {start: 0.0}
    neighbours = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
    )
    while frontier:
        _, current_cost, current = heapq.heappop(frontier)
        if current_cost > best_cost.get(current, math.inf):
            continue
        if current == goal:
            cells = []
            while current is not None:
                cells.append(current)
                current = came_from[current]
            cells.reverse()
            return [_cell_to_world(grid, cell) for cell in cells]
        for dx, dy, step in neighbours:
            candidate = (current[0] + dx, current[1] + dy)
            if not _cell_traversable(
                grid, candidate, lethal_threshold, unknown_is_obstacle
            ):
                continue
            if dx and dy:
                if (not _cell_traversable(
                        grid, (current[0] + dx, current[1]),
                        lethal_threshold, unknown_is_obstacle) or
                        not _cell_traversable(
                            grid, (current[0], current[1] + dy),
                            lethal_threshold, unknown_is_obstacle)):
                    continue
            raw_cost = max(0, _cell_cost(grid, candidate))
            candidate_cost = current_cost + step * (1.0 + 2.0 * raw_cost / 100.0)
            if candidate_cost >= best_cost.get(candidate, math.inf):
                continue
            best_cost[candidate] = candidate_cost
            came_from[candidate] = current
            heapq.heappush(
                frontier,
                (candidate_cost + heuristic(candidate), candidate_cost, candidate),
            )
    return None


def path_lookahead_point(path, lookahead):
    if not path:
        return None
    travelled = 0.0
    for previous, current in zip(path, path[1:]):
        travelled += math.hypot(current[0] - previous[0], current[1] - previous[1])
        if travelled >= lookahead:
            return current
    return path[-1]


def body_error_xy(world_x, world_y, pose):
    yaw = yaw_of(pose.orientation)
    dx = world_x - pose.position.x
    dy = world_y - pose.position.y
    forward = math.cos(yaw) * dx + math.sin(yaw) * dy
    left = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return forward, left, math.hypot(dx, dy), math.atan2(left, forward)


def control_command(
    distance,
    heading,
    max_linear_speed,
    turn_linear_speed=0.0,
    max_angular_speed=0.10,
    arc_linear_speed=0.08,
    turn_radius=0.8,
):
    """Return (mode, forward, yaw-rate); turn gain tightens walking arcs."""
    def walking_turn_rate(linear_speed):
        proportional_rate = 0.8 * abs(heading)
        radius_rate = abs(linear_speed) / turn_radius if linear_speed else 0.0
        return math.copysign(
            min(max_angular_speed, max(proportional_rate, radius_rate)), heading
        )

    angle = abs(heading)
    if angle > math.radians(45.0):
        mode = "大角度行走转向" if turn_linear_speed > 0.0 else "大角度原地转向"
        return (
            mode,
            turn_linear_speed,
            walking_turn_rate(turn_linear_speed),
        )
    if angle > math.radians(14.0):
        linear = min(max_linear_speed, arc_linear_speed)
        return (
            "低速弧线转向",
            linear,
            walking_turn_rate(linear),
        )
    return (
        "航点跟踪",
        clamp(0.6 * max(distance - 0.12, 0.0), 0.02, max_linear_speed),
        clamp(0.5 * heading, -min(0.08, max_angular_speed), min(0.08, max_angular_speed)),
    )


def local_avoidance_command(
    distance,
    heading,
    max_linear_speed,
    max_angular_speed,
    align_angle=math.radians(10.0),
    align_angular_speed=0.5,
    turn_radius=0.8,
):
    """Align with the A* path before translating, so arcs cannot cut corners."""
    if abs(heading) > align_angle:
        return (
            "局部A*原地对准",
            0.0,
            math.copysign(min(max_angular_speed, align_angular_speed), heading),
        )
    return control_command(
        distance,
        heading,
        max_linear_speed,
        turn_linear_speed=max_linear_speed,
        max_angular_speed=max_angular_speed,
        arc_linear_speed=max_linear_speed,
        turn_radius=turn_radius,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--max-route-distance", type=float, default=20.0)
    parser.add_argument("--max-goal-distance", type=float, default=20.0)
    parser.add_argument("--max-travel", type=float, default=None)
    parser.add_argument("--max-linear-speed", type=float, default=0.4)
    parser.add_argument("--turn-linear-speed", type=float, default=0.4)
    parser.add_argument("--arc-linear-speed", type=float, default=0.4)
    parser.add_argument("--max-angular-speed", type=float, default=0.6)
    parser.add_argument("--turn-radius", type=float, default=0.8)
    parser.add_argument("--planner-timeout", type=float, default=15.0)
    parser.add_argument("--costmap-topic", default="/my_nav/costmap")
    parser.add_argument("--costmap-lookahead", type=float, default=1.2)
    parser.add_argument("--costmap-footprint-radius", type=float, default=0.45)
    parser.add_argument("--costmap-threshold", type=int, default=65)
    parser.add_argument("--costmap-max-age", type=float, default=2.0)
    parser.add_argument("--costmap-block-timeout", type=float, default=5.0)
    parser.add_argument("--local-plan-lookahead", type=float, default=0.8)
    parser.add_argument("--local-align-angle", type=float, default=10.0)
    parser.add_argument("--local-align-angular-speed", type=float, default=0.5)
    parser.add_argument("--allow-unknown-costmap", action="store_true")
    parser.add_argument("--turn-test-only", action="store_true")
    parser.add_argument("--turn-angular-speed", type=float, default=0.50)
    parser.add_argument("--max-turn-degrees", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--goal-tolerance", type=float, default=0.18)
    args = parser.parse_args()
    if not 0.2 <= args.max_route_distance <= 20.0:
        parser.error("--max-route-distance 必须在 0.2..20.0 m 内")
    if not args.max_route_distance <= args.max_goal_distance <= 20.0:
        parser.error("--max-goal-distance 必须在 --max-route-distance..20.0 m 内")
    if args.max_travel is None:
        args.max_travel = default_max_travel(args.max_route_distance)
    if not 0.1 <= args.max_travel <= args.max_route_distance:
        parser.error("--max-travel 必须在 0.1..--max-route-distance m 内")
    if not 0.02 <= args.max_linear_speed <= 0.4:
        parser.error("--max-linear-speed 必须在 0.02..0.4 m/s 内")
    if not 0.0 <= args.turn_linear_speed <= 0.4:
        parser.error("--turn-linear-speed 必须在 0.0..0.4 m/s 内")
    if not 0.02 <= args.arc_linear_speed <= args.max_linear_speed:
        parser.error("--arc-linear-speed 必须在 0.02..--max-linear-speed m/s 内")
    if not 0.05 <= args.max_angular_speed <= 0.80:
        parser.error("--max-angular-speed 必须在 0.05..0.80 rad/s 内")
    if not 0.5 <= args.turn_radius <= 2.0:
        parser.error("--turn-radius 必须在 0.5..2.0 m 内")
    if not 5.0 <= args.planner_timeout <= 120.0:
        parser.error("--planner-timeout 必须在 5..120 s 内")
    if not 0.4 <= args.costmap_lookahead <= 3.0:
        parser.error("--costmap-lookahead 必须在 0.4..3.0 m 内")
    if not 0.30 <= args.costmap_footprint_radius <= 0.80:
        parser.error("--costmap-footprint-radius 必须在 0.30..0.80 m 内")
    if not 1 <= args.costmap_threshold <= 100:
        parser.error("--costmap-threshold 必须在 1..100 内")
    if not 0.5 <= args.costmap_max_age <= 5.0:
        parser.error("--costmap-max-age 必须在 0.5..5.0 s 内")
    if not 1.0 <= args.costmap_block_timeout <= 15.0:
        parser.error("--costmap-block-timeout 必须在 1..15 s 内")
    if not 0.3 <= args.local_plan_lookahead <= 1.5:
        parser.error("--local-plan-lookahead 必须在 0.3..1.5 m 内")
    if not 5.0 <= args.local_align_angle <= 30.0:
        parser.error("--local-align-angle 必须在 5..30° 内")
    if not 0.2 <= args.local_align_angular_speed <= args.max_angular_speed:
        parser.error(
            "--local-align-angular-speed 必须在 0.2..--max-angular-speed rad/s 内"
        )
    if not 10.0 <= args.timeout <= 300.0:
        parser.error("--timeout 必须在 10..300 s 内")
    if args.turn_test_only:
        if args.max_route_distance > 1.0 or args.timeout > 15.0:
            parser.error("原地转向试验要求 --max-route-distance <= 1.0 且 --timeout <= 15")
        speed_problem = turn_trial_speed_problem(args.turn_angular_speed)
        if speed_problem:
            parser.error(speed_problem)
        if not 5.0 <= args.max_turn_degrees <= 15.0:
            parser.error("--max-turn-degrees 必须在 5..15° 内")

    import rclpy
    from diagnostic_msgs.msg import DiagnosticStatus
    from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Bool

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def on_terminate(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_terminate)
    signal.signal(signal.SIGTERM, on_terminate)
    node = rclpy.create_node("far_waypoint_follower_trial")
    keys = (
        "goal", "waypoint", "odom", "costmap", "feedback", "walk_speed",
        "rl_state", "reached",
    )
    latest = {key: None for key in keys}
    received = {key: 0.0 for key in keys}

    def callback_for(key):
        def callback(msg):
            latest[key] = msg
            received[key] = time.monotonic()
        return callback

    goal_point_publisher = node.create_publisher(PointStamped, "/far_eval/goal_point", 10)
    local_path_publisher = node.create_publisher(Path, "/my_nav/local_avoid_path", 10)

    def publish_local_path(points):
        message = Path()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = "fastlio_odom"
        for x, y in points or ():
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        local_path_publisher.publish(message)

    def goal_pose_callback(msg):
        if latest["odom"] is None:
            print("忽略 2D Goal Pose：尚无 /my_nav/odom。", flush=True)
            return
        if msg.header.frame_id not in ("", "fastlio_odom"):
            print(f"忽略 2D Goal Pose：frame_id={msg.header.frame_id}。", flush=True)
            return
        point = PointStamped()
        point.header.stamp = msg.header.stamp
        point.header.frame_id = "fastlio_odom"
        point.point.x = msg.pose.position.x
        point.point.y = msg.pose.position.y
        point.point.z = latest["odom"].pose.pose.position.z
        goal_point_publisher.publish(point)
        print(
            f"已把 2D Goal Pose 转为 FAR 目标：({point.point.x:.3f}, "
            f"{point.point.y:.3f}, {point.point.z:.3f})",
            flush=True,
        )

    best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = [
        node.create_subscription(PointStamped, "/far_eval/goal_point", callback_for("goal"), 10),
        node.create_subscription(PointStamped, "/far_eval/way_point", callback_for("waypoint"), 10),
        node.create_subscription(Odometry, "/my_nav/odom", callback_for("odom"), best_effort),
        node.create_subscription(OccupancyGrid, args.costmap_topic, callback_for("costmap"), 10),
        node.create_subscription(TwistStamped, "/hric/robot/cmd_vel_status", callback_for("feedback"), best_effort),
        node.create_subscription(TwistStamped, "/hric/robot/walk_speed", callback_for("walk_speed"), best_effort),
        node.create_subscription(DiagnosticStatus, "/hric/robot/rl_state", callback_for("rl_state"), best_effort),
        node.create_subscription(Bool, "/far_reach_goal_status", callback_for("reached"), 10),
        node.create_subscription(PoseStamped, "/goal_pose", goal_pose_callback, 10),
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
        for _ in range(80):
            send_velocity()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.025)

    try:
        print("等待 RViz 点击一个新的、已确认空旷的 FAR 目标。", flush=True)
        wait_start = time.monotonic()
        goal_time = None
        observed_goal_distance = None
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if latest["goal"] is None:
                if now - wait_start >= 60.0:
                    raise RuntimeError("60 秒内未收到新的 RViz 目标")
                continue
            if goal_time is None:
                goal_time = received["goal"]
                if args.turn_test_only:
                    print("收到目标，等待里程计与安全反馈...", flush=True)
                else:
                    print("收到目标，等待该目标对应的 FAR 航点...", flush=True)
            if (latest["odom"] is not None and
                    now - received["odom"] <= 0.5):
                observed_goal_distance = body_error(
                    latest["goal"].point, latest["odom"].pose.pose
                )[2]
                problem = goal_range_problem(
                    observed_goal_distance,
                    (args.max_route_distance if args.turn_test_only
                     else args.max_goal_distance),
                )
                if problem:
                    raise RuntimeError(problem)
            if not args.turn_test_only and (
                    latest["waypoint"] is None or received["waypoint"] < goal_time):
                if now - goal_time >= args.planner_timeout:
                    if observed_goal_distance is None:
                        raise RuntimeError(
                            "收到目标但没有新鲜 /my_nav/odom，无法判断目标距离"
                        )
                    raise RuntimeError(planner_timeout_message(
                        observed_goal_distance, args.planner_timeout
                    ))
                continue
            required = ("odom", "feedback", "walk_speed", "rl_state")
            if not args.turn_test_only:
                required += ("costmap",)
            if any(latest[key] is None for key in required):
                continue
            if (now - received["odom"] > 0.5 or now - received["feedback"] > 1.0 or
                    now - received["walk_speed"] > 1.0 or
                    (not args.turn_test_only and
                     now - received["costmap"] > args.costmap_max_age) or
                    (not args.turn_test_only and now - received["waypoint"] > 1.0)):
                continue
            break

        goal = latest["goal"]
        odom = latest["odom"]
        pre_go_pose = odom.pose.pose
        waypoint = latest["waypoint"]
        costmap = latest["costmap"]
        if (goal.header.frame_id != "fastlio_odom" or
                (not args.turn_test_only and waypoint.header.frame_id != "fastlio_odom") or
                (not args.turn_test_only and costmap.header.frame_id != "fastlio_odom") or
                odom.header.frame_id != "fastlio_odom" or
                odom.child_frame_id != "my_nav_base_link"):
            raise RuntimeError("目标、航点、代价地图或里程计坐标系不符")
        goal_error = body_error(goal.point, odom.pose.pose)
        waypoint_error = (body_error(waypoint.point, odom.pose.pose)
                          if not args.turn_test_only else None)
        if args.turn_test_only:
            print(
                f"原地转向预检：目标距离 {goal_error[2]:.3f} m，"
                f"偏角 {math.degrees(goal_error[3]):+.1f}°",
                flush=True,
            )
        if args.turn_test_only:
            problem = turn_trial_input_problem(goal_error, args.max_route_distance)
            if problem:
                raise RuntimeError(problem)
        else:
            problem = planning_problem(
                goal_error[2], waypoint_error[2], args.max_goal_distance,
                args.max_route_distance, args.goal_tolerance,
            )
            if problem:
                raise RuntimeError(problem + "；未创建速度发布者")
        state = {item.key: item.value for item in latest["rl_state"].values}
        if state.get("current_state") != "SWR" or state.get("status") != "running":
            raise RuntimeError(f"FSM 状态不符：{state}")
        if (abs(latest["feedback"].twist.linear.x) > 0.02 or
                abs(latest["feedback"].twist.angular.z) > 0.02 or
                abs(latest["walk_speed"].twist.linear.x) > 0.02 or
                abs(latest["walk_speed"].twist.angular.z) > 0.02):
            raise RuntimeError("开始前速度反馈不是零")
        existing = speed_publisher_names(node)
        consumers = {ep.node_name for ep in node.get_subscriptions_info_by_topic("/hric/robot/cmd_vel")}
        if existing != ["joystick_bridge_node"] or "hric_topic_bridge" not in consumers:
            raise RuntimeError(f"速度话题端点异常：发布者={existing}，订阅者={consumers}")

        if not args.turn_test_only:
            print(
                f"全局目标距机器人 {goal_error[2]:.3f} m；首个 FAR 航点距机器人 "
                f"{waypoint_error[2]:.3f} m、方向 {math.degrees(waypoint_error[3]):+.1f}°"
            )
        if args.turn_test_only:
            print(
                f"原地转向试验：v=0，|w|={args.turn_angular_speed:.2f} rad/s，"
                f"目标约 40 Hz 发布；累计航向最多 {args.max_turn_degrees:.1f}°，"
                "平移超过 0.05 m 或 5 秒未转动即停；本次不会前进。"
            )
        else:
            print(
                f"持续读取 FAR 航点；最终目标距离上限 {args.max_goal_distance:.2f} m，"
                f"局部航点距离上限 {args.max_route_distance:.2f} m，"
                f"本次移动上限 {args.max_travel:.2f} m，"
                f"速度上限 {args.max_linear_speed:.2f} m/s，角速度上限 "
                f"{args.max_angular_speed:.2f} rad/s，"
                f"弧线速度 {args.arc_linear_speed:.2f} m/s，目标转弯半径 "
                f"{args.turn_radius:.2f} m，超时 {args.timeout:.0f} s。"
            )
            print(
                f"局部避障已启用：{args.costmap_topic}，前视 "
                f"{args.costmap_lookahead:.2f} m，足迹半径 "
                f"{args.costmap_footprint_radius:.2f} m，代价阈值 "
                f"{args.costmap_threshold}，未知区域"
                f"{'允许' if args.allow_unknown_costmap else '禁止'}；"
                f"A* 局部跟踪前视 {args.local_plan_lookahead:.2f} m，"
                f"偏差超过 {args.local_align_angle:.1f}° 时先以 "
                f"{args.local_align_angular_speed:.2f} rad/s 原地对准。"
            )
            preview_mode, preview_linear, preview_angular = control_command(
                waypoint_error[2],
                waypoint_error[3],
                args.max_linear_speed,
                args.turn_linear_speed,
                args.max_angular_speed,
                args.arc_linear_speed,
                args.turn_radius,
            )
            preview_blockage = costmap_blockage(
                costmap,
                odom.pose.pose,
                preview_linear,
                preview_angular,
                args.costmap_lookahead,
                args.costmap_footprint_radius,
                args.costmap_threshold,
                not args.allow_unknown_costmap,
            )
            if preview_blockage is None:
                print("Costmap 预检：当前直接控制段可通行。")
            else:
                preview_path = costmap_path(
                    costmap,
                    (odom.pose.pose.position.x, odom.pose.pose.position.y),
                    (waypoint.point.x, waypoint.point.y),
                    args.costmap_threshold,
                    not args.allow_unknown_costmap,
                )
                preview_point = path_lookahead_point(
                    preview_path, args.local_plan_lookahead
                )
                if preview_point is None:
                    raise RuntimeError(
                        "Costmap 预检发现直达方向有障碍，且局部 A* 无可达路径；"
                        "未创建速度发布者"
                    )
                publish_local_path(preview_path)
                print(
                    f"Costmap 预检：直达方向命中代价 {preview_blockage[2]}；"
                    f"局部 A* 生成 {len(preview_path)} 个栅格点，首个跟踪点 "
                    f"({preview_point[0]:.2f}, {preview_point[1]:.2f})。"
                )
        if not args.execute:
            print("仅预检；加 --execute 后仍需现场输入 GO。")
            return 0
        print("确认全程空旷、E 上拨、专人手持实体急停。", flush=True)
        if input("确认后输入大写 GO；其他输入取消：").strip() != "GO":
            print("已取消。")
            return 1

        confirmation_time = time.monotonic()
        required = ("odom", "feedback", "walk_speed", "rl_state")
        if not args.turn_test_only:
            required += ("costmap",)
        deadline = confirmation_time + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if all(received[key] > confirmation_time for key in required):
                break
        else:
            missing = [key for key in required if received[key] <= confirmation_time]
            raise RuntimeError(f"GO 后缺少新消息：{missing}")
        if received["goal"] > goal_time:
            raise RuntimeError("确认期间目标发生变化，请重新运行")
        if not args.turn_test_only:
            problem = cached_waypoint_problem(
                goal_time, received["waypoint"], time.monotonic()
            ) or pose_changed_problem(pre_go_pose, latest["odom"].pose.pose)
            if problem:
                raise RuntimeError(problem + "；GO 后未发送运动命令")
        if speed_publisher_names(node) != ["joystick_bridge_node"]:
            raise RuntimeError("GO 后出现其他速度发布者，未启动")
        latest_goal_distance = body_error(goal.point, latest["odom"].pose.pose)[2]
        if args.turn_test_only:
            latest_goal_error = body_error(goal.point, latest["odom"].pose.pose)
            problem = turn_trial_input_problem(latest_goal_error, args.max_route_distance)
            if problem:
                raise RuntimeError(problem + "；GO 后未发送运动命令")
        else:
            latest_waypoint_distance = body_error(
                latest["waypoint"].point, latest["odom"].pose.pose
            )[2]
            problem = planning_problem(
                latest_goal_distance, latest_waypoint_distance,
                args.max_goal_distance, args.max_route_distance,
                args.goal_tolerance,
            )
            if problem:
                raise RuntimeError(problem + "；GO 后未发送运动命令")

        publisher = node.create_publisher(TwistStamped, "/hric/robot/cmd_vel", 10)
        moving = True
        discovery_deadline = time.monotonic() + 2.0
        while time.monotonic() < discovery_deadline and publisher.get_subscription_count() == 0:
            send_velocity()
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError("未连接到 hric_topic_bridge")
        for _ in range(5):
            send_velocity()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.1)
        expected_speed_publishers = ["far_waypoint_follower_trial", "joystick_bridge_node"]
        if speed_publisher_names(node) != expected_speed_publishers:
            raise RuntimeError("建立速度连接后出现其他发布者，未发送非零命令")

        start_pose = latest["odom"].pose.pose
        trial_turn_direction = (1 if latest_goal_error[3] > 0 else -1) if args.turn_test_only else 0
        travel = FilteredTravelAccumulator(start_pose.position.x, start_pose.position.y)
        motion_start = time.monotonic()
        first_command_time = None
        command_feedback_seen = False
        close_since = None
        last_waypoint_xy = None
        last_mode = None
        mode_started = motion_start
        mode_start_yaw = yaw_of(start_pose.orientation)
        mode_start_xy = (start_pose.position.x, start_pose.position.y)
        last_report = 0.0
        premature_reach_warned = False
        costmap_blocked_since = None
        detour_cache_key = None
        detour_path = None
        last_endpoint_check = motion_start
        turn_command_count = 0
        last_turn_command_at = None
        outcome = "达到时间上限"

        while True:
            loop_started = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.02)
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            if now - last_endpoint_check >= 0.5:
                if speed_publisher_names(node) != expected_speed_publishers:
                    outcome = "运行中出现其他速度发布者或原发布者消失"
                    break
                last_endpoint_check = now
            if now - motion_start >= args.timeout:
                break
            if (now - received["odom"] > 0.5 or now - received["feedback"] > 1.0 or
                    now - received["walk_speed"] > 1.0):
                outcome = "里程计或速度反馈中断"
                break
            if (not args.turn_test_only and
                    now - received["costmap"] > args.costmap_max_age):
                outcome = "局部代价地图中断或过期"
                break
            if not args.turn_test_only and now - received["waypoint"] > 1.0:
                # FAR may publish once while stationary and resume after motion.
                # The validated pre-GO waypoint has only a short bootstrap window.
                using_bootstrap = (
                    received["waypoint"] <= confirmation_time
                    and now - motion_start <= 5.0
                    and cached_waypoint_problem(
                        goal_time, received["waypoint"], now
                    ) is None
                )
                if not using_bootstrap:
                    outcome = "FAR 航点流中断；缓存航点的短暂启动窗口已结束"
                    break
            state = {item.key: item.value for item in latest["rl_state"].values}
            if state.get("current_state") != "SWR" or state.get("status") != "running":
                outcome = f"FSM 状态变化：{state}"
                break
            odom = latest["odom"]
            waypoint = latest["waypoint"]
            if (odom.header.frame_id != "fastlio_odom" or
                    odom.child_frame_id != "my_nav_base_link" or
                    (not args.turn_test_only and
                     waypoint.header.frame_id != "fastlio_odom")):
                outcome = "运行中坐标系变化"
                break
            goal_distance = body_error(goal.point, odom.pose.pose)[2]
            far_reached = (not args.turn_test_only and
                latest["reached"] is not None
                and received["reached"] >= goal_time
                and latest["reached"].data
            )
            if accept_goal_reached(far_reached, goal_distance, args.goal_tolerance):
                outcome = "FAR 报告到达且 FAST-LIO 距离进入目标容差"
                break
            if far_reached and not premature_reach_warned:
                print(
                    f"警告：FAR 提前报告到达，但仍距原始目标 {goal_distance:.3f} m；"
                    "暂不判定完成。",
                    flush=True,
                )
                premature_reach_warned = True
            position = odom.pose.pose.position
            travel.update(position.x, position.y)
            if travel.path_length >= args.max_travel:
                outcome = f"达到 {args.max_travel:.2f} m 移动安全上限"
                break
            allowed_angular = (args.turn_angular_speed if args.turn_test_only
                               else args.max_angular_speed)
            if (abs(latest["walk_speed"].twist.linear.x) > args.max_linear_speed + 0.05 or
                    abs(latest["walk_speed"].twist.angular.z) > allowed_angular + 0.05):
                outcome = "实际速度反馈超限"
                break
            if (abs(latest["feedback"].twist.linear.x) > 0.01 or
                    abs(latest["feedback"].twist.angular.z) > 0.03):
                command_feedback_seen = True
            if (first_command_time is not None and now - first_command_time > 2.0 and
                    not command_feedback_seen):
                outcome = "cmd_vel_status 未确认非零命令"
                break

            if args.turn_test_only:
                reason = turn_trial_guard(
                    start_pose, odom.pose.pose, trial_turn_direction,
                    args.max_turn_degrees, now - motion_start,
                )
                if reason:
                    outcome = reason
                    break
                current_heading = body_error(goal.point, odom.pose.pose)[3]
                if trial_turn_direction * current_heading <= math.radians(10.0):
                    outcome = "目标方向已进入 10°，原地转向试验结束"
                    break
                mode, linear, angular = turn_trial_command(
                    current_heading, args.turn_angular_speed
                )
                if mode != last_mode:
                    print(f"控制阶段：{mode}；命令 v=0.00 m/s, w={angular:+.2f} rad/s")
                    last_mode = mode
                if last_turn_command_at is not None and now - last_turn_command_at > 0.12:
                    outcome = f"转向发令间隔 {now - last_turn_command_at:.3f} s 超过 0.12 s"
                    break
                send_velocity(linear, angular)
                last_turn_command_at = time.monotonic()
                turn_command_count += 1
                if first_command_time is None:
                    first_command_time = now
                if now - last_report >= 1.0:
                    progress = trial_turn_direction * wrap_angle(
                        yaw_of(odom.pose.pose.orientation) - yaw_of(start_pose.orientation)
                    )
                    print(
                        f"转向状态：目标方向航向变化 {math.degrees(progress):+.1f}°；"
                        f"平移 {travel.net_displacement:.3f} m；"
                        f"目标偏角 {math.degrees(current_heading):+.1f}°；"
                        f"cmd_vel_status w="
                        f"{latest['feedback'].twist.angular.z:+.2f} rad/s",
                        flush=True,
                    )
                    last_report = now
                time.sleep(max(0.0, 0.025 - (time.monotonic() - loop_started)))
                continue

            forward, left, distance, heading = body_error(waypoint.point, odom.pose.pose)
            problem = planning_problem(
                goal_distance, distance, args.max_goal_distance,
                args.max_route_distance, args.goal_tolerance,
            )
            if problem:
                outcome = problem
                break
            waypoint_xy = (waypoint.point.x, waypoint.point.y)
            if (last_waypoint_xy is None or
                    math.hypot(waypoint_xy[0] - last_waypoint_xy[0],
                               waypoint_xy[1] - last_waypoint_xy[1]) > 0.05):
                print(
                    f"采用 FAR 航点 ({waypoint_xy[0]:.3f}, {waypoint_xy[1]:.3f})；"
                    f"当前距离 {distance:.3f} m、方向 {math.degrees(heading):+.1f}°",
                    flush=True,
                )
                last_waypoint_xy = waypoint_xy

            if distance <= args.goal_tolerance:
                send_velocity()
                if close_since is None:
                    close_since = now
                elif now - close_since > 3.0:
                    outcome = "已到当前航点，但 FAR 未在 3 秒内更新或确认到达"
                    break
                time.sleep(0.1)
                continue
            close_since = None
            mode, linear, angular = control_command(
                distance,
                heading,
                args.max_linear_speed,
                args.turn_linear_speed,
                args.max_angular_speed,
                args.arc_linear_speed,
                args.turn_radius,
            )
            display_mode = mode
            blockage = costmap_blockage(
                latest["costmap"],
                odom.pose.pose,
                linear,
                angular,
                args.costmap_lookahead,
                args.costmap_footprint_radius,
                args.costmap_threshold,
                not args.allow_unknown_costmap,
            )
            if blockage is not None:
                cache_key = (received["costmap"], waypoint_xy)
                if cache_key != detour_cache_key:
                    detour_path = costmap_path(
                        latest["costmap"],
                        (position.x, position.y),
                        waypoint_xy,
                        args.costmap_threshold,
                        not args.allow_unknown_costmap,
                    )
                    detour_cache_key = cache_key
                    publish_local_path(detour_path)
                local_point = path_lookahead_point(
                    detour_path, args.local_plan_lookahead
                )
                if local_point is not None:
                    _, _, local_distance, local_heading = body_error_xy(
                        local_point[0], local_point[1], odom.pose.pose
                    )
                    local_mode, local_linear, local_angular = local_avoidance_command(
                        local_distance,
                        local_heading,
                        args.max_linear_speed,
                        args.max_angular_speed,
                        math.radians(args.local_align_angle),
                        args.local_align_angular_speed,
                        args.turn_radius,
                    )
                    local_blockage = costmap_blockage(
                        latest["costmap"],
                        odom.pose.pose,
                        local_linear,
                        local_angular,
                        min(args.costmap_lookahead, local_distance),
                        args.costmap_footprint_radius,
                        args.costmap_threshold,
                        not args.allow_unknown_costmap,
                    )
                    if local_blockage is None:
                        mode, linear, angular = (
                            local_mode, local_linear, local_angular
                        )
                        display_mode = "局部A*绕障/" + local_mode
                        blockage = None
                        if costmap_blocked_since is not None:
                            print(
                                "Costmap 已生成可执行绕障段，恢复运动。",
                                flush=True,
                            )
            if blockage is not None:
                send_velocity()
                if costmap_blocked_since is None:
                    costmap_blocked_since = now
                    print(
                        f"代价地图停车：前方扫掠足迹命中代价 {blockage[2]}，"
                        f"位置 ({blockage[0]:.2f}, {blockage[1]:.2f})；"
                        "等待局部 A* 或 FAR 给出安全路径。",
                        flush=True,
                    )
                if now - costmap_blocked_since >= args.costmap_block_timeout:
                    outcome = (
                        f"代价地图障碍持续 {args.costmap_block_timeout:.1f} 秒，"
                        "FAR 未给出可安全执行的新航点"
                    )
                    break
                mode_started = now
                mode_start_yaw = yaw_of(odom.pose.pose.orientation)
                mode_start_xy = (position.x, position.y)
                last_mode = "代价地图停车"
                time.sleep(max(0.0, 0.1 - (time.monotonic() - loop_started)))
                continue
            if costmap_blocked_since is not None:
                print("代价地图扫掠区域已有安全路径，恢复运动。", flush=True)
                costmap_blocked_since = None
                last_mode = None
            if display_mode != last_mode:
                print(
                    f"控制阶段：{display_mode}；"
                    f"命令 v={linear:.2f} m/s, w={angular:+.2f} rad/s",
                    flush=True,
                )
                last_mode = display_mode
                mode_started = now
                mode_start_yaw = yaw_of(odom.pose.pose.orientation)
                mode_start_xy = (position.x, position.y)
            mode_translation = math.hypot(
                position.x - mode_start_xy[0], position.y - mode_start_xy[1]
            )
            signed_yaw_progress = wrap_angle(
                yaw_of(odom.pose.pose.orientation) - mode_start_yaw
            ) * (1 if angular >= 0.0 else -1)
            if motion_stalled(
                now - mode_started, mode_translation, signed_yaw_progress,
                turning=(mode != "航点跟踪"),
            ):
                actual = latest["walk_speed"].twist
                outcome = (
                    f"5 秒内未检测到有效{'转向' if mode != '航点跟踪' else '平移'}；"
                    f"平移 {mode_translation:.3f} m，目标方向航向变化 "
                    f"{math.degrees(signed_yaw_progress):+.1f}°，"
                    f"walk_speed v={actual.linear.x:.2f} m/s, "
                    f"w={actual.angular.z:+.2f} rad/s"
                )
                break
            send_velocity(linear, angular)
            if first_command_time is None:
                first_command_time = now

            if now - last_report >= 2.0:
                print(
                    f"状态：路线累计 {travel.path_length:.2f} m，"
                    f"距当前航点 {distance:.2f} m，距全局目标 {goal_distance:.2f} m；"
                    f"walk_speed v={latest['walk_speed'].twist.linear.x:.2f}, "
                    f"w={latest['walk_speed'].twist.angular.z:+.2f}",
                    flush=True,
                )
                last_report = now
            time.sleep(max(0.0, 0.1 - (time.monotonic() - loop_started)))

        final_goal_distance = body_error(goal.point, latest["odom"].pose.pose)[2]
        print(
            f"停止原因：{outcome}；过滤路线累计 {travel.path_length:.3f} m；"
            f"净位移 {travel.net_displacement:.3f} m；距全局目标 {final_goal_distance:.3f} m；"
            f"航向变化 {math.degrees(wrap_angle(yaw_of(latest['odom'].pose.pose.orientation) - yaw_of(start_pose.orientation))):+.1f}°；"
            f"cmd_vel_status 非零确认={'是' if command_feedback_seen else '否'}"
        )
        if args.turn_test_only:
            duration = max(time.monotonic() - motion_start, 0.001)
            print(f"转向非零命令 {turn_command_count} 条，试验阶段平均 {turn_command_count / duration:.1f} Hz")
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
