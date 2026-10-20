import importlib.util
import math
import pathlib
import unittest
from types import SimpleNamespace


SCRIPT = (
    pathlib.Path(__file__).resolve().parents[1]
    / "scripts"
    / "far_waypoint_follower_trial.py"
)
SPEC = importlib.util.spec_from_file_location("far_waypoint_follower_trial", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FilteredTravelAccumulatorTest(unittest.TestCase):
    def test_millimetre_oscillation_is_not_route_travel(self):
        travel = MODULE.FilteredTravelAccumulator(0.0, 0.0)
        for index in range(400):
            travel.update(0.003 if index % 2 else -0.003, 0.0)
        self.assertEqual(travel.path_length, 0.0)
        self.assertLess(travel.net_displacement, 0.004)

    def test_slow_motion_accumulates_in_chunks(self):
        travel = MODULE.FilteredTravelAccumulator(0.0, 0.0)
        for index in range(1, 101):
            travel.update(index * 0.001, 0.0)
        self.assertGreaterEqual(travel.path_length, 0.08)
        self.assertAlmostEqual(travel.net_displacement, 0.1, places=6)


class ControlCommandTest(unittest.TestCase):
    def test_small_heading_uses_requested_speed_cap(self):
        mode, linear, angular = MODULE.control_command(1.0, 0.0, 0.4)
        self.assertEqual(mode, "航点跟踪")
        self.assertEqual(linear, 0.4)
        self.assertEqual(angular, 0.0)

    def test_medium_heading_uses_slow_arc(self):
        mode, linear, angular = MODULE.control_command(
            1.0, math.radians(20.0), 0.4
        )
        self.assertEqual(mode, "低速弧线转向")
        self.assertEqual(linear, 0.08)
        self.assertGreater(angular, 0.0)

    def test_medium_heading_uses_explicit_arc_speed(self):
        mode, linear, angular = MODULE.control_command(
            1.0, math.radians(41.0), 0.4, 0.05, 0.2,
            arc_linear_speed=0.2,
        )
        self.assertEqual(mode, "低速弧线转向")
        self.assertEqual(linear, 0.2)
        self.assertEqual(angular, 0.2)

    def test_large_heading_can_use_configured_walking_turn(self):
        mode, linear, angular = MODULE.control_command(
            1.0, math.radians(-60.0), 0.4, 0.05, 0.2
        )
        self.assertEqual(mode, "大角度行走转向")
        self.assertEqual(linear, 0.05)
        self.assertEqual(angular, -0.2)

    def test_turn_trial_uses_pure_yaw_command(self):
        self.assertEqual(
            MODULE.turn_trial_command(math.radians(-30), 0.3),
            ("原地转向试验", 0.0, -0.3),
        )

    def test_walking_turn_uses_requested_radius(self):
        mode, linear, angular = MODULE.control_command(
            1.0,
            math.radians(-26.0),
            0.4,
            turn_linear_speed=0.4,
            max_angular_speed=0.6,
            arc_linear_speed=0.4,
            turn_radius=0.8,
        )
        self.assertEqual(mode, "低速弧线转向")
        self.assertAlmostEqual(linear, 0.4)
        self.assertAlmostEqual(angular, -0.5)
        self.assertAlmostEqual(abs(linear / angular), 0.8)


def pose_at(x=0.0, yaw_degrees=0.0):
    angle = math.radians(yaw_degrees) / 2.0
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(angle), w=math.cos(angle)),
    )


def occupancy_grid(width=20, height=20, resolution=0.1, origin=(-1.0, -1.0)):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="fastlio_odom"),
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin[0], y=origin[1]),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=[0] * (width * height),
    )


def set_cost(grid, world_x, world_y, cost):
    gx = int((world_x - grid.info.origin.position.x) / grid.info.resolution)
    gy = int((world_y - grid.info.origin.position.y) / grid.info.resolution)
    grid.data[gy * grid.info.width + gx] = cost


class CostmapGuardTest(unittest.TestCase):
    def test_clear_corridor_allows_forward_command(self):
        grid = occupancy_grid()
        self.assertIsNone(
            MODULE.costmap_blockage(
                grid, pose_at(), linear=0.4, angular=0.0,
                lookahead=0.8, footprint_radius=0.20,
            )
        )

    def test_lethal_cell_in_swept_footprint_blocks_command(self):
        grid = occupancy_grid()
        set_cost(grid, 0.55, 0.05, 100)
        blockage = MODULE.costmap_blockage(
            grid, pose_at(), linear=0.4, angular=0.0,
            lookahead=0.8, footprint_radius=0.20,
        )
        self.assertIsNotNone(blockage)
        self.assertEqual(blockage[2], 100)

    def test_curved_command_checks_arc_not_only_straight_line(self):
        grid = occupancy_grid(width=40, height=40, origin=(-2.0, -2.0))
        set_cost(grid, 0.55, 0.18, 100)
        blockage = MODULE.costmap_blockage(
            grid, pose_at(), linear=0.4, angular=0.5,
            lookahead=0.8, footprint_radius=0.12,
        )
        self.assertIsNotNone(blockage)
        self.assertEqual(blockage[2], 100)

    def test_obstacle_outside_footprint_does_not_block(self):
        grid = occupancy_grid()
        set_cost(grid, 0.55, 0.45, 100)
        self.assertIsNone(
            MODULE.costmap_blockage(
                grid, pose_at(), linear=0.4, angular=0.0,
                lookahead=0.8, footprint_radius=0.20,
            )
        )

    def test_unknown_cell_blocks_by_default(self):
        grid = occupancy_grid()
        set_cost(grid, 0.55, 0.05, -1)
        blockage = MODULE.costmap_blockage(
            grid, pose_at(), linear=0.4, angular=0.0,
            lookahead=0.8, footprint_radius=0.20,
        )
        self.assertIsNotNone(blockage)
        self.assertEqual(blockage[2], -1)


class CostmapLocalPlannerTest(unittest.TestCase):
    def test_astar_routes_around_blocking_obstacle(self):
        grid = occupancy_grid(width=40, height=40, origin=(-1.0, -2.0))
        for row in range(-4, 5):
            set_cost(grid, 0.55, row * 0.1 + 0.05, 100)
        path = MODULE.costmap_path(
            grid, (0.0, 0.0), (1.5, 0.0), lethal_threshold=65,
            unknown_is_obstacle=True,
        )
        self.assertIsNotNone(path)
        self.assertGreater(max(abs(y) for _, y in path), 0.40)
        self.assertGreater(path[-1][0], 1.3)

    def test_astar_reports_no_path_through_complete_wall(self):
        grid = occupancy_grid(width=30, height=30, origin=(-1.0, -1.5))
        for row in range(30):
            set_cost(grid, 0.55, -1.45 + row * 0.1, 100)
        path = MODULE.costmap_path(
            grid, (0.0, 0.0), (1.5, 0.0), lethal_threshold=65,
            unknown_is_obstacle=True,
        )
        self.assertIsNone(path)

    def test_path_lookahead_selects_a_local_detour_point(self):
        path = [(0.0, 0.0), (0.2, 0.0), (0.4, 0.2), (0.6, 0.4), (1.0, 0.4)]
        point = MODULE.path_lookahead_point(path, 0.55)
        self.assertEqual(point, (0.6, 0.4))

    def test_large_local_path_heading_rotates_before_translation(self):
        mode, linear, angular = MODULE.local_avoidance_command(
            distance=0.5,
            heading=math.radians(30.0),
            max_linear_speed=0.4,
            max_angular_speed=0.6,
            align_angle=math.radians(10.0),
            align_angular_speed=0.5,
        )
        self.assertEqual(mode, "局部A*原地对准")
        self.assertEqual(linear, 0.0)
        self.assertAlmostEqual(angular, 0.5)

    def test_aligned_local_path_translates(self):
        mode, linear, angular = MODULE.local_avoidance_command(
            distance=0.5,
            heading=math.radians(5.0),
            max_linear_speed=0.4,
            max_angular_speed=0.6,
            align_angle=math.radians(10.0),
            align_angular_speed=0.5,
        )
        self.assertNotEqual(mode, "局部A*原地对准")
        self.assertGreater(linear, 0.0)


class TurnTrialGuardTest(unittest.TestCase):
    def test_stops_at_small_yaw_limit(self):
        reason = MODULE.turn_trial_guard(pose_at(), pose_at(yaw_degrees=10.2), 1, 10.0, 1.0)
        self.assertIn("转角上限", reason)

    def test_stops_if_robot_translates_during_pure_turn(self):
        reason = MODULE.turn_trial_guard(pose_at(), pose_at(x=0.06), 1, 10.0, 1.0)
        self.assertIn("平移", reason)

    def test_stops_if_yaw_does_not_change(self):
        reason = MODULE.turn_trial_guard(pose_at(), pose_at(yaw_degrees=0.2), 1, 10.0, 5.1)
        self.assertIn("未检测到有效转向", reason)

    def test_stops_if_yaw_goes_the_wrong_way(self):
        reason = MODULE.turn_trial_guard(pose_at(), pose_at(yaw_degrees=-2.1), 1, 10.0, 1.0)
        self.assertIn("反向", reason)

    def test_allows_initial_yaw_progress(self):
        self.assertIsNone(
            MODULE.turn_trial_guard(pose_at(), pose_at(yaw_degrees=4.0), 1, 10.0, 1.0)
        )


class TurnTrialInputTest(unittest.TestCase):
    def test_accepts_near_side_target_without_far_waypoint(self):
        self.assertIsNone(
            MODULE.turn_trial_input_problem((0.0, 0.0, 0.3, math.radians(100)), 1.0)
        )

    def test_rejects_target_too_near_with_distance(self):
        problem = MODULE.turn_trial_input_problem(
            (0.0, 0.0, 0.1, math.radians(35)), 1.0
        )
        self.assertIn("0.100", problem)
        self.assertIn("距离", problem)

    def test_rejects_rear_ambiguous_heading_with_angle(self):
        problem = MODULE.turn_trial_input_problem(
            (0.0, 0.0, 0.5, math.radians(175)), 1.0
        )
        self.assertIn("175.0", problem)
        self.assertIn("偏角", problem)


class TurnTrialSpeedTest(unittest.TestCase):
    def test_allows_measured_joystick_range(self):
        self.assertIsNone(MODULE.turn_trial_speed_problem(0.5))
        self.assertIsNone(MODULE.turn_trial_speed_problem(0.6))

    def test_rejects_joystick_peak_rate(self):
        self.assertIsNotNone(MODULE.turn_trial_speed_problem(2.0))


class RouteLimitTest(unittest.TestCase):
    def test_five_metre_goal_does_not_default_to_five_metre_travel(self):
        self.assertAlmostEqual(MODULE.default_max_travel(5.0), 0.2)

    def test_small_route_keeps_travel_within_route(self):
        self.assertAlmostEqual(MODULE.default_max_travel(0.2), 0.2)


class GoWaypointTest(unittest.TestCase):
    def test_allows_recent_waypoint_from_current_goal_before_go(self):
        self.assertIsNone(MODULE.cached_waypoint_problem(10.0, 11.0, 14.0))

    def test_rejects_waypoint_from_older_goal(self):
        self.assertIn("早于", MODULE.cached_waypoint_problem(10.0, 9.0, 11.0))

    def test_rejects_stale_waypoint(self):
        self.assertIn(
            "未更新",
            MODULE.cached_waypoint_problem(10.0, 11.0, 17.0, max_age=5.0),
        )

    def test_rejects_pose_change_while_confirming(self):
        self.assertIn("位姿变化", MODULE.pose_changed_problem(pose_at(), pose_at(x=0.06)))
        self.assertIn("位姿变化", MODULE.pose_changed_problem(pose_at(), pose_at(yaw_degrees=6)))

    def test_accepts_stationary_pose(self):
        self.assertIsNone(MODULE.pose_changed_problem(pose_at(), pose_at(x=0.01)))


class GoalReachedTest(unittest.TestCase):
    def test_premature_far_reached_signal_is_rejected(self):
        self.assertFalse(MODULE.accept_goal_reached(True, 0.471, 0.18))

    def test_far_reached_signal_inside_tolerance_is_accepted(self):
        self.assertTrue(MODULE.accept_goal_reached(True, 0.17, 0.18))


class WaypointSafetyTest(unittest.TestCase):
    def test_rejects_waypoint_beyond_trial_range(self):
        self.assertIsNotNone(MODULE.waypoint_problem(0.94, 1.99, 1.0, 0.18))

    def test_rejects_self_waypoint_when_goal_is_far(self):
        self.assertIsNotNone(MODULE.waypoint_problem(0.71, 0.002, 1.0, 0.18))

    def test_accepts_nearby_plausible_waypoint(self):
        self.assertIsNone(MODULE.waypoint_problem(0.8, 0.6, 1.0, 0.18))


class GoalPlanningTest(unittest.TestCase):
    def test_rejects_far_goal_before_waiting_for_waypoint(self):
        problem = MODULE.goal_range_problem(7.2, 5.0)
        self.assertIn("7.200", problem)
        self.assertIn("5.00", problem)
        self.assertIn("不会继续等待 FAR", problem)

    def test_accepts_goal_inside_configured_range(self):
        self.assertIsNone(MODULE.goal_range_problem(4.9, 5.0))

    def test_missing_waypoint_message_identifies_planner_failure(self):
        message = MODULE.planner_timeout_message(4.2, 60.0)
        self.assertIn("4.200", message)
        self.assertIn("未生成首个航点", message)
        self.assertIn("V-Graph", message)

    def test_far_goal_with_local_waypoint_is_allowed(self):
        self.assertIsNone(
            MODULE.planning_problem(
                goal_distance=8.891,
                waypoint_distance=4.2,
                max_goal_distance=15.0,
                max_waypoint_distance=5.0,
                tolerance=0.18,
            )
        )

    def test_far_goal_limit_is_independent_from_waypoint_limit(self):
        problem = MODULE.planning_problem(16.0, 4.2, 15.0, 5.0, 0.18)
        self.assertIn("16.000", problem)

    def test_local_waypoint_limit_is_still_enforced(self):
        problem = MODULE.planning_problem(8.891, 5.2, 15.0, 5.0, 0.18)
        self.assertIn("航点", problem)
        self.assertIn("5.200", problem)


class MotionWatchdogTest(unittest.TestCase):
    def test_turn_without_yaw_progress_stalls_even_if_it_translates(self):
        self.assertTrue(MODULE.motion_stalled(5.1, 0.1, 0.0, turning=True))

    def test_turn_with_yaw_progress_does_not_stall(self):
        self.assertFalse(MODULE.motion_stalled(5.1, 0.01, math.radians(3), turning=True))

    def test_forward_without_translation_stalls(self):
        self.assertTrue(MODULE.motion_stalled(5.1, 0.003, 0.0, turning=False))

    def test_watchdog_waits_five_seconds(self):
        self.assertFalse(MODULE.motion_stalled(4.9, 0.0, 0.0, turning=True))


class SpeedEndpointTest(unittest.TestCase):
    def test_duplicate_or_new_speed_publisher_is_visible(self):
        class Endpoint:
            def __init__(self, name):
                self.node_name = name

        class Node:
            def get_publishers_info_by_topic(self, topic):
                self_topic = "/hric/robot/cmd_vel"
                assert topic == self_topic
                return [
                    Endpoint("joystick_bridge_node"),
                    Endpoint("far_waypoint_follower_trial"),
                    Endpoint("hric_path_tracker"),
                ]

        self.assertEqual(
            MODULE.speed_publisher_names(Node()),
            ["far_waypoint_follower_trial", "hric_path_tracker", "joystick_bridge_node"],
        )


if __name__ == "__main__":
    unittest.main()
