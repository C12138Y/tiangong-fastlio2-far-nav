import importlib.util
import pathlib
import unittest


SCRIPT = (
    pathlib.Path(__file__).resolve().parents[1]
    / "scripts"
    / "far_single_waypoint_trial.py"
)
SPEC = importlib.util.spec_from_file_location("far_single_waypoint_trial", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MotionAccumulatorTest(unittest.TestCase):
    def test_oscillating_odometry_does_not_look_like_net_translation(self):
        motion = MODULE.MotionAccumulator(0.0, 0.0)
        for index in range(200):
            motion.update(0.001 if index % 2 else -0.001, 0.0)

        self.assertGreater(motion.path_length, 0.30)
        self.assertLessEqual(motion.net_displacement, 0.0011)

    def test_straight_motion_reaches_net_translation_limit(self):
        motion = MODULE.MotionAccumulator(0.0, 0.0)
        for index in range(1, 16):
            motion.update(index * 0.01, 0.0)

        self.assertAlmostEqual(motion.net_displacement, 0.15, places=6)
        self.assertAlmostEqual(motion.path_length, 0.15, places=6)


if __name__ == "__main__":
    unittest.main()
