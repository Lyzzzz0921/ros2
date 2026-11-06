"""实验层测试：到达精度、双关节代价、重力前馈、端点刚度、圆轨迹跟踪。"""

import unittest

from mujoco_muscle_arm_stiffness import extras, runs


class TestSingleMuscle(unittest.TestCase):
    def test_muscle_lifts_arm(self):
        res = runs.run_a1()
        self.assertGreater(res["peak_shoulder"], 0.5)
        self.assertGreater(res["peak_force"], 50.0)


class TestReach(unittest.TestCase):
    def test_reach_accuracy(self):
        res = runs.run_a2()
        self.assertLess(res["mean_err_mm"], 30.0)
        self.assertEqual(len(res["per_target"]), 3)


class TestBiarticular(unittest.TestCase):
    def test_biceps_saves_activation(self):
        res = runs.run_a3()
        self.assertLess(res["cost_ratio"], 1.0)

    def test_biceps_costs_accuracy(self):
        res = runs.run_a3()
        self.assertGreater(res["err_ratio"], 1.0)


class TestGravityFeedforward(unittest.TestCase):
    def test_without_feedforward_degrades(self):
        res = runs.run_a4()
        self.assertGreater(res["degradation"], 2.0)


class TestStiffness(unittest.TestCase):
    def test_stiffness_grows_with_coactivation(self):
        res = extras.run_a5()
        ks = [lv["k_major"] for lv in res["levels"]]
        self.assertGreater(ks[-1], ks[0] * 1.5)

    def test_stiffness_is_anisotropic(self):
        res = extras.run_a5()
        for lv in res["levels"]:
            self.assertGreater(lv["anisotropy"], 1.5)
            self.assertGreater(lv["k_major"], lv["k_minor"])


class TestTracking(unittest.TestCase):
    def test_circle_tracking_accuracy(self):
        res = extras.run_a6()
        self.assertLess(res["mean_err_mm"], 25.0)
        self.assertGreater(len(res["traj"]), 50)




if __name__ == "__main__":
    unittest.main()

