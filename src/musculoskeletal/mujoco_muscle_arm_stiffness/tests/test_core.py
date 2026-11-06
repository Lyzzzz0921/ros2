"""核心单元测试：肌肉模型、IK/FK、分配、QP。"""

import unittest

import numpy as np

from mujoco_muscle_arm_stiffness.core import (
    HillMuscle, ik_target, forward_kinematics, allocate_joint, solve_qp,
    arm_matrix, MONO, BIA, F_MAX, R_SHOULDER, R_ELBOW,
)


class TestMuscle(unittest.TestCase):
    def test_fl_peak_at_optimal(self):
        self.assertAlmostEqual(HillMuscle.fl(1.0), 1.0, places=6)
        self.assertLess(HillMuscle.fl(1.3), 1.0)

    def test_fv_centrifugal_stronger(self):
        self.assertGreater(HillMuscle.fv(-0.5), 1.0)
        self.assertLess(HillMuscle.fv(0.5), 1.0)

    def test_passive_only_when_stretched(self):
        self.assertEqual(HillMuscle.fp(1.0), 0.0)
        self.assertGreater(HillMuscle.fp(1.2), 0.0)


class TestKinematics(unittest.TestCase):
    def test_ik_fk_roundtrip(self):
        for target in [(0.30, -0.30), (0.15, -0.42), (0.38, -0.10)]:
            q1, q2 = ik_target(*target)
            x, z = forward_kinematics(q1, q2)
            self.assertAlmostEqual(x, target[0], places=6)
            self.assertAlmostEqual(z, target[1], places=6)

    def test_ik_respects_reach(self):
        q1, q2 = ik_target(0.30, -0.30)
        self.assertGreaterEqual(q2, 0.0)
        self.assertLessEqual(q2, np.pi)


class TestAllocation(unittest.TestCase):
    def test_elbow_positive_uses_flexor(self):
        """MONO 顺序是 [sh_flex, sh_ext, el_flex, el_ext]，
        正力矩（屈肘）必须落在 el_flex（索引 2），不能再交换。"""
        a2 = allocate_joint(+1.5, R_ELBOW)
        self.assertGreater(a2[0], a2[1])

    def test_shoulder_positive_uses_flexor(self):
        a1 = allocate_joint(+1.5, R_SHOULDER)
        self.assertGreater(a1[0], a1[1])


class TestQP(unittest.TestCase):
    def test_bia_solves_requested_torque(self):
        _, A = arm_matrix(BIA)
        tau = np.array([2.0, 1.0])
        a = solve_qp(A * F_MAX, tau)
        self.assertTrue(np.allclose(A * F_MAX @ a, tau, atol=1e-9))
        self.assertTrue(np.all(a >= -1e-12) and np.all(a <= 1.0 + 1e-12))

    def test_mono_has_four_muscles(self):
        names, A = arm_matrix(MONO)
        self.assertEqual(len(names), 4)
        self.assertEqual(A.shape, (2, 4))

    def test_bia_has_four_muscles(self):
        names, A = arm_matrix(BIA)
        self.assertEqual(len(names), 4)
        self.assertIn("biceps", names)


if __name__ == "__main__":
    unittest.main()
