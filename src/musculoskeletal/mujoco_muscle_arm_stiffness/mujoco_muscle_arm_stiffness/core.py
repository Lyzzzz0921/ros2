"""核心：Hill 肌肉、肩肘两关节模型、肌肉分配。

设计要点
--------
- MuJoCo 只负责刚体动力学与关节限位，肌力经 qfrc_applied 施加；
- 肌肉几何用解析矩臂：L = L0 - (a_s*θ_s + a_e*θ_e)；
- 双关节肱二头肌同时跨肩、肘（a_s>0, a_e>0），是电机模型没有的现象。
"""

from __future__ import annotations

import itertools

import numpy as np

DT = 0.002

F_MAX = 400.0        # 单条肌肉最大等长力 (N)
L0_MUS = 0.30        # 关节中立位的肌肉长度 (m)
R_SHOULDER = 0.035   # 肩关节矩臂 (m)
R_ELBOW = 0.030      # 肘关节矩臂 (m)

L_UPPER = 0.25       # 上臂长 (m)
L_FORE = 0.25        # 前臂长 (m)
M_UPPER, M_FORE = 2.0, 1.2


class HillMuscle:
    """Hill 三元件模型（与仓库其他模块同一组曲线形式）。"""

    def __init__(self, l_opt: float, f_max: float = F_MAX):
        self.l_opt = float(l_opt)
        self.f_max = float(f_max)
        self.v_max = 10.0 * self.l_opt
        self.a = 0.0

    @staticmethod
    def fl(ln):
        return float(np.exp(-((ln - 1.0) / 0.45) ** 2))

    @staticmethod
    def fl_prime(ln):
        return float(-2.0 * (ln - 1.0) / 0.45 ** 2 * np.exp(-((ln - 1.0) / 0.45) ** 2))

    @staticmethod
    def fp(ln):
        if ln <= 1.0:
            return 0.0
        return float((np.exp(5.0 * (ln - 1.0)) - 1.0) / (np.exp(5.0 * 0.6) - 1.0))

    @staticmethod
    def fv(vn):
        if vn >= 0:
            return float(0.3125 / (vn + 0.25) - 0.25)
        x = min(1.0, -vn)
        return float(1.0 + 0.4 * x / (0.25 + x))

    def update_activation(self, u, dt, tau_act=0.01, tau_deact=0.04):
        tau = tau_act if u > self.a else tau_deact
        self.a = float(np.clip(self.a + (u - self.a) * min(1.0, dt / tau), 0.0, 1.0))
        return self.a

    def force(self, length, length_dot):
        ln = length / self.l_opt
        vn = -length_dot / self.v_max
        return self.f_max * (self.a * self.fl(ln) * self.fv(vn) + self.fp(ln))


def arm_xml(gravity="0 0 -9.81"):
    """肩 + 肘两关节平面臂。"""
    return f"""
<mujoco model="muscle_arm">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{DT}" integrator="implicitfast" gravity="{gravity}"/>
  <visual>
    <global offwidth="640" offheight="480" fovy="45"/>
    <headlight ambient="0.45 0.45 0.5" diffuse="0.6 0.6 0.6"/>
    <map znear="0.01" zfar="20"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.26 0.26 0.30"
             rgb2="0.17 0.17 0.20" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="6 6" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="0.5 -0.8 1.2" dir="-0.2 0.4 -1"/>
    <camera name="side" pos="0.9 -0.9 0.15" xyaxes="0.7 0.7 0 -0.2 0.2 1"/>
    <geom name="floor" type="plane" size="1.5 1.5 0.01" pos="0 0 -0.75" material="grid"/>
    <body name="torso" pos="0 0 0">
      <geom type="box" size="0.06 0.05 0.12" pos="0 0 0.06"
            rgba="0.55 0.57 0.62 1" mass="5"/>
      <body name="upper" pos="0 0 0">
        <joint name="shoulder" type="hinge" axis="0 -1 0" range="-1.4 1.8"
               damping="0.20" armature="0.004"/>
        <geom type="capsule" size="0.026" fromto="0 0 0 0 0 -{L_UPPER}"
              mass="{M_UPPER}" rgba="0.20 0.55 0.95 1"/>
        <body name="fore" pos="0 0 -{L_UPPER}">
          <joint name="elbow" type="hinge" axis="0 -1 0" range="0.0 2.2"
                 damping="0.15" armature="0.002"/>
          <geom type="capsule" size="0.023" fromto="0 0 0 0 0 -{L_FORE}"
                mass="{M_FORE}" rgba="0.95 0.55 0.20 1"/>
          <site name="ee" pos="0 0 -{L_FORE}" size="0.015" rgba="1 0.85 0.2 1"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


# 肌肉力臂 (a_shoulder, a_elbow)
MONO = {
    "sh_flex": (R_SHOULDER, 0.0),
    "sh_ext": (-R_SHOULDER, 0.0),
    "el_flex": (0.0, R_ELBOW),
    "el_ext": (0.0, -R_ELBOW),
}
# 双关节版：肘屈肌换成跨肩肘的肱二头肌（屈肘 + 屈肩）
BIA = {
    "sh_flex": (R_SHOULDER, 0.0),
    "sh_ext": (-R_SHOULDER, 0.0),
    "el_ext": (0.0, -R_ELBOW),
    "biceps": (R_SHOULDER, R_ELBOW),
}


def arm_matrix(model_dict):
    names = list(model_dict)
    return names, np.array([model_dict[n] for n in names]).T


def ik_target(x, z, l1=L_UPPER, l2=L_FORE):
    """两连杆逆解：末端 (x,z) -> (θ_shoulder, θ_elbow)。

    几何约定：θ1 从竖直向下起算，向 +x 为正；θ2 为肘屈曲（向前为正）。
    """
    r2 = x * x + z * z
    c2 = (r2 - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    c2 = float(np.clip(c2, -1.0, 1.0))
    q2 = float(np.arccos(c2))
    q1 = float(np.arctan2(x, -z) - np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2)))
    return q1, q2


def forward_kinematics(q1, q2, l1=L_UPPER, l2=L_FORE):
    x = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    z = -l1 * np.cos(q1) - l2 * np.cos(q1 + q2)
    return float(x), float(z)


def joint_pd(q, qd, q_des, kp, kd):
    return kp * (np.asarray(q_des) - np.asarray(q)) - kd * np.asarray(qd)


def allocate_joint(tau, arm, f_max=F_MAX, base=0.02):
    """单关节两肌分配：主肌出力 + 对侧 base 共激活。返回 (正力矩肌, 负力矩肌)。"""
    a = np.array([base, base])
    a[0 if tau >= 0 else 1] = float(np.clip(abs(tau) / (abs(arm) * f_max), 0.0, 1.0))
    return a


def solve_qp(A, tau, a_max=1.0):
    """min Σa² s.t. A·a = tau, 0 ≤ a ≤ a_max（肌肉少，枚举子集即精确解）。"""
    A = np.asarray(A, float)
    tau = np.asarray(tau, float)
    n = A.shape[1]
    if np.allclose(tau, 0.0):
        return np.zeros(n)
    for k in (1, 2, 3):
        best = None
        for combo in itertools.combinations(range(n), k):
            As = A[:, combo]
            if np.linalg.matrix_rank(As) < 2:
                continue
            a, *_ = np.linalg.lstsq(As, tau, rcond=None)
            if np.all(a >= -1e-9) and np.all(a <= a_max + 1e-9):
                cost = float(np.sum(a ** 2))
                if best is None or cost < best[0]:
                    best = (cost, combo, a)
        if best is not None:
            out = np.zeros(n)
            out[list(best[1])] = np.clip(best[2], 0.0, a_max)
            return out
    a, *_ = np.linalg.lstsq(A, tau, rcond=None)
    return np.clip(a, 0.0, a_max)


