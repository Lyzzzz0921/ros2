"""A5 端点刚度与 A6 圆轨迹跟踪。

单独成文件，便于 `bench`/`gif` 复用，也让 A1-A4 的代码保持不动。
"""

from __future__ import annotations

import mujoco
import numpy as np

from .core import (
    DT, F_MAX, L0_MUS, MONO, HillMuscle, arm_xml, arm_matrix,
    ik_target, forward_kinematics, allocate_joint,
)

R_SHOULDER = 0.035
R_ELBOW = 0.030
REF_XY = (0.30, -0.30)
CO_ACT_LEVELS = (0.05, 0.15, 0.30)


# ---------------- A5：端点刚度 ----------------

def joint_torque(q, base):
    """固定共激活下的净关节力矩（主动 + 被动）。"""
    names, A = arm_matrix(MONO)
    L = L0_MUS - A.T @ np.asarray(q, float)
    tau = np.zeros(2)
    for j in range(len(names)):
        ln = L[j] / (L0_MUS * 1.10)
        f = F_MAX * (base * HillMuscle.fl(ln) + HillMuscle.fp(ln))
        tau += A[:, j] * f
    return tau


def jacobian(q, h=1e-6):
    J = np.zeros((2, 2))
    for i in range(2):
        qp = np.array(q, float); qp[i] += h
        qm = np.array(q, float); qm[i] -= h
        xp, zp = forward_kinematics(qp[0], qp[1])
        xm, zm = forward_kinematics(qm[0], qm[1])
        J[0, i] = (xp - xm) / (2 * h)
        J[1, i] = (zp - zm) / (2 * h)
    return J


def endpoint_stiffness(q, base, h=1e-5):
    """Kx = J^-T · Kj · J^-1，Kj = -dτ/dq。"""
    Kj = np.zeros((2, 2))
    for i in range(2):
        qp = np.array(q, float); qp[i] += h
        qm = np.array(q, float); qm[i] -= h
        Kj[:, i] = -(joint_torque(qp, base) - joint_torque(qm, base)) / (2 * h)
    J = jacobian(q)
    Jinv = np.linalg.inv(J)
    K = Jinv.T @ Kj @ Jinv
    return 0.5 * (K + K.T)


def run_a5():
    """三个共激活水平下的末端刚度椭圆（特征值与方向）。"""
    q = np.array(ik_target(*REF_XY))
    x0, z0 = forward_kinematics(q[0], q[1])
    levels = []
    for base in CO_ACT_LEVELS:
        K = endpoint_stiffness(q, base)
        w, V = np.linalg.eigh(K)
        order = np.argsort(w)[::-1]
        w, V = w[order], V[:, order]
        levels.append({
            "co_activation": base,
            "k_major": float(max(w[0], 0.0)),
            "k_minor": float(max(w[1], 0.0)),
            "anisotropy": float(max(w[0], 1e-9) / max(w[1], 1e-9)),
            "dir_major": [float(V[0, 0]), float(V[1, 0])],
            "dir_minor": [float(V[0, 1]), float(V[1, 1])],
        })
    return {"ref_x": float(x0), "ref_z": float(z0), "levels": levels}


# ---------------- A6：圆轨迹跟踪 ----------------

def track_circle(seconds=4.0, radius=0.08, center=(0.30, -0.30), rec=None):
    m = mujoco.MjModel.from_xml_string(arm_xml())
    d = mujoco.MjData(m)
    q_init = np.array(ik_target(center[0] + radius, center[1]))
    d.qpos[0], d.qpos[1] = q_init[0], q_init[1]
    mujoco.mj_forward(m, d)

    _, A = arm_matrix(MONO)
    muscles = [HillMuscle(L0_MUS * 1.10) for _ in range(4)]
    kp, kd = np.array([60.0, 40.0]), np.array([6.0, 4.0])
    omega = 2 * np.pi / 2.0

    rows = []
    for i in range(int(seconds / DT)):
        t = i * DT
        tx = center[0] + radius * np.cos(omega * t)
        tz = center[1] + radius * np.sin(omega * t)
        q_des = np.array(ik_target(tx, tz))
        tx2 = center[0] + radius * np.cos(omega * (t + DT))
        tz2 = center[1] + radius * np.sin(omega * (t + DT))
        qd_des = (np.array(ik_target(tx2, tz2)) - q_des) / DT

        q = np.array([d.qpos[0], d.qpos[1]])
        qd = np.array([d.qvel[0], d.qvel[1]])
        tau = (kp * (q_des - q) + kd * (qd_des - qd)
               + np.array([d.qfrc_bias[0], d.qfrc_bias[1]]))

        a = np.zeros(4)
        a[0:2] = allocate_joint(tau[0], R_SHOULDER)
        ak = allocate_joint(tau[1], R_ELBOW)
        a[2], a[3] = ak[0], ak[1]
        a = np.maximum(a, 0.02)

        L = L0_MUS - A.T @ q
        Ld = -A.T @ qd
        for j in range(4):
            muscles[j].update_activation(a[j], DT)
        F = np.array([muscles[j].force(L[j], Ld[j]) for j in range(4)])
        d.qfrc_applied[:] = A @ F
        mujoco.mj_step(m, d)
        if rec:
            rec.maybe(d, i)

        ex, ez = float(d.site("ee").xpos[0]), float(d.site("ee").xpos[2])
        rows.append((t, ex, ez, tx, tz, float(np.hypot(ex - tx, ez - tz))))

    return np.array(rows)


def run_a6(rec=None):
    arr = track_circle(rec=rec)
    ss = arr[arr[:, 0] > 1.0]
    step = max(1, len(arr) // 120)
    traj = arr[::step, [0, 1, 2, 3, 4]]
    return {
        "radius_m": 0.08,
        "period_s": 2.0,
        "mean_err_mm": float(ss[:, 5].mean() * 1000),
        "max_err_mm": float(ss[:, 5].max() * 1000),
        "traj": [[float(v) for v in row] for row in traj],
    }
