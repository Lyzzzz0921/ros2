"""四个实验：单肌肉、到达、双关节肱二头肌对比、重力前馈必要性。

约定：θ_s 肩关节角（从小臂自然下垂起算，向前为正），θ_e 肘屈曲角。
"""

from __future__ import annotations

import os

import mujoco
import numpy as np
from PIL import Image

from .core import (
    DT, F_MAX, L0_MUS, R_SHOULDER, R_ELBOW, MONO, BIA,
    HillMuscle, arm_xml, arm_matrix, ik_target, forward_kinematics,
    joint_pd, allocate_joint, solve_qp,
)


def _make():
    m = mujoco.MjModel.from_xml_string(arm_xml())
    return m, mujoco.MjData(m)


def _endpoint(d):
    p = d.site("ee").xpos
    return float(p[0]), float(p[2])


def run_a1(rec=None):
    """单条肌肉（肩屈肌）阶跃：肌肉能不能把上臂抬起来。"""
    m, d = _make()
    mus = [HillMuscle(L0_MUS * 1.10) for _ in range(4)]
    A = arm_matrix(MONO)[1]
    ts, ang, frc = [], [], []
    for i in range(int(2.0 / DT)):
        t = i * DT
        q = np.array([d.qpos[0], d.qpos[1]])
        qd = np.array([d.qvel[0], d.qvel[1]])
        u = np.full(4, 0.05)
        u[0] = 0.35 if t >= 0.4 else 0.05          # sh_flex 主肌
        L = L0_MUS - A.T @ q
        Ld = -A.T @ qd
        for j in range(4):
            mus[j].update_activation(u[j], DT)
        F = np.array([mus[j].force(L[j], Ld[j]) for j in range(4)])
        d.qfrc_applied[:] = A @ F
        mujoco.mj_step(m, d)
        if rec:
            rec.maybe(d, i)
        ts.append(t); ang.append(float(d.qpos[0])); frc.append(float(F[0]))
    ang = np.array(ang)
    return {"peak_shoulder": float(ang.max()), "final_shoulder": float(ang[-1]),
            "peak_force": float(np.max(frc))}


TARGETS_XY = [(0.30, -0.30), (0.15, -0.42), (0.38, -0.10)]


def _targets_q():
    return [np.array(ik_target(x, z)) for x, z in TARGETS_XY]


def _simulate(model_dict, mode, seconds_per_target=1.2, ff=True, rec=None):
    m, d = _make()
    names, A = arm_matrix(model_dict)
    muscles = [HillMuscle(L0_MUS * 1.10) for _ in names]
    q_targets = _targets_q()
    kp, kd = np.array([40.0, 25.0]), np.array([4.0, 2.5])

    rows = []
    for k, q_des in enumerate(q_targets):
        for i in range(int(seconds_per_target / DT)):
            q = np.array([d.qpos[0], d.qpos[1]])
            qd = np.array([d.qvel[0], d.qvel[1]])
            tau = joint_pd(q, qd, q_des, kp, kd)
            if ff:
                tau = tau + np.array([d.qfrc_bias[0], d.qfrc_bias[1]])

            if mode == "qp":
                a = np.maximum(solve_qp(A * F_MAX, tau), 0.02)
            else:                                   # 逐关节分配（仅单关节肌肉模型可用）
                a = np.zeros(len(names))
                a[0:2] = allocate_joint(tau[0], R_SHOULDER)
                ak = allocate_joint(tau[1], R_ELBOW)
                a[2], a[3] = ak[0], ak[1]
                a = np.maximum(a, 0.02)

            L = L0_MUS - A.T @ q
            Ld = -A.T @ qd
            for j in range(len(names)):
                muscles[j].update_activation(a[j], DT)
            F = np.array([muscles[j].force(L[j], Ld[j]) for j in range(len(names))])
            d.qfrc_applied[:] = A @ F
            mujoco.mj_step(m, d)
            if rec:
                rec.maybe(d, i)

            ex, ez = _endpoint(d)
            tx, tz = TARGETS_XY[k]
            rows.append((k, float(np.hypot(ex - tx, ez - tz)),
                         float(np.sum(a ** 2)), ex, ez))

    arr = np.array(rows)
    per_target = []
    for k in range(len(q_targets)):
        sub = arr[arr[:, 0] == k]
        tail = sub[int(len(sub) * 0.7):]            # 取后 30% 视作稳态
        per_target.append({"target": TARGETS_XY[k],
                           "err_mm": float(tail[:, 1].mean() * 1000),
                           "act_cost": float(sub[:, 2].mean())})
    return {"per_target": per_target,
            "mean_err_mm": float(np.mean([p["err_mm"] for p in per_target])),
            "act_cost": float(np.mean([p["act_cost"] for p in per_target]))}


def run_a2(rec=None):
    """单关节肌肉模型 + 逐关节分配：到达三个目标点。"""
    return _simulate(MONO, "joint", rec=rec)


def run_a3():
    """双关节肱二头肌 vs 四条单关节肌，同任务对比。"""
    mono = _simulate(MONO, "joint")
    bia = _simulate(BIA, "qp")
    return {"mono": mono, "bia": bia,
            "err_ratio": bia["mean_err_mm"] / max(mono["mean_err_mm"], 1e-9),
            "cost_ratio": bia["act_cost"] / max(mono["act_cost"], 1e-9)}


def run_a3_demo(rec=None):
    """双关节肱二头肌版到达（用于录制动图）。"""
    return _simulate(BIA, "qp", rec=rec)


def run_a4():
    """重力前馈：有无 qfrc_bias 补偿的到达误差对比。"""
    with_ff = _simulate(MONO, "joint", ff=True)
    without = _simulate(MONO, "joint", ff=False)
    return {"with_ff": with_ff, "without_ff": without,
            "degradation": without["mean_err_mm"] / max(with_ff["mean_err_mm"], 1e-9)}


class Recorder:
    """MuJoCo 离屏渲染 -> GIF（懒初始化）。"""

    def __init__(self, every=20, width=640, height=480,
                 lookat=(0.10, 0.0, -0.26), distance=0.95, elevation=-5.0, azimuth=150.0):
        self.renderer = None
        self.width, self.height = width, height
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.lookat[:] = lookat
        self.cam.distance = distance
        self.cam.elevation = elevation
        self.cam.azimuth = azimuth
        self.every = every
        self.frames = []

    def maybe(self, d, i):
        if i % self.every != 0:
            return
        if self.renderer is None:
            self.renderer = mujoco.Renderer(d.model, self.height, self.width)
        self.renderer.update_scene(d, camera=self.cam)
        self.frames.append(self.renderer.render().copy())

    def save(self, path, fps=25, scale=0.62):
        if not self.frames:
            raise RuntimeError("没有录到帧")
        imgs = [Image.fromarray(f).convert("RGB").resize(
            (int(f.shape[1] * scale), int(f.shape[0] * scale)), Image.LANCZOS)
            for f in self.frames]
        imgs[0].save(path, save_all=True, append_images=imgs[1:],
                     duration=int(1000 / fps), loop=0, optimize=False, disposal=1)
        if self.renderer is not None:
            self.renderer.close()
        return os.path.getsize(path)



