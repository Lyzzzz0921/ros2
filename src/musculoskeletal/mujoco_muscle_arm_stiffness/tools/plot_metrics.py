#!/usr/bin/env python3
"""从 metrics.json 生成四张图：A3 权衡、A4 前馈、A5 刚度椭圆、A6 轨迹误差。

    python3 tools/plot_metrics.py out/metrics.json out
"""

import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def plot_a3(d, outdir):
    a3 = d["a3"]
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.4))
    ax[0].bar(["mono", "biceps"], [a3["mono"]["mean_err_mm"], a3["bia"]["mean_err_mm"]],
              color=["#4c78a8", "#e45756"])
    ax[0].set_ylabel("endpoint error (mm)")
    ax[0].set_title("reach accuracy")
    ax[1].bar(["mono", "biceps"], [a3["mono"]["act_cost"], a3["bia"]["act_cost"]],
              color=["#4c78a8", "#54a24b"])
    ax[1].set_ylabel("activation cost  mean sum(a^2)")
    ax[1].set_title("energy cost")
    for a in ax:
        a.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    p = os.path.join(outdir, "a3_biceps_tradeoff.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def plot_a4(d, outdir):
    a4 = d["a4"]
    fig, ax = plt.subplots(figsize=(5.2, 3.2))
    ax.bar(["with feedforward", "without"],
           [a4["with_ff"]["mean_err_mm"], a4["without_ff"]["mean_err_mm"]],
           color=["#4c78a8", "#f58518"])
    ax.set_ylabel("endpoint error (mm)")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    p = os.path.join(outdir, "a4_gravity_feedforward.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def plot_a5(d, outdir):
    a5 = d["a5"]
    x0, z0 = a5["ref_x"], a5["ref_z"]
    colors = ["#4c78a8", "#f58518", "#54a24b"]
    fig, ax = plt.subplots(figsize=(6, 5))
    t = np.linspace(0, 2 * np.pi, 200)
    for lv, c in zip(a5["levels"], colors):
        dm = np.array(lv["dir_major"])
        dn = np.array(lv["dir_minor"])
        a = np.sqrt(max(lv["k_major"], 0.0)) * 0.02
        b = np.sqrt(max(lv["k_minor"], 0.0)) * 0.02
        pts = np.outer(dm, a * np.cos(t)) + np.outer(dn, b * np.sin(t))
        ax.plot(x0 + pts[0], z0 + pts[1], color=c, lw=1.8,
                label=f"co-act {lv['co_activation']:.2f}: "
                      f"{lv['k_major']:.1f} / {lv['k_minor']:.1f} N/m")
        ax.plot([x0], [z0], "o", color=c, ms=3)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("z (m)")
    ax.set_title("endpoint stiffness ellipse vs co-activation")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    p = os.path.join(outdir, "a5_stiffness_ellipses.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def plot_a6(d, outdir):
    a6 = d["a6"]
    tr = np.array(a6["traj"])
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    ax[0].plot(tr[:, 3], tr[:, 4], "--", color="#888", label="target")
    ax[0].plot(tr[:, 1], tr[:, 2], "-", color="#4c78a8", label="endpoint")
    ax[0].set_aspect("equal")
    ax[0].set_xlabel("x (m)")
    ax[0].set_ylabel("z (m)")
    ax[0].set_title(f"circular tracking ({a6['period_s']:.1f} s/rev, "
                    f"r={a6['radius_m'] * 100:.0f} cm)")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)

    ex = tr[:, 1] - tr[:, 3]
    ez = tr[:, 2] - tr[:, 4]
    ax[1].plot(tr[:, 0], np.hypot(ex, ez) * 1000, color="#e45756")
    ax[1].set_xlabel("t (s)")
    ax[1].set_ylabel("endpoint error (mm)")
    ax[1].set_title(f"mean {a6['mean_err_mm']:.1f} mm, max {a6['max_err_mm']:.1f} mm")
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    p = os.path.join(outdir, "a6_tracking_error.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "out/metrics.json"
    outdir = sys.argv[2] if len(sys.argv) > 2 else (os.path.dirname(path) or ".")
    os.makedirs(outdir, exist_ok=True)
    with open(path, encoding="utf-8") as f:
        d = json.load(f)

    plotters = [("a3", plot_a3), ("a4", plot_a4), ("a5", plot_a5), ("a6", plot_a6)]
    for key, fn in plotters:
        if key not in d:
            print(f"skip {key}（metrics.json 里没有）")
            continue
        print(fn(d, outdir))


if __name__ == "__main__":
    main()
