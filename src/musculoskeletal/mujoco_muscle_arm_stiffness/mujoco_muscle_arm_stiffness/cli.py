"""命令行入口：check / run / bench / gif。"""

import argparse
import json
import os

import mujoco

from . import core, extras, runs


def cmd_check(_args):
    print(f"mujoco {mujoco.__version__}")
    m = mujoco.MjModel.from_xml_string(core.arm_xml())
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mujoco.mj_step(m, d)
    print(f"  模型 nq={m.nq}（肩 + 肘），末端位置 "
          f"({d.site('ee').xpos[0]:+.3f}, {d.site('ee').xpos[2]:+.3f}) OK")
    q1, q2 = core.ik_target(0.30, -0.30)
    x, z = core.forward_kinematics(q1, q2)
    print(f"  IK 往返：目标 (0.30, -0.30) -> 关节 ({q1:+.3f}, {q2:+.3f}) -> "
          f"FK ({x:+.3f}, {z:+.3f})")


def cmd_run(args):
    print(json.dumps(SCENES[args.scene](), ensure_ascii=False, indent=2))


def cmd_bench(args):
    os.makedirs(args.out, exist_ok=True)
    res = {name: SCENES[name]() for name in ("a1", "a2", "a3", "a4", "a5", "a6")}
    path = os.path.join(args.out, "metrics.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    print(f"指标已写入 {path}")
    print(f"  A5 主刚度 {res['a5']['levels'][0]['k_major']:.1f} -> "
          f"{res['a5']['levels'][-1]['k_major']:.1f} N/m")
    print(f"  A6 稳态误差 {res['a6']['mean_err_mm']:.2f} mm")


def cmd_gif(args):
    os.makedirs(args.out, exist_ok=True)
    jobs = [
        ("a1_single_muscle.gif", runs.run_a1, dict(every=15), 0.62),
        ("a2_reach.gif", runs.run_a2, dict(every=20), 0.62),
        ("a3_biceps_reach.gif", runs.run_a3_demo, dict(every=20), 0.62),
        ("a6_circle_tracking.gif", extras.run_a6, dict(every=20), 0.55),
    ]
    for name, fn, kw, scale in jobs:
        rec = runs.Recorder(**kw)
        fn(rec=rec)
        size = rec.save(os.path.join(args.out, name), scale=scale)
        print(f"  {name}  {len(rec.frames)} 帧, {size / 1e6:.2f} MB")


SCENES = {"a1": runs.run_a1, "a2": runs.run_a2, "a3": runs.run_a3,
          "a4": runs.run_a4, "a5": extras.run_a5, "a6": extras.run_a6}


def main(argv=None):
    ap = argparse.ArgumentParser(description="肌肉骨骼上肢（肩-肘）模块")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("check", help="模型自检").set_defaults(func=cmd_check)

    p = sub.add_parser("run", help="运行单个实验")
    p.add_argument("--scene", required=True, choices=sorted(SCENES))
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("bench", help="运行全部实验")
    p.add_argument("--out", default="out")
    p.set_defaults(func=cmd_bench)

    p = sub.add_parser("gif", help="渲染动图")
    p.add_argument("--out", default="gif")
    p.set_defaults(func=cmd_gif)

    args = ap.parse_args(argv)
    args.func(args)


