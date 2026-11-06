# MuJoCo 肌肉驱动的手臂刚度与轨迹跟踪

用 **Hill 肌肉**驱动一个肩-肘两关节平面臂，重点研究**共激活如何改变末端刚度**，
以及双关节肌在精度与能耗之间的权衡。

仓库已有 `msk_arm_reach`（ROS 闭环到达，6 条肌肉），本模块不重复它的工作：
我们用 Python 实现肌肉模型、不带 ROS 节点，专注**定量分析**——刚度椭圆、
能耗代价、重力前馈必要性、圆轨迹精度。

## 运行环境

| 依赖 | 版本 |
|---|---|
| Python | 3.8+（实测 3.10 / 3.14） |
| MuJoCo | 3.0+（实测 3.16.0） |
| numpy | 1.20+ |
| Pillow | 仅 `gif` 需要 |
| matplotlib | 仅出图需要 |

不依赖 ROS 运行；`launch/` 与 `package.xml` 只是与其他模块统一的包装。

## 运行

```bash
python3 main.py check                 # 模型 + IK 往返自检
python3 main.py run --scene a5        # 单个实验
python3 main.py bench --out out       # 六组实验，导出 out/metrics.json
python3 main.py gif --out gif         # 渲染 4 段动图
python3 tools/plot_metrics.py out/metrics.json out   # 4 张对比图
python3 -m unittest discover -s tests # 18 项测试
```

## 六组实验

| 级别 | 内容 | 结果 |
|---|---|---|
| A1 | 单条肩屈肌阶跃（模型基线） | 峰值肩角 0.913 rad，峰值肌力 130 N |
| A2 | 四条单关节肌 + PD 到达三目标（基线） | 平均端点误差 11.1 mm |
| A3 | 双关节肱二头肌 vs 单关节肌 | 误差 +27%，激活代价 **−38%** |
| A4 | 去掉重力前馈 | 误差 11.1 → **44.4 mm（4 倍）** |
| A5 | 共激活调末端刚度 | 主刚度 23.7 → **56.6 N/m（2.4 倍）**，各向异性 36.5 → 16.9 |
| A6 | 圆轨迹跟踪（r=8 cm，2 s/圈） | 稳态误差 **10.7 mm** |

核心结论：

1. **共激活能在同一姿态下改变末端刚度**，且刚度高度各向异性；
2. **双关节肌是"省力但不精确"的折中**；
3. **重力前馈不可省**，误差放大 4 倍。

## 文件结构

```
mujoco_muscle_arm_stiffness/
├── main.py                 # check / run / bench / gif
├── package.xml / setup.py  # ament_python 包装（可选）
├── launch/main.launch.py
├── resource/
├── mujoco_muscle_arm_stiffness/
│   ├── core.py             # Hill 肌肉、MJCF、IK/FK、分配、QP
│   ├── runs.py             # A1-A4 与录制
│   ├── extras.py           # A5 端点刚度、A6 圆轨迹
│   └── cli.py
├── tests/                  # 18 项单元/集成测试
└── tools/plot_metrics.py   # 4 张对比图
```

## 已知限制

1. **平面矢状面**：只有肩屈伸与肘屈伸，未建模肩外展/旋转与腕关节。
2. **解析矩臂**：真实肌肉力臂随构型变化，这里用常数矩臂简化。
3. **参数为缩放模型**：上臂 0.25 m、前臂 0.25 m，质量 2.0/1.2 kg，不对应具体人体。
4. **A6 是关节空间 PD 跟踪逆解轨迹**，未做阻抗/自适应控制。
5. **不含 ROS 节点**：不接受外部目标话题，ROS 闭环见 `msk_arm_reach`。
6. 未做 sim2real。

## 参考

- A. V. Hill, *The heat of shortening and the dynamic constants of muscle*, 1938
- F. E. Zajac, *Muscle and tendon*, 1989
