# CARLA 停车场场景生成

## 概述
在 CARLA 0.9.16 仿真器的 Town04 地图中，通过 Python API 自动生成一个带车位线的停车场场景，包含 6 个垂直停车位，并自动将镜头移动到停车场上方俯视。

## 环境要求
- CARLA 0.9.16（服务端运行于 Windows 有 GPU 的宿主机）
- Ubuntu 22.04 + Python 3.10（运行客户端脚本）
- Python 依赖：`carla==0.9.16`

## 安装步骤

### 1. 下载安装 Windows 版 CARLA
从官方 Release 页面下载 CARLA 0.9.16 Windows 版：
https://github.com/carla-simulator/carla/releases/tag/0.9.16

下载 `CARLA_0.9.16.zip`，解压到 `D:\carla` 目录。

### 2. 启动 CARLA 服务端
```powershell
D:\carla\CarlaUE4.exe -quality-level=Low

