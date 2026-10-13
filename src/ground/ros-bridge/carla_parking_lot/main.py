#!/usr/bin/env python3
"""
CARLA 停车场场景生成
在 Town04 地图中生成带车位线的停车场，并自动将镜头移动到俯视位置。
"""

import carla
import time
import argparse


def main():
    parser = argparse.ArgumentParser(description='CARLA 停车场场景生成')
    parser.add_argument('--host', default='192.168.43.1', help='CARLA 服务器 IP')
    parser.add_argument('--port', default=2000, type=int, help='CARLA 服务器端口')
    args = parser.parse_args()

    # 连接 CARLA 服务器
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)

    # 切换到 Town04 地图
    print("切换到 Town04 ...")
    world = client.load_world('Town04')
    time.sleep(2)

    # 停车场参数
    cx = 280.1       # 停车场中心 x 坐标
    cy = -223.7      # 停车场中心 y 坐标
    cz = 0.3         # 地面高度
    slot_width = 2.8  # 车位宽度（米）
    slot_length = 5.5 # 车位长度（米）
    num_slots = 6    # 车位数量

    # 清空旧车辆
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()
    for actor in world.get_actors().filter('sensor.*'):
        actor.destroy()
    print("已清空旧车辆")

    # 绘制车位线
    print("正在画车位线...")
    for i in range(num_slots):
        sy = cy + i * slot_width

        # 车位四个角点
        p1 = carla.Location(cx - slot_length/2, sy - slot_width/2, cz)
        p2 = carla.Location(cx + slot_length/2, sy - slot_width/2, cz)
        p3 = carla.Location(cx + slot_length/2, sy + slot_width/2, cz)
        p4 = carla.Location(cx - slot_length/2, sy + slot_width/2, cz)

        # 画绿色车位线
        world.debug.draw_line(p1, p2, thickness=0.1, color=carla.Color(0, 255, 0), life_time=0)
        world.debug.draw_line(p2, p3, thickness=0.1, color=carla.Color(0, 255, 0), life_time=0)
        world.debug.draw_line(p3, p4, thickness=0.1, color=carla.Color(0, 255, 0), life_time=0)
        world.debug.draw_line(p4, p1, thickness=0.1, color=carla.Color(0, 255, 0), life_time=0)

        # 标注车位编号
        world.debug.draw_string(
            carla.Location(cx, sy, cz + 0.5),
            f"P{i+1}",
            color=carla.Color(255, 255, 0),
            life_time=0
        )

    print(f"已画 {num_slots} 个车位")

    # 自动将镜头移动到停车场上方俯视
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(
        carla.Location(x=cx, y=cy, z=15),
        carla.Rotation(pitch=-90, yaw=0)
    ))
    print("镜头已移到停车场上方俯视")
    print("\n✅ 停车场场景生成完成！")


if __name__ == '__main__':
    main()
