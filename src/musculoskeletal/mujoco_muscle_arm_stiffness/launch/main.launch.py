"""ROS 2 launch：按场景启动一次实验。

    ros2 launch mujoco_muscle_arm_stiffness main.launch.py scene:=a2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _run(context):
    scene = LaunchConfiguration("scene").perform(context)
    share = get_package_share_directory("mujoco_muscle_arm_stiffness")
    return [
        ExecuteProcess(
            cmd=["python3", os.path.join(share, "main.py"), "run", "--scene", scene],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="a2",
                              description="实验编号：a1 / a2 / a3 / a4"),
        OpaqueFunction(function=_run),
    ])
