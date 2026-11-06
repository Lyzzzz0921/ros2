import os
from glob import glob

from setuptools import setup

package_name = "mujoco_muscle_arm_stiffness"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "main.py", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenHUTB",
    maintainer_email="openhutb@example.com",
    description="肌肉骨骼上肢（肩-肘）仿真",
    license="MIT",
    entry_points={
        "console_scripts": [
            "muscle_arm_stiffness = mujoco_muscle_arm_stiffness.cli:main",
        ],
    },
)
