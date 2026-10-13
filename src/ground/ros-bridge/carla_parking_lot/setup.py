from setuptools import setup

package_name = 'carla_parking_lot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='CARLA Town04 停车场场景生成',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = carla_parking_lot.main:main',
        ],
    },
)
