from setuptools import setup
import os
from glob import glob

# sun: ROS 2 Python 包安装描述：注册 quadsim_node 控制台入口，并安装 launch、参数和 MJCF 资源。

package_name = 'quadsim_mujoco'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'mjcf'), glob(os.path.join('mjcf', '*.*'))),
        (os.path.join('share', package_name, 'mjcf/assets'), glob(os.path.join('mjcf/assets', '*.stl'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lyt',
    maintainer_email='lyt@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quadsim_node = quadsim_mujoco.quadsim_node:main',
            'quad = quadsim_mujoco.quad:main',
            'dryden_wind_field = quadsim_mujoco.dryden_wind_field:main',
        ],
    },
)
