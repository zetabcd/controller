from glob import glob
from setuptools import find_packages, setup

setup(
    name='flight_data_recorder',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/flight_data_recorder']),
        ('share/flight_data_recorder', ['package.xml', 'README.md']),
        ('share/flight_data_recorder/config', glob('config/*.yaml')),
        ('share/flight_data_recorder/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orion',
    maintainer_email='orion@todo.todo',
    description='ROS 2 numeric flight-data ULog recorder',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'flight_data_recorder_node = flight_data_recorder.node:main',
        'recover_ulog = flight_data_recorder.ulog:recover_main',
    ]},
)
