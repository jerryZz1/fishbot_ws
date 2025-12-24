from setuptools import setup
import os
from glob import glob

package_name = 'cmd_vel_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@todo.todo',
    description='Safety filters for command topics',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_filter_node = cmd_vel_safety.twist_filter_node:main',
            'float_filter_node = cmd_vel_safety.float_filter_node:main',
            'limit_switch_demux_node = cmd_vel_safety.limit_switch_demux_node:main',
            'axis_filter_node = cmd_vel_safety.axis_filter_node:main',
            'safety_supervisor_node = cmd_vel_safety.safety_supervisor_node:main',
            'dc_interlock_node = cmd_vel_safety.dc_interlock_node:main',
        ],
    },
)
