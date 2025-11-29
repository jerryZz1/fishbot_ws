import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'turtle_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 关键：用绝对路径 glob
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join(os.path.dirname(__file__), 'launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bobo',
    maintainer_email='1607266788@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_joy_node = turtle_joy.turtle_joy_node:main',
        ],
    },
)