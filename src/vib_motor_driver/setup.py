from setuptools import setup

package_name = 'vib_motor_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymodbus'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='you@example.com',
    description='ROS2 driver for vibration motor via Modbus RTU',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vib_motor_node = vib_motor_driver.vib_motor_node:main',
        ],
    },
)
