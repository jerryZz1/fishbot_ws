from setuptools import setup

package_name = 'ultrasonic_rs485'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='RS485 ultrasonic sensor driver (Modbus RTU)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_rs485_node = ultrasonic_rs485.ultrasonic_rs485_node:main',
        ],
    },
)
