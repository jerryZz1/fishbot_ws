from setuptools import setup

package_name = 'fishbot_limit_switch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='862524030@qq.com',
    description='Limit switch publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'limit_switch_node = fishbot_limit_switch.limit_switch_node:main'
        ],
    },
)
