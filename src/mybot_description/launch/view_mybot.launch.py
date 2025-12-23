import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    pkg = get_package_share_directory('mybot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'mybot.urdf.xacro')
    rviz_cfg = os.path.join(pkg, 'rviz', 'mybot.rviz')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', xacro_file])}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
    )

    return LaunchDescription([rsp, rviz])
