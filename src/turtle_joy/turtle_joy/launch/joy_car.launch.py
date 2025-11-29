from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 手柄驱动
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'),

        # 2. 我们自己写的 joy→cmd_vel 节点
        Node(
            package='turtle_joy',        # 你的包名
            executable='turtle_joy_node', # 对应 setup.py 里注册的入口
            name='joy_to_car',
            output='screen',
            parameters=[{
                'axis_linear': 1,      # 左摇杆上下
                'axis_angular': 0,     # 左摇杆左右
                'scale_linear': 0.08,   # m/s
                'scale_angular': 1.2,  # rad/s
                'deadzone': 0.05
            }]),
    ])