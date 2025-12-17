from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ========== 1. 手柄硬件 → /joy ==========
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',   # 默认手柄设备
                'deadzone': 0.05,
                'autorepeat_rate': 0.0,
            }]
        ),

        # ========== 2. /joy → /cmd_vel, /cmd_vel_axis, /dc_motor_cmd, /vib_motor_cmd ==========
        Node(
            package='turtle_joy',
            executable='turtle_joy_node',
            name='turtle_joy',
            output='screen'
            # 如果以后要覆盖参数，也可以加 parameters=[{...}]
        ),

        # ========== 3. micro-ROS Agent (串口) ==========
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200']
        ),

        # ========== 4. 振动电机 Modbus 驱动 ==========
        Node(
            package='vib_motor_driver',
            executable='vib_motor_node',
            name='vib_motor_driver',
            output='screen'
        ),

        # 后面你还可以继续加：超声节点 / 限位开关 / URDF / TF 等
    ])
