from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 0) 底盘安全监督器（超声波融合，输出 /cmd_vel_safe）
    supervisor = Node(
        package='cmd_vel_safety',
        executable='safety_supervisor_node',
        name='safety_supervisor',
        output='screen',
        parameters=[{
            'in_cmd': '/cmd_vel_raw',
            'out_cmd': '/cmd_vel_safe',
            'estop_topic': '/estop',

            # 默认：s1=front, s2=left, s3=right
            'front_topic': '/ultrasonic/s1',
            'left_topic':  '/ultrasonic/s2',
            'right_topic': '/ultrasonic/s3',

            'publish_hz': 50.0,
            'sensor_timeout_sec': 0.30,
            'slow_dist_m': 0.60,
            'stop_dist_m': 0.30,

            'side_slow_dist_m': 0.35,
            'side_stop_dist_m': 0.20,

            'ultra_fail_policy': 'ignore',
        }]
    )

    # 1) 限位解复用：/limit_switch -> /axis/limit_*
    demux = Node(
        package='cmd_vel_safety',
        executable='limit_switch_demux_node',
        name='limit_switch_demux',
        output='screen',
        parameters=[{
            'in_topic': '/limit_switch',
            'idx_z_max': 0,
            'idx_z_min': 2,
            'idx_y_max': 3,
            'idx_y_min': 5,
            'active_high': True,
        }]
    )

    # 2) 底盘 Twist 最终安全层：/cmd_vel_safe -> /cmd_vel
    chassis = Node(
        package='cmd_vel_safety',
        executable='twist_filter_node',
        name='chassis_safety',
        output='screen',
        parameters=[{
            'in_topic':  '/cmd_vel_safe',
            'out_topic': '/cmd_vel',
            'estop_topic': '/estop',

            'cmd_timeout_sec': 0.25,
            'publish_hz': 50.0,

            'v_max': 0.30,
            'w_max': 1.20,
            'a_v_max': 0.80,
            'a_w_max': 2.50,
        }]
    )

    # 3) 轴 Axis 专用安全层（带限位）：/cmd_vel_axis_raw -> /cmd_vel_axis
    axis = Node(
        package='cmd_vel_safety',
        executable='axis_filter_node',
        name='axis_safety',
        output='screen',
        parameters=[{
            'in_topic':  '/cmd_vel_axis_raw',
            'out_topic': '/cmd_vel_axis',
            'estop_topic': '/estop',

            'cmd_timeout_sec': 0.25,
            'publish_hz': 50.0,

            'y_vel_max': 0.20,
            'z_vel_max': 0.20,
            'y_acc_max': 0.50,
            'z_acc_max': 0.50,

            'y_min_topic': '/axis/limit_y_min',
            'y_max_topic': '/axis/limit_y_max',
            'z_min_topic': '/axis/limit_z_min',
            'z_max_topic': '/axis/limit_z_max',

            'y_min_active_high': True,
            'y_max_active_high': True,
            'z_min_active_high': True,
            'z_max_active_high': True,

            'latch_limit': False,
        }]
    )

    # 4) DC 联锁（只做 estop=0），输出 /dc_motor_cmd_raw_ilk
    dc_interlock = Node(
        package='cmd_vel_safety',
        executable='dc_interlock_node',
        name='dc_interlock',
        output='screen',
        parameters=[{
            'dc_in_topic': '/dc_motor_cmd_raw',
            'dc_out_topic': '/dc_motor_cmd_raw_ilk',
            'estop_topic': '/estop',

            'x_min': -1.0,
            'x_max':  1.0,

            'publish_zero_on_estop_edge': True,
        }]
    )

    # 5) DC Float32 安全层：/dc_motor_cmd_raw_ilk -> /dc_motor_cmd
    dc = Node(
        package='cmd_vel_safety',
        executable='float_filter_node',
        name='dc_safety',
        output='screen',
        parameters=[{
            'in_topic':  '/dc_motor_cmd_raw_ilk',
            'out_topic': '/dc_motor_cmd',
            'estop_topic': '/estop',

            'cmd_timeout_sec': 0.25,
            'publish_hz': 50.0,

            'x_min': -1.0,
            'x_max':  1.0,
            'a_x_max': 4.0,
        }]
    )

    return LaunchDescription([
        supervisor,
        demux,
        chassis,
        axis,
        dc_interlock,
        dc,
    ])
