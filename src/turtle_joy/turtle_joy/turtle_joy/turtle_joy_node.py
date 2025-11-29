#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class JoyToCar(Node):
    def __init__(self):
        super().__init__('joy_to_car')

        # ------------ 底盘参数（左摇杆） ------------
        self.declare_parameter('axis_linear', 1)        # 左摇杆前后
        self.declare_parameter('axis_angular', 0)       # 左摇杆左右
        self.declare_parameter('scale_linear', 0.08)
        self.declare_parameter('scale_angular', 1.2)
        self.declare_parameter('deadzone', 0.05)

        # ------------ 导轨参数（右摇杆，始终生效） ------------
        # 你自己调好的轴号：
        self.declare_parameter('axis_flap_lr', 2)       # 右摇杆左右 → Y 轴导轨 (tgt_y)
        self.declare_parameter('axis_flap_ud', 3)       # 右摇杆上下 → Z 轴导轨 (tgt_z)
        self.declare_parameter('scale_flap_lr', 0.08)
        self.declare_parameter('scale_flap_ud', 0.08)
        self.declare_parameter('deadzone_flap', 0.05)

        # ------------ 直流电机参数（扳机） ------------
        # 已知：一个扳机是 axis 4
        self.declare_parameter('axis_dc_trigger', 4)    # 扳机轴
        self.declare_parameter('scale_dc', 0.5)         # 最大 0.7（适中启动/最高速度）
        self.declare_parameter('deadzone_dc', 0.1)      # 扳机死区

        # ------------ ROS 接口 ------------
        self.sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)

        # 底盘差速控制
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        # 导轨控制给 ESP32：cmd_vel_axis
        self.pub_axis = self.create_publisher(Twist, 'cmd_vel_axis', 10)
        # 直流电机控制给 ESP32：dc_motor_cmd（std_msgs/Float32）
        self.pub_dc = self.create_publisher(Float32, 'dc_motor_cmd', 10)

        self.get_logger().info(
            "Joy → /cmd_vel + /cmd_vel_axis + /dc_motor_cmd 已启动"
        )

    def joy_cb(self, joy: Joy):
        # ==================== 1. 底盘控制（左摇杆） ====================
        ax_l = self.get_parameter('axis_linear').value
        ax_a = self.get_parameter('axis_angular').value
        sc_l = self.get_parameter('scale_linear').value
        sc_a = self.get_parameter('scale_angular').value
        dz = self.get_parameter('deadzone').value

        twist = Twist()

        # 线速度
        if 0 <= ax_l < len(joy.axes):
            v = joy.axes[ax_l] * sc_l
            twist.linear.x = 0.0 if abs(v) < dz else v
        else:
            twist.linear.x = 0.0

        # 角速度
        if 0 <= ax_a < len(joy.axes):
            w = joy.axes[ax_a] * sc_a
            twist.angular.z = 0.0 if abs(w) < dz else w
        else:
            twist.angular.z = 0.0

        self.pub_cmd_vel.publish(twist)

        # ==================== 2. 导轨控制（右摇杆） ====================
        ax_lr = self.get_parameter('axis_flap_lr').value
        ax_ud = self.get_parameter('axis_flap_ud').value
        sc_lr = self.get_parameter('scale_flap_lr').value
        sc_ud = self.get_parameter('scale_flap_ud').value
        dz_f = self.get_parameter('deadzone_flap').value

        axis_twist = Twist()

        # Y 轴导轨（左右）
        v_lr = joy.axes[ax_lr] * sc_lr if 0 <= ax_lr < len(joy.axes) else 0.0
        axis_twist.linear.x = 0.0 if abs(v_lr) < dz_f else v_lr

        # Z 轴导轨（上下）
        v_ud = joy.axes[ax_ud] * sc_ud if 0 <= ax_ud < len(joy.axes) else 0.0
        axis_twist.linear.y = 0.0 if abs(v_ud) < dz_f else v_ud

        self.pub_axis.publish(axis_twist)

        # ==================== 3. 直流电机控制（扳机 axis 4） ====================
        ax_dc = self.get_parameter('axis_dc_trigger').value
        sc_dc = self.get_parameter('scale_dc').value      # 最大速度（0~1 里取 0.7）
        dz_dc = self.get_parameter('deadzone_dc').value   # 扳机死区

        cmd = Float32()
        dc_val = 0.0

        if 0 <= ax_dc < len(joy.axes):
            raw = joy.axes[ax_dc]

            # 很多手柄扳机是：松开 ≈ 1.0，捏到底 ≈ -1.0
            # 我们统一映射到 [0, 1]：t = (1 - raw) / 2
            # raw =  1  -> t = 0   （松开）
            # raw = -1  -> t = 1   （捏到底）
            t = (1.0 - raw) / 2.0

            # 死区处理：小于 deadzone_dc 时认为 0
            if t < dz_dc:
                dc_val = 0.0
            else:
                # 线性映射到 [0, sc_dc]，比如 [0, 0.7]
                dc_val = sc_dc * t
        else:
            dc_val = 0.0

        cmd.data = float(dc_val)
        self.pub_dc.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToCar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
