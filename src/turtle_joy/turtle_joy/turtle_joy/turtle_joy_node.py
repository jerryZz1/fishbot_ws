#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class JoyToCar(Node):
    def __init__(self):
        super().__init__('joy_to_car')

        # ------------ 底盘参数（左摇杆，简化为固定速度的前/后/左/右） ------------
        self.declare_parameter('axis_linear', 1)        # 左摇杆前后（ly）
        self.declare_parameter('axis_angular', 0)       # 左摇杆左右（lx）

        # 固定速度，而不是按幅度比例
        self.declare_parameter('fixed_linear_speed', 0.20)   # 前进/后退固定速度 m/s
        self.declare_parameter('fixed_angular_speed', 0.6)   # 左右转固定角速度 rad/s

        # 死区：摇杆绝对值小于这个就当作 0（防止抖动）
        self.declare_parameter('deadzone', 0.15)

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
        self.declare_parameter('scale_dc', 0.5)         # 最大约 0.5（对应 ESP32 端 -1~1 的幅度）
        self.declare_parameter('deadzone_dc', 0.1)      # 扳机死区

        # ------------ 振动电机参数（Button 7 启停，固定 40%） ------------
        self.declare_parameter('btn_vib_toggle', 7)     # 按钮编号（你说用 button7）
        self.declare_parameter('vib_default_speed', 40.0)  # 振动电机默认速度：40%

        # 状态：当前振动电机是否认为“开启”
        self.vib_on = False
        # 上一次 button7 状态，用于边沿检测
        self.last_vib_btn_state = False

        # ------------ ROS 接口 ------------
        self.sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)

        # 底盘差速控制
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        # 导轨控制给 ESP32：cmd_vel_axis
        self.pub_axis = self.create_publisher(Twist, 'cmd_vel_axis', 10)
        # 直流电机控制给 ESP32：dc_motor_cmd（std_msgs/Float32）
        self.pub_dc = self.create_publisher(Float32, 'dc_motor_cmd', 10)
        # 振动电机控制给 Modbus 驱动节点：vib_motor_cmd（std_msgs/Float32，单位 %）
        self.pub_vib = self.create_publisher(Float32, 'vib_motor_cmd', 10)

        self.get_logger().info(
            "Joy → /cmd_vel(简化四向) + /cmd_vel_axis + /dc_motor_cmd + /vib_motor_cmd 已启动"
        )

    def joy_cb(self, joy: Joy):
        # ==================== 1. 底盘控制（左摇杆，简化四向） ====================
        ax_l = self.get_parameter('axis_linear').value      # 前后（ly）
        ax_a = self.get_parameter('axis_angular').value     # 左右（lx）
        v_fixed = self.get_parameter('fixed_linear_speed').value
        w_fixed = self.get_parameter('fixed_angular_speed').value
        dz = self.get_parameter('deadzone').value

        twist = Twist()

        # 读取左摇杆当前值
        ly = joy.axes[ax_l] if 0 <= ax_l < len(joy.axes) else 0.0
        lx = joy.axes[ax_a] if 0 <= ax_a < len(joy.axes) else 0.0

        # 应用死区
        ly = 0.0 if abs(ly) < dz else ly
        lx = 0.0 if abs(lx) < dz else lx

        if ly == 0.0 and lx == 0.0:
            # 摇杆在死区内：停止
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            # 根据哪个方向更大，判断是“前后”还是“转向”
            if abs(ly) >= abs(lx):
                # 前后优先：只前/后，不转向
                twist.angular.z = 0.0
                if ly > 0:
                    twist.linear.x = v_fixed     # 前进
                else:
                    twist.linear.x = -v_fixed    # 后退
            else:
                # 左右优先：原地转向
                twist.linear.x = 0.0
                if lx > 0:
                    # 一般约定：z < 0 为右转
                    twist.angular.z = -w_fixed   # 右转
                else:
                    twist.angular.z = w_fixed    # 左转

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
        sc_dc = self.get_parameter('scale_dc').value      # 最大速度（0~1 里取 0.5）
        dz_dc = self.get_parameter('deadzone_dc').value   # 扳机死区

        cmd_dc = Float32()
        dc_val = 0.0

        if 0 <= ax_dc < len(joy.axes):
            raw = joy.axes[ax_dc]

            # 很多手柄扳机是：松开 ≈ 1.0，捏到底 ≈ -1.0
            # 统一映射到 [0, 1]：t = (1 - raw) / 2
            t = (1.0 - raw) / 2.0

            # 死区处理
            if t < dz_dc:
                dc_val = 0.0
            else:
                dc_val = sc_dc * t
        else:
            dc_val = 0.0

        cmd_dc.data = float(dc_val)
        self.pub_dc.publish(cmd_dc)

        # ==================== 4. 振动电机控制（Button 7 启停，40%） ====================
        btn_vib = self.get_parameter('btn_vib_toggle').value
        vib_speed = self.get_parameter('vib_default_speed').value  # 例如 40.0 (%)

        current_btn_state = (
            len(joy.buttons) > btn_vib and joy.buttons[btn_vib] == 1
        )

        # 上升沿检测：从未按下 → 按下
        if current_btn_state and not self.last_vib_btn_state:
            # 切换状态
            self.vib_on = not self.vib_on

            msg_vib = Float32()
            if self.vib_on:
                # 开启：40%
                msg_vib.data = float(vib_speed)
                self.get_logger().info(f"振动电机：ON，速度 = {vib_speed}%")
            else:
                # 关闭：0%
                msg_vib.data = 0.0
                self.get_logger().info("振动电机：OFF")

            self.pub_vib.publish(msg_vib)

        # 更新上一次按钮状态
        self.last_vib_btn_state = current_btn_state


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
