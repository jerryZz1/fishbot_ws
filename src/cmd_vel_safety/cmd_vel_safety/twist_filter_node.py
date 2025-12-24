#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class TwistSafetyFilter(Node):
    """
    通用 Twist 安全过滤器（可多实例并行）：
    - 订阅 in_topic(Twist)，发布 out_topic(Twist)
    - 超时置零、急停置零
    - v/w 限幅
    - dv/dt, dw/dt 加速度限制（slew-rate）
    - 定时发布（publish_hz），确保下游（ESP32）持续收到心跳式 cmd_vel
    """

    def __init__(self):
        super().__init__("twist_safety_filter")

        # -------- params (fully parameterized) --------
        self.declare_parameter("in_topic", "/cmd_vel_raw")
        self.declare_parameter("out_topic", "/cmd_vel")
        self.declare_parameter("estop_topic", "/estop")

        self.declare_parameter("cmd_timeout_sec", 0.25)
        self.declare_parameter("publish_hz", 50.0)

        self.declare_parameter("v_max", 0.30)       # m/s
        self.declare_parameter("w_max", 1.20)       # rad/s
        self.declare_parameter("a_v_max", 0.80)     # m/s^2
        self.declare_parameter("a_w_max", 2.50)     # rad/s^2

        self.in_topic = str(self.get_parameter("in_topic").value)
        self.out_topic = str(self.get_parameter("out_topic").value)
        self.estop_topic = str(self.get_parameter("estop_topic").value)

        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.pub_period = 1.0 / self.publish_hz if self.publish_hz > 0 else 0.02

        self.v_max = float(self.get_parameter("v_max").value)
        self.w_max = float(self.get_parameter("w_max").value)
        self.a_v_max = float(self.get_parameter("a_v_max").value)
        self.a_w_max = float(self.get_parameter("a_w_max").value)

        # -------- state --------
        self.estop = False
        self.last_in_time = None

        self.tgt_v = 0.0
        self.tgt_w = 0.0

        self.out_v = 0.0
        self.out_w = 0.0
        self.last_update_time = self.get_clock().now()

        # -------- ROS I/O --------
        self.sub_cmd = self.create_subscription(Twist, self.in_topic, self.on_cmd, 10)
        self.sub_estop = self.create_subscription(Bool, self.estop_topic, self.on_estop, 10)
        self.pub_cmd = self.create_publisher(Twist, self.out_topic, 10)

        self.timer = self.create_timer(self.pub_period, self.on_timer)

        self.get_logger().info(
            f"[TwistFilter] in={self.in_topic} out={self.out_topic} "
            f"timeout={self.cmd_timeout}s pub={self.publish_hz}Hz estop={self.estop_topic}"
        )

    def on_estop(self, msg: Bool):
        self.estop = bool(msg.data)

    def on_cmd(self, msg: Twist):
        now = self.get_clock().now()
        self.last_in_time = now

        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # 限幅（安全层第一道）
        v = clamp(v, -self.v_max, self.v_max)
        w = clamp(w, -self.w_max, self.w_max)

        self.tgt_v = v
        self.tgt_w = w

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = self.pub_period
        self.last_update_time = now

        # 输入超时判断
        timed_out = True
        if self.last_in_time is not None:
            age = (now - self.last_in_time).nanoseconds * 1e-9
            timed_out = (age > self.cmd_timeout)

        if self.estop or timed_out:
            tgt_v = 0.0
            tgt_w = 0.0
        else:
            tgt_v = self.tgt_v
            tgt_w = self.tgt_w

        # slew-rate 限制（加速度限制）
        dv = tgt_v - self.out_v
        dw = tgt_w - self.out_w

        max_dv = self.a_v_max * dt
        max_dw = self.a_w_max * dt

        dv = clamp(dv, -max_dv, max_dv)
        dw = clamp(dw, -max_dw, max_dw)

        self.out_v += dv
        self.out_w += dw

        out = Twist()
        out.linear.x = self.out_v
        out.angular.z = self.out_w
        self.pub_cmd.publish(out)

def main():
    rclpy.init()
    node = TwistSafetyFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
