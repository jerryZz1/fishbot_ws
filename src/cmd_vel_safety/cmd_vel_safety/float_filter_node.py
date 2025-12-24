#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class FloatSafetyFilter(Node):
    """
    通用 Float32 安全过滤器（可多实例并行）：
    - 订阅 in_topic(Float32)，发布 out_topic(Float32)
    - 超时置零、急停置零
    - 限幅
    - 一阶 slew-rate（相当于加速度限制）
    - 定时发布
    """

    def __init__(self):
        super().__init__("float_safety_filter")

        self.declare_parameter("in_topic", "/dc_motor_cmd_raw")
        self.declare_parameter("out_topic", "/dc_motor_cmd")
        self.declare_parameter("estop_topic", "/estop")

        self.declare_parameter("cmd_timeout_sec", 0.25)
        self.declare_parameter("publish_hz", 50.0)

        self.declare_parameter("x_min", -1.0)
        self.declare_parameter("x_max",  1.0)
        self.declare_parameter("a_x_max", 4.0)  # 单位：每秒最大变化量（slew-rate）

        self.in_topic = str(self.get_parameter("in_topic").value)
        self.out_topic = str(self.get_parameter("out_topic").value)
        self.estop_topic = str(self.get_parameter("estop_topic").value)

        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.pub_period = 1.0 / self.publish_hz if self.publish_hz > 0 else 0.02

        self.x_min = float(self.get_parameter("x_min").value)
        self.x_max = float(self.get_parameter("x_max").value)
        self.a_x_max = float(self.get_parameter("a_x_max").value)

        self.estop = False
        self.last_in_time = None

        self.tgt_x = 0.0
        self.out_x = 0.0
        self.last_update_time = self.get_clock().now()

        self.sub_cmd = self.create_subscription(Float32, self.in_topic, self.on_cmd, 10)
        self.sub_estop = self.create_subscription(Bool, self.estop_topic, self.on_estop, 10)
        self.pub_cmd = self.create_publisher(Float32, self.out_topic, 10)
        self.timer = self.create_timer(self.pub_period, self.on_timer)

        self.get_logger().info(
            f"[FloatFilter] in={self.in_topic} out={self.out_topic} "
            f"timeout={self.cmd_timeout}s pub={self.publish_hz}Hz estop={self.estop_topic}"
        )

    def on_estop(self, msg: Bool):
        self.estop = bool(msg.data)

    def on_cmd(self, msg: Float32):
        now = self.get_clock().now()
        self.last_in_time = now

        x = float(msg.data)
        x = clamp(x, self.x_min, self.x_max)
        self.tgt_x = x

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = self.pub_period
        self.last_update_time = now

        timed_out = True
        if self.last_in_time is not None:
            age = (now - self.last_in_time).nanoseconds * 1e-9
            timed_out = (age > self.cmd_timeout)

        if self.estop or timed_out:
            tgt_x = 0.0
        else:
            tgt_x = self.tgt_x

        dx = tgt_x - self.out_x
        max_dx = self.a_x_max * dt
        dx = clamp(dx, -max_dx, max_dx)
        self.out_x += dx

        out = Float32()
        out.data = float(self.out_x)
        self.pub_cmd.publish(out)

def main():
    rclpy.init()
    node = FloatSafetyFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
