#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

class DcInterlock(Node):
    """
    DC 联锁节点（用于蛟龙螺旋推进）：
    - 输入：dc_in_topic (Float32, raw)
    - 输入：estop_topic (Bool)
    - 输出：dc_out_topic (Float32, raw after interlock)
    规则：
      - estop=True -> 输出强制为 0
      - estop=False -> 透传（可选限幅）
    说明：
      - 本节点不做平滑、不做超时；这些由下游 float_filter_node 负责
      - 该节点的意义是把“联动规则”从 float_filter 中解耦出来，便于扩展
    """

    def __init__(self):
        super().__init__("dc_interlock")

        # ---- params ----
        self.declare_parameter("dc_in_topic", "/dc_motor_cmd_raw")
        self.declare_parameter("dc_out_topic", "/dc_motor_cmd_raw_ilk")
        self.declare_parameter("estop_topic", "/estop")

        # optional clamp (defensive)
        self.declare_parameter("x_min", -1.0)
        self.declare_parameter("x_max",  1.0)

        # if True: when estop asserted, publish 0 immediately even if no new dc cmd arrives
        self.declare_parameter("publish_zero_on_estop_edge", True)

        self.dc_in_topic = str(self.get_parameter("dc_in_topic").value)
        self.dc_out_topic = str(self.get_parameter("dc_out_topic").value)
        self.estop_topic = str(self.get_parameter("estop_topic").value)

        self.x_min = float(self.get_parameter("x_min").value)
        self.x_max = float(self.get_parameter("x_max").value)

        self.publish_zero_on_estop_edge = bool(self.get_parameter("publish_zero_on_estop_edge").value)

        # ---- state ----
        self.estop = False
        self.last_dc = 0.0

        # ---- ROS ----
        self.pub = self.create_publisher(Float32, self.dc_out_topic, 10)
        self.sub_dc = self.create_subscription(Float32, self.dc_in_topic, self.on_dc, 10)
        self.sub_estop = self.create_subscription(Bool, self.estop_topic, self.on_estop, 10)

        self.get_logger().info(
            f"[DcInterlock] in={self.dc_in_topic} out={self.dc_out_topic} estop={self.estop_topic} "
            f"clamp=[{self.x_min},{self.x_max}] edge_zero={self.publish_zero_on_estop_edge}"
        )

    def on_estop(self, msg: Bool):
        prev = self.estop
        self.estop = bool(msg.data)

        # rising edge: publish zero immediately (optional)
        if self.publish_zero_on_estop_edge and (not prev) and self.estop:
            out = Float32()
            out.data = 0.0
            self.pub.publish(out)

    def on_dc(self, msg: Float32):
        x = clamp(float(msg.data), self.x_min, self.x_max)
        self.last_dc = x

        out = Float32()
        out.data = 0.0 if self.estop else x
        self.pub.publish(out)

def main():
    rclpy.init()
    node = DcInterlock()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
