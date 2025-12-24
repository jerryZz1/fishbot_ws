#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Bool

class LimitSwitchDemux(Node):
    def __init__(self):
        super().__init__("limit_switch_demux")

        self.declare_parameter("in_topic", "/limit_switch")

        # 这四个 index 默认按你当前发布顺序
        self.declare_parameter("idx_z_max", 0)  # Z正
        self.declare_parameter("idx_z_min", 2)  # Z负
        self.declare_parameter("idx_y_max", 3)  # Y正
        self.declare_parameter("idx_y_min", 5)  # Y负

        # 若你的硬件是常闭/反逻辑，这里可反转
        self.declare_parameter("active_high", True)

        self.in_topic = self.get_parameter("in_topic").value
        self.idx_z_max = int(self.get_parameter("idx_z_max").value)
        self.idx_z_min = int(self.get_parameter("idx_z_min").value)
        self.idx_y_max = int(self.get_parameter("idx_y_max").value)
        self.idx_y_min = int(self.get_parameter("idx_y_min").value)
        self.active_high = bool(self.get_parameter("active_high").value)

        self.sub = self.create_subscription(UInt8MultiArray, self.in_topic, self.cb, 10)

        self.pub_y_min = self.create_publisher(Bool, "/axis/limit_y_min", 10)
        self.pub_y_max = self.create_publisher(Bool, "/axis/limit_y_max", 10)
        self.pub_z_min = self.create_publisher(Bool, "/axis/limit_z_min", 10)
        self.pub_z_max = self.create_publisher(Bool, "/axis/limit_z_max", 10)

    def _norm(self, v: int) -> bool:
        raw = bool(v)
        return raw if self.active_high else (not raw)

    def cb(self, msg: UInt8MultiArray):
        data = list(msg.data)
        def safe_get(i: int) -> int:
            return data[i] if 0 <= i < len(data) else 0

        y_min = self._norm(safe_get(self.idx_y_min))
        y_max = self._norm(safe_get(self.idx_y_max))
        z_min = self._norm(safe_get(self.idx_z_min))
        z_max = self._norm(safe_get(self.idx_z_max))

        self.pub_y_min.publish(Bool(data=y_min))
        self.pub_y_max.publish(Bool(data=y_max))
        self.pub_z_min.publish(Bool(data=z_min))
        self.pub_z_max.publish(Bool(data=z_max))

def main():
    rclpy.init()
    n = LimitSwitchDemux()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
