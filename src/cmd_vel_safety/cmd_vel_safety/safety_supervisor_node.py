#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def is_valid_range(r: float) -> bool:
    return (r is not None) and (not math.isnan(r)) and (r > 0.0)

class SafetySupervisor(Node):
    """
    综合安全监督器（底盘）：
    - 输入：/cmd_vel_raw + 超声波(front/left/right) + /estop
    - 输出：/cmd_vel_safe （再交给 twist_filter_node 做最终限速/加速度/超时/心跳）
    - 当前：只实现超声波约束；后续可在同节点内加入激光雷达约束（k_lidar）
    """

    def __init__(self):
        super().__init__("safety_supervisor")

        # ---- params ----
        self.declare_parameter("in_cmd", "/cmd_vel_raw")
        self.declare_parameter("out_cmd", "/cmd_vel_safe")
        self.declare_parameter("estop_topic", "/estop")

        # ultrasonic topics (default mapping: s1 front, s2 left, s3 right)
        self.declare_parameter("front_topic", "/ultrasonic/s1")
        self.declare_parameter("left_topic",  "/ultrasonic/s2")
        self.declare_parameter("right_topic", "/ultrasonic/s3")

        # behavior
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("sensor_timeout_sec", 0.30)

        # thresholds for forward motion
        self.declare_parameter("slow_dist_m", 0.60)
        self.declare_parameter("stop_dist_m", 0.30)

        # optional: limit angular speed when side obstacles too close
        self.declare_parameter("side_slow_dist_m", 0.35)
        self.declare_parameter("side_stop_dist_m", 0.20)

        # policy when ultrasonic stale/invalid: "ignore" or "stop"
        self.declare_parameter("ultra_fail_policy", "ignore")  # or "stop"

        self.in_cmd = str(self.get_parameter("in_cmd").value)
        self.out_cmd = str(self.get_parameter("out_cmd").value)
        self.estop_topic = str(self.get_parameter("estop_topic").value)

        self.front_topic = str(self.get_parameter("front_topic").value)
        self.left_topic  = str(self.get_parameter("left_topic").value)
        self.right_topic = str(self.get_parameter("right_topic").value)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.period = 1.0 / self.publish_hz if self.publish_hz > 0 else 0.02
        self.sensor_timeout = float(self.get_parameter("sensor_timeout_sec").value)

        self.slow_dist = float(self.get_parameter("slow_dist_m").value)
        self.stop_dist = float(self.get_parameter("stop_dist_m").value)

        self.side_slow = float(self.get_parameter("side_slow_dist_m").value)
        self.side_stop = float(self.get_parameter("side_stop_dist_m").value)

        self.ultra_fail_policy = str(self.get_parameter("ultra_fail_policy").value).lower().strip()

        # ---- state ----
        self.estop = False
        self.last_cmd = Twist()
        self.last_cmd_time = None

        self.front = float("nan")
        self.left  = float("nan")
        self.right = float("nan")
        self.front_time = None
        self.left_time  = None
        self.right_time = None

        # ---- ROS I/O ----
        self.sub_cmd = self.create_subscription(Twist, self.in_cmd, self.cb_cmd, 10)
        self.sub_estop = self.create_subscription(Bool, self.estop_topic, self.cb_estop, 10)

        self.sub_front = self.create_subscription(Range, self.front_topic, self.cb_front, 10)
        self.sub_left  = self.create_subscription(Range, self.left_topic,  self.cb_left, 10)
        self.sub_right = self.create_subscription(Range, self.right_topic, self.cb_right, 10)

        self.pub_cmd = self.create_publisher(Twist, self.out_cmd, 10)
        self.timer = self.create_timer(self.period, self.on_timer)

        self.get_logger().info(
            f"[SafetySupervisor] in={self.in_cmd} out={self.out_cmd} "
            f"ultra(front={self.front_topic},left={self.left_topic},right={self.right_topic}) "
            f"policy={self.ultra_fail_policy}"
        )

    def cb_estop(self, msg: Bool):
        self.estop = bool(msg.data)

    def cb_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def cb_front(self, msg: Range):
        self.front = float(msg.range)
        self.front_time = self.get_clock().now()

    def cb_left(self, msg: Range):
        self.left = float(msg.range)
        self.left_time = self.get_clock().now()

    def cb_right(self, msg: Range):
        self.right = float(msg.range)
        self.right_time = self.get_clock().now()

    def _is_fresh(self, t) -> bool:
        if t is None:
            return False
        age = (self.get_clock().now() - t).nanoseconds * 1e-9
        return age <= self.sensor_timeout

    def _scale_from_dist(self, d: float, stop_d: float, slow_d: float) -> float:
        """
        Return k in [0,1].
        d <= stop_d => 0
        d >= slow_d => 1
        else linear between.
        """
        if not is_valid_range(d):
            return None
        if d <= stop_d:
            return 0.0
        if d >= slow_d:
            return 1.0
        # linear ramp
        return (d - stop_d) / max(slow_d - stop_d, 1e-6)

    def on_timer(self):
        out = Twist()
        if self.estop:
            self.pub_cmd.publish(out)
            return

        # if no cmd received yet, publish zeros (safe)
        if self.last_cmd_time is None:
            self.pub_cmd.publish(out)
            return

        v = float(self.last_cmd.linear.x)
        w = float(self.last_cmd.angular.z)

        # --- ultrasonic freshness ---
        f_fresh = self._is_fresh(self.front_time)
        l_fresh = self._is_fresh(self.left_time)
        r_fresh = self._is_fresh(self.right_time)

        # compute scaling
        k_ultra_v = 1.0
        k_ultra_w = 1.0

        # forward constraint based on front sensor
        if v > 0.0:
            if f_fresh:
                k = self._scale_from_dist(self.front, self.stop_dist, self.slow_dist)
                if k is not None:
                    k_ultra_v = min(k_ultra_v, k)
            else:
                if self.ultra_fail_policy == "stop":
                    k_ultra_v = 0.0

        # side constraint based on left/right: limit turning when too close
        # (optional conservative behavior)
        def side_k(d, fresh):
            if not fresh:
                return None
            return self._scale_from_dist(d, self.side_stop, self.side_slow)

        kl = side_k(self.left, l_fresh)
        kr = side_k(self.right, r_fresh)
        ks = []
        if kl is not None:
            ks.append(kl)
        if kr is not None:
            ks.append(kr)
        if ks:
            k_ultra_w = min(k_ultra_w, min(ks))

        # apply
        v_out = v * k_ultra_v
        w_out = w * k_ultra_w

        out.linear.x = float(v_out)
        out.angular.z = float(w_out)
        self.pub_cmd.publish(out)

def main():
    rclpy.init()
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
