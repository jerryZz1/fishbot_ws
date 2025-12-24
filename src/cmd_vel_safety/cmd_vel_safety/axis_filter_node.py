#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def apply_slew(cur: float, tgt: float, a_max: float, dt: float) -> float:
    """Slew-rate limiter: limit |tgt-cur| <= a_max*dt. If a_max<=0, no limit."""
    if a_max <= 0.0:
        return tgt
    delta = tgt - cur
    max_delta = a_max * dt
    delta = clamp(delta, -max_delta, max_delta)
    out = cur + delta
    if abs(out) < 1e-6:
        out = 0.0
    return out

class AxisSafetyFilter(Node):
    """
    轴专用安全过滤器（Twist）：
    - 订阅 in_topic(Twist) -> 发布 out_topic(Twist)
    - 约定：linear.x = Y 轴速度；linear.y = Z 轴速度
    - Y/Z 独立：限速、加速度限制（slew-rate）
    - 超时置零、急停置零
    - 限位开关：若继续朝触发方向运动，则该轴速度强制为0
    """

    def __init__(self):
        super().__init__("axis_safety_filter")

        # ---------- topics ----------
        self.declare_parameter("in_topic", "/cmd_vel_axis_raw")
        self.declare_parameter("out_topic", "/cmd_vel_axis")
        self.declare_parameter("estop_topic", "/estop")

        # ---------- timing ----------
        self.declare_parameter("cmd_timeout_sec", 0.25)
        self.declare_parameter("publish_hz", 50.0)

        # ---------- limits (per-axis) ----------
        self.declare_parameter("y_vel_max", 0.20)
        self.declare_parameter("z_vel_max", 0.20)
        self.declare_parameter("y_acc_max", 0.50)
        self.declare_parameter("z_acc_max", 0.50)

        # ---------- limit switch topics (Bool) ----------
        self.declare_parameter("y_min_topic", "/axis/limit_y_min")
        self.declare_parameter("y_max_topic", "/axis/limit_y_max")
        self.declare_parameter("z_min_topic", "/axis/limit_z_min")
        self.declare_parameter("z_max_topic", "/axis/limit_z_max")

        # active level: True means msg.data==True is "HIT"
        self.declare_parameter("y_min_active_high", True)
        self.declare_parameter("y_max_active_high", True)
        self.declare_parameter("z_min_active_high", True)
        self.declare_parameter("z_max_active_high", True)

        # optional: latch hit until released (usually False is fine)
        self.declare_parameter("latch_limit", False)

        # ---------- read params ----------
        self.in_topic = str(self.get_parameter("in_topic").value)
        self.out_topic = str(self.get_parameter("out_topic").value)
        self.estop_topic = str(self.get_parameter("estop_topic").value)

        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.pub_period = 1.0 / self.publish_hz if self.publish_hz > 0 else 0.02

        self.y_vel_max = float(self.get_parameter("y_vel_max").value)
        self.z_vel_max = float(self.get_parameter("z_vel_max").value)
        self.y_acc_max = float(self.get_parameter("y_acc_max").value)
        self.z_acc_max = float(self.get_parameter("z_acc_max").value)

        self.y_min_topic = str(self.get_parameter("y_min_topic").value)
        self.y_max_topic = str(self.get_parameter("y_max_topic").value)
        self.z_min_topic = str(self.get_parameter("z_min_topic").value)
        self.z_max_topic = str(self.get_parameter("z_max_topic").value)

        self.y_min_active_high = bool(self.get_parameter("y_min_active_high").value)
        self.y_max_active_high = bool(self.get_parameter("y_max_active_high").value)
        self.z_min_active_high = bool(self.get_parameter("z_min_active_high").value)
        self.z_max_active_high = bool(self.get_parameter("z_max_active_high").value)
        self.latch_limit = bool(self.get_parameter("latch_limit").value)

        # ---------- state ----------
        self.estop = False
        self.last_in_time = None

        self.tgt_y = 0.0
        self.tgt_z = 0.0
        self.out_y = 0.0
        self.out_z = 0.0
        self.last_update_time = self.get_clock().now()

        self._y_min_hit = False
        self._y_max_hit = False
        self._z_min_hit = False
        self._z_max_hit = False

        self._y_min_latched = False
        self._y_max_latched = False
        self._z_min_latched = False
        self._z_max_latched = False

        # ---------- ROS I/O ----------
        self.sub_cmd = self.create_subscription(Twist, self.in_topic, self.on_cmd, 10)
        self.sub_estop = self.create_subscription(Bool, self.estop_topic, self.on_estop, 10)

        self.sub_ymin = self.create_subscription(Bool, self.y_min_topic, self.on_ymin, 10)
        self.sub_ymax = self.create_subscription(Bool, self.y_max_topic, self.on_ymax, 10)
        self.sub_zmin = self.create_subscription(Bool, self.z_min_topic, self.on_zmin, 10)
        self.sub_zmax = self.create_subscription(Bool, self.z_max_topic, self.on_zmax, 10)

        self.pub_cmd = self.create_publisher(Twist, self.out_topic, 10)
        self.timer = self.create_timer(self.pub_period, self.on_timer)

        self.get_logger().info(
            f"[AxisFilter] in={self.in_topic} out={self.out_topic} "
            f"timeout={self.cmd_timeout}s pub={self.publish_hz}Hz "
            f"limits: y({self.y_min_topic},{self.y_max_topic}) z({self.z_min_topic},{self.z_max_topic})"
        )

    # ---------- helpers ----------
    def _norm_hit(self, raw: bool, active_high: bool) -> bool:
        return raw if active_high else (not raw)

    def _eff_hit(self, hit: bool, latched: bool) -> bool:
        return (hit or latched) if self.latch_limit else hit

    def _gate_by_limits(self, y: float, z: float) -> tuple[float, float]:
        """
        Convention:
          y < 0 -> toward y_min; y > 0 -> toward y_max
          z < 0 -> toward z_min; z > 0 -> toward z_max
        """
        y_min = self._eff_hit(self._y_min_hit, self._y_min_latched)
        y_max = self._eff_hit(self._y_max_hit, self._y_max_latched)
        z_min = self._eff_hit(self._z_min_hit, self._z_min_latched)
        z_max = self._eff_hit(self._z_max_hit, self._z_max_latched)

        if y_min and y < 0.0:
            y = 0.0
        if y_max and y > 0.0:
            y = 0.0
        if z_min and z < 0.0:
            z = 0.0
        if z_max and z > 0.0:
            z = 0.0
        return y, z

    # ---------- callbacks ----------
    def on_estop(self, msg: Bool):
        self.estop = bool(msg.data)

    def on_ymin(self, msg: Bool):
        hit = self._norm_hit(bool(msg.data), self.y_min_active_high)
        self._y_min_hit = hit
        if self.latch_limit:
            self._y_min_latched = hit

    def on_ymax(self, msg: Bool):
        hit = self._norm_hit(bool(msg.data), self.y_max_active_high)
        self._y_max_hit = hit
        if self.latch_limit:
            self._y_max_latched = hit

    def on_zmin(self, msg: Bool):
        hit = self._norm_hit(bool(msg.data), self.z_min_active_high)
        self._z_min_hit = hit
        if self.latch_limit:
            self._z_min_latched = hit

    def on_zmax(self, msg: Bool):
        hit = self._norm_hit(bool(msg.data), self.z_max_active_high)
        self._z_max_hit = hit
        if self.latch_limit:
            self._z_max_latched = hit

    def on_cmd(self, msg: Twist):
        now = self.get_clock().now()
        self.last_in_time = now

        y = float(msg.linear.x)
        z = float(msg.linear.y)

        # clamp
        y = clamp(y, -self.y_vel_max, self.y_vel_max)
        z = clamp(z, -self.z_vel_max, self.z_vel_max)

        # gate by limits
        y, z = self._gate_by_limits(y, z)

        self.tgt_y = y
        self.tgt_z = z

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = self.pub_period
        self.last_update_time = now

        # timeout
        timed_out = True
        if self.last_in_time is not None:
            age = (now - self.last_in_time).nanoseconds * 1e-9
            timed_out = (age > self.cmd_timeout)

        if self.estop or timed_out:
            tgt_y, tgt_z = 0.0, 0.0
        else:
            tgt_y, tgt_z = self.tgt_y, self.tgt_z

        # apply gating again (safe even if 0)
        tgt_y, tgt_z = self._gate_by_limits(tgt_y, tgt_z)

        # slew-rate per axis
        self.out_y = apply_slew(self.out_y, tgt_y, self.y_acc_max, dt)
        self.out_z = apply_slew(self.out_z, tgt_z, self.z_acc_max, dt)

        out = Twist()
        out.linear.x = float(self.out_y)
        out.linear.y = float(self.out_z)
        self.pub_cmd.publish(out)

def main():
    rclpy.init()
    node = AxisSafetyFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
