#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray


class AxisSafetyNode(Node):
    """
    Axis limit arbitration node

    Subscribe:
      - /cmd_vel_axis_raw (geometry_msgs/Twist)
          linear.x -> Y axis velocity
          linear.y -> Z axis velocity

      - /limit_switch (std_msgs/UInt8MultiArray)
          [Z+, Z0, Z-, Y+, Y0, Y-]

    Publish:
      - /cmd_vel_axis (geometry_msgs/Twist)
    """

    def __init__(self):
        super().__init__('axis_safety_node')

        # -------- parameters --------
        self.declare_parameter('cmd_in_topic',  '/cmd_vel_axis_raw')
        self.declare_parameter('cmd_out_topic', '/cmd_vel_axis')
        self.declare_parameter('limit_topic',   '/limit_switch')

        # 触发限位后保持禁止时间（ms），用于抑制抖动
        self.declare_parameter('hold_ms', 150)

        # 限位是否为高电平触发
        self.declare_parameter('active_high', True)

        self.cmd_in_topic  = self.get_parameter('cmd_in_topic').value
        self.cmd_out_topic = self.get_parameter('cmd_out_topic').value
        self.limit_topic   = self.get_parameter('limit_topic').value
        self.hold_ms       = int(self.get_parameter('hold_ms').value)
        self.active_high   = bool(self.get_parameter('active_high').value)

        # -------- ROS interfaces --------
        self.pub_cmd = self.create_publisher(Twist, self.cmd_out_topic, 10)
        self.sub_cmd = self.create_subscription(
            Twist, self.cmd_in_topic, self.cb_cmd, 10)
        self.sub_lim = self.create_subscription(
            UInt8MultiArray, self.limit_topic, self.cb_limit, 10)

        # -------- state --------
        self.last_cmd = Twist()
        self.limit_state = [0] * 6

        # latch time for each dangerous direction
        self.block_until = {
            'z_pos': 0.0,
            'z_neg': 0.0,
            'y_pos': 0.0,
            'y_neg': 0.0,
        }

        self.get_logger().info(
            f"AxisSafetyNode started. in={self.cmd_in_topic}, out={self.cmd_out_topic}"
        )

    def _is_active(self, v: int) -> bool:
        return (v == 1) if self.active_high else (v == 0)

    # -------- callbacks --------
    def cb_limit(self, msg: UInt8MultiArray):
        if len(msg.data) < 6:
            self.get_logger().warn("limit_switch data length < 6")
            return

        self.limit_state = list(msg.data[:6])

        # 限位变化也立即裁剪一次
        self._publish_safe_cmd(self.last_cmd)

    def cb_cmd(self, msg: Twist):
        self.last_cmd = msg
        self._publish_safe_cmd(msg)

    # -------- core logic --------
    def _publish_safe_cmd(self, cmd_in: Twist):
        now = time.time()
        hold_s = self.hold_ms / 1000.0

        # limit order: [Z+, Z0, Z-, Y+, Y0, Y-]
        z_p, _, z_n, y_p, _, y_n = self.limit_state

        if self._is_active(z_p):
            self.block_until['z_pos'] = max(self.block_until['z_pos'], now + hold_s)
        if self._is_active(z_n):
            self.block_until['z_neg'] = max(self.block_until['z_neg'], now + hold_s)
        if self._is_active(y_p):
            self.block_until['y_pos'] = max(self.block_until['y_pos'], now + hold_s)
        if self._is_active(y_n):
            self.block_until['y_neg'] = max(self.block_until['y_neg'], now + hold_s)

        y_cmd = float(cmd_in.linear.x)
        z_cmd = float(cmd_in.linear.y)

        # only block dangerous directions
        if now < self.block_until['z_pos'] and z_cmd > 0.0:
            z_cmd = 0.0
        if now < self.block_until['z_neg'] and z_cmd < 0.0:
            z_cmd = 0.0
        if now < self.block_until['y_pos'] and y_cmd > 0.0:
            y_cmd = 0.0
        if now < self.block_until['y_neg'] and y_cmd < 0.0:
            y_cmd = 0.0

        out = Twist()
        out.linear.x = y_cmd
        out.linear.y = z_cmd
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = 0.0

        self.pub_cmd.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AxisSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
