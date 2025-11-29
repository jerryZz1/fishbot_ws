#!/usr/bin/env python3
"""
vib_motor_node.py

把原来的 Modbus 振动电机测试脚本封装成一个 ROS2 节点：

- 订阅: /vib_motor_cmd  (std_msgs/Float32, 0~100, <=0 停止)
- （可选）发布: /vib_motor_state (std_msgs/Float32, 当前转速百分比)

串口参数、寄存器地址都做成参数，可以在 launch 里改。
"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException


class VibMotorNode(Node):
    def __init__(self) -> None:
        super().__init__('vib_motor_node')

        # ---------- 参数 ----------
        self.declare_parameter('port', '/dev/ttyS3')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('speed_reg', 0x1552)   # 写入目标转速 %
        self.declare_parameter('ctrl_reg',  0x1551)   # 启停控制寄存器 (1=启, 0=停)
        self.declare_parameter('fb_reg',    0x1510)   # 反馈转速 %

        port      = self.get_parameter('port').get_parameter_value().string_value
        baudrate  = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.slave_id = self.get_parameter('slave_id').get_parameter_value().integer_value
        self.speed_reg= self.get_parameter('speed_reg').get_parameter_value().integer_value
        self.ctrl_reg = self.get_parameter('ctrl_reg').get_parameter_value().integer_value
        self.fb_reg   = self.get_parameter('fb_reg').get_parameter_value().integer_value

        # ---------- Modbus 客户端 ----------
        # timeout 改为 3.0，和 simple_motor_control.py 一致
        self.client: Optional[ModbusSerialClient] = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            parity='N',
            stopbits=1,
            bytesize=8,
            timeout=3.0,
        )

        self.connected = False
        self._connect_modbus()

        # ---------- ROS 接口 ----------
        # 订阅控制指令：0~100 -> 设置转速并启动；<=0 -> 停止
        self.sub_cmd = self.create_subscription(
            Float32,
            'vib_motor_cmd',
            self.cmd_callback,
            10
        )

        # 状态发布（暂时可以不用，先专注把启停跑通）
        self.pub_state = self.create_publisher(
            Float32,
            'vib_motor_state',
            10
        )

        # 周期读取当前转速（调试阶段可以先关掉，等启停正常了再打开）
        # self.timer = self.create_timer(0.5, self.read_state_timer_cb)

        self.get_logger().info(
            f"Vibration motor node started. port={port}, baudrate={baudrate}, slave={self.slave_id}"
        )

    # ---------- Modbus 连接 / 重连 ----------
    def _connect_modbus(self) -> None:
        if self.client is None:
            return
        if self.connected:
            return

        self.get_logger().info("Trying to connect Modbus device...")
        ok = self.client.connect()
        if not ok:
            self.get_logger().error("Failed to connect Modbus device.")
            self.connected = False
        else:
            self.get_logger().info("Modbus connected.")
            self.connected = True

    def _ensure_connected(self) -> bool:
        """确保已连接，若断开则尝试重连一次。"""
        if not self.connected:
            self._connect_modbus()
        return self.connected

    # ---------- ROS 回调：接收目标转速 ----------
    def cmd_callback(self, msg: Float32) -> None:
        """
        收到目标速度指令:
        - msg.data <= 0: 停止电机
        - msg.data > 0: 设置转速并启动
        """
        target_speed = float(msg.data)
        if not self._ensure_connected():
            self.get_logger().warn("No Modbus connection, ignore command.")
            return

        try:
            if target_speed <= 0.0:
                # 写停止
                self.get_logger().info("Command: STOP motor")
                self._write_register(self.ctrl_reg, 0)   # 停止
            else:
                # 限幅到 0~100
                if target_speed > 100.0:
                    target_speed = 100.0
                if target_speed < 0.0:
                    target_speed = 0.0

                speed_val = int(target_speed)
                self.get_logger().info(f"Command: set speed {speed_val}% and START motor")

                # 1. 先写速度寄存器（0x1552）
                self._write_register(self.speed_reg, speed_val)

                # 2. 模仿 simple_motor_control.py，加一点延时
                time.sleep(0.5)

                # 3. 再写启动寄存器（0x1551）
                self._write_register(self.ctrl_reg, 1)

        except Exception as e:
            # 这里的 e 里已经包含更详细的信息（见 _write_register）
            self.get_logger().error(f"Error sending Modbus command: {e}")
            self.connected = False

    # ---------- 定时读取当前转速并发布（可暂时不用） ----------
    def read_state_timer_cb(self) -> None:
        if not self._ensure_connected():
            # 没连上就暂时不读
            return

        try:
            resp = self.client.read_holding_registers(self.fb_reg, 1, slave=self.slave_id)
            if resp.isError():
                # 出错时，标记断线，下次再重连
                self.get_logger().warn("Read feedback failed (Modbus error).")
                self.connected = False
                return

            current_speed = resp.registers[0]
            msg = Float32()
            msg.data = float(current_speed)
            self.pub_state.publish(msg)
        except ModbusIOException as e:
            self.get_logger().warn(f"Read feedback ModbusIOException: {e}")
            self.connected = False
        except Exception as e:
            self.get_logger().error(f"Read feedback exception: {e}")
            self.connected = False

    # ---------- 写单寄存器的小工具（增强日志） ----------
    def _write_register(self, address: int, value: int) -> None:
        """
        写单个寄存器，如果设备返回异常，会打印详细信息并抛出异常。
        """
        if not self._ensure_connected():
            raise RuntimeError("No Modbus connection")

        resp = self.client.write_register(address, value, slave=self.slave_id)

        if resp.isError():
            # 尽量把错误信息打全，便于对照驱动器手册
            err_text = f"Modbus write error: addr=0x{address:X}, value={value}, resp={resp}"
            try:
                func_code = getattr(resp, "function_code", None)
                exc_code  = getattr(resp, "exception_code", None)
                err_text += f", func={func_code}, exc={exc_code}"
            except Exception:
                pass

            self.get_logger().error(err_text)
            raise RuntimeError(err_text)

    # ---------- 关闭 ----------
    def destroy_node(self):
        self.get_logger().info("Shutting down vib_motor_node...")
        try:
            if self.client is not None:
                self.client.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VibMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
