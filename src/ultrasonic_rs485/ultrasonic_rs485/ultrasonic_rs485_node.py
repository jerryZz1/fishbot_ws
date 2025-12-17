#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import UInt16MultiArray

import serial
import struct
import math
import time


class ModbusRTUClient:
    """Modbus-RTU CRC16, function 0x03 read holding registers."""
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    @staticmethod
    def crc16(data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 1:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def read_holding_registers(self, slave_id: int, start_addr: int, count: int) -> list[int] | None:
        # Request: [id][0x03][addr_hi][addr_lo][cnt_hi][cnt_lo][crc_lo][crc_hi]
        pdu = struct.pack(">B B H H", slave_id, 0x03, start_addr, count)
        crc = self.crc16(pdu)
        req = pdu + struct.pack("<H", crc)

        # Response: [id][0x03][byte_count][data...][crc_lo][crc_hi]
        resp_len = 3 + 2 * count + 2
        self.ser.reset_input_buffer()
        self.ser.write(req)
        resp = self.ser.read(resp_len)
        if len(resp) != resp_len:
            return None

        recv_crc = struct.unpack("<H", resp[-2:])[0]
        calc_crc = self.crc16(resp[:-2])
        if recv_crc != calc_crc:
            return None

        if resp[0] != slave_id or resp[1] != 0x03:
            return None

        byte_count = resp[2]
        if byte_count != 2 * count:
            return None

        regs = []
        data = resp[3:3 + byte_count]
        for i in range(count):
            regs.append(struct.unpack(">H", data[2*i:2*i+2])[0])  # high byte first
        return regs

    def close(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass


class UltrasonicA22Rs485Node(Node):
    """
    Read DYP-A22 RS485(Modbus-RTU) realtime distance register 0x0101 (mm).
    Publish:
      - /ultrasonic/<id> : sensor_msgs/Range
      - /ultrasonic/raw_mm : UInt16MultiArray (same order as slave_ids)
    """

    def __init__(self):
        super().__init__("ultrasonic_a22_rs485_node")

        # ---- params ----
        self.declare_parameter("port", "/dev/ttyS4")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout", 0.10)
        self.declare_parameter("poll_hz", 10.0)
        self.declare_parameter("slave_ids", [1, 2, 3])

        # A22 realtime distance register (mm): 0x0101
        self.declare_parameter("dist_reg", 0x0101)

        # Range msg config (按你场景改)
        self.declare_parameter("min_range_m", 0.01)
        self.declare_parameter("max_range_m", 3.50)
        self.declare_parameter("field_of_view_rad", 0.5)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.poll_hz = float(self.get_parameter("poll_hz").value)
        self.slave_ids = list(self.get_parameter("slave_ids").value)
        self.dist_reg = int(self.get_parameter("dist_reg").value)

        self.min_range_m = float(self.get_parameter("min_range_m").value)
        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.fov = float(self.get_parameter("field_of_view_rad").value)

        self.client = ModbusRTUClient(self.port, self.baudrate, self.timeout)

        self.pubs = {}
        for sid in self.slave_ids:
            self.pubs[sid] = self.create_publisher(Range, f"/ultrasonic/s{sid}", 10)

        self.pub_raw = self.create_publisher(UInt16MultiArray, "/ultrasonic/raw_mm", 10)

        period = 1.0 / max(self.poll_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"Ultrasonic A22 RS485 node: port={self.port}, baud={self.baudrate}, ids={self.slave_ids}, poll_hz={self.poll_hz}"
        )

    def on_timer(self):
        stamp = self.get_clock().now().to_msg()

        raw_list = []
        for sid in self.slave_ids:
            regs = self.client.read_holding_registers(sid, self.dist_reg, 1)
            if not regs:
                # 读失败用 0 填一下（你也可以用 0xFFFF）
                raw_mm = 0xFFFF
                raw_list.append(raw_mm)
                continue

            raw_mm = regs[0] & 0xFFFF
            raw_list.append(raw_mm)

            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = f"ultrasonic_{sid}"
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self.fov
            msg.min_range = self.min_range_m
            msg.max_range = self.max_range_m

            # 手册：0xFFFE=同频干扰，0xFFFD=测不到物体（我们用 NaN 表示无效）
            if raw_mm in (0xFFFE, 0xFFFD, 0xFFFF):
                msg.range = float("nan")
            else:
                msg.range = float(raw_mm) / 1000.0  # mm -> m

            self.pubs[sid].publish(msg)

        raw_msg = UInt16MultiArray()
        raw_msg.data = raw_list
        self.pub_raw.publish(raw_msg)

    def destroy_node(self):
        try:
            self.client.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicA22Rs485Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
