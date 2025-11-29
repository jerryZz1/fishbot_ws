#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import serial
import time
import threading

# =================== Modbus RTU Client ===================
class ModbusRTUClient:
    def __init__(self, port='/dev/ttyS2', baudrate=38400, slave_id=1, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.timeout = timeout
        self.ser = None

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            return True
        except Exception as e:
            print(f"Modbus connect error: {e}")
            return False

    def crc16(self, data):
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for i in range(8):
                if (crc & 1):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    def read_discrete_inputs(self, address=0, count=8):
        """功能码0x02: 读取离散输入"""
        if self.ser is None or not self.ser.is_open:
            return None
        request = [self.slave_id, 0x02, (address >> 8) & 0xFF, address & 0xFF,
                   (count >> 8) & 0xFF, count & 0xFF]
        crc = self.crc16(request)
        request += [crc & 0xFF, (crc >> 8) & 0xFF]

        try:
            self.ser.reset_input_buffer()
            self.ser.write(bytearray(request))
            resp = self.ser.read(5 + ((count + 7) // 8))
            if len(resp) < 5:
                return None
            # 校验CRC
            received_crc = (resp[-1] << 8) | resp[-2]
            calc_crc = self.crc16(list(resp[:-2]))
            if received_crc != calc_crc:
                return None
            # 解析位
            byte_count = resp[2]
            data_bytes = resp[3:3+byte_count]
            states = []
            for i in range(count):
                byte_index = i // 8
                bit_index = i % 8
                state = (data_bytes[byte_index] >> bit_index) & 0x01 if byte_index < len(data_bytes) else 0
                states.append(state)
            return states
        except Exception as e:
            print(f"Modbus read error: {e}")
            return None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

# =================== ROS2 Node ===================
class LimitSwitchNode(Node):
    def __init__(self):
        super().__init__('limit_switch_node')
        self.pub = self.create_publisher(UInt8MultiArray, 'limit_switch', 10)
        self.client = ModbusRTUClient('/dev/ttyS2', 38400, 1)
        if not self.client.connect():
            self.get_logger().error("Modbus connect failed!")
        self.lock = threading.Lock()

        # 配置限位顺序: 正 -> 原 -> 负
        # 原始索引: Z原0, Z正1, Z负2, Y正3, Y原4, Y负5
        # 发布顺序: Z正, Z原, Z负, Y正, Y原, Y负
        self.order_map = [1,0,2,3,4,5]

        # 去抖动缓存
        self.last_states = [0]*6
        self.stable_states = [0]*6
        self.debounce_count = 2  # 连续2次相同才认为有效
        self.current_count = [0]*6

        # 开启线程循环
        self.thread = threading.Thread(target=self.loop)
        self.thread.daemon = True
        self.thread.start()
        self.get_logger().info("Limit switch node started")

    def loop(self):
        poll_interval = 0.05  # 50ms轮询
        while rclpy.ok():
            raw_states = self.client.read_discrete_inputs(0, 6)
            if raw_states:
                with self.lock:
                    # 去抖动处理
                    for i, idx in enumerate(self.order_map):
                        if raw_states[idx] == self.last_states[i]:
                            self.current_count[i] += 1
                        else:
                            self.current_count[i] = 1
                        if self.current_count[i] >= self.debounce_count:
                            self.stable_states[i] = raw_states[idx]
                        self.last_states[i] = raw_states[idx]

                # 发布消息
                msg = UInt8MultiArray()
                msg.data = self.stable_states.copy()
                self.pub.publish(msg)
                self.get_logger().info(f"Pub: {msg.data}")
            time.sleep(poll_interval)

    def destroy_node(self):
        self.client.close()
        super().destroy_node()

# =================== Main ===================
def main(args=None):
    rclpy.init(args=args)
    node = LimitSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
