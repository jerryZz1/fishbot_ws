# fishbot_ws

基于 ROS2 Humble 的地坪整平机器人上位机工作空间。

## 功能模块

- `turtle_joy`：手柄控制底盘 + YZ 轴
- `vib_motor_driver`：通过 Modbus-RTU 控制振动电机
- `micro_ros_agent`：与 ESP32-S3 micro-ROS 客户端通讯
- `a22_ultrasonic_driver`（计划）：多路超声 Modbus 采集
- `fishbot_bringup`：整机启动、底盘和传感器配置
- …

## 运行环境

- Ubuntu 22.04
- ROS2 Humble
- …

## 快速启动

```bash
colcon build
source install/setup.bash
ros2 launch fishbot_bringup bringup.launch.py
