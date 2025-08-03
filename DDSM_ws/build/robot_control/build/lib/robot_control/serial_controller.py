#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import threading
import time
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SerialController(Node):
    def __init__(self):
        super().__init__('serial_controller')
        
        # 串口配置参数
        self.serial_port = self.declare_parameter('serial_port', '/dev/car').value
        self.baud_rate = self.declare_parameter('baud_rate', 115200).value
        
        # 机器人参数（与TwistController保持一致）
        self.wheel_separation = 0.3  # 轮距，单位米
        self.wheel_radius = 0.05    # 轮半径，单位米
        self.max_rpm = 280         # 最大转速
        
        # 初始化串口
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            self.get_logger().info(f'成功打开串口 {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'无法打开串口: {str(e)}')
            return

        # 创建速度命令订阅者
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 创建串口数据发布者
        self.serial_pub = self.create_publisher(
            String,
            'serial_data',
            10
        )
        
        # 创建串口读取线程
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        self.get_logger().info('串口控制器已初始化')

    def motor_command_json(self, motor_id: int, rpm: int, act: int = 3) -> dict:
        """生成单个电机的JSON命令"""
        cmd = {
            "T": 10010,
            "id": motor_id,
            "cmd": rpm,
            "act": act
        }
        return cmd  # 返回字典而不是字符串

    def ctrl_rpm_side_json(self, side: str, rpm: int, act: int = 3) -> list:
        """生成一侧电机的JSON命令"""
        commands = []
        if side == 'left':
            commands.append(self.motor_command_json(3, rpm, act))
            commands.append(self.motor_command_json(2, rpm, act))
        elif side == 'right':
            commands.append(self.motor_command_json(1, rpm, act))
            commands.append(self.motor_command_json(4, rpm, act))
        return commands

    def twist_to_motor_commands(self, linear_x: float, angular_z: float) -> list:
        """将Twist消息转换为电机命令"""
        # 计算左右轮的线速度 (m/s)
        v_left = linear_x - (angular_z * self.wheel_separation / 2.0)
        v_right = linear_x + (angular_z * self.wheel_separation / 2.0)
        
        # 转换为RPM
        rpm_left = int((v_left * 60) / (2 * 3.14159 * self.wheel_radius))
        rpm_right = int((v_right * 60) / (2 * 3.14159 * self.wheel_radius))
        
        # 限制在最大转速范围内
        rpm_left = max(min(rpm_left, self.max_rpm), -self.max_rpm)
        rpm_right = max(min(rpm_right, self.max_rpm), -self.max_rpm)
        
        # 生成电机命令
        commands = []
        commands.extend(self.ctrl_rpm_side_json('left', rpm_left))
        commands.extend(self.ctrl_rpm_side_json('right', -rpm_right))  # 注意：右侧电机需要反向
        
        # 打印速度和RPM信息
        self.get_logger().info(f'左轮速度: {v_left:.2f} m/s, RPM: {rpm_left}')
        self.get_logger().info(f'右轮速度: {v_right:.2f} m/s, RPM: {rpm_right}')
        
        return commands

    def cmd_vel_callback(self, msg: Twist):
        """处理接收到的Twist消息"""
        # 打印接收到的Twist消息
        self.get_logger().info('收到Twist消息:')
        self.get_logger().info(f'  - linear: x={msg.linear.x:.2f}, y={msg.linear.y:.2f}, z={msg.linear.z:.2f}')
        self.get_logger().info(f'  - angular: x={msg.angular.x:.2f}, y={msg.angular.y:.2f}, z={msg.angular.z:.2f}')
        
        # 生成电机命令
        commands = self.twist_to_motor_commands(msg.linear.x, msg.angular.z)
        
        # 分别发送每个电机的命令
        for i, cmd in enumerate(commands):
            json_str = json.dumps(cmd)  # 将字典转换为JSON字符串
            self.get_logger().info(f'发送电机 {cmd["id"]} 命令: {json_str}')
            try:
                # 发送命令并添加换行符
                self.ser.write((json_str + '\n').encode())
                # 等待一小段时间，确保命令被分开发送
                time.sleep(0.01)
            except serial.SerialException as e:
                self.get_logger().error(f'发送数据失败: {str(e)}')

    def read_serial(self):
        """持续读取串口数据的线程"""
        while rclpy.ok():
            if self.ser.is_open:
                try:
                    if self.ser.in_waiting:
                        data = self.ser.readline().decode('utf-8').strip()
                        try:
                            # 尝试解析JSON数据
                            json_data = json.loads(data)
                            # 发布格式化的JSON字符串
                            msg = String()
                            msg.data = json.dumps(json_data, indent=2)
                            self.serial_pub.publish(msg)
                            self.get_logger().info(f'收到JSON数据: {msg.data}')
                        except json.JSONDecodeError:
                            # 如果不是JSON格式，直接发布原始数据
                            msg = String()
                            msg.data = data
                            self.serial_pub.publish(msg)
                            self.get_logger().info(f'收到数据: {data}')
                except serial.SerialException as e:
                    self.get_logger().error(f'读取数据失败: {str(e)}')
                    break
            time.sleep(0.01)

    def destroy_node(self):
        """清理节点资源"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
