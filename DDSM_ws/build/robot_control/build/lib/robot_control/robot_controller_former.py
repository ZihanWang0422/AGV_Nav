#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
import aiohttp
import json
from typing import List, Dict
import threading

class TwistController(Node):
    def __init__(self):
        super().__init__('twist_controller')
        
        # ESP32 configuration
        self.board_ip = "192.168.4.1"
        self.base_url = f"http://{self.board_ip}/js"
        
        # Robot parameters
        self.wheel_separation = 0.3  # 轮距，单位米
        self.wheel_radius = 0.05    # 轮半径，单位米
        self.max_rpm = 280         # 最大转速
        
        # Create subscriber for twist commands
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)
            
        self.get_logger().info('Twist controller node started')
        
        # Create event loop for async operations
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self.thread.start()
        
    def _run_event_loop(self):
        """Run the event loop in a separate thread"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def motor_command_json(self, motor_id: int, rpm: int, act: int = 3) -> str:
        """Generate JSON command for a single motor"""
        cmd = {
            "T": 10010,
            "id": motor_id,
            "cmd": rpm,
            "act": act
        }
        return json.dumps(cmd)

    def ctrl_rpm_side_json(self, side: str, rpm: int, act: int = 3) -> List[str]:
        """Generate JSON commands for one side's motors"""
        commands = []
        if side == 'left':
            commands.append(self.motor_command_json(3, rpm, act))
            commands.append(self.motor_command_json(2, rpm, act))
        elif side == 'right':
            commands.append(self.motor_command_json(1, rpm, act))
            commands.append(self.motor_command_json(4, rpm, act))
        return commands

    def twist_to_motor_commands(self, linear_x: float, angular_z: float) -> List[str]:
        """Convert twist to motor commands
        
        Args:
            linear_x: Linear velocity in m/s
            angular_z: Angular velocity in rad/s
            
        Returns:
            List of JSON command strings
        """
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

    async def send_json_command_async(self, session: aiohttp.ClientSession, json_cmd_str: str):
        """Send a single JSON command asynchronously"""
        params = {'json': json_cmd_str}
        try:
            async with session.get(self.base_url, params=params, timeout=5) as response:
                result = await response.text()
                self.get_logger().info(f'发送命令: {json_cmd_str}')
                self.get_logger().info(f'收到响应: {result}')
        except Exception as e:
            self.get_logger().error(f'发送失败: {str(e)}')

    async def send_commands_concurrent(self, json_cmd_list: List[str]):
        """Send multiple JSON commands concurrently"""
        async with aiohttp.ClientSession() as session:
            tasks = [self.send_json_command_async(session, cmd) for cmd in json_cmd_list]
            await asyncio.gather(*tasks)

    def twist_callback(self, msg: Twist):
        """Handle received twist commands"""
        # 只使用linear.x（前进/后退）和angular.z（旋转）
        self.get_logger().info('收到Twist消息:')
        self.get_logger().info(f'  - linear: x={msg.linear.x:.2f}, y={msg.linear.y:.2f}, z={msg.linear.z:.2f}')
        self.get_logger().info(f'  - angular: x={msg.angular.x:.2f}, y={msg.angular.y:.2f}, z={msg.angular.z:.2f}')
        
        commands = self.twist_to_motor_commands(msg.linear.x, msg.angular.z)
        
        # 打印生成的JSON命令
        self.get_logger().info('生成的JSON命令:')
        for i, cmd in enumerate(commands):
            self.get_logger().info(f'  {i+1}. {cmd}')
        
        # 发送命令到ESP32
        asyncio.run_coroutine_threadsafe(
            self.send_commands_concurrent(commands), 
            self.loop
        )

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    twist_controller = TwistController()
    try:
        rclpy.spin(twist_controller)
    except KeyboardInterrupt:
        pass
    finally:
        twist_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 