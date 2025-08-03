#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        
        # 创建订阅者
        self.nav_sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.nav_callback,
            10)
            
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
            
        self.get_logger().info('cmd_vel_mux node initialized')
        
    def nav_callback(self, msg):
        # 直接转发消息
        self.cmd_vel_pub.publish(msg)
        self.get_logger().debug('Forwarded velocity command')

def main():
    rclpy.init()
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 