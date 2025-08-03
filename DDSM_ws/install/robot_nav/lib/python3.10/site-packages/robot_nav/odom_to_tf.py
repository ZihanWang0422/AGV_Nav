#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import StaticTransformBroadcaster
import traceback
import sys

class OdomToTF(Node):
    def __init__(self):
        try:
            super().__init__('odom_to_tf')
            self.get_logger().info('Initializing odom_to_tf node...')
            
            # 创建TF广播器
            self.get_logger().info('Creating TF broadcasters...')
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            self.static_broadcaster = StaticTransformBroadcaster(self)
            self.get_logger().info('TF broadcasters created successfully')
            
            # 发布静态TF: map -> odom
            self.get_logger().info('Publishing static transforms...')
            self.publish_static_transforms()
            
            # 定义要订阅的话题名称
            self.odom_topic = '/visual_slam/tracking/odometry'  # 使用视觉里程计
            self.get_logger().info(f'Setting up subscription to {self.odom_topic}')
            
            # 创建订阅者 - 使用视觉里程计
            self.odom_sub = self.create_subscription(
                Odometry,
                self.odom_topic,
                self.odom_callback,
                10)
            self.get_logger().info(f'Created subscription to {self.odom_topic}')
            
            # 添加计数器用于调试
            self.callback_count = 0
            self.last_callback_time = self.get_clock().now()
            
            # 创建定时器来检查订阅状态
            self.create_timer(5.0, self.check_subscription_status)
            self.get_logger().info('Initialization completed successfully')
            
        except Exception as e:
            error_msg = f'Error during initialization: {str(e)}\n{traceback.format_exc()}'
            # 如果logger已经创建，使用logger
            if hasattr(self, 'get_logger'):
                self.get_logger().error(error_msg)
            # 否则直接打印到stderr
            else:
                print(error_msg, file=sys.stderr)
            # 重新抛出异常，让ROS2知道初始化失败
            raise
            
    def check_subscription_status(self):
        """定期检查订阅状态"""
        try:
            now = self.get_clock().now()
            time_since_last = (now - self.last_callback_time).nanoseconds / 1e9
            self.get_logger().info(f'Odometry callbacks received: {self.callback_count}')
            self.get_logger().info(f'Time since last callback: {time_since_last:.1f} seconds')
            self.get_logger().info(f'Subscribed to topic: {self.odom_topic}')
        except Exception as e:
            self.get_logger().error(f'Error in status check: {str(e)}\n{traceback.format_exc()}')
        
    def publish_static_transforms(self):
        try:
            # 创建map -> odom的静态转换
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            # 初始位置设为原点
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            # 发布静态转换
            self.static_broadcaster.sendTransform(t)
            self.get_logger().info('Successfully published static transform: map -> odom')
        except Exception as e:
            error_msg = f'Error publishing static transform: {str(e)}\n{traceback.format_exc()}'
            self.get_logger().error(error_msg)
            raise
        
    def odom_callback(self, msg):
        try:
            self.callback_count += 1
            self.last_callback_time = self.get_clock().now()
            
            if self.callback_count == 1 or self.callback_count % 100 == 0:
                self.get_logger().info(f'Received odometry message #{self.callback_count}')
            
            # 创建odom -> base_link的转换
            t = TransformStamped()
            
            # 设置时间戳和坐标系
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            
            # 设置位置
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            
            # 设置方向
            t.transform.rotation = msg.pose.pose.orientation
            
            # 发布转换
            self.tf_broadcaster.sendTransform(t)
            
            if self.callback_count == 1 or self.callback_count % 100 == 0:
                self.get_logger().info(f'Published transform #{self.callback_count}: odom -> base_link')
                
        except Exception as e:
            self.get_logger().error(f'Error in odometry callback: {str(e)}\n{traceback.format_exc()}')

def main():
    try:
        rclpy.init()
        node = OdomToTF()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            node.get_logger().error(f'Error in main loop: {str(e)}\n{traceback.format_exc()}')
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f'Critical error: {str(e)}\n{traceback.format_exc()}', file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main() 