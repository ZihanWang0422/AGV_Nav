#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

class MapConverter(Node):
    def __init__(self):
        super().__init__('map_converter')
        
        # 创建订阅者和发布者
        qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            '/visual_slam/vis/observations_cloud',
            self.cloud_callback,
            qos_sub)
        
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', qos_pub)
        
        # 地图参数
        self.resolution = 0.05  # 5cm per cell
        self.width = 400      # 20m / 0.05
        self.height = 400     # 20m / 0.05
        
        # 创建空地图
        self.map_msg = OccupancyGrid()
        self.map_msg.info.resolution = self.resolution
        self.map_msg.info.width = self.width
        self.map_msg.info.height = self.height
        self.map_msg.info.origin.position.x = -10.0  # -width*resolution/2
        self.map_msg.info.origin.position.y = -10.0  # -height*resolution/2
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0
        
        # 初始化地图数据
        self.map_data = np.full((self.height, self.width), -1, dtype=np.int8)
        self.map_msg.data = list(self.map_data.flatten())
        
        # 创建计时器，定期发布地图
        self.create_timer(1.0, self.publish_map)  # 每秒发布一次地图
        
        self.get_logger().info('Map converter node initialized')
        self.point_count = 0
        
    def cloud_callback(self, msg):
        try:
            # 从点云中读取点
            points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
            
            if len(points) == 0:
                self.get_logger().warn('Received empty point cloud')
                return
                
            # 将点云坐标转换为地图坐标
            map_points_x = ((points[:, 0] - self.map_msg.info.origin.position.x) / self.resolution).astype(int)
            map_points_y = ((points[:, 1] - self.map_msg.info.origin.position.y) / self.resolution).astype(int)
            
            # 过滤掉地图范围外的点
            valid_points = (map_points_x >= 0) & (map_points_x < self.width) & \
                         (map_points_y >= 0) & (map_points_y < self.height)
            map_points_x = map_points_x[valid_points]
            map_points_y = map_points_y[valid_points]
            
            # 更新地图
            self.map_data[map_points_y, map_points_x] = 100
            
            # 膨胀障碍物
            kernel_size = 3
            occupied = self.map_data == 100
            dilated = np.zeros_like(self.map_data)
            
            for i in range(-(kernel_size//2), kernel_size//2 + 1):
                for j in range(-(kernel_size//2), kernel_size//2 + 1):
                    rolled = np.roll(np.roll(occupied, i, axis=0), j, axis=1)
                    dilated |= rolled
            
            # 更新地图数据
            self.map_data[dilated] = 100
            # 将未占用的区域标记为自由空间
            self.map_data[~dilated & (self.map_data == -1)] = 0
            
            # 更新时间戳
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_msg.header.frame_id = 'map'
            self.map_msg.info.map_load_time = self.get_clock().now().to_msg()
            
            # 更新地图消息数据
            self.map_msg.data = list(self.map_data.flatten())
            
            # 打印调试信息
            self.point_count += len(points)
            if self.point_count % 1000 == 0:  # 每处理1000个点打印一次
                self.get_logger().info(f'Total points processed: {self.point_count}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
    
    def publish_map(self):
        """定期发布地图"""
        if hasattr(self, 'map_msg'):
            self.map_publisher.publish(self.map_msg)
            self.get_logger().info('Published map')

def main():
    rclpy.init()
    node = MapConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 