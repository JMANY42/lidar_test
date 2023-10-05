#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class print_lidar_output(Node):
    
    def __init__(self):
        super().__init__("print_lidar_output")
        self.get_logger().info("its working")
        self.lidar_subscriber = self.create_subscription(
            PointCloud2, "/wamv/sensors/lidars/lidar_wamv_sensor/points", self.PointCloud2Callback, 10)
    
    def PointCloud2Callback(self, msg:PointCloud2):
        self.get_logger().info(str(msg))
        #self.get_logger().info(str(msg.point_step))
        
        

def main(args=None):
    rclpy.init(args=args)
    node = print_lidar_output()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
