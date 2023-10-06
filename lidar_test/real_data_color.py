#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
from sklearn.cluster import DBSCAN
from sensor_msgs_py import point_cloud2

class publish_lidar_with_color(Node):
    def __init__(self):
        super().__init__("real_data_color")
        
        self.get_logger().info("this is working")
        self.pcd_publisher = self.create_publisher(PointCloud2,'pc2_with_color_real_data',10)
        self.subscription = self.create_subscription(PointCloud2,
            "/wamv/sensors/lidars/lidar_wamv_sensor/points",self.listener_callback,10)
        self.subscription

        timer_period = .5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.pcd = PointCloud2()
    def timer_callback(self):
        self.pcd_publisher.publish(self.pcd)
    
    def listener_callback(self,msg:PointCloud2):
        print("data received")
        self.pcd = point_cloud(msg)


def point_cloud(msg):

    points = np.array(list(point_cloud2.read_points(msg)))

    mask = np.isinf(points).any(axis=1)

    points = points[~mask]
    

    db = DBSCAN(eps=.5, min_samples=2).fit(points)
    labels = db.labels_
    max = np.max(labels)


    points = np.delete(points, np.where(labels == -1),axis=0)
    labels = np.delete(labels, np.where(labels == -1))


    print("max: "+str(max))
    print(len(points))
    print(len(labels))

    colors = np.random.randint(255,size=(max+2,3))
    colors[0] = [0,0,0]


    fields = [PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1),
            PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1),
            PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1),
            PointField(name='rgba',offset=12,datatype=PointField.UINT32,count=1)]

    header = Header()
    header.frame_id = "map"

    point_struct = struct.Struct("<fffBBBB")

    buffer = bytearray(point_struct.size * len(points))

    for i, point in enumerate(points):
        point_struct.pack_into(
            buffer, i * point_struct.size, point[0], point[1], point[2], colors[labels[i]+1][0], colors[labels[i]+1][1], colors[labels[i]+1][2],255
        )
    return PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(16),
        row_step=(24*points.shape[0]),
        data=buffer
    )

def main(args=None):
    rclpy.init(args=args)
    publisher = publish_lidar_with_color()
    rclpy.spin(publisher)

if __name__ == '__main__':
    main()