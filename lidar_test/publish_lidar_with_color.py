#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import random
from sklearn.cluster import DBSCAN

class publish_lidar_with_color(Node):
    def __init__(self):
        super().__init__("publish_lidar_with_color")
        
        self.get_logger().info("this is working")
        self.pcd_publisher = self.create_publisher(PointCloud2,'pc2_with_color',10)
        timer_period = .5
        self.timer = self.create_timer(timer_period,self.timer_callback)
    
    def timer_callback(self):
        self.pcd = point_cloud()
        self.pcd_publisher.publish(self.pcd)


def point_cloud():

    points = np.empty((100000,3),float)

    for i in range(len(points)):
        points[i][0] = random.uniform(-10.0,10.0)
        points[i][1] = random.uniform(-10.0,10.0)
        points[i][2] = random.uniform(-10.0,10.0)


    print(points)
    print(len(points))

    db = DBSCAN(eps=.5, min_samples=10).fit(points)
    labels = db.labels_
    print(labels)

    max = np.max(labels)
    i=0

    for x in labels:
        if x==-1:
            points = np.delete(points,i,axis=0)
            labels = np.delete(labels,i)
            i-=1
        i+=1
    print("max: "+str(max))

    colors = np.empty((max+2,3),int)
    for color in colors:
        color[0] = random.randint(0,255)
        color[1] = random.randint(0,255)
        color[2] = random.randint(0,255)
    colors[0] = [0,0,0]


    fields = [PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1),
            PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1),
            PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1),
            PointField(name='rgba',offset=12,datatype=PointField.UINT32,count=1)]

    header = Header()
    header.frame_id = "map"

    itemsize1 = 4#np.dtype(PointField.FLOAT32).itemsize
    itemsize2= 4#np.dtype(PointField.UINT32).itemsize

    point_struct = struct.Struct("<fffBBBB")

    buffer = bytearray(point_struct.size * len(points))
    i=0
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
        row_step=((itemsize1*3 + itemsize2*3)*points.shape[0]),
        data=buffer
    )

def main(args=None):
    rclpy.init(args=args)
    publisher = publish_lidar_with_color()
    rclpy.spin(publisher)

if __name__ == '__main__':
    main()