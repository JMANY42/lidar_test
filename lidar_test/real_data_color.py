#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
from sklearn.cluster import DBSCAN

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

    points = np.array(list(read_points(msg)))

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





import sys
from collections import namedtuple
import ctypes
import math
import struct

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

def main(args=None):
    rclpy.init(args=args)
    publisher = publish_lidar_with_color()
    rclpy.spin(publisher)

if __name__ == '__main__':
    main()