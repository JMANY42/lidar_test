import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField  # Import PointField
import struct

class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            "/wamv/sensors/lidars/lidar_wamv_sensor/points",
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        # Extract point cloud data
        data = msg.data
        point_step = msg.point_step
        fields = msg.fields

        num_points = int(len(data) / point_step)

        for i in range(num_points):
            offset = i * point_step
            point = {}
            for field in fields:
                field_name = field.name
                field_offset = field.offset
                field_dtype = field.datatype
                if field_dtype == PointField.FLOAT32:
                    value = struct.unpack('f', data[offset + field_offset:offset + field_offset + 4])[0]
                elif field_dtype == PointField.FLOAT64:
                    value = struct.unpack('d', data[offset + field_offset:offset + field_offset + 8])[0]
                elif field_dtype == PointField.INT32 or field_dtype == PointField.UINT32:
                    value = struct.unpack('I', data[offset + field_offset:offset + field_offset + 4])[0]
                # Add more cases for other datatypes as needed
                point[field_name] = value

            x = point['x']
            y = point['y']
            z = point['z']
            if z>0.5:
            	self.get_logger().info(f"Point: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            # You can add your processing logic here
            # For example, filtering, transformation, object detection, etc.

def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    rclpy.spin(subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()