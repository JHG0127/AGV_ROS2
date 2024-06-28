import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Float32MultiArray, Int32MultiArray
import numpy as np
import struct
import time


class ObjectCalcNode(Node):
    def __init__(self):
        super().__init__('object_calc_node')

        self.person_detected = False
        self.camera_info_received = False
        self.person_positions = None
        self.point_cloud_data = None
        self.last_detection_time = time.time()

        self.person_detected_sub = self.create_subscription(
            String, '/person_detection_status', self.object_detection_callback, 10)

        self.position_sub = self.create_subscription(
            Int32MultiArray, '/closest_person_position', self.position_callback, 10)

        self.depth_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.camera_info_callback, 10)

        self.distance_pub = self.create_publisher(
            Float32MultiArray, '/person_camera_distance', 10)  # Use Float32MultiArray

        self.timer = self.create_timer(1.0, self.calculate_distance_to_person)

    def object_detection_callback(self, msg):
        if msg.data == "person":
            self.person_detected = True
            self.last_detection_time = time.time()
        else:
            self.person_detected = False

    def position_callback(self, msg):
        self.person_positions = np.array(msg.data).reshape(-1, 2)

    def camera_info_callback(self, msg):
        self.point_cloud_data = msg
        self.camera_info_received = True

    def calculate_distance_to_person(self):
        if self.person_detected and self.camera_info_received and self.person_positions is not None \
                and len(self.person_positions) > 0 and time.time() - self.last_detection_time < 6:
            for person_position in self.person_positions:
                u, v = person_position
                width, height = self.point_cloud_data.width, self.point_cloud_data.height
                point_step = self.point_cloud_data.point_step
                row_step = self.point_cloud_data.row_step

                data = self.point_cloud_data.data
                format = 'fff'
                offset = (v * row_step) + (u * point_step)
                x, y, z = struct.unpack_from(format, data, offset)

                distance = np.sqrt(x ** 2 + y ** 2 + z ** 2)

                if not np.isinf(distance):
                    msg = Float32MultiArray(data=[x, z])
                    self.distance_pub.publish(msg)
                    self.get_logger().info(f'x: {x}, y: {y}, z: {z}, distance: {distance}')
                else:
                    self.get_logger().info("Distance to closest person is infinity. Not publishing.")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectCalcNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
