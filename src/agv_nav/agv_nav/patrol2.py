import rclpy
import numpy as np
import struct
import time
import tf2_ros
import threading
import queue
import math

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Float32, Int32MultiArray

class AGVNavigator(Node):
    def __init__(self):
        super().__init__('agv_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = [
            self.create_pose(3.2413, -1.1711, (0.0, 0.0, 0.95645, 0.29188)),  # 첫
            self.create_pose(-3.4533, 3.5679, (0.0, 0.0, 0.92379, 0.3829))  # 두
        ]
        self.current_waypoint_index = 0

        self.person_detected = False
        self.camera_info_received = False
        self.person_positions = None  # 수정: self.person_positions 초기화
        self.point_cloud_data = None
        self.last_detection_time = time.time()  # 마지막으로 감지된 시간 기록

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.target_frame = 'map'
        self.base_link_frame = 'base_link'

        # 손든 사람 감지 결과를 받는 subscriber
        self.person_detected_sub = self.create_subscription(
            String, '/person_detection_status',self.object_detection_callback, 10)

        # 사람 위치를 받는 subscriber
        self.position_sub = self.create_subscription(
            Int32MultiArray, '/closest_person_position', self.position_callback, 10)

        # 카메라 포인트 클라우드 정보를 받는 subscriber
        self.depth_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.camera_info_callback, 10)

        self.navigate_to_next_pose()

    def object_detection_callback(self, msg):
        # 사람이 감지되었음을 수신
        if msg.data == "person":
            self.person_detected = True
            self.last_detection_time = time.time()  # 감지된 시간 기록
        else:
            self.person_detected = False        

    def position_callback(self, msg):
        self.person_positions = np.array(msg.data).reshape(-1, 2)

    def camera_info_callback(self, msg):
        # 카메라 포인트 클라우드 정보를 수신
        self.point_cloud_data = msg
        self.camera_info_received = True

    def calculate_distance_to_person(self):
        if self.person_detected and self.camera_info_received and self.person_positions is not None \
                and len(self.person_positions) > 0 and time.time() - self.last_detection_time < 6:  # 최근 6초 이내에 감지된 경우
            self.cancel_goal()
            for person_position in self.person_positions:  # 모든 사람의 위치에 대해 반복
                u, v = person_position  # 사람의 위치에서 u, v 추출
                width, height = self.point_cloud_data.width, self.point_cloud_data.height
                point_step = self.point_cloud_data.point_step
                row_step = self.point_cloud_data.row_step

                data = self.point_cloud_data.data
                format = 'fff'
                offset = (v * row_step) + (u * point_step)
                a, b, z = struct.unpack_from(format, data, offset)

                self.move_forward(a, b)

                # 거리를 계산
                distance = np.sqrt(a ** 2 + b ** 2 + z ** 2)

                # 거리가 무한대이면 pub하지 않음
                if not np.isinf(distance):
                    # 거리를 터미널에 출력
                    self.get_logger().info("Distance to person: {:.2f} meters".format(distance))
                else:
                    # 무한대인 경우에는 중지 메시지를 게시하지 않고 로그만 출력
                    self.get_logger().info("Distance to closest person is infinity. Not publishing.")

    def move_forward(self, a, b):
        if self.current_pose is None:
            self.get_logger().info('현재 위치를 알 수 없습니다. 앞으로 이동할 수 없습니다.')
            return

        # 현재 방향(쿼터니언)을 yaw 각도로 변환
        orientation = self.current_pose.pose.orientation
        yaw = self.quaternion_to_yaw(orientation)

        # yaw 각도와 입력받은 거리를 사용하여 새로운 목표 위치 계산
        x = self.current_pose.pose.position.x + a * math.cos(yaw) - b * math.sin(yaw)
        y = self.current_pose.pose.position.y + b * math.cos(yaw) + a * math.sin(yaw)

        new_pose = self.create_pose(x, y, (
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w,
        ))

        self.navigate_to_pose(new_pose)                    

    def create_pose(self, x, y, quaternion):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

    def navigate_to_next_pose(self):
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.current_waypoint_index = 0

        goal = NavigateToPose.Goal()
        goal.pose = self.waypoints[self.current_waypoint_index]
        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

        # 3초간 대기
        time.sleep(3)

        self.current_waypoint_index += 1

        # 다음 목적지로 이동
        self.navigate_to_next_pose()

    def update_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.base_link_frame, rclpy.time.Time())
            position = transform.transform.translation
            orientation = transform.transform.rotation

            self.current_pose = self.create_pose(position.x, position.y, (orientation.x, orientation.y, orientation.z, orientation.w))
            self.get_logger().info(f'현재 위치 업데이트: ({self.current_pose.pose.position.x}, {self.current_pose.pose.position.y})')
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().info(f'변환을 얻는 중 오류 발생: {e}')

    def cancel_goal(self):
        if self.goal_handle:
            self.get_logger().info('목표를 취소합니다...')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

def main(args=None):
    rclpy.init(args=args)
    navigator = AGVNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()